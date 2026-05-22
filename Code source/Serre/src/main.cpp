// Bibliothèques
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "esp32cam.h"
#include "esp_camera.h"
#include <SensirionI2cSht4x.h>
#include <Wire.h>

// Résolution de la caméra et intervalle de capture.
const auto RESOLUTION = esp32cam::Resolution::find(320, 240);
const uint32_t CAPTURE_INTERVAL_MS = 5000;
#define MAX_PAYLOAD 241
static uint16_t imageIdCounter = 0;

// Paramètres de veille profonde et limite de fonctionnement de la pompe.
#define uS_TO_S       1000000ULL
#define SLEEP_NORMAL  (10 * 60 * uS_TO_S)  // 10 minutes
#define PUMP_TIMEOUT_MS (60 * 1000)        // 1 minute max pompe active

// Broches matérielles utilisées par le système.
#define BOOT_BUTTON 0
#define ENABLE 1
#define POMPE 48
#define SDA_PIN 4
#define SCL_PIN 5
#define NO_ERROR 0

static char errorMessage[64];
static int16_t error;
SensirionI2cSht4x sensor;
static bool espNowReady = false;

#define MAC_MAISON {0x80, 0xB5, 0x4E, 0xE8, 0x13, 0x18}
uint8_t MACM[6] = MAC_MAISON;
uint8_t broadcastAddr[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Structure de paquet pour l'envoi d'images en plusieurs chunks.
typedef struct __attribute__((packed)){
  uint8_t type;
  uint16_t imageId;
  uint16_t chunkNumber;
  uint16_t totalChunks;
  uint16_t payloadLen;
  uint8_t imageData[MAX_PAYLOAD];
} ImagePayload;

typedef struct{
  uint8_t type;
  uint8_t temperature;
  uint8_t humidite;
  uint8_t batterie1;
  uint8_t batterie2;
  uint8_t humidite1;
  uint8_t humidite2;
} ENVOIE_DONNEE;

typedef struct {
  uint8_t humidite;
  uint8_t batterie;
} CAPTEURS;

typedef struct {
  uint8_t humidite;
} HUMIDITEVOULU;

typedef struct {
  uint8_t type;
} ETAT;

ENVOIE_DONNEE serre = {0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
float aTemperature = 0.0;
float aHumidity = 0.0;
bool pompeActive = false;

// RTC_DATA_ATTR : survivent au deep sleep
RTC_DATA_ATTR HUMIDITEVOULU maison = {50};
RTC_DATA_ATTR bool pompeEtaitActive = false;  // pour reprendre l'état de la pompe au réveil

void goToSleep() {
  // Éteint la pompe et l'alimentation, envoie un signal de veille, puis passe en deep sleep.
  digitalWrite(POMPE, LOW);
  digitalWrite(ENABLE, LOW);
  Serial.flush();
  ETAT tmp={0x02};
  esp_now_send(broadcastAddr, (uint8_t *)&tmp, sizeof(tmp));
  delay(200);
  esp_sleep_enable_timer_wakeup(SLEEP_NORMAL);
  esp_deep_sleep_start();
}

void initCamera() {
  // Initialise la caméra et ajuste les paramètres d'image.
  using namespace esp32cam;
  constexpr Pins cameraPins{
    D0: 11, D1: 9,  D2: 8,  D3: 10,
    D4: 12, D5: 18, D6: 17, D7: 16,
    XCLK: 15, PCLK: 13, VSYNC: 6, HREF: 7,
    SDA: 4, SCL: 5, RESET: -1, PWDN: 14,
  };
  Config cfg;
  cfg.setPins(cameraPins);
  cfg.setResolution(RESOLUTION);
  cfg.setBufferCount(2);
  cfg.setJpeg(80);
  bool ok = Camera.begin(cfg);
  delay(500);
  if (!ok) {
  } else {
    sensor_t* s = esp_camera_sensor_get();
    if (s != nullptr) {
      s->set_whitebal(s, 1);
      s->set_awb_gain(s, 1);
      s->set_wb_mode(s, 1);
      s->set_saturation(s, 2);
      s->set_contrast(s, 2);
      s->set_colorbar(s, 0);
      s->set_special_effect(s, 0);
    }
  }
}

void initEspNow() {
  // Initialise ESP-NOW et ajoute le peer principal ainsi que le broadcast.
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  if (esp_now_init() != ESP_OK) {
    while (true) delay(1000);
  }
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, MACM, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    while (true) delay(1000);
  }
  esp_now_peer_info_t bcastPeer{};
  memcpy(bcastPeer.peer_addr, broadcastAddr, 6);
  bcastPeer.channel = 0;
  bcastPeer.encrypt = false;
  esp_now_add_peer(&bcastPeer);
  espNowReady = true;
}

void sendFrame(const uint8_t* data, size_t len) {
  // Envoie les données d'image en plusieurs paquets.
  uint16_t totalChunks = (len + MAX_PAYLOAD - 1) / MAX_PAYLOAD;
  for (uint16_t i = 0; i < totalChunks; i++) {
    size_t offset   = i * MAX_PAYLOAD;
    size_t chunkLen = min((size_t)MAX_PAYLOAD, len - offset);
    ImagePayload pkt{};
    pkt.type        = 0x03;
    pkt.imageId     = imageIdCounter;
    pkt.chunkNumber = i;
    pkt.totalChunks = totalChunks;
    pkt.payloadLen  = (uint16_t)chunkLen;
    memcpy(pkt.imageData, data + offset, chunkLen);
    esp_now_send(MACM, (const uint8_t*)&pkt,
                 offsetof(ImagePayload, imageData) + chunkLen);
    delay(10);
  }
  imageIdCounter++;
}

void captureAndSend() {
  // Capture une image depuis la caméra et envoie les données si la capture est valide.
  auto frame = esp32cam::capture();
  if (!frame) { return; }
  sendFrame(frame->data(), frame->size());
}

void initSensor() {
  // Initialise le capteur SHT40 sur le bus I2C.
  sensor.begin(Wire, SHT40_I2C_ADDR_44);
  sensor.softReset();
  delay(10);
  uint32_t serialNumber = 0;
  error = sensor.serialNumber(serialNumber);
}

void readSensor() {
  // Lit la température et l'humidité du capteur SHT40.
  delay(20);
  error = sensor.measureLowestPrecision(aTemperature, aHumidity);
}

void sendSensor(const uint8_t *mac) {
  // Prépare le paquet de données et l'envoie au peer ESP-NOW.
  serre.temperature = int(aTemperature - 10);
  serre.humidite    = int(aHumidity);
  esp_now_send(mac, (uint8_t *)&serre, sizeof(serre));
}

void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // Callback de réception ESP-NOW pour récupérer la commande et les données capteurs.
  if (len == sizeof(HUMIDITEVOULU)) {
    memcpy(&maison, incomingData, sizeof(HUMIDITEVOULU));
  }
  if (len == sizeof(CAPTEURS)) {
    CAPTEURS tmp;
    memcpy(&tmp, incomingData, sizeof(CAPTEURS));
    if (mac[5] == 0x54) {
      serre.humidite1 = tmp.humidite;
      serre.batterie1 = tmp.batterie;
    } else {
      serre.humidite2 = tmp.humidite;
      serre.batterie2 = tmp.batterie;
    }
  }
}

bool gererPompe() {
  uint8_t h = (serre.humidite1 + serre.humidite2) / 2;  // adapte selon tes capteurs actifs
  if (h <= (maison.humidite - 10)) {
    digitalWrite(POMPE, HIGH);
    pompeActive = true;
    pompeEtaitActive = true;
    return false;  // pas prêt à dormir
  } else if (h >= (maison.humidite + 10)) {
    digitalWrite(POMPE, LOW);
    pompeActive = false;
    pompeEtaitActive = false;
    return true;   // OK pour dormir
  }
  // Zone neutre
  digitalWrite(POMPE, pompeActive ? HIGH : LOW);
  return !pompeActive;
}

void setup() {
  Serial.begin(9600);
  delay(1000);

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  pinMode(ENABLE, OUTPUT);
  pinMode(POMPE, OUTPUT);
  digitalWrite(ENABLE, HIGH);
  digitalWrite(POMPE, LOW);

  // Restaurer l'état de la pompe si elle était active avant le sleep.
  pompeActive = pompeEtaitActive;

  // Initialisation des périphériques.
  initCamera();
  Wire.begin(SDA_PIN, SCL_PIN);
  initEspNow();
  initSensor();
  esp_now_register_recv_cb(onDataRecv);

  ETAT tmp={0x01};
  esp_now_send(MACM, (uint8_t *)&tmp, sizeof(tmp));

  // Travail principal : capture d'image, lecture et envoi des données.
  captureAndSend();
  readSensor();
  sendSensor(MACM);

  bool pret = gererPompe();
  if (!pret) {
    // Pompe active — attendre jusqu'à ce qu'elle puisse s'arrêter ou timeout.
    uint32_t start = millis();
    while (!pret && (millis() - start < PUMP_TIMEOUT_MS)) {
      delay(5000);
      readSensor();
      sendSensor(MACM);
      pret = gererPompe();
    }
  }
  delay(5000);
  sendSensor(MACM);
  goToSleep();
}

void loop() {
  // Tout est dans setup() avec deep sleep — loop() ne sera jamais appelé
}