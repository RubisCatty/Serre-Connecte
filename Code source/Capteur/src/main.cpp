// Bibliothèques Arduino pour ESP32
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Définitions des broches
#define BOOT_BUTTON 0
#define ADC_PIN 9           // Broche analogique du capteur d'humidité
#define ADC_BATTERIE 11    // Broche analogique pour la mesure de batterie
#define ENABLE 10          // Broche pour activer/désactiver la lecture de la batterie
#define LED 3

// Temps d'attente pour le réveil profond : 10 minutes
#define uS_TO_S       1000000ULL
#define SLEEP_NORMAL  (10 * 60 * uS_TO_S)

// Structure de données envoyée via ESP-NOW
typedef struct {
  uint8_t humidite;
  uint8_t batterie;
} HUMIDITE;

typedef struct {
  uint8_t type;
} ETAT;

// Variables globales pour stocker les pourcentages mesurés
int pourcent_hum = 0;    // Pourcentage d'humidité calculé
int pourcent_bat = 0;    // Pourcentage de batterie calculé

// Adresse MAC du récepteur (Serre)
#define MAC_SERRE {0xD0, 0xCF, 0x13, 0x18, 0x30, 0x30}
uint8_t receiverMAC[6] = MAC_SERRE;

// Fonction pour envoyer les données de capteur à un appareil distant
void SendSettingsTo(const uint8_t *mac) {
  HUMIDITE setting;
  setting.humidite = pourcent_hum;  // Assigner le pourcentage d'humidité
  setting.batterie = pourcent_bat;  // Assigner le pourcentage de batterie

  // Envoi des données de capteur via ESP-NOW
  esp_err_t result = esp_now_send(mac, (uint8_t *)&setting, sizeof(setting));
}

// Fonction de rappel (callback) pour gérer la réception de données ESP-NOW
void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // Convertir l'adresse MAC en chaîne formatée
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  
  if (len > 0) {
    uint8_t msgType = incomingData[0];
    // Si le type de message est 0x02, mettre le capteur en mode sommeil profond
    if (msgType == 0x02) {
      digitalWrite(ENABLE, LOW); // éteindre la lecture de la batterie
      esp_sleep_enable_timer_wakeup(SLEEP_NORMAL);
      esp_deep_sleep_start();
    }
  }
}

// Initialisation du système
void setup() {
  // Initialiser la communication série pour le débogage
  Serial.begin(9600);
  delay(1000);

  // Configuration des convertisseurs analogiques-numériques (ADC)
  analogReadResolution(12);                    // résolution 12 bits (0 à 4095)
  analogSetPinAttenuation(ADC_PIN, ADC_11db);  // atténuation pour plage ~0-3.3..3.9V
  analogSetPinAttenuation(ADC_BATTERIE, ADC_11db);
  pinMode(ADC_PIN, INPUT);      // capteur d'humidité en entrée
  pinMode(ADC_BATTERIE, INPUT); // capteur de batterie en entrée

  // Configuration des sorties numériques
  pinMode(ENABLE, OUTPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(ENABLE, HIGH); // activer la lecture de la batterie

  // Initialisation de la communication sans fil ESP-NOW
  WiFi.mode(WIFI_STA);           // Mode station WiFi (client)
  WiFi.disconnect();              // éviter les connexions automatiques WiFi
  
  if (esp_now_init() != ESP_OK) {
  } else {
    // Déclarer le pair destinataire avec son adresse MAC
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, receiverMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false; 
    esp_now_add_peer(&peerInfo);
  }
  // Enregistrer la fonction de rappel pour la réception
  esp_now_register_recv_cb(onDataRecv);
}

// Boucle principale - exécutée continuellement
void loop() {
  // Lire les valeurs analogiques brutes des capteurs
  int raw = analogRead(ADC_PIN);        // lecture du capteur d'humidité
  int raw_bat = analogRead(ADC_BATTERIE); // lecture du capteur de batterie

  // ===== Conversion de l'humidité en pourcentage =====
  int tmp1 = raw - 1400; // soustraire la valeur de calibration de base (1300 pour le premier capteur et 1400 pour le second)
  if (tmp1 < 0) {
    // Valeur en dessous du minimum : saturation complète (100%)
    tmp1 = 100;
  } else if (tmp1 > 1900) {
    // Valeur au-dessus du maximum : sec complètement (0%)
    tmp1 = 0;
  } else {
    // Valeur dans la plage normale : normaliser de 0 à 100
    tmp1 = 100 - (tmp1 / 19);// normalisation : (tmp1 * 100 / 1900) = (tmp1 / 19)
  }
  pourcent_hum = tmp1;

  // ===== Conversion de la batterie en pourcentage =====
  int tmp2 = raw_bat - 2300; // soustraire la valeur de calibration
  if (tmp2 < 0) {
    // Tension trop basse : batterie vide (0%)
    tmp2 = 0;
  } else if (tmp2 > 700) {// Comparation avec la plage maximale attendue (600 pour le premier capteur et 700 pour le second)
    // Tension trop haute : batterie pleine (100%)
    tmp2 = 100;
  } else {
    // Tension dans la plage normale : calculer le pourcentage
    tmp2 = tmp2 / 7; // normalisation :(tmp2 * 100 / 600) = (tmp2 / 6) ou (tmp2 * 100 / 700) = (tmp2 / 7) selon le capteur utilisé
  }
  pourcent_bat = tmp2;
  // Envoyer les données calculées au récepteur via ESP-NOW
  SendSettingsTo(receiverMAC);
  // Attendre 1 seconde avant la prochaine mesure et envoi
  delay(1000);
}
