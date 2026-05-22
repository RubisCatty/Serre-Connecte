
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <TJpg_Decoder.h>

//GPIO des bouttons
#define BOOT_BUTTON 0
#define PAGE_BUTTON 47
#define UP_BUTTON 45
#define DOWN_BUTTON 48

//Pages
#define PAGE_CAM 0
#define PAGE_DONNEE 1

//Identifiant du type de message
#define REVEIL 0X01
#define SLEEP 0X02
#define IMAGE 0X03
#define DONNEE 0X04

#define MAX_PAYLOAD 241
#define MAX_IMAGE_SIZE 16384

#define  MAC_SERRE {0xD0, 0xCF, 0x13, 0x18, 0x30, 0x30}
uint8_t MACS[6] = MAC_SERRE;

static bool espNowReady = false;

//Structures de données
typedef struct{
  uint8_t type;
  uint8_t temperature;
  uint8_t humidite;
  uint8_t batterie1;
  uint8_t batterie2;
  uint8_t humidite1;
  uint8_t humidite2;
}CAPTEUR_DATA;

typedef struct {
  uint8_t humidite;
} HUMIDITE;

typedef struct __attribute__((packed)){
  uint8_t type;
  uint16_t imageId;         // ID pour suivre le morceau d'image
  uint16_t chunkNumber;     // Numéro de séquence du morceau (0, 1, 2, ...)
  uint16_t totalChunks;     // Nombre total de morceaux pour cette image
  uint16_t payloadLen;      // Longueur réelle des données d'image dans ce morceau
  uint8_t imageData[MAX_PAYLOAD];  // Données du morceau d'image
} ImagePayload;

//Variable globale
TFT_eSPI tft = TFT_eSPI();
uint8_t currentPage = PAGE_CAM;
HUMIDITE humidite = {50};  // Valeurs par défaut
CAPTEUR_DATA serre;
bool serreNewImageReceived = false;
bool awake=false;

// Gestion du rebond du bouton
unsigned long lastButtonPress = 0;
unsigned long lastIncPress = 0;
unsigned long lastDecPress = 0;
const unsigned long DEBOUNCE_TIME = 300;  // millisecondes
const uint8_t HUMIDITY_STEP = 1;

// Buffer d'assemblage JPEG
uint8_t  imgBuffer[MAX_IMAGE_SIZE];
size_t   imgLen        = 0;
uint16_t currentImgId  = 0xFFFF; // ID de l'image en cours d'assemblage
volatile bool imageReady = false;

// Timestamp de la dernière image reçue
unsigned long lastImageTime = 0;
unsigned long lastTimeUpdate = 0; // Pour mettre a jour le temps chaque seconde

bool tft_output(int16_t x, int16_t y, uint16_t w, uint16_t h, uint16_t* bitmap) {
  if (y >= tft.height()) return 0;
  
  // Swap manuel des bytes si les couleurs sont mauvaises
  for (int i = 0; i < w * h; i++) {
    bitmap[i] = (bitmap[i] >> 8) | (bitmap[i] << 8);
  }
  
  tft.pushImage(x, y, w, h, bitmap);
  return 1;
}

// ── Affichage du temps écoulé depuis la dernière photo ────────────────────────

String getElapsedTimeString(unsigned long lastTime) {
  if (lastTime == 0) {
    return "Aucune photo";
  }
  unsigned long elapsed = (millis() - lastTime) / 1000; // convertir en secondes
  unsigned long seconds = elapsed;
  String timeStr = "";
  timeStr += String(seconds) + "s";
  return timeStr;
}

// ── Mise à jour du temps écoulé (sans redessiner l'image) ──────────────────────

void updateImageTime() {
  if (currentPage == PAGE_CAM && lastImageTime > 0) {
    unsigned long now = millis();
    // Mettre a jour le temps uniquement toutes les secondes
    if (now - lastTimeUpdate >= 1000) {
      lastTimeUpdate = now;
      
      String elapsedStr = getElapsedTimeString(lastImageTime);
      tft.fillRect(10, 260, 240, 25, TFT_BLACK);
      tft.setTextColor(TFT_CYAN);
      tft.setTextSize(1);
      tft.drawString(elapsedStr, 10, 265, 2);
    }
  }
}

// ── Affichage de l'image JPEG assemblée ──────────────────────────────────────

void displayJpeg() {
  TJpgDec.setJpgScale(1);
  uint16_t w = 0, h = 0;
  TJpgDec.getJpgSize(&w, &h, imgBuffer, imgLen);

  uint16_t srcW = 320, srcH = 240;
  uint16_t x = (tft.width()  - srcW) / 2; // centré horizontalement → x=80
  uint16_t y = (280 - srcH) / 2;          // centré dans les 280px dispo → y=20

  TJpgDec.drawJpg(x, y, imgBuffer, imgLen);

  // Afficher le temps écoulé depuis la dernière photo
  lastTimeUpdate = millis();
  updateImageTime();
}

void displayImagePage() {
  tft.fillScreen(TFT_BLACK);
  
  // Pied de page
  tft.setTextColor(TFT_MAGENTA);
  tft.setTextSize(2);
  tft.drawString("Appuyez sur maison pour les parametres", 10, 290, 1);
  tft.drawLine(410,290,407,287,TFT_MAGENTA);
  tft.drawLine(410,291,407,288,TFT_MAGENTA);
}

void displaySettingsPage() {
  tft.fillScreen(TFT_BLACK);
  
  // Tracer la bordure
  tft.drawRect(10, 10, 460, 270, TFT_CYAN);
  
  // Affichage des paramètres
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  tft.drawString("Humidite", 65, 50, 1);
  //Accent
  tft.drawLine(153, 50, 157, 47, TFT_WHITE);
  tft.drawLine(153, 51, 157, 48, TFT_WHITE);
  tft.drawFastHLine(55, 70, 110, TFT_PINK); // Ligne de titre
  if (humidite.humidite < 10) {
    tft.drawString(String(humidite.humidite) + "%", 95, 110, 2);
  } else if (humidite.humidite > 99) {
    tft.drawString(String(humidite.humidite) + "%", 80, 110, 2);
  } else {
    tft.drawString(String(humidite.humidite) + "%", 90, 110, 2);
  }
  
  //bloc de séparation
  tft.drawFastVLine(200, 10, 270, TFT_CYAN); // Ligne verticale de séparation
  tft.drawFastHLine(200, 90, 270, TFT_PINK); 
  tft.drawFastHLine(200, 190, 270, TFT_PINK);
  
  // Indiquer boutons +/- (visuel)
  tft.setTextColor(TFT_GREEN);
  tft.setTextSize(4);
  tft.drawString("+", 30, 160, 1);
  tft.drawString("-", 30, 220, 1);

  //Affichage données température et humidité
  tft.setTextColor(TFT_YELLOW);
  tft.setTextSize(2);
  tft.drawString("Temperature", 220, 30, 1);
  tft.drawLine(273, 30, 278, 27, TFT_YELLOW);
  tft.drawLine(273, 31, 278, 28, TFT_YELLOW);
  if (serre.temperature < 10) {
    tft.drawString(String(serre.temperature), 400, 20, 2);
  } else if (serre.temperature > 99) {
    tft.drawString(String(serre.temperature), 370, 20, 2);
  } else {
    tft.drawString(String(serre.temperature), 390, 20, 2);
  }
  tft.drawCircle(425, 20, 2,TFT_YELLOW);
  tft.drawString("C", 430, 20, 2);

  tft.setTextColor(TFT_YELLOW);
  tft.drawString("Humidite", 250, 60, 1);
  tft.drawLine(340, 60, 345, 57, TFT_YELLOW);
  tft.drawLine(340, 61, 345, 58, TFT_YELLOW);
  if (serre.humidite < 10) {
    tft.drawString(String(serre.humidite) + "%", 400, 50, 2);
  } else if (serre.humidite > 99) {
    tft.drawString(String(serre.humidite) + "%", 370, 50, 2);
  } else {
    tft.drawString(String(serre.humidite) + "%", 390, 50, 2);
  }

  //Chiffre capteur
  tft.drawString("1", 210, 100, 1);
  tft.drawString("2", 210, 200, 1);
  tft.drawFastHLine(200, 120, 30, TFT_BLUE);
  tft.drawFastVLine(230, 90, 30, TFT_BLUE);
  tft.drawFastHLine(200, 220, 30, TFT_BLUE);
  tft.drawFastVLine(230, 190, 30, TFT_BLUE);

  //Affichage données humidité du sol capteur 1
  tft.setTextColor(TFT_ORANGE);
  tft.drawString("Humidite", 250, 110, 1);
  tft.drawLine(340, 110, 345, 107, TFT_ORANGE);
  tft.drawLine(340, 111, 345, 108, TFT_ORANGE);
  if (serre.humidite1 < 10) {
    tft.drawString(String(serre.humidite1) + "%", 400, 100, 2);
  } else if (serre.humidite1 > 99) {
    tft.drawString(String(serre.humidite1) + "%", 370, 100, 2);
  } else {
    tft.drawString(String(serre.humidite1) + "%", 390, 100, 2);
  }
  //Batterie
  tft.setTextColor(TFT_DARKGREEN);
  tft.drawString("Batterie", 250, 150, 1);
  if (serre.batterie1 < 10) {
    tft.drawString(String(serre.batterie1) + "%", 400, 140, 2);
  } else if (serre.batterie1 > 99) {
    tft.drawString(String(serre.batterie1) + "%", 370, 140, 2);
  } else {
    tft.drawString(String(serre.batterie1) + "%", 390, 140, 2);
  }

  //Affichage données humidité du sol capteur 2
  tft.setTextColor(TFT_ORANGE);
  tft.drawString("Humidite", 250, 210, 1);
  tft.drawLine(340, 210, 345, 207, TFT_ORANGE);
  tft.drawLine(340, 211, 345, 208, TFT_ORANGE);
  if (serre.humidite2 < 10) {
    tft.drawString(String(serre.humidite2) + "%", 400, 200, 2);
  } else if (serre.humidite2 > 99) {
    tft.drawString(String(serre.humidite2) + "%", 370, 200, 2);
  } else {
    tft.drawString(String(serre.humidite2) + "%", 390, 200, 2);
  }
  //Batterie
  tft.setTextColor(TFT_DARKGREEN);
  tft.drawString("Batterie", 250, 250, 1);
  if (serre.batterie2 < 10) {
    tft.drawString(String(serre.batterie2) + "%", 400, 240, 2);
  } else if (serre.batterie2 > 99) {
    tft.drawString(String(serre.batterie2) + "%", 370, 240, 2);
  } else {
    tft.drawString(String(serre.batterie2) + "%", 390, 240, 2);
  }

  // Pied de page
  tft.setTextColor(TFT_MAGENTA);
  tft.drawString("Appuyez sur maison pour l'image", 10, 290, 1);
}

void updateValues(){
  //Température
  tft.fillRect(370, 20, 80, 30, TFT_BLACK);
  tft.setTextColor(TFT_YELLOW);
  tft.setTextSize(2);
  if (serre.temperature < 10) {
      tft.drawString(String(serre.temperature), 400, 20, 2);
    } else if (serre.temperature > 99) {
      tft.drawString(String(serre.temperature), 370, 20, 2);
    } else {
      tft.drawString(String(serre.temperature), 390, 20, 2);
    }
  tft.drawCircle(425, 20, 2,TFT_YELLOW);
  tft.drawString("C", 430, 20, 2);

  //Humidité
  tft.fillRect(370, 50, 80, 30, TFT_BLACK);
  tft.setTextColor(TFT_YELLOW);
  if (serre.humidite < 10) {
    tft.drawString(String(serre.humidite) + "%", 400, 50, 2);
  } else if (serre.humidite > 99) {
    tft.drawString(String(serre.humidite) + "%", 370, 50, 2); 
  } else {
    tft.drawString(String(serre.humidite) + "%", 390, 50, 2);
  }

  //Humidité1
  tft.fillRect(370, 100, 80, 30, TFT_BLACK);
  tft.setTextColor(TFT_ORANGE);
  if (serre.humidite1 < 10) {
    tft.drawString(String(serre.humidite1) + "%", 400, 100, 2); 
  } else if (serre.humidite1 > 99) {
    tft.drawString(String(serre.humidite1) + "%", 370, 100, 2);
  } else {
    tft.drawString(String(serre.humidite1) + "%", 390, 100, 2);
  }

  //Batterie1
  tft.fillRect(370, 140, 80, 30, TFT_BLACK);
  tft.setTextColor(TFT_DARKGREEN);
  if (serre.batterie1 < 10) {
    tft.drawString(String(serre.batterie1) + "%", 400, 140, 2);
  } else if (serre.batterie1 > 99) {
    tft.drawString(String(serre.batterie1) + "%", 370, 140, 2);
  } else {
    tft.drawString(String(serre.batterie1) + "%", 390, 140, 2);
  }

  //Humidité2
  tft.fillRect(370, 200, 80, 30, TFT_BLACK);
  tft.setTextColor(TFT_ORANGE);
  if (serre.humidite2 < 10) {
    tft.drawString(String(serre.humidite2) + "%", 400, 200, 2);
  } else if (serre.humidite2 > 99) {
    tft.drawString(String(serre.humidite2) + "%", 370, 200, 2);
  } else {
    tft.drawString(String(serre.humidite2) + "%", 390, 200, 2);
  }
      
  //Batterie2
  tft.fillRect(370, 240, 80, 30, TFT_BLACK);
  tft.setTextColor(TFT_DARKGREEN);
  if (serre.batterie2 < 10) {
    tft.drawString(String(serre.batterie2) + "%", 400, 240, 2);
  } else if (serre.batterie2 > 99) {
    tft.drawString(String(serre.batterie2) + "%", 370, 240, 2);
  } else {
    tft.drawString(String(serre.batterie2) + "%", 390, 240, 2);
  }
}

void updateHumiditeVoulue(uint8_t newHum) {
  tft.fillRect(70, 95, 120, 50, TFT_BLACK);
  tft.setTextColor(TFT_WHITE);
  tft.setTextSize(2);
  if (newHum < 10) {
    tft.drawString(String(newHum) + "%", 95, 110, 2);
  } else if (newHum > 99) {
    tft.drawString(String(newHum) + "%", 80, 110, 2);
  } else {
    tft.drawString(String(newHum) + "%", 90, 110, 2);
  }
}

void SendSettingsTo(const uint8_t *mac) {
  esp_err_t result = esp_now_send(mac, (uint8_t *)&humidite, sizeof(humidite));
}

void handleSettingsButtons() {
  unsigned long now = millis();

  // Bouton augmenter
  if (digitalRead(UP_BUTTON) == LOW && (now - lastIncPress) > DEBOUNCE_TIME) {
    lastIncPress = now;
    if (humidite.humidite + HUMIDITY_STEP <= 100) {
      humidite.humidite += HUMIDITY_STEP;
    } else {
      humidite.humidite = 100;
    }
    if(awake){
    SendSettingsTo(MACS);
    }
    updateHumiditeVoulue(humidite.humidite);
  }

  // Bouton diminuer
  if (digitalRead(DOWN_BUTTON) == LOW && (now - lastDecPress) > DEBOUNCE_TIME) {
    lastDecPress = now;
    if (humidite.humidite >= HUMIDITY_STEP) {
      humidite.humidite -= HUMIDITY_STEP;
    } else {
      humidite.humidite = 0;
    }
    SendSettingsTo(MACS);
    updateHumiditeVoulue(humidite.humidite);
  }
}

void handleButtonPress() {
  // Gérer la pression du bouton avec rebond
  unsigned long currentTime = millis();
  
  if (currentTime - lastButtonPress > DEBOUNCE_TIME) {
    lastButtonPress = currentTime;
    
    // Basculer entre les pages
    if (currentPage == PAGE_CAM) {
      currentPage = PAGE_DONNEE;
      displaySettingsPage();
    } else {
      currentPage = PAGE_CAM;
      displayImagePage();
      if (imageReady) {
        if (currentPage == PAGE_CAM) {
          displayJpeg();
        }
      }
    }
  }
}

void initEspNow() {
  // ESP-NOW fonctionne en mode STA sans se connecter réellement à un point d'accès
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
 
  if (esp_now_init() != ESP_OK) {
    while (true) delay(1000);
  }
 
  // Enregistrer le pair
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, MACS, 6);
  peer.channel = 0;   // 0 = current channel
  peer.encrypt = false;
 
  if (esp_now_add_peer(&peer) != ESP_OK) {
    while (true) delay(1000);
  }
  espNowReady = true;
}

void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // Rappel reçu des données ESP-NOW
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  if (len > 0) {
    uint8_t msgType = incomingData[0];
    if (msgType == DONNEE && len == sizeof(CAPTEUR_DATA)) {
      memcpy(&serre, incomingData, sizeof(CAPTEUR_DATA));
      if(currentPage == PAGE_DONNEE) {
        updateValues();
      }
    }else if(msgType == IMAGE) {
      imageReady = false;
      ImagePayload image;
      memcpy(&image,incomingData,sizeof(ImagePayload));
      // Nouvelle image ou premier chunk : réinitialiser le buffer
    if (image.imageId != currentImgId || image.chunkNumber == 0) {
      currentImgId = image.imageId;
      imgLen = 0;
    }

    // Copier le chunk dans le buffer
    if (imgLen + image.payloadLen <= MAX_IMAGE_SIZE) {
      memcpy(imgBuffer + imgLen, image.imageData, image.payloadLen);
      imgLen += image.payloadLen;
    }

    // Dernier chunk reçu → signaler au loop()
    if (image.chunkNumber == image.totalChunks - 1) {
      imageReady = true;
      serreNewImageReceived=true;
      lastImageTime = millis(); // Enregistrer le temps de réception
    }
    }else if(msgType == REVEIL) {
      awake=true;
      SendSettingsTo(MACS);
    }else if(msgType == SLEEP) {
      awake=false;
    }
  }
}

void setup() {
  Serial.begin(9600);
  delay(100);

  // Initialiser le bouton de page
  pinMode(PAGE_BUTTON, INPUT);
  // Initialiser boutons + / -
  pinMode(UP_BUTTON, INPUT);
  pinMode(DOWN_BUTTON, INPUT);

  // Initialiser l'affichage TFT
  tft.init();
  tft.setRotation(3);// Orientation paysage
  tft.fillScreen(TFT_BLACK);

  // Config TJpgDec
  TJpgDec.setJpgScale(1);       // 1 = taille originale
  TJpgDec.setSwapBytes(false);   
  TJpgDec.setCallback(tft_output);

  initEspNow();
  esp_now_register_recv_cb(onDataRecv);
  
  displayImagePage();
}

void loop() {
  if (imageReady) {
    if ((currentPage == PAGE_CAM)&&serreNewImageReceived) {
      serreNewImageReceived = false;
      displayJpeg();
    }
  }
  // Vérifier la pression du bouton de changement de page
  if (digitalRead(PAGE_BUTTON) == LOW) {
    handleButtonPress();
    // petite pause pour éviter une double lecture si c'est pour changer de page
    delay(200);
  }
  // Si on est sur la page paramètres, gérer les boutons + / -
  if (currentPage == PAGE_DONNEE) {
    handleSettingsButtons();
  }
  
  // Mettre a jour le temps de la dernière photo en continu
  updateImageTime();
  
  delay(50);
}