#include "ota_update.h"

OTAUpdate::OTAUpdate() {
  currentState = OTA_IDLE;
  buttonPressed = false;
  lastBlinkTime = 0;
  blinkCount = 0;
  progressPixels = 0;
  littlefsCompleted = false;
  firmwareCompleted = false;
}

bool OTAUpdate::shouldStartOTA() {
  // Vérifier si le bouton 1 est pressé au démarrage
  pinMode(22, INPUT_PULLUP); // BUTTON_1_PIN
  delay(100); // Attendre la stabilisation
  
  bool buttonState = !digitalRead(22); // Inversé car INPUT_PULLUP
  Serial.print("État du bouton 1 au démarrage: ");
  Serial.println(buttonState ? "PRESSÉ" : "RELAXÉ");
  
  return buttonState;
}

void OTAUpdate::beginOTA() {
  Serial.println("=== DÉMARRAGE OTA UPDATE ===");
  currentState = OTA_CHECKING_BUTTON;
  
  // 5 clignottements bleus pour annoncer le début
  Serial.println("Pattern LED: 5 clignottements bleus");
  delay(2000); // Simuler le pattern LED
  
  currentState = OTA_CONNECTING_WIFI;
}

void OTAUpdate::updateOTA() {
  switch (currentState) {
    case OTA_CONNECTING_WIFI:
      Serial.println("Connexion au WiFi...");
      if (connectToWiFi()) {
        Serial.println("WiFi connecté avec succès");
        currentState = OTA_DOWNLOADING_LITTLEFS;
      } else {
        Serial.println("Échec de connexion WiFi");
        currentState = OTA_ERROR;
      }
      break;
      
    case OTA_DOWNLOADING_LITTLEFS:
    case OTA_INSTALLING_LITTLEFS:
      Serial.println("Téléchargement et installation LittleFS...");
      if (downloadAndInstallLittleFS()) {
        Serial.println("LittleFS mis à jour avec succès");
        littlefsCompleted = true;
        currentState = OTA_DOWNLOADING_FIRMWARE;
      } else {
        Serial.println("Échec de mise à jour LittleFS");
        currentState = OTA_ERROR;
      }
      break;
      
    case OTA_DOWNLOADING_FIRMWARE:
    case OTA_INSTALLING_FIRMWARE:
      Serial.println("Téléchargement et installation Firmware...");
      if (downloadAndInstallFirmware()) {
        Serial.println("Firmware mis à jour avec succès");
        firmwareCompleted = true;
        currentState = OTA_SUCCESS;
      } else {
        Serial.println("Échec de mise à jour Firmware");
        currentState = OTA_ERROR;
      }
      break;
      
    case OTA_SUCCESS:
      Serial.println("Pattern LED: 5 clignottements verts");
      delay(2000); // Simuler le pattern LED
      currentState = OTA_RESTART_NEEDED;
      break;
      
    case OTA_ERROR:
      Serial.println("Pattern LED: 5 clignottements rouges");
      delay(2000); // Simuler le pattern LED
      currentState = OTA_RESTART_NEEDED;
      break;
      
    default:
      break;
  }
}

bool OTAUpdate::isOTAComplete() {
  return (currentState == OTA_RESTART_NEEDED);
}

bool OTAUpdate::needsRestart() {
  return (currentState == OTA_RESTART_NEEDED);
}

void OTAUpdate::restartESP32() {
  Serial.println("Redémarrage de l'ESP32...");
  delay(1000);
  ESP.restart();
}

bool OTAUpdate::checkButton1Pressed() {
  pinMode(22, INPUT_PULLUP);
  return !digitalRead(22);
}

bool OTAUpdate::connectToWiFi() {
  Serial.print("Connexion au WiFi: ");
  Serial.println(OTA_WIFI_SSID);
  
  WiFi.begin(OTA_WIFI_SSID, OTA_WIFI_PASSWORD);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("WiFi connecté! Adresse IP: ");
    Serial.println(WiFi.localIP());
    return true;
  } else {
    Serial.println();
    Serial.println("Échec de connexion WiFi");
    return false;
  }
}

bool OTAUpdate::downloadAndInstallLittleFS() {
  Serial.println("Téléchargement LittleFS...");
  
  httpClient.begin(wifiClient, OTA_LITTLEFS_URL);
  int httpCode = httpClient.GET();
  
  if (httpCode != HTTP_CODE_OK) {
    Serial.print("Erreur HTTP LittleFS: ");
    Serial.println(httpCode);
    httpClient.end();
    return false;
  }
  
  int contentLength = httpClient.getSize();
  Serial.print("Taille LittleFS: ");
  Serial.println(contentLength);
  
  if (!Update.begin(contentLength, U_SPIFFS)) {
    Serial.print("Erreur début update LittleFS: ");
    Serial.println(Update.errorString());
    httpClient.end();
    return false;
  }
  
  WiFiClient* stream = httpClient.getStreamPtr();
  int downloadedBytes = 0;
  
  while (httpClient.connected() && downloadedBytes < contentLength) {
    size_t size = stream->available();
    if (size) {
      uint8_t buffer[1024];
      size_t bytesRead = stream->readBytes(buffer, min(size, sizeof(buffer)));
      
      if (Update.write(buffer, bytesRead) != bytesRead) {
        Serial.println("Erreur écriture LittleFS");
        Update.abort();
        httpClient.end();
        return false;
      }
      
      downloadedBytes += bytesRead;
      
      // Afficher la progression
      if (downloadedBytes % (contentLength / 10) == 0) {
        Serial.print("Progression LittleFS: ");
        Serial.print((downloadedBytes * 100) / contentLength);
        Serial.println("%");
      }
    }
    delay(1);
  }
  
  if (Update.end()) {
    Serial.println("LittleFS mis à jour avec succès");
    httpClient.end();
    return true;
  } else {
    Serial.print("Erreur fin update LittleFS: ");
    Serial.println(Update.errorString());
    httpClient.end();
    return false;
  }
}

bool OTAUpdate::downloadAndInstallFirmware() {
  Serial.println("Téléchargement Firmware...");
  
  httpClient.begin(wifiClient, OTA_FIRMWARE_URL);
  int httpCode = httpClient.GET();
  
  if (httpCode != HTTP_CODE_OK) {
    Serial.print("Erreur HTTP Firmware: ");
    Serial.println(httpCode);
    httpClient.end();
    return false;
  }
  
  int contentLength = httpClient.getSize();
  Serial.print("Taille Firmware: ");
  Serial.println(contentLength);
  
  if (!Update.begin(contentLength)) {
    Serial.print("Erreur début update Firmware: ");
    Serial.println(Update.errorString());
    httpClient.end();
    return false;
  }
  
  WiFiClient* stream = httpClient.getStreamPtr();
  int downloadedBytes = 0;
  
  while (httpClient.connected() && downloadedBytes < contentLength) {
    size_t size = stream->available();
    if (size) {
      uint8_t buffer[1024];
      size_t bytesRead = stream->readBytes(buffer, min(size, sizeof(buffer)));
      
      if (Update.write(buffer, bytesRead) != bytesRead) {
        Serial.println("Erreur écriture Firmware");
        Update.abort();
        httpClient.end();
        return false;
      }
      
      downloadedBytes += bytesRead;
      
      // Afficher la progression
      if (downloadedBytes % (contentLength / 10) == 0) {
        Serial.print("Progression Firmware: ");
        Serial.print((downloadedBytes * 100) / contentLength);
        Serial.println("%");
      }
    }
    delay(1);
  }
  
  if (Update.end()) {
    Serial.println("Firmware mis à jour avec succès");
    httpClient.end();
    return true;
  } else {
    Serial.print("Erreur fin update Firmware: ");
    Serial.println(Update.errorString());
    httpClient.end();
    return false;
  }
}