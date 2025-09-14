#ifndef OTA_UPDATE_H
#define OTA_UPDATE_H

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Update.h>
#include <LittleFS.h>

// Configuration OTA
#define OTA_WIFI_SSID "mrVOOlpy"
#define OTA_WIFI_PASSWORD "youhououhou"
#define OTA_LITTLEFS_URL "http://ksoloti_kontrol.gaetanstreel.com/littlefs.bin"
#define OTA_FIRMWARE_URL "http://ksoloti_kontrol.gaetanstreel.com/firmware.bin"

// États de l'OTA
enum OTAState {
  OTA_IDLE,
  OTA_CHECKING_BUTTON,
  OTA_CONNECTING_WIFI,
  OTA_DOWNLOADING_LITTLEFS,
  OTA_INSTALLING_LITTLEFS,
  OTA_DOWNLOADING_FIRMWARE,
  OTA_INSTALLING_FIRMWARE,
  OTA_SUCCESS,
  OTA_ERROR,
  OTA_RESTART_NEEDED
};

class OTAUpdate {
private:
  OTAState currentState;
  bool buttonPressed;
  WiFiClient wifiClient;
  HTTPClient httpClient;
  unsigned long lastBlinkTime;
  int blinkCount;
  int progressPixels;
  bool littlefsCompleted;
  bool firmwareCompleted;
  
  // Fonctions privées
  bool checkButton1Pressed();
  bool connectToWiFi();
  bool downloadAndInstallLittleFS();
  bool downloadAndInstallFirmware();
  
public:
  OTAUpdate();
  bool shouldStartOTA();
  void beginOTA();
  void updateOTA();
  bool isOTAComplete();
  bool needsRestart();
  void restartESP32();
};

#endif