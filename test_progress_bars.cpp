/*
 * Test des barres de progression OTA améliorées
 * 
 * Ce fichier teste les nouvelles barres de progression avec feedback visuel :
 * - Barre de progression jaune pendant la connexion WiFi
 * - Barre de progression cyan pendant le téléchargement LittleFS
 * - Barre de progression rose pendant le téléchargement Firmware
 * 
 * Instructions de test :
 * 1. Compiler et uploader ce code sur l'ESP32
 * 2. Au démarrage, maintenir le bouton 1 enfoncé pour tester l'OTA
 * 3. Observer les barres de progression colorées
 * 4. Relâcher le bouton pour voir le mode normal
 */

#include <Arduino.h>
#include <FastLED.h>

// Configuration LED
#define LED_PIN 4
#define NUM_LEDS 144
#define BRIGHTNESS 64

CRGB leds[NUM_LEDS];

void showBlinkPattern(CRGB color, int count, int duration) {
  Serial.print("Pattern LED: ");
  Serial.print(count);
  Serial.println(" clignottements");
  
  for (int i = 0; i < count; i++) {
    // Allumer toutes les LEDs
    for (int j = 0; j < NUM_LEDS; j++) {
      leds[j] = color;
    }
    FastLED.show();
    delay(duration);
    
    // Éteindre toutes les LEDs
    for (int j = 0; j < NUM_LEDS; j++) {
      leds[j] = CRGB(0, 0, 0);
    }
    FastLED.show();
    delay(duration);
  }
}

void showProgressBar(CRGB color, int pixels) {
  // Effacer toutes les LEDs
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
  
  // Allumer les pixels de progression
  for (int i = 0; i < pixels && i < NUM_LEDS; i++) {
    leds[i] = color;
  }
  
  FastLED.show();
  
  Serial.print("Progression: ");
  Serial.print(pixels);
  Serial.print("/");
  Serial.println(NUM_LEDS);
}

void clearLEDs() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
}

void simulateWiFiConnection() {
  Serial.println("=== SIMULATION CONNEXION WIFI ===");
  
  int maxAttempts = 20;
  int progressStep = NUM_LEDS / maxAttempts;
  
  for (int attempts = 0; attempts < maxAttempts; attempts++) {
    delay(500);
    Serial.print(".");
    
    // Mettre à jour la barre de progression jaune
    int progressPixels = (attempts + 1) * progressStep;
    showProgressBar(CRGB(255, 255, 0), progressPixels); // Jaune
  }
  
  Serial.println();
  Serial.println("WiFi connecté!");
  
  // Barre de progression complète en vert pour indiquer le succès
  showProgressBar(CRGB(0, 255, 0), NUM_LEDS);
  delay(1000);
  clearLEDs();
}

void simulateLittleFSDownload() {
  Serial.println("=== SIMULATION TÉLÉCHARGEMENT LITTLEFS ===");
  
  int totalBytes = 1000000; // 1MB simulé
  int progressStep = totalBytes / NUM_LEDS;
  int downloadedBytes = 0;
  
  while (downloadedBytes < totalBytes) {
    int bytesToAdd = random(1000, 5000); // Ajout aléatoire de bytes
    downloadedBytes += bytesToAdd;
    if (downloadedBytes > totalBytes) downloadedBytes = totalBytes;
    
    // Mettre à jour la barre de progression cyan
    int progressPixels = downloadedBytes / progressStep;
    if (progressPixels > NUM_LEDS) progressPixels = NUM_LEDS;
    
    showProgressBar(CRGB(0, 255, 255), progressPixels); // Cyan
    delay(50);
  }
  
  Serial.println("LittleFS téléchargé!");
  delay(1000);
  clearLEDs();
}

void simulateFirmwareDownload() {
  Serial.println("=== SIMULATION TÉLÉCHARGEMENT FIRMWARE ===");
  
  int totalBytes = 2000000; // 2MB simulé
  int progressStep = totalBytes / NUM_LEDS;
  int downloadedBytes = 0;
  
  while (downloadedBytes < totalBytes) {
    int bytesToAdd = random(1000, 5000); // Ajout aléatoire de bytes
    downloadedBytes += bytesToAdd;
    if (downloadedBytes > totalBytes) downloadedBytes = totalBytes;
    
    // Mettre à jour la barre de progression rose
    int progressPixels = downloadedBytes / progressStep;
    if (progressPixels > NUM_LEDS) progressPixels = NUM_LEDS;
    
    showProgressBar(CRGB(255, 0, 255), progressPixels); // Rose
    delay(50);
  }
  
  Serial.println("Firmware téléchargé!");
  delay(1000);
  clearLEDs();
}

void setup_OTA() {
  Serial.println("=== MODE OTA ACTIVÉ ===");
  
  // Initialiser FastLED pour l'OTA
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);        
  FastLED.setBrightness(BRIGHTNESS);        
  
  // 5 clignottements bleus pour annoncer le début
  showBlinkPattern(CRGB(0, 0, 255), 5, 200);
  
  // Simulation connexion WiFi avec barre jaune
  simulateWiFiConnection();
  
  // Simulation téléchargement LittleFS avec barre cyan
  simulateLittleFSDownload();
  
  // Simulation téléchargement Firmware avec barre rose
  simulateFirmwareDownload();
  
  // 5 clignottements verts pour le succès
  showBlinkPattern(CRGB(0, 255, 0), 5, 200);
  
  // Redémarrage après OTA        
  Serial.println("Redémarrage après mise à jour OTA...");        
  delay(2000);
  ESP.restart();
}

void setup_normal() {
  Serial.println("=== MODE NORMAL ACTIVÉ ===");
  
  // Initialiser FastLED pour le mode normal
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);        
  FastLED.setBrightness(BRIGHTNESS);        
  
  // Test des LEDs en mode normal
  Serial.println("Test des LEDs en mode normal...");
  
  // Clignotement vert pour indiquer le mode normal
  showBlinkPattern(CRGB(0, 255, 0), 3, 500);
  
  Serial.println("=== Initialisation mode normal terminée ===");
  Serial.println("Le système est prêt à fonctionner");
}

void setup() {
  Serial.begin(115200);
  Serial.println("=== TEST BARRES DE PROGRESSION OTA ===");
  Serial.println("Initialisation...");
  
  // === VÉRIFICATION DU MODE DE DÉMARRAGE ===        
  Serial.println("Vérification du mode de démarrage...");
  
  // Vérifier si le bouton 1 est pressé au démarrage
  pinMode(22, INPUT_PULLUP); // BUTTON_1_PIN
  delay(100); // Attendre la stabilisation
  
  bool buttonPressed = !digitalRead(22); // Inversé car INPUT_PULLUP
  Serial.print("État du bouton 1 au démarrage: ");
  Serial.println(buttonPressed ? "PRESSÉ" : "RELAXÉ");
  
  if (buttonPressed) {
    setup_OTA(); // Cette fonction termine par un redémarrage
  } else {
    setup_normal(); // Mode normal
  }
}

void loop() {
  // Mode normal - clignotement lent pour indiquer que le système fonctionne
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  
  if (millis() - lastBlink > 2000) {
    ledState = !ledState;
    
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = ledState ? CRGB(0, 255, 0) : CRGB(0, 0, 0);
    }
    FastLED.show();
    
    Serial.print("Mode normal actif - ");
    Serial.println(ledState ? "LEDs ON" : "LEDs OFF");
    
    lastBlink = millis();
  }
  
  delay(10);
}
