/*
 * Test de la fonctionnalité OTA
 * 
 * Ce fichier permet de tester la fonctionnalité OTA sans avoir besoin
 * de fichiers binaires réels sur le serveur.
 * 
 * Instructions de test :
 * 1. Compiler et uploader ce code sur l'ESP32
 * 2. Au démarrage, maintenir le bouton 1 enfoncé
 * 3. Observer les patterns LED :
 *    - 5 clignottements bleus : début OTA
 *    - Barre de progression cyan : téléchargement LittleFS (simulé)
 *    - Barre de progression rose : téléchargement Firmware (simulé)
 *    - 5 clignottements verts : succès (ou rouges si erreur)
 * 4. L'ESP32 redémarre automatiquement
 * 
 * Note : En mode test, les téléchargements sont simulés pour éviter
 * les erreurs de connexion réseau.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <FastLED.h>

// Configuration LED pour test
#define LED_PIN 4
#define NUM_LEDS 144
#define BRIGHTNESS 64

// Couleurs pour le feedback
#define COLOR_BLUE CRGB(0, 0, 255)
#define COLOR_CYAN CRGB(0, 255, 255)
#define COLOR_PINK CRGB(255, 0, 255)
#define COLOR_GREEN CRGB(0, 255, 0)
#define COLOR_RED CRGB(255, 0, 0)
#define COLOR_OFF CRGB(0, 0, 0)

CRGB leds[NUM_LEDS];

void showBlinkPattern(CRGB color, int count, int duration) {
  Serial.print("Pattern clignotant: ");
  Serial.print(count);
  Serial.println(" fois");
  
  for (int i = 0; i < count; i++) {
    // Allumer toutes les LEDs
    for (int j = 0; j < NUM_LEDS; j++) {
      leds[j] = color;
    }
    FastLED.show();
    delay(duration);
    
    // Éteindre toutes les LEDs
    for (int j = 0; j < NUM_LEDS; j++) {
      leds[j] = COLOR_OFF;
    }
    FastLED.show();
    delay(duration);
  }
}

void showProgressBar(CRGB color, int pixels) {
  // Effacer toutes les LEDs
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = COLOR_OFF;
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

void simulateOTA() {
  Serial.println("=== SIMULATION OTA UPDATE ===");
  
  // 1. 5 clignottements bleus pour annoncer le début
  Serial.println("Étape 1: Annonce du début OTA");
  showBlinkPattern(COLOR_BLUE, 5, 200);
  delay(500);
  
  // 2. Simulation téléchargement LittleFS (cyan)
  Serial.println("Étape 2: Simulation téléchargement LittleFS");
  for (int i = 0; i <= NUM_LEDS; i += 5) {
    showProgressBar(COLOR_CYAN, i);
    delay(50); // Simulation du téléchargement
  }
  delay(500);
  
  // 3. Simulation téléchargement Firmware (rose)
  Serial.println("Étape 3: Simulation téléchargement Firmware");
  for (int i = 0; i <= NUM_LEDS; i += 5) {
    showProgressBar(COLOR_PINK, i);
    delay(50); // Simulation du téléchargement
  }
  delay(500);
  
  // 4. 5 clignottements verts pour le succès
  Serial.println("Étape 4: Succès de la mise à jour");
  showBlinkPattern(COLOR_GREEN, 5, 200);
  
  Serial.println("Simulation OTA terminée - Redémarrage dans 3 secondes...");
  delay(3000);
  ESP.restart();
}

void setup() {
  Serial.begin(115200);
  Serial.println("=== TEST OTA UPDATE ===");
  
  // Initialiser FastLED
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  
  // Effacer les LEDs
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = COLOR_OFF;
  }
  FastLED.show();
  
  // Vérifier le bouton 1
  pinMode(22, INPUT_PULLUP); // BUTTON_1_PIN
  delay(100);
  
  bool buttonPressed = !digitalRead(22);
  Serial.print("État du bouton 1: ");
  Serial.println(buttonPressed ? "PRESSÉ" : "RELAXÉ");
  
  if (buttonPressed) {
    Serial.println("Mode OTA détecté - Lancement de la simulation...");
    simulateOTA();
  } else {
    Serial.println("Mode normal - Test des patterns LED...");
    
    // Test de tous les patterns
    showBlinkPattern(COLOR_BLUE, 3, 200);
    delay(500);
    showBlinkPattern(COLOR_CYAN, 3, 200);
    delay(500);
    showBlinkPattern(COLOR_PINK, 3, 200);
    delay(500);
    showBlinkPattern(COLOR_GREEN, 3, 200);
    delay(500);
    showBlinkPattern(COLOR_RED, 3, 200);
    delay(500);
    
    Serial.println("Test terminé - Mode normal actif");
  }
}

void loop() {
  // Mode normal - clignotement lent pour indiquer que le système fonctionne
  static unsigned long lastBlink = 0;
  static bool ledState = false;
  
  if (millis() - lastBlink > 1000) {
    ledState = !ledState;
    
    for (int i = 0; i < NUM_LEDS; i++) {
      leds[i] = ledState ? COLOR_GREEN : COLOR_OFF;
    }
    FastLED.show();
    
    lastBlink = millis();
  }
  
  delay(10);
}
