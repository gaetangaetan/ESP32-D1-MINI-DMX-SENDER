/*
 * Test de la nouvelle architecture OTA modulaire
 * 
 * Ce fichier teste la nouvelle approche avec setup_OTA() et setup_normal() séparés.
 * 
 * Instructions de test :
 * 1. Compiler et uploader ce code sur l'ESP32
 * 2. Au démarrage, maintenir le bouton 1 enfoncé pour tester setup_OTA()
 * 3. Relâcher le bouton pour tester setup_normal()
 * 4. Observer les messages série pour confirmer le bon fonctionnement
 * 
 * Avantages de cette architecture :
 * - Séparation complète des deux modes
 * - Pas de conflits entre OTA et mode normal
 * - Code facilement modifiable ou supprimable
 * - setup_OTA() termine par un redémarrage, donc pas de loop() en mode OTA
 */

#include <Arduino.h>
#include <FastLED.h>

// Configuration LED
#define LED_PIN 4
#define NUM_LEDS 144
#define BRIGHTNESS 64

CRGB leds[NUM_LEDS];

void setup_OTA() {
  Serial.println("=== MODE OTA ACTIVÉ ===");
  
  // Initialiser FastLED pour l'OTA
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);        
  FastLED.setBrightness(BRIGHTNESS);        
  
  // 5 clignottements bleus pour annoncer le début
  Serial.println("Pattern LED: 5 clignottements bleus");
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < NUM_LEDS; j++) {
      leds[j] = CRGB(0, 0, 255); // Bleu
    }
    FastLED.show();
    delay(200);
    FastLED.clear();
    FastLED.show();
    delay(200);
  }
  
  // Simulation du processus OTA
  Serial.println("Simulation téléchargement LittleFS...");
  delay(2000);
  
  Serial.println("Simulation téléchargement Firmware...");
  delay(2000);
  
  // 5 clignottements verts pour le succès
  Serial.println("Pattern LED: 5 clignottements verts");
  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < NUM_LEDS; j++) {
      leds[j] = CRGB(0, 255, 0); // Vert
    }
    FastLED.show();
    delay(200);
    FastLED.clear();
    FastLED.show();
    delay(200);
  }
  
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
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < NUM_LEDS; j++) {
      leds[j] = CRGB(0, 255, 0); // Vert
    }
    FastLED.show();
    delay(500);
    FastLED.clear();
    FastLED.show();
    delay(500);
  }
  
  Serial.println("=== Initialisation mode normal terminée ===");
  Serial.println("Le système est prêt à fonctionner");
}

void setup() {
  Serial.begin(115200);
  Serial.println("=== TEST ARCHITECTURE OTA MODULAIRE ===");
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
