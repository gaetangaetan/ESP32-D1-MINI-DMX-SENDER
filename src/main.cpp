/*Paramètres Ksoloti
1 autopan depth
2 pitch
3 vibrato speed
4 vibrato depth
5 delay time
6 delay feedback
7 osc waveform
8 gate threshold
9 portamento time
10 scale
11 octave low high
12 OSC 2 volume
13 OSC 2 pitch offset
14 autopan frequency
15 scale tonic
16 volume drums
17 trig kick
18 trig snare
19 trig hh
20 master volume (inverted)
21 filter on-off 
22 filter cutoff
23 filter reso
24 filter type
*/
#define VERSION 160
/*
// Contrôleur interactif ESP32 avec capteurs Sharp IR
// Utilise ESP-NOW pour transmettre les données DMX
// Fréquence d'émission : 50Hz
// Canal DMX 102 : valeur du capteur Sharp IR (0-255)
*/

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <PCF8574.h>
#include <TM1637.h>

// Définitions pour les capteurs Sharp IR
#define DIST_SENSOR_1_PIN 35    // GPIO35 - Premier capteur Sharp IR
#define DIST_SENSOR_2_PIN 36    // GPIO36 - Deuxième capteur Sharp IR

// Définitions pour les faders analogiques
#define FADER_1_PIN 32    // GPIO32 - Premier fader
#define FADER_2_PIN 33    // GPIO33 - Deuxième fader
#define FADER_3_PIN 34    // GPIO34 - Troisième fader

// Définitions pour l'encodeur rotatif KY-040
#define ENCODER_A_PIN 26    // GPIO26 - Pin A de l'encodeur
#define ENCODER_B_PIN 27    // GPIO27 - Pin B de l'encodeur
#define ENCODER_BUTTON_PIN 25  // GPIO25 - Bouton de l'encodeur

// Définitions pour l'afficheur TM1637
#define TM1637_CLK_PIN 18    // GPIO18 - CLK de l'afficheur
#define TM1637_DIO_PIN 19    // GPIO19 - DIO de l'afficheur

// Configuration I2C pour PCF8574
#define PCF8574_ADDRESS 0x20    // Adresse I2C du PCF8574
#define I2C_SDA_PIN 21         // GPIO21 - SDA
#define I2C_SCL_PIN 22         // GPIO22 - SCL

// Configuration ESP-NOW
#define EMISSION_FREQUENCY 50  // Hz (20ms entre chaque émission)
#define DMX_CHANNEL_IR_1 102  // Canal DMX pour le premier capteur Sharp IR
#define DMX_CHANNEL_IR_2 103  // Canal DMX pour le deuxième capteur Sharp IR
#define MAX_DISTANCE_CM 80  // Distance maximale en cm pour Sharp IR (80cm = 0, 4cm = 255)
#define MIN_DISTANCE_CM 4   // Distance minimale en cm pour Sharp IR

// Configuration des presets
#define PRESET_SIZE 24  // Nombre de paramètres par preset (24 au lieu de 20)
#define MAX_PRESETS 10  // Nombre maximum de presets

// Structure pour un paramètre
typedef struct {
  const char* name;     // Nom du paramètre
  uint16_t dmxChannel;  // Canal DMX (1-512)
  uint8_t value;        // Valeur actuelle (0-255)
  uint8_t defaultValue; // Valeur par défaut
} Parameter;

// Structure pour un preset complet
typedef struct {
  char name[16];        // Nom du preset
  uint8_t values[PRESET_SIZE]; // Valeurs des paramètres
} Preset;

// Création des objets
PCF8574 pcf8574(PCF8574_ADDRESS);
TM1637 display(TM1637_CLK_PIN, TM1637_DIO_PIN);
ESP32Encoder encoder;

// Définition des 24 paramètres du theremin selon la liste fournie
Parameter parameters[PRESET_SIZE] = {
  {"autopan_depth", 101, 0, 0},
  {"pitch", 102, 0, 0},
  {"vibrato_speed", 103, 0, 0},
  {"vibrato_depth", 104, 0, 0},
  {"delay_time", 105, 0, 0},
  {"delay_feedback", 106, 0, 0},
  {"osc_waveform", 107, 0, 0},
  {"gate_threshold", 108, 0, 0},
  {"portamento_time", 109, 0, 0},
  {"scale", 110, 0, 0},
  {"octave_low_high", 111, 0, 0},
  {"osc2_volume", 112, 0, 0},
  {"osc2_pitch_offset", 113, 0, 0},
  {"autopan_frequency", 114, 0, 0},
  {"scale_tonic", 115, 0, 0},
  {"volume_drums", 116, 0, 0},
  {"trig_kick", 117, 0, 0},
  {"trig_snare", 118, 0, 0},
  {"trig_hh", 119, 0, 0},
  {"master_volume", 120, 0, 0},
  {"filter_on_off", 121, 0, 0},
  {"filter_cutoff", 122, 0, 0},
  {"filter_reso", 123, 0, 0},
  {"filter_type", 124, 0, 0}
};

// Tableau des presets
Preset presets[MAX_PRESETS];

// Tableau des valeurs DMX (512 canaux)
uint8_t dmxValues[512];

// Structure pour les paquets DMX
typedef struct struct_dmx_packet
{
  uint8_t blockNumber; // on divise les 512 adresses en 4 blocs de 128 adresses
  uint8_t dmxvalues[128];
} struct_dmx_packet;

// Structure pour les données de retour du Ksoloti
typedef struct struct_ksoloti_feedback
{
  uint8_t startByte;    // 0xCC pour identifier les paquets de retour
  uint8_t val1;         // Première valeur du Ksoloti
  uint8_t val2;         // Deuxième valeur du Ksoloti
  uint8_t checksum;     // Checksum pour validation
} struct_ksoloti_feedback;

struct_dmx_packet outgoingDMXPacket;
struct_ksoloti_feedback incomingKsolotiData;

// Adresse de diffusion ESP-NOW
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Variables pour stocker les données reçues du Ksoloti
uint8_t ksoloti_val1 = 0;
uint8_t ksoloti_val2 = 0;
bool ksoloti_data_updated = false;

esp_now_peer_info_t peerInfo;

// Variables de timing
unsigned long lastEmissionTime = 0;
const unsigned long EMISSION_INTERVAL = 1000 / EMISSION_FREQUENCY; // 20ms pour 50Hz

// Variables pour les boutons PCF8574
bool buttonStates[3] = {false, false, false};
bool lastButtonStates[3] = {false, false, false};
unsigned long lastButtonPress[3] = {0, 0, 0};
const unsigned long BUTTON_DEBOUNCE = 200; // 200ms de debounce

// Variables pour les faders
uint8_t faderValues[3] = {0, 0, 0};
uint8_t lastFaderValues[3] = {0, 0, 0};

// Variables pour l'encodeur
int32_t lastEncoderValue = 0;
uint8_t selectedPreset = 1;  // Index du preset sélectionné (0-9)
bool encoderButtonPressed = false;
unsigned long lastEncoderButtonPress = 0;

// Variables pour l'affichage
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_UPDATE_INTERVAL = 100; // 100ms entre les mises à jour

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  // Callback pour le statut d'envoi (optionnel)
}

// Callback pour la réception de données ESP-NOW
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  // Vérifier si c'est un paquet de données du Ksoloti
  if (data_len == sizeof(struct_ksoloti_feedback)) {
    memcpy(&incomingKsolotiData, data, sizeof(struct_ksoloti_feedback));
    
    // Vérifier le byte de démarrage et le checksum
    if (incomingKsolotiData.startByte == 0xCC) {
      uint8_t calculated_checksum = incomingKsolotiData.startByte + 
                                   incomingKsolotiData.val1 + 
                                   incomingKsolotiData.val2;
      
      if (calculated_checksum == incomingKsolotiData.checksum) {
        ksoloti_val1 = incomingKsolotiData.val1;
        ksoloti_val2 = incomingKsolotiData.val2;
        ksoloti_data_updated = true;
        
        // Serial.print("Données Ksoloti reçues: val1=");
        // Serial.print(ksoloti_val1);
        // Serial.print(", val2=");
        // Serial.println(ksoloti_val2);
      }
    }
  }
}

// Fonction pour mettre à jour un paramètre par son nom
void setParameter(const char* paramName, uint8_t value) {
  for (int i = 0; i < PRESET_SIZE; i++) {
    if (strcmp(parameters[i].name, paramName) == 0) {
      parameters[i].value = value;
      // Mettre à jour le tableau DMX
      dmxValues[parameters[i].dmxChannel - 1] = value;
      //Serial.println("Paramètre " + String(paramName) + " = " + String(value));
      return;
    }
  }
  //Serial.println("Paramètre " + String(paramName) + " non trouvé");
}

// Fonction pour obtenir la valeur d'un paramètre par son nom
uint8_t getParameter(const char* paramName) {
  for (int i = 0; i < PRESET_SIZE; i++) {
    if (strcmp(parameters[i].name, paramName) == 0) {
      return parameters[i].value;
    }
  }
  return 0;
}

// Fonction pour sauvegarder un preset
void savePreset(uint8_t presetIndex, const char* presetName) {
  if (presetIndex >= MAX_PRESETS) {
    Serial.println("Index de preset invalide");
    return;
  }
  
  // Copier le nom du preset
  strncpy(presets[presetIndex].name, presetName, 15);
  presets[presetIndex].name[15] = '\0';
  
  // Copier les valeurs des paramètres
  for (int i = 0; i < PRESET_SIZE; i++) {
    presets[presetIndex].values[i] = parameters[i].value;
  }
  
  //Serial.println("Preset " + String(presetIndex) + " sauvegardé en RAM: " + String(presetName));
}

// Fonction pour charger un preset
void loadPreset(uint8_t presetIndex) {
  if (presetIndex >= MAX_PRESETS) {
    Serial.println("Index de preset invalide");
    return;
  }
  
  // Appliquer les valeurs aux paramètres
  for (int i = 0; i < PRESET_SIZE; i++) {
    parameters[i].value = presets[presetIndex].values[i];
    dmxValues[parameters[i].dmxChannel - 1] = parameters[i].value;
  }
  
  //Serial.println("Preset " + String(presetIndex) + " chargé depuis RAM: " + String(presets[presetIndex].name));
}

// Fonction pour afficher tous les paramètres
void printParameters() {
  //Serial.println("=== Paramètres actuels ===");
  for (int i = 0; i < PRESET_SIZE; i++) {
    //Serial.println(String(parameters[i].name) + " (DMX " + String(parameters[i].dmxChannel) + "): " + String(parameters[i].value));
  }
  //Serial.println("==========================");
}

// Fonction pour initialiser les paramètres par défaut
void initializeParameters() {
  for (int i = 0; i < PRESET_SIZE; i++) {
    parameters[i].value = parameters[i].defaultValue;
    dmxValues[parameters[i].dmxChannel - 1] = parameters[i].value;
  }
  //Serial.println("Paramètres initialisés aux valeurs par défaut");
}



// Fonction pour enregistrer tous les paramètres d'un coup (24 arguments)
void setAllParameters(uint8_t autopan_depth, uint8_t pitch, uint8_t vibrato_speed, uint8_t vibrato_depth,
                     uint8_t delay_time, uint8_t delay_feedback, uint8_t osc_waveform, uint8_t gate_threshold,
                     uint8_t portamento_time, uint8_t scale, uint8_t octave_low_high, uint8_t osc2_volume,
                     uint8_t osc2_pitch_offset, uint8_t autopan_frequency, uint8_t scale_tonic,
                     uint8_t volume_drums, uint8_t trig_kick, uint8_t trig_snare, uint8_t trig_hh,
                     uint8_t master_volume, uint8_t filter_on_off, uint8_t filter_cutoff,
                     uint8_t filter_reso, uint8_t filter_type) {
  
  // Mettre à jour tous les paramètres
  parameters[0].value = autopan_depth;
  parameters[1].value = pitch;
  parameters[2].value = vibrato_speed;
  parameters[3].value = vibrato_depth;
  parameters[4].value = delay_time;
  parameters[5].value = delay_feedback;
  parameters[6].value = osc_waveform;
  parameters[7].value = gate_threshold;
  parameters[8].value = portamento_time;
  parameters[9].value = scale;
  parameters[10].value = octave_low_high;
  parameters[11].value = osc2_volume;
  parameters[12].value = osc2_pitch_offset;
  parameters[13].value = autopan_frequency;
  parameters[14].value = scale_tonic;
  parameters[15].value = volume_drums;
  parameters[16].value = trig_kick;
  parameters[17].value = trig_snare;
  parameters[18].value = trig_hh;
  parameters[19].value = master_volume;
  parameters[20].value = filter_on_off;
  parameters[21].value = filter_cutoff;
  parameters[22].value = filter_reso;
  parameters[23].value = filter_type;
  
  // Mettre à jour le tableau DMX
  for (int i = 0; i < PRESET_SIZE; i++) {
    dmxValues[parameters[i].dmxChannel - 1] = parameters[i].value;
  }
  
  //Serial.println("Tous les paramètres mis à jour");
  printParameters();
}

// Fonction pour afficher tous les presets disponibles
void printAllPresets() {
  //Serial.println("=== Presets disponibles ===");
  for (int i = 0; i < MAX_PRESETS; i++) {
    //Serial.print("Preset " + String(i) + ": " + String(presets[i].name));
    //Serial.println(" (valeurs: " + String(presets[i].values[0]) + "," + String(presets[i].values[1]) + ",...)");
  }
  //Serial.println("===========================");
}

// Fonction d'initialisation du rotary encoder (désactivée - encoder non câblé)
/*
void initializeEncoder() {
  // Configuration des pins du rotary encoder
  encoder.attachHalfQuad(ENCODER_A_PIN, ENCODER_B_PIN);
  encoder.setCount(0);
  
  // Configuration du bouton (optionnel)
  pinMode(ENCODER_BUTTON_PIN, INPUT_PULLUP);
  
  Serial.println("=== ROTARY ENCODER INITIALISÉ ===");
  Serial.println("Pins configurés:");
  Serial.print("  - Pin A (CLK): D5 (GPIO18)");
  Serial.println(" ✓");
  Serial.print("  - Pin B (DT): D6 (GPIO19)");
  Serial.println(" ✓");
  Serial.print("  - Bouton (SW): D7 (GPIO23)");
  Serial.println(" ✓");
  Serial.println("");
  Serial.println("=== CONTROLES ===");
  Serial.println("🔄 Rotation: Modifie la valeur du paramètre actuel");
  Serial.println("🔘 Bouton: Change de paramètre (cycle 0-19)");
  Serial.println("📊 Monitor: Affiche les changements en temps réel");
  Serial.println("=====================================");
}
*/

// Fonction pour gérer le rotary encoder (désactivée - encoder non câblé)
/*
void handleEncoder() {
  // Lecture de la valeur actuelle du rotary encoder
  int32_t currentEncoderValue = encoder.getCount();
  
  // Si la valeur a changé, mettre à jour le paramètre sélectionné
  if (currentEncoderValue != lastEncoderValue) {
    int32_t delta = currentEncoderValue - lastEncoderValue;
    
    // Obtenir la valeur actuelle du paramètre sélectionné
    uint8_t currentValue = parameters[selectedParameter].value;
    
    // Ajuster la valeur en fonction de la rotation
    int newValue = currentValue + (delta * 2); // Multiplier par 2 pour un contrôle plus sensible
    
    // Limiter la valeur entre 0 et 255
    if (newValue < 0) newValue = 0;
    if (newValue > 255) newValue = 255;
    
    // Mettre à jour le paramètre
    parameters[selectedParameter].value = (uint8_t)newValue;
    dmxValues[parameters[selectedParameter].dmxChannel - 1] = (uint8_t)newValue;
    
    // Afficher les informations détaillées de debug
    Serial.print("🔄 ROTATION: ");
    if (delta > 0) {
      Serial.print("+");
    }
    Serial.print(delta);
    Serial.print(" | Encoder: ");
    Serial.print(currentEncoderValue);
    Serial.print(" | ");
    Serial.print(parameters[selectedParameter].name);
    Serial.print(" (DMX ");
    Serial.print(parameters[selectedParameter].dmxChannel);
    Serial.print("): ");
    Serial.print(currentValue);
    Serial.print(" → ");
    Serial.print((uint8_t)newValue);
    Serial.print(" [");
    Serial.print((uint8_t)newValue * 100 / 255);
    Serial.println("%]");
    
    lastEncoderValue = currentEncoderValue;
  }
  
  // Gestion du bouton pour changer de paramètre
  bool buttonState = !digitalRead(ENCODER_BUTTON_PIN); // Inversé car INPUT_PULLUP
  
  if (buttonState && !encoderButtonPressed && (millis() - lastButtonPress > BUTTON_DEBOUNCE)) {
    selectedParameter = (selectedParameter + 1) % PRESET_SIZE; // Passer au paramètre suivant
    
    Serial.println("🔘 BOUTON PRESSÉ - Changement de paramètre");
    Serial.print("  📋 Paramètre ");
    Serial.print(selectedParameter);
    Serial.print("/19: '");
    Serial.print(parameters[selectedParameter].name);
    Serial.print("'");
    Serial.println("");
    Serial.print("  📡 DMX Channel: ");
    Serial.print(parameters[selectedParameter].dmxChannel);
    Serial.print(" | Valeur actuelle: ");
    Serial.print(parameters[selectedParameter].value);
    Serial.print(" [");
    Serial.print(parameters[selectedParameter].value * 100 / 255);
    Serial.println("%]");
    Serial.println("  ──────────────────────────────────────");
    
    encoderButtonPressed = true;
    lastButtonPress = millis();
  }
  
  if (!buttonState) {
    encoderButtonPressed = false;
  }
  
  // Affichage périodique du statut du rotary encoder
  if (millis() - lastEncoderStatusTime >= ENCODER_STATUS_INTERVAL) {
    Serial.println("📊 STATUT ROTARY ENCODER:");
    Serial.print("  🎯 Paramètre actuel: ");
    Serial.print(selectedParameter);
    Serial.print("/19 - '");
    Serial.print(parameters[selectedParameter].name);
    Serial.println("'");
    Serial.print("  📡 DMX Channel: ");
    Serial.print(parameters[selectedParameter].dmxChannel);
    Serial.print(" | Valeur: ");
    Serial.print(parameters[selectedParameter].value);
    Serial.print(" [");
    Serial.print(parameters[selectedParameter].value * 100 / 255);
    Serial.println("%]");
    Serial.print("  🔢 Compteur encoder: ");
    Serial.println(encoder.getCount());
    Serial.println("  ──────────────────────────────────────");
    
    lastEncoderStatusTime = millis();
  }
}
*/

// Fonction d'initialisation des presets
void initializePresets() {
  Serial.println("Initialisation des presets...");
  
  // Preset 0 - Simple sans effet
  setAllParameters(0, 0, 64, 10, 0, 0, 100, 0, 75, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0);
  savePreset(0, "Simple");
  
  // Preset 1 - Simple avec effet
  setAllParameters(0, 0, 64, 10, 90, 110, 100, 0, 75, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0);
  savePreset(1, "Simple+Effet");
  
  // Preset 2 - Octaver and growl
  setAllParameters(0, 0, 64, 10, 90, 110, 145, 0, 75, 255, 0, 255, 140, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0);
  savePreset(2, "OctaverGrowl");
  
  // Preset 3 - Modern siren vibrafrenzy
  setAllParameters(0, 0, 162, 129, 140, 167, 205, 0, 108, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0, 0, 0, 0);
  savePreset(3, "ModernSiren");
  
  // Preset 4 - Classical
  setAllParameters(0, 0, 59, 16, 74, 83, 255, 0, 213, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  savePreset(4, "Classical");
  
  // Preset 5 - Furious octaver growl feedbacker
  setAllParameters(0, 0, 221, 10, 74, 241, 255, 0, 91, 255, 0, 255, 196, 0, 0, 0, 0, 0, 0, 50, 0, 0, 0, 0);
  savePreset(5, "FuriousGrowl");
  
  // Preset 6 - À définir
  setAllParameters(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  savePreset(6, "Preset6");
  
  // Preset 7 - À définir
  setAllParameters(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  savePreset(7, "Preset7");
  
  // Preset 8 - À définir
  setAllParameters(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  savePreset(8, "Preset8");
  
  // Preset 9 - À définir
  setAllParameters(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  savePreset(9, "Preset9");
  
  Serial.println("Presets initialisés");
}

// Fonction d'initialisation de l'interface utilisateur
void initializeUserInterface() {
  // Initialisation I2C
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  
  // Initialisation PCF8574
  if (pcf8574.begin()) {
    Serial.println("PCF8574 initialisé ✓");
  } else {
    Serial.println("Erreur PCF8574 ✗");
  }
  
  // Initialisation de l'encodeur KY-040
  encoder.attachHalfQuad(ENCODER_A_PIN, ENCODER_B_PIN);
  encoder.setCount(0);
  pinMode(ENCODER_BUTTON_PIN, INPUT_PULLUP);
  
  // Initialisation de l'afficheur TM1637
  display.init();
  display.setBrightness(7); // 0-7
  display.clearScreen();
  
  Serial.println("Interface utilisateur initialisée");
}

// Fonction pour lire les capteurs Sharp IR
void readSharpIRSensors() {
  // Lecture du premier capteur Sharp IR
  int rawValue1 = analogRead(DIST_SENSOR_1_PIN);
  uint8_t sharpIRValue1 = rawValue1 / 16;
  setParameter("pitch", sharpIRValue1);
  
  // Lecture du deuxième capteur Sharp IR
  int rawValue2 = analogRead(DIST_SENSOR_2_PIN);
  uint8_t sharpIRValue2 = rawValue2 / 16;
  setParameter("vibrato_speed", sharpIRValue2);
}

// Fonction pour lire les faders
void readFaders() {
  faderValues[0] = analogRead(FADER_1_PIN) / 16; // 0-4095 -> 0-255
  faderValues[1] = analogRead(FADER_2_PIN) / 16;
  faderValues[2] = analogRead(FADER_3_PIN) / 16;
  
  // Mise à jour des paramètres selon les faders
  if (faderValues[0] != lastFaderValues[0]) {
    setParameter("filter_cutoff", faderValues[0]);
    lastFaderValues[0] = faderValues[0];
  }
  
  if (faderValues[1] != lastFaderValues[1]) {
    setParameter("filter_reso", faderValues[1]);
    lastFaderValues[1] = faderValues[1];
  }
  
  if (faderValues[2] != lastFaderValues[2]) {
    setParameter("delay_time", faderValues[2]);
    lastFaderValues[2] = faderValues[2];
  }
}

// Fonction pour gérer les boutons PCF8574
void handleButtons() {
  for (int i = 0; i < 3; i++) {
    bool currentState = !pcf8574.digitalRead(i); // Inversé car INPUT_PULLUP
    
    if (currentState && !lastButtonStates[i] && (millis() - lastButtonPress[i] > BUTTON_DEBOUNCE)) {
      // Bouton pressé
      switch (i) {
        case 0: // Bouton 1 - Charger preset 0
          loadPreset(0);
          Serial.println("Bouton 1 - Preset 0 chargé");
          break;
        case 1: // Bouton 2 - Charger preset 1
          loadPreset(1);
          Serial.println("Bouton 2 - Preset 1 chargé");
          break;
        case 2: // Bouton 3 - Charger preset 2
          loadPreset(2);
          Serial.println("Bouton 3 - Preset 2 chargé");
          break;
      }
      lastButtonPress[i] = millis();
    }
    
    lastButtonStates[i] = currentState;
  }
}

// Fonction pour gérer l'encodeur KY-040
void handleEncoder() {
  int32_t currentEncoderValue = encoder.getCount();
  
  // Gestion de la rotation - changement de preset
  if (currentEncoderValue != lastEncoderValue) {
    int32_t delta = currentEncoderValue - lastEncoderValue;
    
    // Afficher les valeurs brutes de l'encodeur
    Serial.print("Encodeur - Brut: ");
    Serial.print(currentEncoderValue);
    Serial.print(" | Delta: ");
    Serial.print(delta);
    
    // Utiliser le modulo 20 puis diviser par 2 pour gérer les 2 deltas par cran physique
    int32_t moduloValue = (currentEncoderValue % 20) / 2;
    int32_t lastModuloValue = (lastEncoderValue % 20) / 2;
    
    // Détecter la direction et changer de preset
    if (moduloValue != lastModuloValue) {
      // Changer de preset directement selon la valeur modulo
      selectedPreset = moduloValue % 10;
      
      // Charger le preset sélectionné
      loadPreset(selectedPreset);
      
      Serial.print(" | Modulo/2: ");
      Serial.print(moduloValue);
      Serial.print(" | Preset: ");
      Serial.print(selectedPreset);
      Serial.print(" - ");
      Serial.println(presets[selectedPreset].name);
    }
    
    lastEncoderValue = currentEncoderValue;
  }
  
  // Gestion du bouton de l'encodeur - toggle filtre on/off
  bool buttonState = !digitalRead(ENCODER_BUTTON_PIN);
  
  if (buttonState && !encoderButtonPressed && (millis() - lastEncoderButtonPress > BUTTON_DEBOUNCE)) {
    // Toggle du filtre on/off
    uint8_t currentFilterState = getParameter("filter_on_off");
    uint8_t newFilterState = (currentFilterState == 0) ? 255 : 0;
    setParameter("filter_on_off", newFilterState);
    
    Serial.print("Bouton encodeur - Filtre: ");
    Serial.println((newFilterState == 255) ? "ON" : "OFF");
    
    encoderButtonPressed = true;
    lastEncoderButtonPress = millis();
  }
  
  if (!buttonState) {
    encoderButtonPressed = false;
  }
}

// Fonction pour mettre à jour l'afficheur TM1637
void updateDisplay() {
  if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    // Afficher le preset sélectionné
    display.display(selectedPreset);
    
    lastDisplayUpdate = millis();
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("=== Contrôleur Interactif ESP32 ===");
  Serial.println("Initialisation...");
  
  // Afficher l'adresse MAC de l'ESP32
  Serial.print("Adresse MAC ESP32: ");
  Serial.println(WiFi.macAddress());
  
  // Initialisation du tableau DMX à 0
  for (int i = 0; i < 512; i++) {
    dmxValues[i] = 0;
  }
  
  // Initialisation des paramètres
  initializeParameters();
  
  // Initialisation des presets
  initializePresets();
  
  // Chargement du preset 1 au démarrage
  loadPreset(1);
  
  // Initialisation de l'interface utilisateur
  initializeUserInterface();
  
  // Configuration ESP-NOW
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erreur d'initialisation ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  // Configuration du peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Erreur d'ajout du peer");
    return;
  }
  
  Serial.println("ESP-NOW initialisé");
  Serial.println("Fréquence d'émission: " + String(EMISSION_FREQUENCY) + "Hz");
  Serial.println("Capteurs Sharp IR sur GPIO35 et GPIO36");
  Serial.println("Faders sur GPIO32, GPIO33, GPIO34");
  Serial.println("Encodeur KY-040 sur GPIO26, GPIO27, GPIO25");
  Serial.println("Afficheur TM1637 sur GPIO18, GPIO19");
  Serial.println("Boutons PCF8574 via I2C (GPIO21, GPIO22)");
  Serial.println("================================");
}

void sendDMXvalues()
{
  // Envoi des 4 paquets DMX (512 canaux divisés en 4 blocs de 128)
  for (int packetNumber = 0; packetNumber < 4; packetNumber++)
  {
    outgoingDMXPacket.blockNumber = packetNumber;
    
    // Remplir le paquet avec les 128 valeurs correspondantes
    for (int i = 0; i < 128; i++)
    {
      int dmxIndex = (packetNumber * 128) + i;
      if (dmxIndex < 512) {
        outgoingDMXPacket.dmxvalues[i] = dmxValues[dmxIndex];
      } else {
        outgoingDMXPacket.dmxvalues[i] = 0;
      }
    }
    
    // Envoi du paquet via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&outgoingDMXPacket, sizeof(outgoingDMXPacket));
    
    if (result == ESP_OK) {
      //Serial.print(" [OK]");
    } else {
      Serial.print(" [ERREUR]");
    }
  }
  
  //Serial.println();
}

void setlights()
{
  // Utilisation de la valeur Sharp IR déjà calculée dans dmxValues
  uint8_t sharpIRValue1 = dmxValues[DMX_CHANNEL_IR_1 - 1]; // -1 car les canaux DMX commencent à 1
  
  // Canal DMX 1 : Mode de contrôle (0 = intensité rouge, 1 = autre mode, etc.)
  dmxValues[0] = 0; // Mode intensité rouge
  
  // Canal DMX 2 : Intensité rouge modulée par les valeurs Ksoloti
  // Utilisation de la division flottante pour un meilleur contrôle
  uint8_t ksoloti_modulation = (uint8_t)((float)ksoloti_val1);
  
  // Modulation finale avec la valeur Sharp IR
  dmxValues[1] = (3* ksoloti_modulation * sharpIRValue1) / 255;
  
  // Debug (optionnel)
  //Serial.print("Sharp IR DMX: "); Serial.print(sharpIRValue1);
  //Serial.print(" | Ksoloti mod: "); Serial.print(ksoloti_modulation);
  //Serial.print(" | DMX2: "); Serial.println(dmxValues[1]);
}

void loop()
{
  // Lecture des capteurs Sharp IR
  readSharpIRSensors();
  
  // Lecture des faders
  readFaders();
  
  // Gestion des boutons PCF8574
  handleButtons();
  
  // Gestion de l'encodeur KY-040
  handleEncoder();
  
  // Mise à jour de l'afficheur TM1637
  updateDisplay();
  
  // Émission à fréquence fixe (50Hz)
  if (millis() - lastEmissionTime >= EMISSION_INTERVAL) {
    setlights();
    sendDMXvalues();
    lastEmissionTime = millis();
  }
  
  delay(1); // Petit délai pour éviter de surcharger le CPU
}

