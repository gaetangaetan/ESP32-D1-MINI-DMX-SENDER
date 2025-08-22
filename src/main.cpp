// adresse mac de l'onirigun : 68:C6:3A:FD:37:17

#define VERSION 163 // onirigun
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
#include <TM1637Display.h>
#include <driver/adc.h>  // Pour les constantes ADC de l'ESP32
#include <FastLED.h>     // Pour le ruban WS2812B

#define CONTROL_TEST 0 // 1 pour tester les entrées, 0 pour le fonctionnement normal

// Définitions pour les capteurs Sharp IR
#define DIST_SENSOR_1_PIN 35    // GPIO35 - Premier capteur Sharp IR (changé pour plus de stabilité)
#define DIST_SENSOR_2_PIN 36    // GPIO36 - Deuxième capteur Sharp IR

// Définitions pour les faders analogiques
#define FADER_1_PIN 32    // GPIO32 - Premier fader
#define FADER_2_PIN 33    // GPIO33 - Deuxième fader (changé pour éviter le conflit)
#define FADER_3_PIN 34    // GPIO34 - Troisième fader

// Définitions pour l'encodeur rotatif KY-040
#define ENCODER_A_PIN 26    // GPIO26 - Pin A de l'encodeur
#define ENCODER_B_PIN 27    // GPIO27 - Pin B de l'encodeur
#define ENCODER_BUTTON_PIN 25  // GPIO25 - Bouton de l'encodeur

// Définitions pour l'afficheur TM1637
#define TM1637_CLK_PIN 18    // GPIO18 - CLK de l'afficheur
#define TM1637_DIO_PIN 19    // GPIO19 - DIO de l'afficheur

// Définitions pour les boutons push (remplacement du PCF8574)
#define BUTTON_1_PIN 22    // GPIO22 - Premier bouton (libéré du PCF8574)
#define BUTTON_2_PIN 21    // GPIO21 - Deuxième bouton (libéré du PCF8574)
#define BUTTON_3_PIN 23    // GPIO23 - Troisième bouton (nouveau GPIO)

// Définitions pour le ruban WS2812B
#define LED_STRIP_PIN 4    // GPIO4 - Signal DATA du WS2812B
#define NUM_LEDS 144         // Nombre de LEDs dans le ruban (ajustable)
#define BRIGHTNESS 64      // Luminosité (0-255)

// Configuration ESP-NOW
#define EMISSION_FREQUENCY 50  // Hz (20ms entre chaque émission)
#define DMX_CHANNEL_IR_1 102  // Canal DMX pour le premier capteur Sharp IR
#define DMX_CHANNEL_IR_2 103  // Canal DMX pour le deuxième capteur Sharp IR
#define MAX_DISTANCE_CM 80  // Distance maximale en cm pour Sharp IR (80cm = 0, 4cm = 255)
#define MIN_DISTANCE_CM 4   // Distance minimale en cm pour Sharp IR

// Configuration des presets
#define PRESET_SIZE 40  // Nombre de paramètres par preset (40 au lieu de 38)
#define MAX_PRESETS 10  // Nombre maximum de presets

// Configuration des trigs
#define TRIG_LENGTH 5   // Durée des trigs en nombre de paquets DMX

// Configuration du dimmer RGB
#define DIMMER_SPEED 0.01 // Vitesse de l'inertie du dimmer (0.0-1.0, plus petit = plus lent)

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
TM1637Display display(TM1637_CLK_PIN, TM1637_DIO_PIN);
ESP32Encoder encoder;

// Définition du tableau de LEDs pour FastLED
CRGB leds[NUM_LEDS];

// Définition des 37 paramètres du theremin (24 existants + 13 nouveaux)
Parameter parameters[PRESET_SIZE] = {
  // Paramètres existants (1-24)
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
  {"filter_type", 124, 0, 0},
  
  // Nouveaux paramètres (25-37)
  {"ir1_target_param", 125, 1, 1},        // 1 - index du paramètre contrôlé par IR1 (1 = pitch)
  {"ir2_target_param", 126, 3, 3},        // 2 - index du paramètre contrôlé par IR2 (3 = vibrato_depth)
  {"fader1_target_param", 127, 2, 2},     // 3 - index du paramètre contrôlé par fader1 (2 = vibrato_speed)
  {"fader2_target_param", 128, 4, 4},     // 4 - index du paramètre contrôlé par fader2 (4 = delay_time)
  {"fader3_target_param", 129, 5, 5},     // 5 - index du paramètre contrôlé par fader3 (5 = delay_feedback)
  {"button3_target_param", 130, 18, 18},  // 6 - index du paramètre contrôlé par bouton3 (18 = trig_hh)
  {"button3_released_value", 131, 0, 0},  // 7 - valeur quand bouton3 est relevé
  {"button3_pressed_value", 132, 5, 5},   // 8 - valeur quand bouton3 est pressé
  {"rgb1_red", 133, 255, 255},            // 9 - première valeur RGB (rouge)
  {"rgb1_green", 134, 0, 0},              // 10 - première valeur RGB (vert)
  {"rgb1_blue", 135, 0, 0},               // 11 - première valeur RGB (bleu)
  {"rgb2_red", 136, 0, 0},                // 12 - deuxième valeur RGB (rouge)
  {"rgb2_green", 137, 255, 255},          // 13 - deuxième valeur RGB (vert)
  {"rgb2_blue", 138, 0, 0},               // 14 - deuxième valeur RGB (bleu)
  {"dimmer1_source", 139, 0, 0},          // 15 - source du dimmer1 (0=IR1, 1=IR2, 2=Fader1, 3=Fader2, 4=Fader3)
  {"dimmer2_source", 140, 1, 1}           // 16 - source du dimmer2 (0=IR1, 1=IR2, 2=Fader1, 3=Fader2, 4=Fader3)
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

// Adresse MAC du récepteur ESP8266
uint8_t receiverAddress[] = {0x2C, 0xF4, 0x32, 0x7A, 0x08, 0x1E};

// Variables pour stocker les données reçues du Ksoloti
uint8_t ksoloti_val1 = 0;
uint8_t ksoloti_val2 = 0;
bool ksoloti_data_updated = false;

esp_now_peer_info_t peerInfo; // Configuration du peer récepteur ESP8266

// Variables de timing
unsigned long lastEmissionTime = 0;
const unsigned long EMISSION_INTERVAL = 1000 / EMISSION_FREQUENCY; // 20ms pour 50Hz

// Variables pour les boutons PCF8574
bool buttonStates[3] = {false, false, false};
bool lastButtonStates[3] = {false, false, false};
unsigned long lastButtonPress[3] = {0, 0, 0};
const unsigned long BUTTON_DEBOUNCE = 100; // 200ms de debounce

// Variables pour les interruptions de boutons
volatile bool buttonInterruptFlags[3] = {false, false, false};
volatile unsigned long buttonInterruptTimes[3] = {0, 0, 0};

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

// Variable pour le compteur d'affichage
uint16_t displayCounter = 0;

// Variables pour la stabilisation des capteurs IR - VERSION SIMPLIFIÉE
#define IR_SAMPLE_SIZE 10        // Taille de la moyenne mobile (ajustable)
#define IR_MAX_DEVIATION 50      // Limite des écarts autorisés (ajustable)

int irBuffer1[IR_SAMPLE_SIZE];   // Buffer pour IR1
int irBuffer2[IR_SAMPLE_SIZE];   // Buffer pour IR2
int irIndex1 = 0;                // Index circulaire IR1
int irIndex2 = 0;                // Index circulaire IR2
int irSum1 = 0;                  // Somme IR1
int irSum2 = 0;                  // Somme IR2
bool irInitialized1 = false;     // Initialisation IR1
bool irInitialized2 = false;     // Initialisation IR2

// Variables pour le dimmer RGB avec inertie
float dimmerValue1 = 0.0;        // Valeur actuelle du dimmer IR1 (0.0-1.0)
float dimmerValue2 = 0.0;        // Valeur actuelle du dimmer IR2 (0.0-1.0)
uint8_t rgb1RedDimmed = 0;       // RGB1 rouge dimmé
uint8_t rgb1GreenDimmed = 0;     // RGB1 vert dimmé
uint8_t rgb1BlueDimmed = 0;      // RGB1 bleu dimmé
uint8_t rgb2RedDimmed = 0;       // RGB2 rouge dimmé
uint8_t rgb2GreenDimmed = 0;     // RGB2 vert dimmé
uint8_t rgb2BlueDimmed = 0;      // RGB2 bleu dimmé

// Variables pour la transposition du pitch
int8_t transpose = 0;           // Valeur de transposition (-12 à +12)
uint8_t transpose_factor = 24;  // Facteur de transposition

// Prototypes
void displayUnified();

// Fonction pour stabiliser un capteur IR avec moyenne mobile et limitation d'aberrants
int stabilizeIRSensor(int newValue, int* buffer, int& index, int& sum, bool& initialized) {
  // Initialisation : remplir le buffer avec la première valeur
  if (!initialized) {
    for (int i = 0; i < IR_SAMPLE_SIZE; i++) {
      buffer[i] = newValue;
    }
    sum = newValue * IR_SAMPLE_SIZE;
    initialized = true;
  }
  
  // Calculer la moyenne actuelle
  int currentAverage = sum / IR_SAMPLE_SIZE;
  
  // Vérifier si la nouvelle valeur est aberrante
  int deviation = abs(newValue - currentAverage);
  
  if (deviation > IR_MAX_DEVIATION) {
    // Valeur aberrante : la limiter à moyenne ± seuil
    int limitedValue;
    if (newValue > currentAverage) {
      limitedValue = currentAverage + IR_MAX_DEVIATION;
    } else {
      limitedValue = currentAverage - IR_MAX_DEVIATION;
    }
    
    // Mettre à jour le buffer avec la valeur limitée
    sum -= buffer[index];
    buffer[index] = limitedValue;
    sum += limitedValue;
  } else {
    // Valeur normale : l'ajouter au buffer
    sum -= buffer[index];
    buffer[index] = newValue;
    sum += newValue;
  }
  
  // Passer à l'index suivant (buffer circulaire)
  index = (index + 1) % IR_SAMPLE_SIZE;
  
  // Retourner la moyenne
  return sum / IR_SAMPLE_SIZE;
}

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
      // Vérifier si c'est le paramètre pitch (indice 1)
      if (i == 1) { // pitch
        // Appliquer la transposition
        int transposedValue = (int)value + (transpose * transpose_factor);
        
        // Limiter la valeur entre 0 et 255
        if (transposedValue < 0) transposedValue = 0;
        if (transposedValue > 255) transposedValue = 255;
        
        parameters[i].value = (uint8_t)transposedValue;
        // Mettre à jour le tableau DMX
        dmxValues[parameters[i].dmxChannel - 1] = (uint8_t)transposedValue;
        
        // Debug de la transposition
        // Serial.print("Pitch transposé: ");
        // Serial.print(value);
        // Serial.print(" + (");
        // Serial.print(transpose);
        // Serial.print(" × ");
        // Serial.print(transpose_factor);
        // Serial.print(") = ");
        // Serial.println(transposedValue);
      } else {
        // Pour les autres paramètres, comportement normal
        parameters[i].value = value;
        // Mettre à jour le tableau DMX
        dmxValues[parameters[i].dmxChannel - 1] = value;
      }
      return;
    }
  }
  //Serial.println("Paramètre " + String(paramName) + " non trouvé");
}

// Fonction pour mettre à jour le gate_threshold avec la transposition
void updateGateThresholdWithTranspose() {
  // Récupérer la valeur par défaut du preset pour gate_threshold
  uint8_t originalGateThreshold = presets[selectedPreset].values[7]; // indice 7 = gate_threshold
  
  // Appliquer la transposition
  int transposedGateThreshold = (int)originalGateThreshold + (transpose * transpose_factor);
  
  // Limiter la valeur entre 0 et 255
  if (transposedGateThreshold < 0) transposedGateThreshold = 0;
  if (transposedGateThreshold > 255) transposedGateThreshold = 255;
  
  // Mettre à jour le paramètre
  parameters[7].value = (uint8_t)transposedGateThreshold;
  dmxValues[parameters[7].dmxChannel - 1] = (uint8_t)transposedGateThreshold;
  
  // Debug
  Serial.print("Gate threshold ajusté: ");
  Serial.print(originalGateThreshold);
  Serial.print(" + (");
  Serial.print(transpose);
  Serial.print(" × ");
  Serial.print(transpose_factor);
  Serial.print(") = ");
  Serial.print(transposedGateThreshold);
  Serial.print(" (DMX canal ");
  Serial.print(parameters[7].dmxChannel);
  Serial.print(", index DMX ");
  Serial.print(parameters[7].dmxChannel - 1);
  Serial.println(")");
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

// Variables globales pour stocker les liens des contrôles physiques
// REMOVED: Les liens sont maintenant lus dynamiquement depuis le preset actuel

// Fonction pour mettre à jour les liens des contrôles physiques depuis le preset actuel
// REMOVED: Plus nécessaire car les liens sont lus à chaque appel

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
  
  // Les liens des contrôles physiques sont maintenant lus dynamiquement à chaque appel
  
  Serial.println("Preset " + String(presetIndex) + " chargé: " + String(presets[presetIndex].name));
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



// Fonction pour enregistrer tous les paramètres d'un coup (40 arguments)
void setAllParameters(
uint8_t autopan_depth,   // 0
uint8_t pitch,          // 1
uint8_t vibrato_speed,  // 2
uint8_t vibrato_depth,  // 3  
uint8_t delay_time,     // 4
uint8_t delay_feedback, // 5
uint8_t osc_waveform,  // 6
uint8_t gate_threshold, // 7
uint8_t portamento_time, // 8
uint8_t scale,          // 9
uint8_t octave_low_high, // 10
uint8_t osc2_volume,    // 11
uint8_t osc2_pitch_offset, // 12
uint8_t autopan_frequency, // 13
uint8_t scale_tonic, // 14
uint8_t volume_drums, // 15
uint8_t trig_kick, // 16
uint8_t trig_snare, // 17
uint8_t trig_hh, // 18
uint8_t master_volume, // 19
uint8_t filter_on_off, // 20
uint8_t filter_cutoff, // 21
uint8_t filter_reso, // 22
uint8_t filter_type, // 23
                     // Nouveaux paramètres
uint8_t ir1_target_param, // 24
uint8_t ir2_target_param, // 25
uint8_t fader1_target_param, // 26
uint8_t fader2_target_param, // 27
uint8_t fader3_target_param, // 28
uint8_t button3_target_param, // 29
uint8_t button3_released_value, // 30
uint8_t button3_pressed_value, // 31
uint8_t rgb1_red, // 32
uint8_t rgb1_green, // 33
uint8_t rgb1_blue, // 34
uint8_t rgb2_red, // 35
uint8_t rgb2_green, // 36
uint8_t rgb2_blue, // 37
uint8_t dimmer1_source, // 38
uint8_t dimmer2_source) { // 39
  
  // Mettre à jour tous les paramètres existants (0-23)
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
  
  // Mettre à jour les nouveaux paramètres (24-39)
  parameters[24].value = ir1_target_param;
  parameters[25].value = ir2_target_param;
  parameters[26].value = fader1_target_param;
  parameters[27].value = fader2_target_param;
  parameters[28].value = fader3_target_param;
  parameters[29].value = button3_target_param;
  parameters[30].value = button3_released_value;
  parameters[31].value = button3_pressed_value;
  parameters[32].value = rgb1_red;
  parameters[33].value = rgb1_green;
  parameters[34].value = rgb1_blue;
  parameters[35].value = rgb2_red;
  parameters[36].value = rgb2_green;
  parameters[37].value = rgb2_blue;
  parameters[38].value = dimmer1_source;
  parameters[39].value = dimmer2_source;
  
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



// Fonction pour lire et stabiliser un capteur IR
int readStabilizedIRSensor(int sensorPin, int* buffer, int& index, int& sum, bool& initialized) {
  // Lecture brute
  int rawValue = analogRead(sensorPin);
  
  // Application du filtre de stabilisation
  int stabilizedValue = stabilizeIRSensor(rawValue, buffer, index, sum, initialized);
  
  return stabilizedValue;
}

// Fonction pour mapping amélioré d'IR1 (pitch)
uint8_t mapIR1Logarithmic(int rawValue) {
  // Normaliser la valeur d'entrée (0-4095 → 0.0-1.0)
  float normalized = (float)rawValue / 4095.0;
  
  // Mapping exponentiel inverse pour plus de contrôle dans les petites distances
  // x^0.3 donne une courbe qui monte rapidement au début puis se stabilise
  float expMapped = pow(normalized, 0.3);
  
  // Convertir en 0-255
  uint8_t result = (uint8_t)(expMapped * 255.0);
  
  return result;
}

// Fonction pour mapping linéaire d'IR1 (dimmer)
uint8_t mapIR1Linear(int rawValue) {
  // Mapping linéaire simple pour les dimmers
  return rawValue / 16; // 0-4095 → 0-255
}

// Fonction pour lire la valeur d'un contrôle assigné (0-4)
uint8_t getControlValue(uint8_t source) {
  switch(source) {
    case 0: // IR1 (mapping linéaire pour dimmer)
      return mapIR1Linear(readStabilizedIRSensor(DIST_SENSOR_1_PIN, irBuffer1, irIndex1, irSum1, irInitialized1));
    case 1: // IR2
      return readStabilizedIRSensor(DIST_SENSOR_2_PIN, irBuffer2, irIndex2, irSum2, irInitialized2) / 16;
    case 2: // Fader1 (inversé)
      return (4095 - analogRead(FADER_1_PIN)) / 16;
    case 3: // Fader2 (inversé)
      return (4095 - analogRead(FADER_2_PIN)) / 16;
    case 4: // Fader3 (inversé)
      return (4095 - analogRead(FADER_3_PIN)) / 16;
    default:
      return 0;
  }
}

// Fonction pour mettre à jour le dimmer RGB avec inertie
void updateRGBWithDimmer() {
  // Lecture des valeurs RGB depuis les paramètres
  uint8_t rgb1Red = getParameter("rgb1_red");
  uint8_t rgb1Green = getParameter("rgb1_green");
  uint8_t rgb1Blue = getParameter("rgb1_blue");
  uint8_t rgb2Red = getParameter("rgb2_red");
  uint8_t rgb2Green = getParameter("rgb2_green");
  uint8_t rgb2Blue = getParameter("rgb2_blue");
  
  // Lecture des sources des dimmers depuis les paramètres
  uint8_t dimmer1Source = getParameter("dimmer1_source");
  uint8_t dimmer2Source = getParameter("dimmer2_source");
  
  // Lecture des valeurs des contrôles assignés
  uint8_t control1Value = getControlValue(dimmer1Source);
  uint8_t control2Value = getControlValue(dimmer2Source);
  
  // Mapping 50-255 → 0-255 (en dessous de 50 = éteint)
  float rawDimmer1 = (float)control1Value / 255.0;
  float rawDimmer2 = (float)control2Value / 255.0;
  
  float targetDimmer1 = (rawDimmer1 < 0.196) ? 0.0 : (rawDimmer1 - 0.196) / (1.0 - 0.196); // 50/255 ≈ 0.196
  float targetDimmer2 = (rawDimmer2 < 0.196) ? 0.0 : (rawDimmer2 - 0.196) / (1.0 - 0.196);
  
  // Application de l'inertie
  dimmerValue1 += (targetDimmer1 - dimmerValue1) * DIMMER_SPEED;
  dimmerValue2 += (targetDimmer2 - dimmerValue2) * DIMMER_SPEED;
  
  // Calcul des valeurs RGB dimmées
  rgb1RedDimmed = (uint8_t)(rgb1Red * dimmerValue2); // Dimmer2 contrôle DMX RGB
  rgb1GreenDimmed = (uint8_t)(rgb1Green * dimmerValue2);
  rgb1BlueDimmed = (uint8_t)(rgb1Blue * dimmerValue2);
  
  rgb2RedDimmed = (uint8_t)(rgb2Red * dimmerValue1); // Dimmer1 contrôle LED strip
  rgb2GreenDimmed = (uint8_t)(rgb2Green * dimmerValue1);
  rgb2BlueDimmed = (uint8_t)(rgb2Blue * dimmerValue1);
  
  // Debug (optionnel)
  static unsigned long lastDimmerDebugTime = 0;
  if (millis() - lastDimmerDebugTime > 2000) { // Debug toutes les 2 secondes
    Serial.print("Dimmer - S1:");
    Serial.print(dimmer1Source);
    Serial.print(" S2:");
    Serial.print(dimmer2Source);
    Serial.print(" V1:");
    Serial.print(control1Value);
    Serial.print(" V2:");
    Serial.print(control2Value);
    Serial.print(" D1:");
    Serial.print(dimmerValue1, 2);
    Serial.print(" D2:");
    Serial.print(dimmerValue2, 2);
    Serial.println();
    lastDimmerDebugTime = millis();
  }
}



// Fonction pour contrôler les paramètres via les entrées physiques
// Les liens des contrôles sont maintenant lus dynamiquement depuis le preset actuel
// Si un paramètre de lien vaut 0, le contrôle correspondant est désactivé
void handlePhysicalControls() {
  // Lecture stabilisée des capteurs IR
  int stabilizedIR1 = readStabilizedIRSensor(DIST_SENSOR_1_PIN, irBuffer1, irIndex1, irSum1, irInitialized1);
  int stabilizedIR2 = readStabilizedIRSensor(DIST_SENSOR_2_PIN, irBuffer2, irIndex2, irSum2, irInitialized2);
  uint8_t ir1Value = mapIR1Logarithmic(stabilizedIR1); // Mapping amélioré pour IR1
  uint8_t ir2Value = stabilizedIR2 / 16; // 0-4095 → 0-255
  
  // Lecture des faders (inversés : 4095-0 → 0-255)
  uint8_t fader1Value = (4095 - analogRead(FADER_1_PIN)) / 16; // 4095-0 → 0-255
  uint8_t fader2Value = (4095 - analogRead(FADER_2_PIN)) / 16;
  uint8_t fader3Value = (4095 - analogRead(FADER_3_PIN)) / 16;
  
  // Lecture du bouton 3
  bool button3State = !digitalRead(BUTTON_3_PIN);
  
  // Lire les liens des contrôles directement depuis le preset actuel
  uint8_t ir1TargetParam = getParameter("ir1_target_param");
  uint8_t ir2TargetParam = getParameter("ir2_target_param");
  uint8_t fader1TargetParam = getParameter("fader1_target_param");
  uint8_t fader2TargetParam = getParameter("fader2_target_param");
  uint8_t fader3TargetParam = getParameter("fader3_target_param");
  uint8_t button3TargetParam = getParameter("button3_target_param");
  
  // Appliquer les valeurs aux paramètres cibles selon les liens du preset
  if (ir1TargetParam >= 0 && ir1TargetParam < PRESET_SIZE) {
    setParameter(parameters[ir1TargetParam].name, ir1Value);
  }
  
  if (ir2TargetParam >= 0 && ir2TargetParam < PRESET_SIZE) {
    setParameter(parameters[ir2TargetParam].name, ir2Value);
  }
  
      if (fader1TargetParam >= 0 && fader1TargetParam < PRESET_SIZE) {
      setParameter(parameters[fader1TargetParam].name, fader1Value);
    }
    
    if (fader2TargetParam >= 0 && fader2TargetParam < PRESET_SIZE) {
      setParameter(parameters[fader2TargetParam].name, fader2Value);
    }
    
    if (fader3TargetParam >= 0 && fader3TargetParam < PRESET_SIZE) {
      setParameter(parameters[fader3TargetParam].name, fader3Value);
    }


  // Gestion conditionnelle des faders selon l'état du filtre
  //uint8_t filterState = getParameter("filter_on_off"); 

  // if (filterState == 0) {
  //   // Filter OFF : fader1 → vibrato_speed, fader2 → delay_time, fader3 → delay_feedback
  //   setParameter("vibrato_speed", fader1Value);   // Paramètre 3
  //   setParameter("delay_time", fader2Value);      // Paramètre 5
  //   setParameter("delay_feedback", fader3Value);  // Paramètre 6
  // } else {
  //   // Filter ON : utiliser les assignations du preset
  //   if (fader1TargetParam >= 0 && fader1TargetParam < PRESET_SIZE) {
  //     setParameter(parameters[fader1TargetParam].name, fader1Value);
  //   }
    
  //   if (fader2TargetParam >= 0 && fader2TargetParam < PRESET_SIZE) {
  //     setParameter(parameters[fader2TargetParam].name, fader2Value);
  //   }
    
  //   if (fader3TargetParam >= 0 && fader3TargetParam < PRESET_SIZE) {
  //     setParameter(parameters[fader3TargetParam].name, fader3Value);
  //   }
  // }
  
  // Gestion du bouton 3 avec valeurs released/pressed
  if (button3TargetParam > 0 && button3TargetParam < PRESET_SIZE) {
    uint8_t button3ReleasedValue = getParameter("button3_released_value");
    uint8_t button3PressedValue = getParameter("button3_pressed_value");
    uint8_t button3TargetValue = button3State ? button3PressedValue : button3ReleasedValue;
    setParameter(parameters[button3TargetParam].name, button3TargetValue);
  }
  
  // Mettre à jour le dimmer RGB
  updateRGBWithDimmer();
  
  // Debug (optionnel)
  static unsigned long lastDebugTime = 0;
  if (millis() - lastDebugTime > 1000) { // Debug toutes les secondes
    Serial.print("Contrôles - IR1:");
    Serial.print(ir1Value);
    Serial.print(" IR2:");
    Serial.print(ir2Value);
    Serial.print(" F1:");
    Serial.print(fader1Value);
    Serial.print(" F2:");
    Serial.print(fader2Value);
    Serial.print(" F3:");
    Serial.print(fader3Value);
    Serial.print(" B3:");
    Serial.print(button3State ? "ON" : "OFF");
    Serial.println();
    lastDebugTime = millis();
  }
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
// Fonction d'initialisation des presets
void initializePresets() {

/*
Targets : (0=IR1, 1=IR2, 2=Fader1, 3=Fader2, 4=Fader3)

0  = autopan_depth           10  = octave_low_high         20 = filter_on_off            30 = button3_released_value
1  = pitch                   11 = osc2_volume              21 = filter_cutoff            31 = button3_pressed_value
2  = vibrato_speed           12 = osc2_pitch_offset        22 = filter_reso              32 = rgb1_red
3  = vibrato_depth           13 = autopan_frequency        23 = filter_type              33 = rgb1_green
4  = delay_time              14 = scale_tonic              24 = ir1_target_param         34 = rgb1_blue
5  = delay_feedback          15 = volume_drums             25 = ir2_target_param         35 = rgb2_red
6  = osc_waveform            16 = trig_kick                26 = fader1_target_param      36 = rgb2_green
7  = gate_threshold          17 = trig_snare               27 = fader2_target_param      37 = rgb2_blue
8  = portamento_time         18 = trig_hh                  28 = fader3_target_param      38 = dimmer1_source
9  = scale                   19 = master_volume            29 = button3_target_param     39 = dimmer2_source
*/
  Serial.println("Initialisation des presets...");

//                  |pandepth        |time           |porta          |osc2off       |kick       |master        |type             |fader2        |b3_pressed     /r2             /dim2
//                       |pitch          |delFB          |scale         |autofreq       |snare      |filter         |ir1             |fader3        /r1             /g2
//                           |vibraspeed     |osc            |octlow        |tonic          |hh         |cutoff         |ir2            |button3        /g1             /b2
//                               |vibradepth     |gate           |osc2vol       |drums         |master     |reso            |fader1         |b3_rel         /b1             /dim1

    // Preset 2 - Classical scale
  //                 0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  
  setAllParameters(  0,  0, 90,  2, 74, 83,255,155,213,255,  0,  0,  0,  0,  0,255,  0,  0,  0,  0,  0,  0,  0,  0,  1, 15, 14,  4,  5, 18,  0,  5,255,  0,  50,255,0,  50,  0,  0);
  savePreset(2, "Classical scale");

      // Preset 3 - Classical scale siren
  //                 0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  
  setAllParameters(  0,  0, 100, 3, 74, 83,100,155,213,255,  0,  0,  0,  0,  0,255,  0,  0,  0,  0,  0,  0,  0,  0,  1, 15, 14,  4,  5, 18,  0,  5,255,  0,  50,255,0,  50,  0,  0);
  savePreset(3, "Classical scale siren");

      // Preset 0 - Classical
  //                 0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  
  setAllParameters(  0,  0, 59, 16, 74, 83,255,155,213,  0,  0,  0,  0,  0,  0,255,  0,  0,  0,  0,  0,  0,  0,  0,  1, 15,  2,  4,  5, 18,  0,  5,255,  0,  50,255,0,  50,  0,  0);
  savePreset(0, "Classical");

  // Preset 6 - Buzz1
  //                 0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  
  setAllParameters(  0,  0, 59, 16, 74, 83,100,155,213,  0,  0,  0,  0,  0,  0,255,  0,  0,  0,  0,  0,  0,  0,  0,  1, 15,  2,  4,  5, 18,  0,  5,255,  0,  50,255,0,  50,  0,  0);
  savePreset(6, "Buzz1");

  // Preset 1 - Bass growler
  //                 0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  
  setAllParameters(  0,  0, 59, 16, 74, 83,255,100,213,  0,  0,255,  0,  0,  0,255,  0,  0,  0,  0,  0,  0,  0,  0,  1,  0, 12,  4,  5, 18,  0,  5,255,  0,  50,255,0,  50,  0,  0);
  savePreset(1, "Bass growler");
  

  // Preset 5 - Furious octaver growl feedbacker
  //                 0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  
  setAllParameters(  0,150,221, 10, 74,241,255,  0, 91,255,  0,255,196,  0,  0,255,  0,  0,  0, 50,  0,  0,  0,  0, 12,  3, 21, 22, 6, 18,  0,  5,255,  0,  0,  0,255,  0,  0,  1);
  savePreset(5, "FuriousGrowl");




    // Preset 7 - À définir
  //                 0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  
  setAllParameters(  0,  0,  0,  0,  0,  0,  0,  120, 75, 50,  0,150, 150,  0, 0,255,  0,  0,  0,  0,  0,  0,  0,  0,  1,  3, 21, 22, 23, 18,  0,  5,  0,255, 15,200,10, 255, 0,  0);
  savePreset(7, "Preset7");



  // Preset 4 - Simple sans effet
  //                 0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  
  setAllParameters(  0,  0, 64, 10,  0,  0,100,  80, 75,  0,  0,  0,  0,  0,  0,255,  0,  0,  0, 0,  0,  0,  0,  0,  1,  3,  2,  4,  5, 18,  0,  5,255,  0,  0,  0,255,  0,  0,  1);
  savePreset(4, "Simple");
  
  // Preset 1 - Simple avec effet
  //                 0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  
  setAllParameters(  0,  0,  0,  0,130,110,100, 100, 30,  0,255,  0,  0,  0,  0,255,  0,  0,  0, 0,  0,  0,  0,  0,  1,  3,  2,  4,  5, 18,  0,  5,255,  0,  0,255,  0,  0,  0,  1);
  savePreset(8, "Simple+Effet");
  
  // // Preset 2 - Octaver and growl
  // //                 0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  
  // setAllParameters(  0,  0, 64, 10, 90,110,145,  0, 75,255,  0,255,140,  0,  0,255,  0,  0,  0, 0,  0,  0,  0,  0,  1,  3,  21, 22, 23, 18,  0,  5,255,255,  50,  0,255,255,  0,  1);
  // savePreset(2, "OctaverGrowl");
  
  // // Preset 3 - Modern siren vibrafrenzy
  // //                 0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  
  // setAllParameters(  0,  0,162,129,140,167,205,  0,108,  0,  0,  0,  0,  0,  0,255,  0,  0,  0,100,  0,  0,  0,  0,  1,  3,  2,  4,  5, 18,  0,  5,255,  0,  0,255,  0, 50,  0,  1);
  // savePreset(3, "ModernSiren");
  

  
  

  

  
  
  // Preset 9 - À définir
  //                 0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  
  setAllParameters(  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,255,  0,  0,  0,  0,  0,  0,  0,  0,  1,  3,  2,  4,  5, 18,  0,  5,255,  0,  0,  0,255,  0,  0,  1);
  savePreset(9, "Preset9");
  
  Serial.println("Presets initialisés");
}
// Fonctions d'interruption pour les boutons
void IRAM_ATTR button1ISR() {
  unsigned long interruptTime = millis();
  if (interruptTime - buttonInterruptTimes[0] > BUTTON_DEBOUNCE) {
    buttonInterruptFlags[0] = true;
    buttonInterruptTimes[0] = interruptTime;
  }
}

void IRAM_ATTR button2ISR() {
  unsigned long interruptTime = millis();
  if (interruptTime - buttonInterruptTimes[1] > BUTTON_DEBOUNCE) {
    buttonInterruptFlags[1] = true;
    buttonInterruptTimes[1] = interruptTime;
  }
}

void IRAM_ATTR button3ISR() {
  unsigned long interruptTime = millis();
  if (interruptTime - buttonInterruptTimes[2] > BUTTON_DEBOUNCE) {
    buttonInterruptFlags[2] = true;
    buttonInterruptTimes[2] = interruptTime;
  }
}

// Fonction d'initialisation des interruptions
void initializeButtonInterrupts() {
  // Configuration des pins en INPUT_PULLUP
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(BUTTON_3_PIN, INPUT_PULLUP);
  
  // Attacher les interruptions (FALLING car INPUT_PULLUP)
  attachInterrupt(digitalPinToInterrupt(BUTTON_1_PIN), button1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_2_PIN), button2ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_3_PIN), button3ISR, FALLING);
  
  Serial.println("Interruptions de boutons configurées");
}

// Fonction d'initialisation de l'interface utilisateur
void initializeUserInterface() {
 
  // Configuration des boutons push avec interruptions
  initializeButtonInterrupts();
  
  // Initialisation du ruban WS2812B
  FastLED.addLeds<WS2812B, LED_STRIP_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();
  
  // Initialisation de l'encodeur KY-040
  encoder.attachHalfQuad(ENCODER_A_PIN, ENCODER_B_PIN);
  encoder.setCount(0);
  pinMode(ENCODER_BUTTON_PIN, INPUT_PULLUP);
  
  // Initialisation de l'afficheur TM1637
  display.setBrightness(7); // 0-7
  display.clear();
 
    // Configuration ADC pour une meilleure résolution
    analogReadResolution(12); // 12 bits (0-4095)

  Serial.println("Interface utilisateur initialisée");
}

// Fonction pour lire les capteurs Sharp IR avec stabilisation - NOUVELLE STRATÉGIE
void readSharpIRSensors() {
  // Lecture stabilisée du premier capteur Sharp IR
  int stabilizedValue1 = readStabilizedIRSensor(DIST_SENSOR_1_PIN, irBuffer1, irIndex1, irSum1, irInitialized1);
  uint8_t sharpIRValue1 = stabilizedValue1 / 16;
  setParameter("pitch", sharpIRValue1);
  
  // Lecture stabilisée du deuxième capteur Sharp IR
  int stabilizedValue2 = readStabilizedIRSensor(DIST_SENSOR_2_PIN, irBuffer2, irIndex2, irSum2, irInitialized2);
  uint8_t sharpIRValue2 = stabilizedValue2 / 16;
  setParameter("vibrato_speed", sharpIRValue2);
  
  // Debug minimal
  Serial.print("IR1: ");
  Serial.print(stabilizedValue1);
  Serial.print(" -> ");
  Serial.print(sharpIRValue1);
  Serial.print(" | IR2: ");
  Serial.print(stabilizedValue2);
  Serial.print(" -> ");
  Serial.println(sharpIRValue2);
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

// Fonction pour gérer les boutons push
void handleButtons() {
  // Lecture des boutons
  bool button1State = !digitalRead(BUTTON_1_PIN); // Inversé car INPUT_PULLUP
  bool button2State = !digitalRead(BUTTON_2_PIN);
  bool button3State = !digitalRead(BUTTON_3_PIN);
  
  // Bouton 1 - Déclencher trig_kick (paramètre 17, canal DMX 117)
  if (button1State && !lastButtonStates[0] && (millis() - lastButtonPress[0] > BUTTON_DEBOUNCE)) {
    dmxValues[116] = 255; // Canal DMX 117 (index 116)
    //Serial.println("Bouton 1 - Trig Kick déclenché");
    lastButtonPress[0] = millis();
  }
  lastButtonStates[0] = button1State;
  
  // Bouton 2 - Déclencher trig_snare (paramètre 18, canal DMX 118)
  if (button2State && !lastButtonStates[1] && (millis() - lastButtonPress[1] > BUTTON_DEBOUNCE)) {
    dmxValues[117] = 255; // Canal DMX 118 (index 117)
    //Serial.println("Bouton 2 - Trig Snare déclenché");
    lastButtonPress[1] = millis();
  }
  lastButtonStates[1] = button2State;
  
  // Bouton 3 - Charger preset 2
  if (button3State && !lastButtonStates[2] && (millis() - lastButtonPress[2] > BUTTON_DEBOUNCE)) {
    //dmxValues[116] = 255; // KICK
    //Serial.println("Bouton 3 - Preset 2 chargé");
    lastButtonPress[2] = millis();
  }
  lastButtonStates[2] = button3State;
}


// Fonction pour gérer les boutons avec interruptions
void handleButtonInterrupts() {
  // Traiter les interruptions du bouton 1 (décrémenter transposition)
  if (buttonInterruptFlags[0]) {
    if (transpose > -12) {
      transpose--;
      Serial.print("Bouton 1 - Transposition décrémentée: ");
      Serial.println(transpose);
      
      // Afficher immédiatement la nouvelle transposition
      displayUnified();
      
      // Recharger le preset pour appliquer la nouvelle transposition
      loadPreset(selectedPreset);
      
      // Mettre à jour le gate_threshold avec la nouvelle transposition
      updateGateThresholdWithTranspose();
    }
    buttonInterruptFlags[0] = false;
  }
  
  // Traiter les interruptions du bouton 2 (incrémenter transposition)
  if (buttonInterruptFlags[1]) {
    if (transpose < 12) {
      transpose++;
      Serial.print("Bouton 2 - Transposition incrémentée: ");
      Serial.println(transpose);
      
      // Afficher immédiatement la nouvelle transposition
      displayUnified();
      
      // Recharger le preset pour appliquer la nouvelle transposition
      loadPreset(selectedPreset);
      
      // Mettre à jour le gate_threshold avec la nouvelle transposition
      updateGateThresholdWithTranspose();
    }
    buttonInterruptFlags[1] = false;
  }
  
  // Traiter les interruptions du bouton 3 (trig_hh)
  if (buttonInterruptFlags[2]) {
    dmxValues[118] = TRIG_LENGTH; // Canal DMX 119 (trig_hh)
    Serial.println("Bouton 3 - Trig HH déclenché (ISR) - Durée: " + String(TRIG_LENGTH));
    buttonInterruptFlags[2] = false;
  }
}


// Fonction pour gérer l'encodeur KY-040
void handleEncoder() {
  int32_t currentEncoderValue = - encoder.getCount();
  
  // Calculer le preset selon la formule : |valeur_encoder / 2| % 10
  // Utiliser la valeur absolue pour éviter les problèmes avec les grandes valeurs négatives
  int32_t normalizedValue = abs(currentEncoderValue / 2);
  uint8_t newPreset = normalizedValue % MAX_PRESETS;
  
  // Si le preset a changé
  if (newPreset != selectedPreset) {
    selectedPreset = newPreset;
    
    // Réinitialiser la transposition lors du changement de preset
    transpose = 0;
    
    // Charger le preset sélectionné
    loadPreset(selectedPreset);
    
    // Pas besoin d'updateGateThresholdWithTranspose() car transpose = 0
    
    // Afficher le nouveau preset avec transposition réinitialisée
    displayUnified();
    
    // Debug sur le moniteur série
    Serial.print("Preset changé: ");
    Serial.print(selectedPreset);
    Serial.print(" - ");
    Serial.println(presets[selectedPreset].name);
    Serial.println("Transposition réinitialisée à 0");
  }
  
  // Gestion du bouton de l'encodeur (toggle filtre)
  bool encoderButtonState = !digitalRead(ENCODER_BUTTON_PIN);
  if (encoderButtonState && !encoderButtonPressed && (millis() - lastEncoderButtonPress > BUTTON_DEBOUNCE)) {
    uint8_t currentFilterState = getParameter("filter_on_off");
    uint8_t newFilterState = (currentFilterState == 0) ? 255 : 0;
    setParameter("filter_on_off", newFilterState);
    
    // Afficher temporairement l'état du filtre
    display.clear();
    if (newFilterState == 255) {
      display.showNumberDec(9999); // Afficher "9999" pour indiquer FILTRE ON
    } else {
      display.showNumberDec(0);    // Afficher "0000" pour indiquer FILTRE OFF
    }
    
    Serial.print("Filtre: ");
    Serial.println((newFilterState == 255) ? "ON" : "OFF");
    
    // Attendre 500ms puis revenir à l'affichage unifié
    delay(500);
    displayUnified();
    
    encoderButtonPressed = true;
    lastEncoderButtonPress = millis();
  }
  
  if (!encoderButtonState) {
    encoderButtonPressed = false;
  }
}

// Fonction pour afficher la transposition et le preset de manière unifiée
void displayUnified() {
  // Format: TTPP où TT = transposition (-12 à +12) et PP = preset (0-9)
  // Exemples: 
  // - Transposition +5, preset 3 → "0503" 
  // - Transposition -2, preset 7 → "-207"
  // - Transposition 0, preset 1 → "0001"
  
  uint8_t segments[4] = {0, 0, 0, 0};
  
  // Calcul pour les digits de la transposition (positions 0 et 1)
  if (transpose == 0) {
    // Transposition = 0 : afficher "00"
    segments[0] = display.encodeDigit(0);
    segments[1] = display.encodeDigit(0);
  } else if (transpose > 0) {
    // Transposition positive : afficher directement le nombre
    if (transpose >= 10) {
      segments[0] = display.encodeDigit(transpose / 10);
      segments[1] = display.encodeDigit(transpose % 10);
    } else {
      segments[0] = display.encodeDigit(0);
      segments[1] = display.encodeDigit(transpose);
    }
  } else {
    // Transposition négative : afficher "-" + valeur absolue
    segments[0] = 0x40; // Segment "-"
    int absTranspose = -transpose;
    if (absTranspose >= 10) {
      segments[1] = display.encodeDigit(absTranspose / 10);
    } else {
      segments[1] = display.encodeDigit(absTranspose);
    }
  }
  
  // Calcul pour les digits du preset (positions 2 et 3)
  segments[2] = display.encodeDigit(0); // Toujours 0 car preset va de 0 à 9
  segments[3] = display.encodeDigit(selectedPreset);
  
  // Afficher les segments
  display.setSegments(segments);
}

// Fonction pour mettre à jour l'afficheur TM1637
void updateDisplay() {
  if (millis() - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    // Utiliser la nouvelle fonction d'affichage unifiée
    displayUnified();
    lastDisplayUpdate = millis();
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("=== Contrôleur Interactif ESP32 ===");
  Serial.println("Initialisation...");
  
  // Désactiver le Bluetooth pour améliorer la stabilité des lectures analogiques
  btStop();
  Serial.println("Bluetooth désactivé pour optimiser les lectures ADC");
  
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
  selectedPreset = 1; // S'assurer que selectedPreset est correct
  
  // Initialisation de l'interface utilisateur
  initializeUserInterface();
  
  // Affichage initial unifié
  displayUnified();
  
  // Configuration ESP-NOW
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erreur d'initialisation ESP-NOW");
    return;
  }
  
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
  
  // Configuration du peer récepteur ESP8266 (unicast au lieu de broadcast)
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Erreur d'ajout du peer");
    return;
  }
  
  Serial.println("ESP-NOW initialisé");
  Serial.println("Mode: Unicast vers récepteur ESP8266");
  Serial.print("Adresse MAC cible: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(receiverAddress[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
  Serial.println("Fréquence d'émission: " + String(EMISSION_FREQUENCY) + "Hz");
  Serial.println("Capteurs Sharp IR sur GPIO35 et GPIO36");
  Serial.println("Faders sur GPIO32, GPIO33, GPIO34");
  Serial.println("Encodeur KY-040 sur GPIO26, GPIO27, GPIO25");
  Serial.println("Afficheur TM1637 sur GPIO18, GPIO19");
  Serial.println("  Format d'affichage: TTPP (TT=transposition, PP=preset)");
  Serial.println("  Exemples: 0501=transpose +5/preset 1, -203=transpose -2/preset 3");
  Serial.println("Boutons push sur GPIO21, GPIO22, GPIO23");
  Serial.println("Bouton 1: Décrémenter transposition (-12 à +12)");
  Serial.println("Bouton 2: Incrémenter transposition (-12 à +12)");
  Serial.println("Bouton 3: Trig HH");
  Serial.println("Bouton encodeur: Toggle filtre ON/OFF");
  Serial.println("Transposition actuelle: " + String(transpose) + " (×" + String(transpose_factor) + ")");
  Serial.println("Transposition appliquée au pitch et gate_threshold");
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
    esp_err_t result = esp_now_send(receiverAddress, (uint8_t *)&outgoingDMXPacket, sizeof(outgoingDMXPacket));
    
    if (result == ESP_OK) {
      //Serial.print(" [OK]");
    } else {
      Serial.print(" [ERREUR]");
    }
  }
  
  // Décrémenter les canaux trig_kick, trig_snare et trig_hh après l'envoi
  if (dmxValues[116] > 0) dmxValues[116]--; // Canal DMX 117 (trig_kick)
  if (dmxValues[117] > 0) dmxValues[117]--; // Canal DMX 118 (trig_snare)
  if (dmxValues[118] > 0) dmxValues[118]--; // Canal DMX 119 (trig_hh)
  
  //Serial.println();
}

void setlights()
{
  // Canal DMX 1 : Mode de contrôle (0 = intensité rouge, 1 = autre mode, etc.)
  dmxValues[0] = 0; // Mode intensité rouge
  
  // Canaux DMX 2, 3, 4 : RGB1 dimmé par IR1
  dmxValues[1] = rgb1RedDimmed;    // Canal DMX 2 (R)
  dmxValues[2] = rgb1GreenDimmed;  // Canal DMX 3 (G) 
  dmxValues[3] = rgb1BlueDimmed;   // Canal DMX 4 (B)
  
  // Mise à jour du ruban WS2812B avec RGB2 dimmé par IR2
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(rgb2RedDimmed, rgb2GreenDimmed, rgb2BlueDimmed);
  }
  
  // Mettre à jour l'affichage
  FastLED.show();
}

void testInputs() {
  // Affichage compact des valeurs brutes
  
  // Lecture des capteurs Sharp IR (valeurs brutes 0-4095)
  int irRaw1 = analogRead(DIST_SENSOR_1_PIN);
  int irRaw2 = analogRead(DIST_SENSOR_2_PIN);
  
  // Lecture des faders (valeurs brutes 0-4095)
  int fader1Raw = analogRead(FADER_1_PIN);
  int fader2Raw = analogRead(FADER_2_PIN);
  int fader3Raw = analogRead(FADER_3_PIN);
  
  // Lecture des boutons (0 ou 1)
  bool button1State = !digitalRead(BUTTON_1_PIN); // Bouton 1 sur GPIO22
  bool button2State = !digitalRead(BUTTON_2_PIN); // Bouton 2 sur GPIO21
  bool button3State = !digitalRead(BUTTON_3_PIN);
  
  // Lecture de l'encodeur
  int32_t testEncoderValue = encoder.getCount();
  bool encoderButtonState = !digitalRead(ENCODER_BUTTON_PIN);
  
  // Mapper les faders sur 0-255 pour DMX 2, 3, 4 (inversés)
  uint8_t fader1Mapped = (4095 - fader1Raw) / 16; // 4095-0 → 0-255
  uint8_t fader2Mapped = (4095 - fader2Raw) / 16;
  uint8_t fader3Mapped = (4095 - fader3Raw) / 16;
  
  // Assigner aux canaux DMX 2, 3, 4
  dmxValues[1] = fader1Mapped; // DMX 2 (Rouge)
  dmxValues[2] = fader2Mapped; // DMX 3 (Vert)
  dmxValues[3] = fader3Mapped; // DMX 4 (Bleu)
  
  // Affichage compact sur une ligne
  Serial.print("IR1 ");
  Serial.print(irRaw1);
  Serial.print(" | IR2 ");
  Serial.print(irRaw2);
  Serial.print(" | F1 ");
  Serial.print(fader1Raw);
  Serial.print(" | F2 ");
  Serial.print(fader2Raw);
  Serial.print(" | F3 ");
  Serial.print(fader3Raw);
  Serial.print(" | B1 ");
  Serial.print(button1State ? "1" : "0");
  Serial.print(" | B2 ");
  Serial.print(button2State ? "1" : "0");
  Serial.print(" | B3 ");
  Serial.print(button3State ? "1" : "0");
  Serial.print(" | ENC ");
  Serial.print(testEncoderValue);
  Serial.print(" | ENCB ");
  Serial.print(encoderButtonState ? "1" : "0");
  Serial.print(" | DMX2 ");
  Serial.print(dmxValues[1]);
  Serial.print(" | DMX3 ");
  Serial.print(dmxValues[2]);
  Serial.print(" | DMX4 ");
  Serial.print(dmxValues[3]);
  Serial.println();
  
  // En mode test, afficher l'encodeur sur les 4 digits
  display.clear();
  uint16_t displayValue = abs(testEncoderValue) % 10000; // Limiter à 4 chiffres
  display.showNumberDec(displayValue);
  
  // Mettre à jour le LED strip avec les valeurs DMX 2, 3, 4
  uint8_t redValue = dmxValues[1];    // Canal DMX 2 (R)
  uint8_t greenValue = dmxValues[2];  // Canal DMX 3 (G) 
  uint8_t blueValue = dmxValues[3];   // Canal DMX 4 (B)
  
  // Appliquer la couleur RGB à tous les LEDs du ruban
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(redValue, greenValue, blueValue);
  }
  
  // Mettre à jour l'affichage
  FastLED.show();
}
void loop()
{
  // Incrémenter le compteur d'affichage
  displayCounter++;
  
  if (CONTROL_TEST == 1) {
    // affichage des valeurs brutes des capteurs, boutons, faders, encodeur
    testInputs();
    delay(500);
  }
  else // fonctionnement normal
  {
    // Gestion des contrôles physiques dynamiques (IR1, IR2, faders, bouton3)
    handlePhysicalControls();
    
    // Gestion de l'encodeur rotatif
    handleEncoder();
  }
  
  // Lecture des faders
 // readFaders();
  
  // Gestion des boutons push
  handleButtonInterrupts();
  
  // Gestion de l'encodeur KY-040
 // handleEncoder();
  
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