#define VERSION 160
/*
// Émetteur DMX sans fil avec capteur ultrasonique
// Utilise ESP-NOW pour transmettre les données DMX
// Fréquence d'émission : 50Hz
// Canal DMX 102 : valeur du capteur ultrasonique (0-255)
*/

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <Ultrasonic.h>
#include <ESP32Encoder.h>

// Définitions pour le capteur HC-SR04
#define TRIG_PIN D4    // D4 (GPIO16) - Pin de déclenchement (Trigger)
#define ECHO_PIN D3    // D3 (GPIO17) - Pin d'écho (Echo)

// Définitions pour le rotary encoder
#define ENCODER_A_PIN D7    // D7 (GPIO23) - Pin A du rotary encoder
#define ENCODER_B_PIN D6    // D6 (GPIO19) - Pin B du rotary encoder
#define ENCODER_BUTTON_PIN D5  // D5 (GPIO18) - Pin bouton du rotary encoder (optionnel)

// Configuration ESP-NOW
#define EMISSION_FREQUENCY 50  // Hz (20ms entre chaque émission)
#define DMX_CHANNEL_ULTRASONIC 102  // Canal DMX pour la valeur ultrasonique
#define MAX_DISTANCE_CM 100  // Distance maximale en cm (100cm = 0, 2cm = 255)

// Configuration des presets
#define PRESET_SIZE 20  // Nombre de paramètres par preset
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

// Création de l'objet Ultrasonic
Ultrasonic ultrasonic(TRIG_PIN, ECHO_PIN);

// Création de l'objet Rotary Encoder
ESP32Encoder encoder;

// Définition des 20 paramètres du theremin
Parameter parameters[PRESET_SIZE] = {
  {"autopan", 101, 0, 0},
  {"pitch", 102, 0, 0},
  {"vibrato_speed", 103, 0, 0},
  {"vibrato_depth", 104, 0, 0},
  {"delay_time", 105, 0, 0},
  {"delay_fbck", 106, 0, 0},
  {"osc", 107, 0, 0},
  {"gate", 108, 0, 0},
  {"glide", 109, 0, 0},
  {"scale", 110, 0, 0},
  {"offset_note", 111, 0, 0},
  {"osc2_vol", 112, 0, 0},
  {"osc2_pitch", 113, 0, 0},
  {"autopan_freq", 114, 0, 0},
  {"scale_tonic", 115, 0, 0},
  {"volume_drums", 116, 0, 0},
  {"kick_trig", 117, 0, 0},
  {"snare_trig", 118, 0, 0},
  {"hh_trig", 119, 0, 0},
  {"reserved", 120, 0, 0}  // Canal réservé pour extensions futures
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

// Variables pour le rotary encoder
int32_t lastEncoderValue = 0;
uint8_t selectedParameter = 0;  // Index du paramètre sélectionné (0-19)
bool encoderButtonPressed = false;
unsigned long lastButtonPress = 0;
const unsigned long BUTTON_DEBOUNCE = 200; // 200ms de debounce
unsigned long lastEncoderStatusTime = 0;
const unsigned long ENCODER_STATUS_INTERVAL = 5000; // Affichage du statut toutes les 5 secondes

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

// Fonction pour mapper la distance (2-MAX_DISTANCE_CM) vers une valeur DMX (255-0)
uint8_t mapDistanceToDMX(long distance) {
  if (distance < 2) return 255;  // Distance minimale = valeur maximale
  if (distance > MAX_DISTANCE_CM) return 0;  // Distance maximale = valeur minimale
  
  // Mapper 2-MAX_DISTANCE_CM vers 255-0 (inversé)
  return map(distance, 2, MAX_DISTANCE_CM, 255, 0);
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



// Fonction pour enregistrer tous les paramètres d'un coup (20 arguments)
void setAllParameters(uint8_t autopan, uint8_t pitch, uint8_t vibrato_speed, uint8_t vibrato_depth,
                     uint8_t delay_time, uint8_t delay_fbck, uint8_t osc, uint8_t gate,
                     uint8_t glide, uint8_t scale, uint8_t offset_note, uint8_t osc2_vol,
                     uint8_t osc2_pitch, uint8_t autopan_freq, uint8_t scale_tonic,
                     uint8_t volume_drums, uint8_t kick_trig, uint8_t snare_trig,
                     uint8_t hh_trig, uint8_t reserved) {
  
  // Mettre à jour tous les paramètres
  parameters[0].value = autopan;
  parameters[1].value = pitch;
  parameters[2].value = vibrato_speed;
  parameters[3].value = vibrato_depth;
  parameters[4].value = delay_time;
  parameters[5].value = delay_fbck;
  parameters[6].value = osc;
  parameters[7].value = gate;
  parameters[8].value = glide;
  parameters[9].value = scale;
  parameters[10].value = offset_note;
  parameters[11].value = osc2_vol;
  parameters[12].value = osc2_pitch;
  parameters[13].value = autopan_freq;
  parameters[14].value = scale_tonic;
  parameters[15].value = volume_drums;
  parameters[16].value = kick_trig;
  parameters[17].value = snare_trig;
  parameters[18].value = hh_trig;
  parameters[19].value = reserved;
  
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

// Fonction d'initialisation du rotary encoder
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

// Fonction pour gérer le rotary encoder
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

// Fonction d'initialisation des presets
void initializePresets() {
  Serial.println("Initialisation des presets...");
  
  // Preset 0 - Preset par défaut
  strcpy(presets[0].name, "Default");
  for (int i = 0; i < PRESET_SIZE; i++) {
    presets[0].values[i] = 0;
  }
  
  // Preset 1 - Configuration de base
  strcpy(presets[1].name, "Preset1");
  presets[1].values[0] = 0;   // autopan (101)
  presets[1].values[1] = 0;   // pitch (102)
  presets[1].values[2] = 61;  // vibrato_speed (103)
  presets[1].values[3] = 8;   // vibrato_depth (104)
  presets[1].values[4] = 107; // delay_time (105)
  presets[1].values[5] = 114; // delay_fbck (106)
  presets[1].values[6] = 0;   // osc (107)
  presets[1].values[7] = 0;   // gate (108)
  presets[1].values[8] = 117; // glide (109)
  presets[1].values[9] = 0;   // scale (110)
  presets[1].values[10] = 0;  // offset_note (111)
  presets[1].values[11] = 0;  // osc2_vol (112)
  presets[1].values[12] = 0;  // osc2_pitch (113)
  presets[1].values[13] = 0;  // autopan_freq (114)
  presets[1].values[14] = 0;  // scale_tonic (115)
  presets[1].values[15] = 0;  // volume_drums (116)
  presets[1].values[16] = 0;  // kick_trig (117)
  presets[1].values[17] = 0;  // snare_trig (118)
  presets[1].values[18] = 0;  // hh_trig (119)
  presets[1].values[19] = 0;  // reserved (120)
  
  // Preset 2 - À définir
  strcpy(presets[2].name, "Preset2");
  for (int i = 0; i < PRESET_SIZE; i++) {
    presets[2].values[i] = 0;
  }
  
  // Preset 3 - À définir
  strcpy(presets[3].name, "Preset3");
  for (int i = 0; i < PRESET_SIZE; i++) {
    presets[3].values[i] = 0;
  }
  
  // Preset 4 - À définir
  strcpy(presets[4].name, "Preset4");
  for (int i = 0; i < PRESET_SIZE; i++) {
    presets[4].values[i] = 0;
  }
  
  // Preset 5 - À définir
  strcpy(presets[5].name, "Preset5");
  for (int i = 0; i < PRESET_SIZE; i++) {
    presets[5].values[i] = 0;
  }
  
  // Preset 6 - À définir
  strcpy(presets[6].name, "Preset6");
  for (int i = 0; i < PRESET_SIZE; i++) {
    presets[6].values[i] = 0;
  }
  
  // Preset 7 - À définir
  strcpy(presets[7].name, "Preset7");
  for (int i = 0; i < PRESET_SIZE; i++) {
    presets[7].values[i] = 0;
  }
  
  // Preset 8 - À définir
  strcpy(presets[8].name, "Preset8");
  for (int i = 0; i < PRESET_SIZE; i++) {
    presets[8].values[i] = 0;
  }
  
  // Preset 9 - À définir
  strcpy(presets[9].name, "Preset9");
  for (int i = 0; i < PRESET_SIZE; i++) {
    presets[9].values[i] = 0;
  }
  
  //Serial.println("Presets initialisés");
}

void setup()
{
  Serial.begin(115200);
  Serial.println("=== Émetteur DMX avec Capteur Ultrasonique ===");
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
  
  // Initialisation du rotary encoder
  initializeEncoder();
  
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
  Serial.println("Canal DMX ultrasonique: " + String(DMX_CHANNEL_ULTRASONIC));
  Serial.println("================================");
}

void sendDMXvalues()
{
  // Mise à jour du paramètre "pitch" avec la valeur ultrasonique (déjà lue dans setlights())
  long distance = ultrasonic.read();
  uint8_t mappedDistance = mapDistanceToDMX(distance);
  setParameter("pitch", mappedDistance);
  
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
  // Utilisation de la valeur ultrasonique déjà calculée dans dmxValues
  uint8_t mappedDistance = dmxValues[DMX_CHANNEL_ULTRASONIC - 1]; // -1 car les canaux DMX commencent à 1
  
  // Canal DMX 1 : Mode de contrôle (0 = intensité rouge, 1 = autre mode, etc.)
  dmxValues[0] = 0; // Mode intensité rouge
  
  // Canal DMX 2 : Intensité rouge modulée par les valeurs Ksoloti
  // Utilisation de la division flottante pour un meilleur contrôle
  uint8_t ksoloti_modulation = (uint8_t)((float)ksoloti_val1);
  
  // Modulation finale avec la valeur ultrasonique
  dmxValues[1] = (3* ksoloti_modulation * mappedDistance) / 255;
  
  // Debug (optionnel)
  //Serial.print("Ultrasonic DMX: "); Serial.print(mappedDistance);
  //Serial.print(" | Ksoloti mod: "); Serial.print(ksoloti_modulation);
  //Serial.print(" | DMX2: "); Serial.println(dmxValues[1]);
}

void loop()
{
  // Gestion du rotary encoder
  handleEncoder();
  
  // Émission à fréquence fixe (50Hz)
  if (millis() - lastEmissionTime >= EMISSION_INTERVAL) {
    setlights();
    sendDMXvalues();
    lastEmissionTime = millis();
  }
  
  delay(1); // Petit délai pour éviter de surcharger le CPU
}

