# Trigger Externe Onirigun - Documentation

## Vue d'ensemble

Le système ESP32 DMX SENDER a été étendu pour recevoir des triggers externes depuis l'Onirigun via ESP-NOW. Quand le bouton blanc de l'Onirigun est pressé, il déclenche la même action que le bouton 3 du contrôleur.

## Configuration

### Adresse MAC de l'Onirigun
- **Adresse MAC** : `68:C6:3A:FD:37:17`
- **Protocole** : ESP-NOW
- **Canal** : 0 (WiFi channel)

### Structure du message
```cpp
typedef struct {
  uint8_t type;            // 0xA0 = animation trigger
  uint8_t animationNumber; // numéro d'animation à déclencher
} AnimationTriggerMessage;
```

## Fonctionnement

### 1. Réception du message
- L'Onirigun envoie un message avec `type = 0xA0`
- Le contrôleur ESP32 reçoit ce message via ESP-NOW
- Le message est validé et traité

### 2. Déclenchement de l'action
- Quand un trigger est reçu, la variable `externalTriggerActive` est mise à `true`
- Le système déclenche immédiatement la même action que le bouton 3 :
  - **Canal DMX** : 119 (trig_hh)
  - **Valeur** : `TRIG_LENGTH` (5 par défaut)
  - **Durée** : 100ms (configurable via `EXTERNAL_TRIGGER_DURATION`)

### 3. Gestion du timing
- Le trigger externe reste actif pendant 100ms
- Après ce délai, il est automatiquement désactivé
- Cette gestion évite les déclenchements multiples

## Code implémenté

### Variables ajoutées
```cpp
// Variables pour le trigger externe de l'Onirigun
bool externalTriggerActive = false;
unsigned long externalTriggerTime = 0;
const unsigned long EXTERNAL_TRIGGER_DURATION = 100; // 100ms
```

### Structure de message
```cpp
// Structure pour les messages de l'Onirigun
typedef struct {
  uint8_t type;            // 0xA0 = animation trigger
  uint8_t animationNumber; // numéro d'animation à déclencher
} AnimationTriggerMessage;
```

### Fonction de gestion
```cpp
void handleExternalTrigger() {
  // Vérifier si le trigger externe est actif et s'il faut le désactiver
  if (externalTriggerActive && (millis() - externalTriggerTime > EXTERNAL_TRIGGER_DURATION)) {
    externalTriggerActive = false;
    Serial.println("Trigger externe désactivé");
  }
}
```

## Utilisation

### Côté Onirigun
1. Appuyer sur le bouton blanc
2. Le système sélectionne l'animation actuelle
3. Un message `0xA0 + numéro_animation` est envoyé en broadcast

### Côté Contrôleur ESP32
1. Réception automatique du message via ESP-NOW
2. Déclenchement immédiat du trig_hh (canal DMX 119)
3. Affichage des informations de debug dans le Serial Monitor

## Debug et monitoring

### Messages de debug
- `"Trigger externe reçu de l'Onirigun: animation X"`
- `"Trigger externe - Trig HH déclenché - Durée: 5"`
- `"Trigger externe désactivé"`

### Informations de configuration
- Affichage de l'adresse MAC de l'Onirigun au démarrage
- Confirmation de l'ajout du peer Onirigun
- Statut de la configuration ESP-NOW

## Évolutions futures

### Utilisation de la variable d'animation
- Actuellement, seule la réception de `animationNumber` est implémentée
- Cette variable pourra être utilisée pour :
  - Sélectionner différents presets
  - Déclencher différentes actions selon l'animation
  - Contrôler des paramètres spécifiques

### Extensions possibles
- Support de plusieurs types de messages
- Gestion de la latence et de la fiabilité
- Intégration avec d'autres systèmes de contrôle

## Tests

### Vérification de la compilation
```bash
cd "ESP32 D1 MINI DMX SENDER"
pio run
```

### Test en fonctionnement
1. Flasher le code sur l'ESP32
2. Ouvrir le Serial Monitor (115200 bauds)
3. Vérifier la configuration ESP-NOW au démarrage
4. Tester le trigger depuis l'Onirigun
5. Observer les messages de debug et l'action DMX

## Dépannage

### Problèmes courants
- **Peer non ajouté** : Vérifier l'adresse MAC de l'Onirigun
- **Messages non reçus** : Vérifier la configuration WiFi et ESP-NOW
- **Trigger non déclenché** : Vérifier le format du message reçu

### Logs utiles
- Messages de configuration au démarrage
- Réception des messages d'animation
- Déclenchement des actions
- Gestion du timing des triggers
