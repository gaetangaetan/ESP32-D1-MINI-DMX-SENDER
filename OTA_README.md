# Fonctionnalité OTA (Over-The-Air Update) v1757978703

## Description

Système de mise à jour sans fil intégré avec progression visuelle LED temps réel. Met à jour automatiquement le firmware ESP32 et le système de fichiers LittleFS contenant l'interface web.

## Améliorations v1757978703

### ✅ Progression Visuelle Complète
- **Barre de progression firmware** corrigée (couleur rose)
- **Segments LED** par incréments de 5 pixels
- **Feedback immédiat** pour chaque étape

### ✅ Optimisations
- **Réinitialisation** des compteurs de progression
- **Gestion d'erreurs** améliorée
- **Stabilité** des connexions TCP

## Fonctionnement

### Déclenchement
- **Au démarrage** : Si le bouton 1 (GPIO 22) est enfoncé, le mode OTA se lance automatiquement
- **Si le bouton n'est pas enfoncé** : Le système démarre normalement

### Processus de mise à jour
1. **Connexion WiFi** : Connexion au réseau `mrVOOlpy` avec le mot de passe `youhououhou`
2. **Téléchargement LittleFS** : Récupération du fichier `littlefs.bin` depuis `http://ksoloti_kontrol.gaetanstreel.com/littlefs.bin`
3. **Installation LittleFS** : Mise à jour du système de fichiers
4. **Téléchargement Firmware** : Récupération du fichier `firmware.bin` depuis `http://ksoloti_kontrol.gaetanstreel.com/firmware.bin`
5. **Installation Firmware** : Mise à jour du firmware
6. **Redémarrage** : Redémarrage automatique de l'ESP32

## Feedback LED

Le ruban WS2812B (144 LEDs) fournit un feedback visuel pendant toute la procédure :

### Séquence des couleurs
- **5 clignottements bleus** (200ms) : Annonce du début de la mise à jour
- **Barre de progression cyan** : Téléchargement et installation LittleFS (incréments de 5 pixels)
- **Barre de progression rose** : Téléchargement et installation Firmware (incréments de 5 pixels)
- **5 clignottements verts** (200ms) : Succès de la mise à jour
- **5 clignottements rouges** (200ms) : Erreur lors de la mise à jour

### Entre chaque étape
Le ruban s'éteint complètement pour une meilleure lisibilité.

## Architecture du code

### Fichiers créés
- `src/ota_update.h` : Déclaration de la classe OTAUpdate
- `src/ota_update.cpp` : Implémentation de la logique OTA
- `test_ota.cpp` : Fichier de test pour valider les patterns LED

### Intégration
- **Indépendant** : Le code OTA est complètement séparé du code principal
- **Non-intrusif** : N'affecte pas le fonctionnement normal de l'application
- **Modulaire** : Peut être facilement désactivé ou modifié

## Utilisation

### Mise à jour normale
1. Maintenir le bouton 1 enfoncé
2. Redémarrer l'ESP32 (ou appuyer sur le bouton reset)
3. Observer les patterns LED pour suivre la progression
4. Attendre le redémarrage automatique

### Test de la fonctionnalité
1. Compiler et uploader `test_ota.cpp`
2. Maintenir le bouton 1 au démarrage pour voir la simulation complète
3. Relâcher le bouton pour voir les patterns de test

## Configuration

### URLs de téléchargement
```cpp
#define OTA_LITTLEFS_URL "http://ksoloti_kontrol.gaetanstreel.com/littlefs.bin"
#define OTA_FIRMWARE_URL "http://ksoloti_kontrol.gaetanstreel.com/firmware.bin"
```

### Configuration WiFi
```cpp
#define OTA_WIFI_SSID "mrVOOlpy"
#define OTA_WIFI_PASSWORD "youhououhou"
```

### Configuration LED
```cpp
#define OTA_LED_PIN 4
#define OTA_NUM_LEDS 144
#define OTA_BRIGHTNESS 64
```

## Sécurité

- **Validation** : Vérification de la taille des fichiers avant téléchargement
- **Rollback** : En cas d'erreur, l'ESP32 redémarre avec l'ancien firmware
- **Timeout** : Limitation du temps de connexion WiFi (20 tentatives)

## Dépannage

### Problèmes courants
1. **Pas de connexion WiFi** : Vérifier le SSID et le mot de passe
2. **Erreur de téléchargement** : Vérifier que les fichiers sont accessibles sur le serveur
3. **Pattern rouge** : Consulter les logs série pour identifier l'erreur

### Logs de débogage
Tous les messages sont affichés sur le port série (115200 baud) :
- État du bouton au démarrage
- Progression de la connexion WiFi
- Taille des fichiers téléchargés
- Statut de chaque étape

## Notes techniques

- **Mémoire** : L'OTA utilise la partition de mise à jour de l'ESP32
- **Stabilité** : Le processus est conçu pour être robuste et récupérer des erreurs
- **Performance** : Téléchargement par blocs de 1024 octets pour optimiser la mémoire
- **Feedback** : Barre de progression mise à jour toutes les 5 LEDs pour une progression fluide
