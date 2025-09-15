# 🎵 Ksoloti Kontrol v1757978703

**Système de contrôle avancé pour synthétiseur Ksoloti**

## 🌟 Fonctionnalités Principales

### 🔄 4 Modes de Fonctionnement
- **Standalone** : Performance maximale, contrôles physiques uniquement
- **WiFi Local** : Interface web via réseau local (`ksolotikontrol.local`)
- **WiFi Access Point** : Point d'accès intégré (`192.168.4.1`)
- **OTA** : Mise à jour sans fil avec progression LED

### 🌐 Interface Web Avancée
- **8 presets** sauvegardables avec interface intuitive
- **21 paramètres audio** contrôlables en temps réel
- **Filtres audio** assignables (HP/BP/LP)
- **Export/Import** presets vers fichiers JSON
- **Console debug** ESP32 temps réel
- **Changement WiFi** depuis l'interface

### 🎛️ Contrôles Physiques
- **Capteurs Sharp IR** (2x) avec mapping logarithmique/linéaire
- **Faders analogiques** (2x) pour contrôle expressif
- **Encodeur rotatif** + boutons pour navigation presets
- **Écran TM1637** affichage octave/preset
- **Ruban LED WS2812B** (144 LEDs) feedback visuel

### 📡 Communication
- **ESP-NOW** 50Hz vers récepteur Ksoloti
- **DMX 512** canaux pour éclairage
- **WiFi 2.4GHz** pour interface web
- **mDNS** résolution automatique `.local`

## 🚀 Installation

### Prérequis
- PlatformIO IDE ou CLI
- ESP32 D1 Mini ou compatible
- Composants hardware (voir schémas)

### Compilation
```bash
# Firmware principal
pio run --target upload

# Interface web (LittleFS)
pio run --target uploadfs
```

### Mise à jour OTA
1. Maintenir **Bouton 1** au démarrage
2. Progression LED automatique
3. Redémarrage après mise à jour

## 📖 Documentation

- **[Manuel Utilisateur](documentation/manuel.html)** - Guide complet
- **[Interface Web](WEB_INTERFACE_README.md)** - Fonctionnalités web
- **[Mises à jour OTA](OTA_README.md)** - Système OTA
- **[Réseaux WiFi](WIFI_NETWORK_README.md)** - Configuration WiFi
- **[Versioning](VERSIONING_README.md)** - Gestion versions

## ⚙️ Spécifications Techniques

### Hardware
- **MCU** : ESP32-D0WD-V3 (240MHz Dual Core)
- **RAM** : 320KB (17.7% utilisée)
- **Flash** : 4MB (61.3% utilisée)
- **ADC** : 12 bits (0-4095)

### Performance
- **Émission ESP-NOW** : 50Hz constant
- **Interface web** : 10Hz (optimisé CPU)
- **Réactivité capteurs** : Temps réel (sans limitation)

### Canaux DMX
- **101-135** : Paramètres audio (35 canaux)
- **1-4** : Contrôle RGB + mode

## 🔧 Architecture

```
src/
├── main.cpp              # Application principale
├── ota_update.h/cpp       # Système OTA modulaire
└── ir_stabilization_config.h

data/
├── index.html            # Interface web
├── script.js             # Logique JavaScript
└── style.css             # Styles CSS

documentation/
└── manuel.html           # Manuel utilisateur standalone
```

## 🎯 Modes d'Utilisation

### 🎧 Mode Standalone
```
Démarrage → Mode par défaut
Performance maximale
Contrôles physiques uniquement
```

### 🌐 Mode WiFi Local
```
Démarrage → Bouton 2 bref
Accès : http://ksolotikontrol.local
Performance excellente
```

### 📱 Mode Access Point
```
Démarrage → Bouton 2 long
Réseau : KsolotiKontrol-AP
Accès : http://192.168.4.1
```

## 🏆 Projet Finalisé

✅ **Toutes fonctionnalités implémentées**  
✅ **Performance optimisée**  
✅ **Documentation complète**  
✅ **Système de mise à jour OTA**  
✅ **Interface web avancée**  
✅ **Modes multiples**  

---

**Version** : 1757978703 (Timestamp Unix)  
**Statut** : Production Ready 🚀  
**Compatibilité** : ESP32, Ksoloti, navigateurs modernes
