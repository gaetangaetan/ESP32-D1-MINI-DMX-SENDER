# Interface Web Ksoloti Kontrol v1757978703

## Vue d'ensemble

L'interface web permet de contrôler les paramètres audio du Ksoloti en temps réel via un navigateur web. Disponible en 3 modes WiFi avec fonctionnalités avancées : debug temps réel, export/import presets, et changement de réseau WiFi.

## Modes d'Accès

### Mode WiFi Local (Recommandé)
- **Accès :** `http://ksolotikontrol.local`
- **Avantages :** Performance optimale, accès depuis votre réseau
- **Configuration :** WiFiManager automatique

### Mode Access Point
- **Réseau :** KsolotiKontrol-AP (mot de passe : ksoloti123)
- **Accès :** `http://192.168.4.1`
- **Note :** Premier chargement peut nécessiter déconnexion/reconnexion

### Mode Standalone
- **Interface web :** Désactivée
- **Performance :** Maximale pour contrôles physiques uniquement

## Nouvelles Fonctionnalités v1757978703

### 🔄 Boutons d'Action (En-tête)
- **💾 Save to File :** Export de tous les presets vers JSON
- **📁 Load from File :** Import de presets depuis JSON
- **🔄 Changer WiFi :** Reconfiguration WiFi (mode local uniquement)
- **🐛 Debug :** Console debug ESP32 temps réel

### 🐛 Console Debug
- **Messages ESP32 :** Affichage avec timestamps
- **Rafraîchissement :** Automatique (2s) + manuel
- **Contrôles :** Actualiser, Effacer
- **Usage :** Débogage en temps réel sans câble série

## Fonctionnalités

### 🎚️ Contrôles Audio
- **21 faders** pour contrôler les paramètres audio principaux :
  1. Autopan Depth
  2. Pitch
  3. Vibrato Speed
  4. Vibrato Depth
  5. Delay Time
  6. Delay Feedback
  7. OSC Waveform
  8. Gate Threshold
  9. Portamento Time
  10. Scale
  11. Octave Low/High
  12. OSC 2 Volume
  13. OSC 2 Pitch Offset
  14. Autopan Frequency
  15. Scale Tonic
  16. Volume Drums
  17. Trig Kick
  18. Trig Snare
  19. Trig HH
  20. Master Volume
  21. Filter On/Off

### 🔗 Assignations des Capteurs
- **4 menus déroulants** pour assigner les contrôles physiques :
  - **Capteur IR 1** → Paramètre audio ou OFF
  - **Capteur IR 2** → Paramètre audio ou OFF
  - **Fader 2** → Paramètre audio ou OFF
  - **Fader 3** → Paramètre audio ou OFF

### 💾 Gestion des Presets Web
- **Sauvegarde** : Créer et sauvegarder des presets avec des noms personnalisés
- **Chargement** : Charger des presets web sauvegardés
- **Stockage** : Les presets sont sauvegardés dans le système de fichiers LittleFS

## Utilisation

### 1. Accès à l'interface
1. Connectez-vous au point d'accès WiFi `DMX_Controller`
2. Mot de passe : `dmx12345`
3. Ouvrez votre navigateur et allez à l'adresse IP affichée (généralement `192.168.4.1`)
4. Sélectionnez le **preset 0** sur l'encodeur rotatif

### 2. Contrôle des paramètres
- **Faders** : Glissez les faders pour ajuster les valeurs (0-255)
- **Temps réel** : Les changements sont appliqués immédiatement
- **Affichage** : Les valeurs actuelles sont affichées sous chaque fader

### 3. Assignation des capteurs
- **OFF** : Le capteur/fader est désactivé
- **Paramètre** : Le capteur/fader contrôle le paramètre sélectionné
- **Sauvegarde automatique** : Les assignations sont sauvegardées automatiquement

### 4. Gestion des presets
- **Sauvegarder** : Entrez un nom et cliquez sur "Sauvegarder"
- **Charger** : Sélectionnez un preset dans la liste et cliquez sur "Charger"
- **Limite** : Maximum 5 presets web

## Architecture Technique

### Mode Web vs Mode Normal
- **Preset 0** : Mode interface web actif
  - Les faders web contrôlent directement les paramètres
  - Les assignations physiques sont respectées
  - L'état est sauvegardé dans le preset 0

- **Presets 1-9** : Mode normal
  - Fonctionnement classique avec les presets sauvegardés
  - L'état web est sauvegardé avant le changement

### Stockage
- **Presets web** : `/web_presets.json` dans LittleFS
- **Assignations** : `/web_assignments.json` dans LittleFS
- **État actuel** : Sauvegardé dans le preset 0

### API REST
- `GET /api/parameters` : Récupérer les valeurs actuelles
- `POST /api/parameters` : Mettre à jour un paramètre
- `GET /api/assignments` : Récupérer les assignations
- `POST /api/assignments` : Mettre à jour les assignations
- `POST /api/save-preset` : Sauvegarder un preset web
- `POST /api/load-preset` : Charger un preset web
- `GET /api/presets` : Lister les presets disponibles

## Avantages

1. **Développement rapide** : Test des paramètres sans recompilation
2. **Interface intuitive** : Contrôle visuel des 21 paramètres
3. **Assignations flexibles** : Configuration des capteurs physiques
4. **Sauvegarde facile** : Création de presets personnalisés
5. **Intégration transparente** : Compatible avec le système existant

## Limitations

- **Preset 0 réservé** : Ne peut plus être utilisé comme preset normal
- **Maximum 5 presets web** : Limité par la mémoire disponible
- **Point d'accès dédié** : Nécessite une connexion WiFi séparée

## Dépannage

### Interface non accessible
- Vérifiez que le preset 0 est sélectionné
- Vérifiez la connexion WiFi au point d'accès
- Vérifiez l'adresse IP affichée sur le moniteur série

### Erreurs de sauvegarde
- Vérifiez l'espace disponible dans LittleFS
- Vérifiez que le nom du preset n'est pas vide
- Vérifiez que le nombre maximum de presets n'est pas atteint

### Contrôles physiques non réactifs
- Vérifiez les assignations dans l'interface web
- Vérifiez que les capteurs/faders sont assignés à des paramètres
- Vérifiez que les assignations ne sont pas sur "OFF"
