# Configuration WiFi et mDNS - Ksoloti Kontrol ESP32

## Vue d'ensemble

Le contrôleur Ksoloti ESP32 peut maintenant se connecter à votre réseau WiFi local et être accessible via `http://ksolotikontrol.local` au lieu de créer son propre point d'accès WiFi.

## Fonctionnalités implémentées

### 1. WiFiManager
- **Connexion automatique** : L'ESP32 se connecte automatiquement au dernier réseau WiFi connu
- **Portail de configuration** : Si aucun réseau connu n'est disponible, un portail captif "KsolotiKontrol-Config" est créé
- **Timeout** : 30 secondes pour la configuration, puis redémarrage automatique
- **Sauvegarde** : Les paramètres WiFi sont sauvegardés en mémoire flash

### 2. mDNS (Multicast DNS)
- **Nom d'hôte** : `ksolotikontrol.local`
- **Résolution automatique** : Accessible depuis n'importe quel appareil sur le réseau local
- **Service HTTP** : Annoncé automatiquement sur le port 80

### 3. Compatibilité ESP-NOW
- **Mode hybride** : ESP-NOW et WiFi coexistent parfaitement
- **Performance** : Aucun impact sur les communications ESP-NOW avec le Ksoloti
- **Stabilité** : Mode `WIFI_STA` utilisé pour les deux protocoles

## Utilisation

### Première configuration
1. **Démarrage** : Allumez l'ESP32
2. **Portail captif** : Si aucun réseau connu, connectez-vous à "KsolotiKontrol-Config"
3. **Configuration** : Ouvrez `http://192.168.4.1` dans votre navigateur
4. **Sélection réseau** : Choisissez votre réseau WiFi et entrez le mot de passe
5. **Sauvegarde** : Les paramètres sont sauvegardés automatiquement

### Utilisation normale
1. **Connexion automatique** : L'ESP32 se connecte au réseau sauvegardé
2. **Accès web** : Ouvrez `http://ksolotikontrol.local` dans votre navigateur
3. **Interface** : Utilisez l'interface web normalement

## Avantages

### Pour l'utilisateur
- **Accès réseau** : Interface accessible depuis n'importe où sur le réseau local
- **Pas de changement de WiFi** : Plus besoin de se connecter au point d'accès de l'ESP32
- **Nom mémorable** : `ksolotikontrol.local` au lieu d'une adresse IP
- **Configuration unique** : Une seule configuration nécessaire

### Pour le développement
- **Débogage facilité** : Accès SSH/telnet possible via le réseau local
- **Mises à jour OTA** : Possibilité d'utiliser le réseau local pour les mises à jour
- **Intégration** : Peut être intégré dans des systèmes domotiques

## Configuration technique

### Constantes modifiées
```cpp
#define MDNS_HOSTNAME "ksolotikontrol"
#define WIFI_TIMEOUT_MS 30000  // 30 secondes
```

### Bibliothèques ajoutées
- `WiFiManager@^2.0.17` (déjà présente)
- `ESPmDNS@^2.0.0` (ajoutée automatiquement)

### Fonctionnement
1. **Initialisation** : `WiFiManager` tente de se connecter au dernier réseau connu
2. **Portail captif** : Si échec, création d'un portail de configuration
3. **mDNS** : Une fois connecté, annonce du service `ksolotikontrol.local`
4. **Serveur web** : Démarrage normal du serveur web sur le port 80

## Dépannage

### L'ESP32 ne se connecte pas
- Vérifiez que le réseau WiFi est dans la portée
- Le mot de passe WiFi est-il correct ?
- Le réseau utilise-t-il un chiffrement supporté (WPA/WPA2) ?

### mDNS ne fonctionne pas
- Votre routeur supporte-t-il mDNS/Bonjour ?
- Sur Windows : Installez "Bonjour Print Services"
- Sur Linux : Installez `avahi-daemon`

### ESP-NOW ne fonctionne plus
- Vérifiez que l'adresse MAC du récepteur est correcte
- Le récepteur Ksoloti est-il allumé et à portée ?
- Consultez les logs série pour les erreurs ESP-NOW

## Retour en arrière

Si vous souhaitez revenir au mode point d'accès :
1. Remplacez le contenu de `setupWebInterface()` par l'ancienne version
2. Remettez les constantes `AP_SSID` et `AP_PASSWORD`
3. Supprimez les includes `WiFiManager` et `ESPmDNS`

## Logs de débogage

Les messages suivants apparaissent dans le moniteur série :
- `"Configuration WiFiManager..."`
- `"Tentative de connexion au WiFi..."`
- `"Connexion WiFi établie !"`
- `"mDNS démarré avec le nom: ksolotikontrol.local"`
- `"Interface web accessible via: http://ksolotikontrol.local"`

## Version
Implémenté dans la version 1757857752
