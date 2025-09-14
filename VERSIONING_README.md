# Système de Versioning - Ksoloti Kontrol ESP32

## 📋 Vue d'ensemble

Le projet utilise un système de versioning basé sur les timestamps Unix pour une gestion simple et automatique des versions.

## 🔧 Structure du système

### Fichiers concernés :
- **`src/main.cpp`** : `#define VERSION 1757857752`
- **`data/index.html`** : Titre et en-tête avec version
- **`data/script.js`** : `const VERSION = 1757857752;`

### Affichage de la version :
- **Console série** : Affichage au démarrage de l'ESP32
- **Interface web** : Titre de la page et en-tête
- **Console JavaScript** : Log au chargement de la page

## 🚀 Scripts disponibles

### 1. `update_version.py`
Script Python pour mettre à jour automatiquement la version dans tous les fichiers.

**Usage :**
```bash
# Génération automatique du timestamp actuel
python update_version.py

# Utilisation d'un timestamp spécifique
python update_version.py 1757857752
```

### 2. `build_with_version.bat`
Script Windows Batch pour build complet avec mise à jour de version.

**Usage :**
```cmd
# Build avec génération automatique de version
build_with_version.bat

# Build avec timestamp spécifique
build_with_version.bat 1757857752
```

### 3. `build_with_version.ps1`
Script PowerShell pour build complet avec mise à jour de version.

**Usage :**
```powershell
# Build avec génération automatique de version
.\build_with_version.ps1

# Build avec timestamp spécifique
.\build_with_version.ps1 1757857752
```

## 📅 Format des versions

- **Format** : Timestamp Unix (10 chiffres)
- **Exemple** : `1757857752`
- **Correspondance** : 14 septembre 2025, 15:49:12 UTC

## 🔄 Workflow recommandé

1. **Développement** : Modifier le code normalement
2. **Build** : Utiliser `build_with_version.bat` ou `build_with_version.ps1`
3. **Test** : Vérifier que la version s'affiche correctement
4. **Déploiement** : Uploader `firmware.bin` et `littlefs.bin`

## 📝 Notes importantes

- La version est automatiquement synchronisée entre tous les fichiers
- Le script détecte automatiquement la version actuelle
- Les fichiers binaires (`firmware.bin`, `littlefs.bin`) gardent des noms fixes
- Aucun script PHP n'est nécessaire sur le serveur

## 🛠️ Dépannage

### Erreur "Python n'est pas installé"
- Installer Python depuis [python.org](https://python.org)
- Vérifier que Python est dans le PATH

### Erreur de compilation
- Vérifier que tous les fichiers sont sauvegardés
- Relancer le build après correction des erreurs

### Version non mise à jour
- Vérifier que les fichiers sont modifiables
- Relancer le script de mise à jour de version
