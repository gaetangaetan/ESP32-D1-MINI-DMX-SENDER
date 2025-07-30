# Stabilisation des Capteurs IR Sharp - Solutions

## Problème Résolu

Le problème avec `ADC_ATTEN_DB_11` était dû à un conflit de types dans l'ESP32. La solution a été de :
1. Ajouter `#include <driver/adc.h>` pour les constantes ADC
2. Remplacer `analogSetPinAttenuation()` par `analogReadResolution(12)` pour une meilleure résolution

## Solutions de Stabilisation Implémentées

### 1. **Moyenne Mobile**
- Prend plusieurs échantillons et calcule la moyenne
- Réduit les pics aléatoires
- Paramètre : `IR_SAMPLE_SIZE` (défaut : 10)

### 2. **Filtre Passe-Bas**
- Applique un filtre numérique pour lisser les variations
- Coefficient ajustable : `IR_FILTER_ALPHA` (0.0-1.0)
- 0.3 = bon compromis stabilité/réactivité

### 3. **Debouncing**
- Ignore les variations inférieures à un seuil
- Paramètre : `IR_DEBOUNCE_THRESHOLD` (défaut : 5)
- Élimine le bruit de fond

### 4. **Combinaison des Méthodes**
- Utilise les 3 techniques simultanément
- Résultat optimal pour la plupart des applications

## Causes des Pics de Valeurs

### Interférences Électromagnétiques
- **Cause** : Bruit électrique des composants voisins
- **Solution** : Filtrage numérique + blindage si nécessaire

### Variations de Tension d'Alimentation
- **Cause** : Instabilité de l'alimentation 3.3V
- **Solution** : Condensateur de découplage + filtrage

### Réflexions Parasites
- **Cause** : Réflexions IR sur les surfaces environnantes
- **Solution** : Orientation du capteur + filtrage

### Bruit Analogique
- **Cause** : Résistance interne du circuit ADC
- **Solution** : Moyennage + filtre passe-bas

## Configuration

### Ajustement des Paramètres

1. **Pour plus de stabilité** :
   ```cpp
   #define IR_SAMPLE_SIZE 20        // Plus d'échantillons
   #define IR_FILTER_ALPHA 0.1      // Filtrage plus fort
   #define IR_DEBOUNCE_THRESHOLD 10 // Seuil plus élevé
   ```

2. **Pour plus de réactivité** :
   ```cpp
   #define IR_SAMPLE_SIZE 5         // Moins d'échantillons
   #define IR_FILTER_ALPHA 0.5      // Filtrage plus faible
   #define IR_DEBOUNCE_THRESHOLD 2  // Seuil plus bas
   ```

### Calibration

Ajustez ces valeurs selon votre capteur :
```cpp
#define IR_MIN_DISTANCE_CM 4    // Distance minimale
#define IR_MAX_DISTANCE_CM 80   // Distance maximale
#define IR_MIN_ADC_VALUE 0      // Valeur ADC min
#define IR_MAX_ADC_VALUE 4095   // Valeur ADC max
```

## Debug et Monitoring

Le code inclut maintenant un système de debug qui affiche :
- Valeurs brutes du capteur
- Valeurs stabilisées
- Valeurs mappées (0-255)

Pour activer le debug, décommentez dans `ir_stabilization_config.h` :
```cpp
#define IR_DEBUG_ENABLED
```

## Tests Recommandés

1. **Test de stabilité** : Placez le capteur à distance fixe et observez les variations
2. **Test de réactivité** : Déplacez rapidement la main devant le capteur
3. **Test de plage** : Testez de la distance minimale à maximale
4. **Test de bruit** : Observez les valeurs en l'absence d'objet

## Optimisations Possibles

### Hardware
- Condensateur de découplage sur l'alimentation du capteur
- Blindage électromagnétique
- Filtre RC sur la ligne analogique

### Software
- Filtre médian au lieu de moyenne
- Filtre de Kalman pour une prédiction plus précise
- Calibration automatique au démarrage

## Fichiers Modifiés

- `src/main.cpp` : Ajout des fonctions de stabilisation
- `src/ir_stabilization_config.h` : Configuration des paramètres
- `IR_STABILIZATION_README.md` : Ce fichier d'explication

## Utilisation

1. Compilez et téléversez le code
2. Ouvrez le moniteur série (115200 bauds)
3. Observez les valeurs de debug
4. Ajustez les paramètres selon vos besoins
5. Testez avec votre application

Les valeurs stabilisées sont maintenant utilisées pour contrôler les paramètres "pitch" et "vibrato_speed" du theremin. 