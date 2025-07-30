/*
 * Configuration pour la stabilisation des capteurs IR Sharp
 * 
 * Ce fichier permet d'ajuster facilement les paramètres de stabilisation
 * sans modifier le code principal.
 */

#ifndef IR_STABILIZATION_CONFIG_H
#define IR_STABILIZATION_CONFIG_H

// === PARAMÈTRES DE STABILISATION ===

// Nombre d'échantillons pour la moyenne mobile
// Plus le nombre est élevé, plus la stabilisation est forte mais la réactivité diminue
#define IR_SAMPLE_SIZE 200

// Coefficient du filtre passe-bas (0.0 à 1.0)
// 0.0 = pas de changement (très stable)
// 1.0 = pas de filtrage (très réactif)
// 0.3 = bon compromis
#define IR_FILTER_ALPHA 0.1

// Seuil de variation pour ignorer le bruit (en unités ADC)
// Plus le seuil est élevé, plus les petites variations sont ignorées
#define IR_DEBOUNCE_THRESHOLD 10

// === MODES DE STABILISATION ===

// Décommentez le mode que vous souhaitez utiliser :

// Mode 1 : Moyenne mobile simple
#define USE_MOVING_AVERAGE

// Mode 2 : Filtre passe-bas
 #define USE_LOW_PASS_FILTER

// Mode 3 : Debouncing simple
 #define USE_DEBOUNCING

// Mode 4 : Combinaison de toutes les méthodes
 #define USE_COMBINED_FILTERING

// === PARAMÈTRES DE DEBUG ===

// Activer l'affichage des valeurs brutes et stabilisées
#define IR_DEBUG_ENABLED

// Fréquence d'affichage des valeurs de debug (en millisecondes)
#define IR_DEBUG_INTERVAL 100

// === CALIBRATION ===

// Valeurs de calibration pour mapper correctement les distances
// Ces valeurs peuvent être ajustées selon votre capteur et votre environnement

// Distance minimale détectée (en cm)
#define IR_MIN_DISTANCE_CM 4

// Distance maximale détectée (en cm)  
#define IR_MAX_DISTANCE_CM 80

// Valeur ADC correspondant à la distance minimale
#define IR_MIN_ADC_VALUE 0

// Valeur ADC correspondant à la distance maximale
#define IR_MAX_ADC_VALUE 4095

#endif // IR_STABILIZATION_CONFIG_H 