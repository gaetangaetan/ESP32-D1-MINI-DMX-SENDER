# Système de Contrôles Physiques Dynamiques

## Vue d'ensemble

Le système de contrôles physiques a été modifié pour être **entièrement dynamique** selon le preset actuel. Les liens entre les contrôles physiques et les paramètres ne sont plus fixes, mais sont lus directement depuis le preset chargé.

## Contrôles Physiques Disponibles

### 1. Capteur IR1 (Sharp IR)
- **Paramètre de lien** : `ir1_target_param` (canal DMX 125)
- **Fonction** : Contrôle le paramètre spécifié dans le preset
- **Désactivation** : Si `ir1_target_param = 0`, le capteur IR1 est désactivé

### 2. Capteur IR2 (Sharp IR)
- **Paramètre de lien** : `ir2_target_param` (canal DMX 126)
- **Fonction** : Contrôle le paramètre spécifié dans le preset
- **Désactivation** : Si `ir2_target_param = 0`, le capteur IR2 est désactivé

### 3. Fader 1 (Potentiomètre)
- **Paramètre de lien** : `fader1_target_param` (canal DMX 127)
- **Fonction** : Contrôle le paramètre spécifié dans le preset
- **Comportement spécial** : Quand `filter_on_off = 0`, contrôle toujours `vibrato_speed`
- **Désactivation** : Si `fader1_target_param = 0`, le fader 1 est désactivé (sauf en mode filtre OFF)

### 4. Fader 2 (Potentiomètre)
- **Paramètre de lien** : `fader2_target_param` (canal DMX 128)
- **Fonction** : Contrôle le paramètre spécifié dans le preset
- **Comportement spécial** : Quand `filter_on_off = 0`, contrôle toujours `delay_time`
- **Désactivation** : Si `fader2_target_param = 0`, le fader 2 est désactivé (sauf en mode filtre OFF)

### 5. Fader 3 (Potentiomètre)
- **Paramètre de lien** : `fader3_target_param` (canal DMX 129)
- **Fonction** : Contrôle le paramètre spécifié dans le preset
- **Comportement spécial** : Quand `filter_on_off = 0`, contrôle toujours `delay_feedback`
- **Désactivation** : Si `fader3_target_param = 0`, le fader 3 est désactivé (sauf en mode filtre OFF)

### 6. Bouton 3
- **Paramètre de lien** : `button3_target_param` (canal DMX 130)
- **Valeurs** : 
  - `button3_released_value` (canal DMX 131) : Valeur quand le bouton est relâché
  - `button3_pressed_value` (canal DMX 132) : Valeur quand le bouton est pressé
- **Fonction** : Bascule entre les deux valeurs du paramètre spécifié
- **Désactivation** : Si `button3_target_param = 0`, le bouton 3 est désactivé

## Exemples de Configuration

### Preset 1 - Configuration Classique
```
ir1_target_param = 1        // IR1 → pitch
ir2_target_param = 3        // IR2 → vibrato_depth
fader1_target_param = 2     // Fader1 → vibrato_speed
fader2_target_param = 4     // Fader2 → delay_time
fader3_target_param = 5     // Fader3 → delay_feedback
button3_target_param = 18   // Bouton3 → trig_hh
```

### Preset 2 - Configuration Alternative
```
ir1_target_param = 21       // IR1 → filter_on_off
ir2_target_param = 22       // IR2 → filter_cutoff
fader1_target_param = 23    // Fader1 → filter_reso
fader2_target_param = 24    // Fader2 → filter_type
fader3_target_param = 20    // Fader3 → master_volume
button3_target_param = 16   // Bouton3 → trig_kick
```

### Preset 3 - Désactivation Partielle
```
ir1_target_param = 1        // IR1 → pitch
ir2_target_param = 0        // IR2 désactivé
fader1_target_param = 2     // Fader1 → vibrato_speed
fader2_target_param = 0     // Fader2 désactivé
fader3_target_param = 5     // Fader3 → delay_feedback
button3_target_param = 0    // Bouton3 désactivé
```

## Avantages du Nouveau Système

1. **Flexibilité totale** : Chaque preset peut avoir une configuration de contrôles complètement différente
2. **Désactivation intelligente** : Les contrôles peuvent être désactivés individuellement (valeur = 0)
3. **Configuration en temps réel** : Les liens sont lus à chaque appel, pas besoin de recharger
4. **Compatibilité** : Le comportement spécial des faders en mode filtre OFF est préservé
5. **Maintenance** : Plus besoin de variables globales ou de fonctions de mise à jour

## Notes Techniques

- Les liens sont lus à chaque appel de `handlePhysicalControls()`
- La vérification `> 0` assure qu'un contrôle est désactivé si son paramètre de lien vaut 0
- Le système est compatible avec l'ancien comportement des faders en mode filtre OFF
- Tous les paramètres de lien sont stockés dans le preset et transmis via DMX
