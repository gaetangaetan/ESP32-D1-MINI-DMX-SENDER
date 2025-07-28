# Configuration du Rotary Encoder

## Branchement du Rotary Encoder

Le rotary encoder du kit AZ Delivery 35 in 1 sensor kit doit être connecté comme suit :

### Connexions physiques :
- **Pin A** → **D5 (GPIO18)** sur l'ESP32 D1 Mini
- **Pin B** → **D6 (GPIO19)** sur l'ESP32 D1 Mini  
- **Pin Bouton** → **D7 (GPIO23)** sur l'ESP32 D1 Mini (optionnel)
- **VCC** → **3.3V** sur l'ESP32 D1 Mini
- **GND** → **GND** sur l'ESP32 D1 Mini

### Schéma de connexion :
```
Rotary Encoder    ESP32 D1 Mini
┌─────────────┐   ┌─────────────┐
│     A       │───│ D5 (GPIO18) │
│     B       │───│ D6 (GPIO19) │
│   Bouton    │───│ D7 (GPIO23) │
│     VCC     │───│ 3.3V        │
│     GND     │───│ GND         │
└─────────────┘   └─────────────┘
```

## Fonctionnalités

### Rotation du bouton :
- **Sens horaire** : Augmente la valeur du paramètre sélectionné
- **Sens anti-horaire** : Diminue la valeur du paramètre sélectionné
- **Sensibilité** : Chaque "clic" change la valeur de 2 unités (0-255)

### Bouton poussoir :
- **Appui court** : Change de paramètre (passe au suivant dans la liste)
- **Debounce** : 200ms pour éviter les appuis multiples

## Paramètres contrôlables

Le rotary encoder peut contrôler n'importe lequel des 20 paramètres du theremin :

1. **autopan** (DMX 101)
2. **pitch** (DMX 102) 
3. **vibrato_speed** (DMX 103)
4. **vibrato_depth** (DMX 104)
5. **delay_time** (DMX 105)
6. **delay_fbck** (DMX 106)
7. **osc** (DMX 107)
8. **gate** (DMX 108)
9. **glide** (DMX 109)
10. **scale** (DMX 110)
11. **offset_note** (DMX 111)
12. **osc2_vol** (DMX 112)
13. **osc2_pitch** (DMX 113)
14. **autopan_freq** (DMX 114)
15. **scale_tonic** (DMX 115)
16. **volume_drums** (DMX 116)
17. **kick_trig** (DMX 117)
18. **snare_trig** (DMX 118)
19. **hh_trig** (DMX 119)
20. **reserved** (DMX 120)

## Utilisation

1. **Compilation et upload** : Le code est prêt à être compilé et uploadé
2. **Monitoring série** : Ouvrez le moniteur série (115200 bauds) pour voir les changements
3. **Contrôle** : 
   - Tournez le bouton pour ajuster la valeur du paramètre actuel
   - Appuyez sur le bouton pour changer de paramètre
   - Les valeurs sont automatiquement envoyées via DMX

## Debug

Le moniteur série affiche :
- Le paramètre sélectionné et sa valeur actuelle
- Les changements de valeur en temps réel
- Le canal DMX correspondant

## Bibliothèque utilisée

- **ESP32Encoder** par madhephaestus
- Version : 1.0.4
- Ajoutée automatiquement via PlatformIO

## Notes techniques

- Le rotary encoder utilise le mode **Half-Quad** pour une meilleure précision
- Les valeurs sont limitées entre 0 et 255 (plage DMX standard)
- Le contrôle est non-volatile : les valeurs persistent tant que l'ESP32 est alimenté
- Compatible avec tous les rotary encoders standard (KY-040, etc.) 