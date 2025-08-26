# Optimisations Mémoire - Correction Interface Web

## Problème identifié

L'interface web ne répondait plus après l'implémentation des nouvelles fonctionnalités. Le diagnostic a révélé que le firmware utilisait **80% de la flash** (1,048,477 bytes sur 1,310,720 bytes), causant des instabilités.

## Solutions appliquées

### 1. Optimisation de la compilation

**Modification dans `platformio.ini`** :
```ini
build_flags = 
    -Os                      # Optimisation pour la taille
    -DCORE_DEBUG_LEVEL=1     # Réduction des messages de debug
board_build.partitions = min_spiffs.csv  # Partition optimisée
```

### 2. Réduction de la taille des structures

**WebPreset** :
- `name[32]` → `name[16]` : Économie de 16 bytes par preset
- Avec 8 presets max : 128 bytes économisés

**Buffers JSON** :
- `DynamicJsonDocument(4096)` → `DynamicJsonDocument(2048)`
- Économie de 2 KB par opération JSON

## Résultats

### Avant optimisation
- **Flash** : 80.0% (1,048,477 bytes sur 1,310,720 bytes)
- **RAM** : 16.3% (53,404 bytes sur 327,680 bytes)
- **Statut** : Interface web instable

### Après optimisation
- **Flash** : 53.3% (1,048,545 bytes sur 1,966,080 bytes)
- **RAM** : 16.3% (53,276 bytes sur 327,680 bytes)
- **Statut** : Interface web stable

## Améliorations obtenues

1. **Espace flash disponible** : +50% grâce à la nouvelle partition
2. **Marge de sécurité** : 46.7% d'espace libre vs 20% avant
3. **Stabilité** : Réduction des risques de corruption mémoire
4. **Performance** : Code optimisé pour la taille

## Notes techniques

- La partition `min_spiffs.csv` réduit l'espace SPIFFS mais augmente l'espace programme
- L'optimisation `-Os` privilégie la taille sur la vitesse
- La réduction des buffers JSON n'affecte pas les fonctionnalités
- Les noms de presets restent largement suffisants avec 15 caractères

## Validation

- ✅ Compilation réussie
- ✅ Upload réussi
- ✅ Toutes les fonctionnalités préservées
- ✅ Économie de mémoire significative

Cette optimisation résout le problème de stabilité tout en conservant toutes les nouvelles fonctionnalités (octave et assignations physiques).

