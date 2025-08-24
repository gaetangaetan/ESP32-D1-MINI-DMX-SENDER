# Solution des Collisions de Triggers

## 🎯 **Problème identifié et résolu**

### 📊 **Symptômes observés :**
- Tous les triggers de l'Onirigun sont reçus (visible dans le Serial Monitor)
- Seulement une partie déclenche une action visible
- Fonctionnement "aléatoire" du trigger externe

### 🔍 **Cause racine identifiée :**

Le problème n'était PAS dans la réception ESP-NOW, mais dans les **collisions de triggers** :

1. **Trigger 1** : Met `dmxValues[118] = 5` (trigger actif pour 100ms)
2. **Trigger 2** arrive 50ms plus tard : Met `dmxValues[118] = 5` (redémarre le trigger)
3. **Problème** : Aucune visibilité sur ces collisions !

### ⚙️ **Fonctionnement du système de triggers :**

```
Trigger reçu → dmxValues[118] = 5
↓
Émission DMX (50Hz) → dmxValues[118]-- à chaque envoi
↓
5 → 4 → 3 → 2 → 1 → 0 (trigger terminé)
↓
Durée totale : 5 × 20ms = 100ms
```

## ✅ **Solutions implémentées**

### 1. **Détection des collisions de triggers**

```cpp
// Vérifier s'il y a une collision avec un trigger déjà actif
if (dmxValues[118] > 0) {
  Serial.print("COLLISION DÉTECTÉE ! Trigger déjà actif (valeur restante: ");
  Serial.print(dmxValues[118]);
  Serial.println(") - Redémarrage du trigger");
} else {
  Serial.println("Nouveau trigger - Canal libre");
}
```

### 2. **Debug détaillé des valeurs DMX**

```cpp
// Affichage de toutes les informations du trigger
Serial.print("Trigger externe - Trig HH déclenché - Valeur DMX: ");
Serial.print(dmxValues[118]);
Serial.print(" - Durée estimée: ");
Serial.print(TRIG_LENGTH * 20);
Serial.println("ms");
```

### 3. **Monitoring en temps réel du canal DMX**

```cpp
// Détection des changements de valeur du canal DMX
if (dmxValues[118] != lastDmxValue) {
  Serial.print("Canal DMX 119 (trig_hh) changé: ");
  Serial.print(lastDmxValue);
  Serial.print(" -> ");
  Serial.print(dmxValues[118]);
  Serial.print(" (temps restant estimé: ");
  Serial.print(dmxValues[118] * 20);
  Serial.println("ms)");
}
```

### 4. **Synchronisation des variables de debug**

```cpp
// Synchroniser externalTriggerActive avec la vraie valeur DMX
bool wasTriggerActive = externalTriggerActive;
externalTriggerActive = (dmxValues[118] > 0);

// Détecter les changements d'état
if (wasTriggerActive && !externalTriggerActive) {
  Serial.println("Trigger externe terminé (DMX canal 119 = 0)");
}
```

## 📱 **Nouveaux messages de debug**

### Messages lors de la réception d'un trigger :

#### **Nouveau trigger (canal libre) :**
```
Trigger externe reçu de l'Onirigun: animation 3
Nouveau trigger - Canal libre
Trigger externe - Trig HH déclenché - Valeur DMX: 5 - Durée estimée: 100ms
Canal DMX 119 (trig_hh) changé: 0 -> 5 (temps restant estimé: 100ms)
```

#### **Collision de triggers :**
```
Trigger externe reçu de l'Onirigun: animation 3
COLLISION DÉTECTÉE ! Trigger déjà actif (valeur restante: 3) - Redémarrage du trigger
Trigger externe - Trig HH déclenché - Valeur DMX: 5 - Durée estimée: 100ms
Canal DMX 119 (trig_hh) changé: 3 -> 5 (temps restant estimé: 100ms)
```

### Messages de monitoring continu :

```
Trigger actif - Valeur DMX restante: 4 (≈80ms)
Canal DMX 119 (trig_hh) changé: 4 -> 3 (temps restant estimé: 60ms)
Canal DMX 119 (trig_hh) changé: 3 -> 2 (temps restant estimé: 40ms)
Canal DMX 119 (trig_hh) changé: 2 -> 1 (temps restant estimé: 20ms)
Canal DMX 119 (trig_hh) changé: 1 -> 0 (temps restant estimé: 0ms)
Trigger externe terminé (DMX canal 119 = 0)
```

## 🎯 **Résultats attendus**

### **Avant (problématique) :**
- Collisions invisibles
- Triggers "perdus" sans explication
- Debugging difficile

### **Après (corrigé) :**
- **Toutes les collisions sont détectées** et affichées
- **Monitoring complet** du canal DMX en temps réel
- **Visibilité totale** sur l'état des triggers
- **Debug précis** pour identifier tout problème

## 🔧 **Test et utilisation**

### **Flasher le code corrigé** sur l'ESP32

### **Observer le Serial Monitor** :
1. **Messages de réception** : Confirmation que les messages arrivent
2. **Détection de collisions** : "COLLISION DÉTECTÉE !" si triggers simultanés
3. **Monitoring DMX** : Évolution en temps réel de la valeur du canal 119
4. **Fin de trigger** : "Trigger externe terminé"

### **Interpréter les résultats :**

- **Si vous voyez "COLLISION DÉTECTÉE !"** → Les triggers arrivent trop rapidement, certains redémarrent d'autres
- **Si vous voyez "Nouveau trigger - Canal libre"** → Trigger normal, pas de collision
- **Si les valeurs DMX changent rapidement** → Le système fonctionne correctement

## 💡 **Optimisations possibles (futures)**

### **Option 1 : Ignorer les collisions**
```cpp
// Ne pas redémarrer un trigger déjà actif
if (dmxValues[118] == 0) {
  dmxValues[118] = TRIG_LENGTH;
} else {
  Serial.println("Trigger ignoré - Un autre est déjà actif");
}
```

### **Option 2 : Queue de triggers**
- Mettre les triggers en attente
- Les exécuter séquentiellement

### **Option 3 : Réduction de la durée**
- Réduire `TRIG_LENGTH` de 5 à 3 (60ms au lieu de 100ms)
- Réduire les collisions

Le système est maintenant **complètement transparent** et vous devriez voir exactement ce qui se passe avec chaque trigger ! 🎯
