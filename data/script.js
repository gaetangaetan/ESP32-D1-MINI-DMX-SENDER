// Script chargé

// Version du système
const VERSION = 1757978703;
// Version: " + VERSION

// Noms des paramètres
const paramNames = [
  'Autopan Depth', 'Pitch', 'Vibrato Speed', 'Vibrato Depth', 'Delay Time',
  'Delay Feedback', 'OSC Waveform', 'Gate Threshold', 'Portamento Time', 'Scale',
  'Octave Low High', 'OSC 2 Volume', 'OSC 2 Pitch Offset', 'Autopan Frequency', 'Scale Tonic',
  'Volume ONIRIGUN', null, null, null, 'Master Volume (Inverted)', // null = paramètres cachés
  'Filter On-Off', 'Filter Cutoff', 'Filter Reso', 'Filter Type', 'RGB Red', 'RGB Green', 'RGB Blue'
];

// Noms des assignations
const assignmentNames = ['Capteur IR 1', 'Capteur IR 2', 'Fader 2', 'Fader 3'];

// Interface unifiée - plus de vues séparées

// Système de debouncing pour éviter le spam de requêtes
let debounceTimers = {};
const DEBOUNCE_DELAY = 150; // 150ms de délai - plus réactif

function debouncedUpdateParameter(paramId, value) {
  // Annuler le timer précédent s'il existe
  if (debounceTimers[paramId]) {
    clearTimeout(debounceTimers[paramId]);
  }
  
  // Programmer une nouvelle requête
  debounceTimers[paramId] = setTimeout(() => {
    updateParameter(paramId, value);
    delete debounceTimers[paramId];
  }, DEBOUNCE_DELAY);
}

// Fonctions API
function updateParameter(paramId, value) {
  fetch('/api/parameters', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({id: parseInt(paramId), value: parseInt(value)})
  })
  .then(r => r.json())
  .then(d => {
    // Notification supprimée pour éviter le spam
    if (!d.success) {
      showStatus('❌ Erreur mise à jour');
    }
  })
  .catch(e => showStatus('❌ Erreur de connexion'));
}

// Version asynchrone pour les presets
async function updateParameterAsync(paramId, value) {
  try {
    const response = await fetch('/api/parameters', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({id: parseInt(paramId), value: parseInt(value)})
    });
    const data = await response.json();
    if (!data.success) {
      throw new Error('Erreur serveur: ' + data.message);
    }
    return data;
  } catch (error) {
    console.error('Erreur updateParameterAsync:', error);
    throw error;
  }
}

function updateAssignment(assignId, value) {
  fetch('/api/assignments', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({index: parseInt(assignId), value: parseInt(value)})
  })
  .then(r => r.json())
  .then(d => {
    if (d.success) {
      // Recharger les assignations pour mettre à jour les couleurs
      loadCurrentAssignments();
    } else {
      showStatus('❌ Erreur assignation');
    }
  })
  .catch(e => showStatus('❌ Erreur de connexion'));
}

function changeOctave(direction) {
  fetch('/api/octave', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({ direction: direction })
  })
  .then(r => r.json())
  .then(d => {
    if (d.success) {
      const octaveDisplay = document.getElementById('octave-display');
      octaveDisplay.textContent = d.octave;
      showStatus('✅ Octave: ' + d.octave);
    } else {
      showStatus('❌ Erreur octave');
    }
  })
  .catch(e => showStatus('❌ Erreur de connexion'));
}

function savePreset(presetId) {
  // Récupérer les valeurs actuelles des 27 paramètres (y compris filter et RGB)
  const values = [];
  
  // Paramètres principaux (0-20)  
  for (let i = 0; i < 21; i++) {
    let value = 0;
    
    if (i === 6) {
      // OSC Waveform - récupérer depuis l'affichage de valeur
      const valueDisplay = document.getElementById('value-6');
      value = valueDisplay ? parseInt(valueDisplay.textContent) : 0;
    } else {
      // Contrôle normal avec slider
      const slider = document.getElementById('param-' + i);
      value = slider ? parseInt(slider.value) : 0;
    }
    
    values.push(value);
  }
  
  // Paramètres filter (21-23) - récupérer depuis les contrôles de filtre
  const filterCutoffSlider = document.getElementById('filter-cutoff');
  values.push(filterCutoffSlider ? parseInt(filterCutoffSlider.value) : 0);
  
  const filterResoSlider = document.getElementById('filter-reso');
  values.push(filterResoSlider ? parseInt(filterResoSlider.value) : 0);
  
  // Déterminer le mode filtre depuis les boutons et convertir en valeurs DMX
  let filterModeValue = 0; // 0 = OFF par défaut
  if (document.getElementById('filter-hp').classList.contains('active')) filterModeValue = 85; // HP
  else if (document.getElementById('filter-bp').classList.contains('active')) filterModeValue = 170; // BP  
  else if (document.getElementById('filter-lp').classList.contains('active')) filterModeValue = 255; // LP
  else if (document.getElementById('filter-off').classList.contains('active')) filterModeValue = 0; // OFF
  values.push(filterModeValue);
  
  // Paramètres RGB (24-26)
  values.push(parseInt(document.getElementById('rgb-red-value').textContent));
  values.push(parseInt(document.getElementById('rgb-green-value').textContent));
  values.push(parseInt(document.getElementById('rgb-blue-value').textContent));
  
  console.log('DEBUG: Sauvegarde preset - values array length:', values.length);
  console.log('DEBUG: Sauvegarde preset - values 20-26:', values.slice(20, 27));

  // Récupérer les assignations physiques actuelles
  const assignments = [];
  for (let i = 0; i < 4; i++) {
    const select = document.getElementById('assignment-' + i);
    assignments.push(select ? parseInt(select.value) : 0);
  }

  fetch('/api/save-preset', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({
      name: 'Preset ' + presetId,
      values: values,
      assignments: assignments,
      slot: parseInt(presetId) - 1 // Convertir l'ID du bouton (1-8) en index de slot (0-7)
    })
  })
  .then(r => r.json())
  .then(d => {
    if (d.success) {
      showStatus('✅ Preset ' + presetId + ' sauvegardé');
    } else {
      showStatus('❌ Erreur sauvegarde');
    }
  })
  .catch(e => showStatus('❌ Erreur de connexion'));
}

async function loadPreset(presetId) {
  console.log('DEBUG: loadPreset() appelée avec ID:', presetId);
  
  try {
    const response = await fetch('/api/load-preset', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({id: parseInt(presetId) - 1}) // Convertir l'ID du bouton (1-8) en index de tableau (0-7)
    });
    
    console.log('DEBUG: Réponse fetch reçue, status:', response.status);
    const data = await response.json();
    console.log('DEBUG: Données JSON reçues:', data);
    
    if (data.success) {
      console.log('DEBUG: Début applyPresetToInterface');
      // Appliquer directement les valeurs du preset sans appeler l'API
      await applyPresetToInterface(data.parameters);
      console.log('DEBUG: applyPresetToInterface terminé');
      
      // Mettre à jour l'affichage de l'octave si présente dans la réponse
      if (typeof data.octave !== 'undefined') {
        console.log('DEBUG: Mise à jour octave:', data.octave);
        const octaveDisplay = document.getElementById('octave-display');
        if (octaveDisplay) {
          octaveDisplay.textContent = data.octave;
          console.log('DEBUG: Octave mise à jour dans le DOM');
        } else {
          console.log('DEBUG: Element octave-display non trouvé');
        }
      } else {
        console.log('DEBUG: Pas d\'octave dans la réponse');
      }
      
      // Mettre à jour l'affichage du preset actuel
      updateCurrentPresetDisplay(presetId);
      
      // Mettre à jour les assignations si présentes dans la réponse
      if (data.assignments) {
        console.log('DEBUG: Mise à jour assignations:', data.assignments);
        applyAssignmentsToInterface(data.assignments);
        console.log('DEBUG: Assignations mises à jour');
      } else {
        console.log('DEBUG: Pas d\'assignations dans la réponse');
      }
      
      console.log('DEBUG: Affichage du status de succès');
      showStatus('✅ Preset ' + presetId + ' chargé');
    } else {
      console.log('DEBUG: Échec du chargement:', data.message);
      showStatus('❌ Erreur chargement');
    }
  } catch (error) {
    console.error('DEBUG: Erreur fetch:', error);
    showStatus('❌ Erreur de connexion');
  }
}

function loadCurrentParams() {
  fetch('/api/parameters')
  .then(r => r.json())
  .then(d => {
    d.parameters.forEach((value, index) => {
      const valueDisplay = document.getElementById('value-' + index);
      
      if (index === 6) {
        // OSC Waveform - mettre à jour l'affichage seulement
        if (valueDisplay) {
          valueDisplay.textContent = value;
        }
      } else if (index >= 20 && index <= 23) {
        // Paramètres filter - ignorer, gérés par les contrôles filter
        return;
      } else {
        // Contrôle normal avec slider
        const slider = document.getElementById('param-' + index);
        if (slider && valueDisplay) {
          slider.value = value;
          valueDisplay.textContent = value;
        }
      }
    });
  })
  .catch(e => console.error('Erreur chargement paramètres:', e));
}

function loadCurrentAssignments() {
  fetch('/api/assignments')
  .then(r => r.json())
  .then(d => {
    d.assignments.forEach((value, index) => {
      const select = document.getElementById('assignment-' + index);
      if (select) {
        select.value = value;
      }
    });
    
    // Appliquer les couleurs d'assignation
    applyAssignmentColors(d.assignments);
  })
  .catch(e => console.error('Erreur chargement assignations:', e));
}

function applyAssignmentsToInterface(assignments) {
  console.log('DEBUG: applyAssignmentsToInterface appelée avec:', assignments);
  
  if (!assignments || assignments.length !== 4) {
    console.error('DEBUG: Assignations invalides, attendu 4 assignations, reçu:', assignments?.length);
    return;
  }
  
  // Appliquer les assignations aux sélecteurs
  assignments.forEach((value, index) => {
    console.log('DEBUG: Application assignation', index, '=', value);
    const select = document.getElementById('assignment-' + index);
    if (select) {
      select.value = value;
      console.log('DEBUG: Assignation', index, 'appliquée');
    } else {
      console.log('DEBUG: Sélecteur assignment-' + index + ' non trouvé');
    }
  });
  console.log('DEBUG: applyAssignmentsToInterface terminée');
  
  // Appliquer les couleurs d'assignation
  applyAssignmentColors(assignments);
}

function applyAssignmentColors(assignments) {
  console.log('DEBUG: applyAssignmentColors appelée avec:', assignments);
  
  // Supprimer toutes les classes d'assignation existantes
  document.querySelectorAll('.param-control').forEach(control => {
    control.classList.remove('assigned-ir1', 'assigned-ir2', 'assigned-fader2', 'assigned-fader3');
  });
  
  document.querySelectorAll('.filter-row').forEach(control => {
    control.classList.remove('assigned-ir1', 'assigned-ir2', 'assigned-fader2', 'assigned-fader3');
  });
  
  document.querySelectorAll('.slider, .fader').forEach(slider => {
    slider.classList.remove('assigned-ir1', 'assigned-ir2', 'assigned-fader2', 'assigned-fader3');
  });
  
  if (!assignments || assignments.length !== 4) return;
  
  // Mapper les assignations aux classes CSS
  const assignmentClasses = ['assigned-ir1', 'assigned-ir2', 'assigned-fader2', 'assigned-fader3'];
  
  assignments.forEach((paramIndex, controlIndex) => {
    if (paramIndex > 0 && paramIndex <= 23) { // Paramètre assigné
      const realParamIndex = paramIndex - 1; // Convertir 1-based en 0-based
      let slider = document.getElementById('param-' + realParamIndex);
      let control = slider ? slider.closest('.param-control') : null;
      
      // Si pas trouvé dans les paramètres normaux, chercher dans les filtres
      if (!slider) {
        if (realParamIndex === 21) { // Filter Cutoff
          slider = document.getElementById('filter-cutoff');
          control = slider ? slider.closest('.filter-row') : null;
        } else if (realParamIndex === 22) { // Filter Reso  
          slider = document.getElementById('filter-reso');
          control = slider ? slider.closest('.filter-row') : null;
        }
      }
      
      if (slider && control) {
        const colorClass = assignmentClasses[controlIndex];
        slider.classList.add(colorClass);
        control.classList.add(colorClass);
        console.log('DEBUG: Couleur appliquée', colorClass, 'au paramètre', realParamIndex);
      }
    }
  });
}

// Nouvelle fonction : appliquer un preset directement à l'interface
async function applyPresetToInterface(presetValues) {
  if (!presetValues || presetValues.length !== 27) {
    console.error('Valeurs de preset invalides, attendu 27 paramètres, reçu:', presetValues?.length);
    return;
  }
  
  console.log('DEBUG: Début applyPresetToInterface avec', presetValues.length, 'paramètres');
  
  // Créer un tableau de promesses pour tous les updateParameter
  const updatePromises = [];
  
  // Appliquer les paramètres principaux (0-20)
  for (let i = 0; i < 21; i++) {
    // Ignorer les paramètres null
    if (paramNames[i] === null) {
      continue;
    }
    
    // Contrôles spéciaux
    if (i === 6) { // OSC Waveform
      const waveform = Math.round(presetValues[i] / 40);
      document.getElementById('value-6').textContent = presetValues[i];
      document.getElementById('waveform-display').textContent = waveform;
    } else if (i === 20) { // Filter - traité avec le paramètre 23
      // Ne rien faire ici, c'est géré dans la section spéciale ci-dessous
    } else {
      // Contrôle normal
      const slider = document.getElementById('param-' + i);
      const valueDisplay = document.getElementById('value-' + i);
      
      if (slider && valueDisplay) {
        slider.value = presetValues[i];
        valueDisplay.textContent = presetValues[i];
        // Ajouter la promesse pour ce paramètre
        updatePromises.push(updateParameterAsync(i, presetValues[i]));
      }
    }
  }
  
  // Appliquer les paramètres filter (21-23)
  const filterCutoff = document.getElementById('filter-cutoff');
  const filterReso = document.getElementById('filter-reso');
  
  if (filterCutoff) {
    filterCutoff.value = presetValues[21];
    document.getElementById('filter-cutoff-value').textContent = presetValues[21];
    updatePromises.push(updateParameterAsync(21, presetValues[21])); // Envoyer Cutoff au serveur
  }
  if (filterReso) {
    filterReso.value = presetValues[22];
    document.getElementById('filter-reso-value').textContent = presetValues[22];
    updatePromises.push(updateParameterAsync(22, presetValues[22])); // Envoyer Résonance au serveur
  }
  
  // Mode filtre (paramètre 23 contient la valeur DMX 0-255)
  const filterModeValue = presetValues[23];
  
  console.log('DEBUG: Filter values - Cutoff:', presetValues[21], 'Reso:', presetValues[22], 'Mode:', filterModeValue);
  
  // Déterminer le mode selon la valeur DMX (0=OFF, 85=HP, 170=BP, 255=LP)
  let filterMode;
  if (filterModeValue === 0) filterMode = 'OFF';
  else if (filterModeValue >= 1 && filterModeValue <= 85) filterMode = 'HP';
  else if (filterModeValue >= 86 && filterModeValue <= 170) filterMode = 'BP';
  else if (filterModeValue >= 171 && filterModeValue <= 255) filterMode = 'LP';
  else filterMode = 'OFF'; // défaut
  
  console.log('DEBUG: Filter mode déterminé:', filterMode);
  
  // Mettre à jour les boutons (tous inactifs puis activer le bon)
  document.getElementById('filter-hp').classList.remove('active');
  document.getElementById('filter-bp').classList.remove('active');
  document.getElementById('filter-lp').classList.remove('active');
  document.getElementById('filter-off').classList.remove('active');
  document.getElementById('filter-' + filterMode.toLowerCase()).classList.add('active');
  
  // Créer/mettre à jour les valeurs cachées pour compatibilité
  if (!document.getElementById('value-20')) {
    const hiddenValue20 = document.createElement('span');
    hiddenValue20.id = 'value-20';
    hiddenValue20.style.display = 'none';
    document.body.appendChild(hiddenValue20);
  }
  if (!document.getElementById('value-23')) {
    const hiddenValue23 = document.createElement('span');
    hiddenValue23.id = 'value-23';
    hiddenValue23.style.display = 'none';
    document.body.appendChild(hiddenValue23);
  }
  // Valeur 20: 255 si filtre actif, 0 si OFF
  const filterOnOffValue = (filterModeValue > 0) ? 255 : 0;
  document.getElementById('value-20').textContent = filterOnOffValue;
  document.getElementById('value-23').textContent = filterModeValue;
  
  // IMPORTANT: Envoyer le paramètre Filter On-Off au serveur
  updatePromises.push(updateParameterAsync(20, filterOnOffValue)); // Filter On/Off 
  updatePromises.push(updateParameterAsync(23, filterModeValue));  // Filter Mode
  
  console.log('DEBUG: Filter On-Off envoyé au serveur:', filterOnOffValue);
  
  // Appliquer les paramètres RGB (24-26)
  const rgbRed = presetValues[24];
  const rgbGreen = presetValues[25];
  const rgbBlue = presetValues[26];
  
  updateRGBDisplay(rgbRed, rgbGreen, rgbBlue);
  updatePromises.push(updateParameterAsync(24, rgbRed));
  updatePromises.push(updateParameterAsync(25, rgbGreen));
  updatePromises.push(updateParameterAsync(26, rgbBlue));
  
  // Attendre que tous les paramètres soient envoyés
  console.log('DEBUG: Attente de', updatePromises.length, 'paramètres...');
  try {
    await Promise.all(updatePromises);
    console.log('DEBUG: Tous les paramètres envoyés avec succès');
    showStatus('✅ Preset chargé et synchronisé');
  } catch (error) {
    console.error('DEBUG: Erreur lors de l\'envoi des paramètres:', error);
    showStatus('❌ Erreur synchronisation preset');
  }
  
  console.log('Preset appliqué à l\'interface (27 paramètres):', presetValues);
}

function showStatus(message) {
  const status = document.getElementById('status');
  status.textContent = message;
  status.className = 'status show ' + (message.includes('✅') ? 'success' : 'error');
  
  setTimeout(() => {
    status.classList.remove('show');
  }, 3000);
}

// Interface unifiée - plus besoin de fonction switchView

// Génération du contenu HTML
function generatePresets() {
  const saveContainer = document.getElementById('savePresetsContainer');
  const loadContainer = document.getElementById('loadPresetsContainer');
  
  let saveHtml = '';
  let loadHtml = '';
  
  // Boutons Save (S1-S8)
  for (let i = 1; i <= 8; i++) {
    saveHtml += '<button class="btn btn-save" onclick="savePreset(' + i + ')">S' + i + '</button>';
  }
  
  // Boutons Load (L1-L8)
  for (let i = 1; i <= 8; i++) {
    loadHtml += '<button class="btn btn-load" onclick="loadPreset(' + i + ')">L' + i + '</button>';
  }
  
  saveContainer.innerHTML = saveHtml;
  loadContainer.innerHTML = loadHtml;
}

function generateAssignments() {
  const container = document.getElementById('assignmentsGrid');
  let html = '';
  
  for (let i = 0; i < 4; i++) {
    html += '<div class="assignment-control">';
    html += '<label for="assignment-' + i + '">' + assignmentNames[i] + '</label>';
    html += '<select class="custom-select" id="assignment-' + i + '" onchange="updateAssignment(' + i + ', this.value)">';
    html += '<option value="0">OFF</option>';
    
    for (let j = 0; j < 24; j++) {
      html += '<option value="' + (j + 1) + '">' + paramNames[j] + '</option>';
    }
    
    html += '</select>';
    html += '</div>';
  }
  
  container.innerHTML = html;
}

function generateParameters() {
  const container = document.getElementById('paramsGrid');
  let html = '';
  
  for (let i = 0; i < 24; i++) {
    // Ignorer les paramètres null (16, 17, 18) et les filtres (20, 21, 22, 23)
    if (paramNames[i] === null || (i >= 20 && i <= 23)) {
      continue;
    }
    
    // Contrôles spéciaux
    if (i === 6) { // OSC Waveform
      html += '<div class="param-control">';
      html += '<div class="param-label-wrapper">';
      html += '<span class="param-name">OSC Waveform</span>';
      html += '<span class="param-value" id="value-6">0</span>';
      html += '</div>';
      html += '<div class="waveform-controls">';
      html += '<button class="btn-waveform" onclick="changeWaveform(-1)">-</button>';
      html += '<span class="waveform-display" id="waveform-display">0</span>';
      html += '<button class="btn-waveform" onclick="changeWaveform(1)">+</button>';
      html += '</div>';
      html += '</div>';
    } else {
      // Contrôle normal
      html += '<div class="param-control">';
      html += '<div class="param-label-wrapper">';
      html += '<span class="param-name">' + paramNames[i] + '</span>';
      html += '<span class="param-value" id="value-' + i + '">0</span>';
      html += '</div>';
      html += '<input type="range" class="slider" id="param-' + i + '" min="0" max="255" value="0" ';
      html += 'oninput="debouncedUpdateParameter(' + i + ', this.value); document.getElementById(\'value-' + i + '\').textContent = this.value">';
      html += '</div>';
    }
  }
  
  container.innerHTML = html;
}

// Fonctions pour les contrôles RGB
function updateRGBDisplay(red, green, blue) {
  document.getElementById('rgb-red-value').textContent = red;
  document.getElementById('rgb-green-value').textContent = green;
  document.getElementById('rgb-blue-value').textContent = blue;
  
  // Mettre à jour le color picker
  const hex = rgbToHex(red, green, blue);
  document.getElementById('color-picker').value = hex;
}

function rgbToHex(r, g, b) {
  return "#" + ((1 << 24) + (r << 16) + (g << 8) + b).toString(16).slice(1);
}

function hexToRgb(hex) {
  const result = /^#?([a-f\d]{2})([a-f\d]{2})([a-f\d]{2})$/i.exec(hex);
  return result ? {
    r: parseInt(result[1], 16),
    g: parseInt(result[2], 16),
    b: parseInt(result[3], 16)
  } : null;
}

function setupFilterControls() {
  // Configuration des faders filter
  const filterCutoff = document.getElementById('filter-cutoff');
  const filterReso = document.getElementById('filter-reso');
  const filterType = document.getElementById('filter-type');
  
  if (filterCutoff) {
    filterCutoff.addEventListener('input', function() {
      const value = parseInt(this.value);
      document.getElementById('filter-cutoff-value').textContent = value;
      debouncedUpdateParameter(21, value); // Index 21 = filter_cutoff
    });
  }
  
  if (filterReso) {
    filterReso.addEventListener('input', function() {
      const value = parseInt(this.value);
      document.getElementById('filter-reso-value').textContent = value;
      debouncedUpdateParameter(22, value); // Index 22 = filter_reso
    });
  }
  
  // Pas de slider pour filter-type, géré par les boutons setFilterMode()
}

function setupRGBControls() {
  // Configuration du sélecteur de couleur
  const colorPicker = document.getElementById('color-picker');
  
  if (colorPicker) {
    colorPicker.addEventListener('change', function() {
      const rgb = hexToRgb(this.value);
      if (rgb) {
        updateRGBDisplay(rgb.r, rgb.g, rgb.b);
        updateParameter(24, rgb.r); // Index 24 = rgb1_red
        updateParameter(25, rgb.g); // Index 25 = rgb1_green
        updateParameter(26, rgb.b); // Index 26 = rgb1_blue
      }
    });
  }
}

// === SYSTÈME DE DEBUG ===
let debugPanelVisible = false;
let debugRefreshInterval = null;

// Fonction pour charger les messages de debug
function loadDebugMessages() {
  fetch('/api/debug')
    .then(response => response.json())
    .then(data => {
      const debugMessagesDiv = document.getElementById('debug-messages');
      if (data.messages) {
        debugMessagesDiv.textContent = data.messages;
        // Scroll vers le bas pour voir les derniers messages
        debugMessagesDiv.scrollTop = debugMessagesDiv.scrollHeight;
      } else {
        debugMessagesDiv.textContent = 'Aucun message de debug disponible';
      }
    })
    .catch(error => {
      console.error('Erreur lors du chargement des messages de debug:', error);
      document.getElementById('debug-messages').textContent = 'Erreur lors du chargement des messages de debug';
    });
}

// Fonction pour effacer les messages de debug
function clearDebugMessages() {
  document.getElementById('debug-messages').textContent = 'Messages de debug effacés';
}

// Fonction pour basculer l'affichage du panneau de debug
function toggleDebugPanel() {
  const debugPanel = document.getElementById('debug-panel');
  const debugToggle = document.getElementById('debug-toggle');
  
  debugPanelVisible = !debugPanelVisible;
  
  if (debugPanelVisible) {
    debugPanel.style.display = 'block';
    debugToggle.textContent = '🐛 Masquer Debug';
    debugToggle.classList.add('active');
    
    // Charger les messages et démarrer le rafraîchissement automatique
    loadDebugMessages();
    debugRefreshInterval = setInterval(loadDebugMessages, 2000); // Rafraîchir toutes les 2 secondes
  } else {
    debugPanel.style.display = 'none';
    debugToggle.textContent = '🐛 Debug';
    debugToggle.classList.remove('active');
    
    // Arrêter le rafraîchissement automatique
    if (debugRefreshInterval) {
      clearInterval(debugRefreshInterval);
      debugRefreshInterval = null;
    }
  }
}

// Initialisation
document.addEventListener('DOMContentLoaded', function() {
  console.log('DEBUG: DOMContentLoaded - Interface DMX ESP32 initialisée');
  console.log('DEBUG: Début initialisation des écouteurs d\'événements');
  
  // Écouteurs pour les boutons d'octave
  const octaveDownBtn = document.getElementById('octave-down');
  const octaveUpBtn = document.getElementById('octave-up');
  
  if (octaveDownBtn) {
    octaveDownBtn.addEventListener('click', () => changeOctave('down'));
  }
  
  if (octaveUpBtn) {
    octaveUpBtn.addEventListener('click', () => changeOctave('up'));
  }
  
  // Générer le contenu
  generatePresets();
  generateAssignments();
  generateParameters();
  
  // Configurer les nouveaux contrôles
  setupFilterControls();
  setupRGBControls();
  
  // Charger les données initiales (désactivé pour éviter les erreurs au démarrage)
  // Les paramètres sont déjà initialisés à 0 dans le HTML
  // loadCurrentParams();
  // loadCurrentAssignments();
  
  // Initialiser l'affichage du preset actuel
  updateCurrentPresetDisplay(1);
  
  // Polling désactivé (trop lourd)
  // startStatusPolling();
  
  // Gestion des boutons d'export/import
  setupBackupButtons();
  
  // === EVENT LISTENERS DE DEBUG ===
  // Bouton toggle debug
  const debugToggle = document.getElementById('debug-toggle');
  if (debugToggle) {
    debugToggle.addEventListener('click', toggleDebugPanel);
    console.log('DEBUG: Event listener debug attaché avec succès');
  }
  
  // Bouton actualiser debug
  const debugRefresh = document.getElementById('debug-refresh');
  if (debugRefresh) {
    debugRefresh.addEventListener('click', loadDebugMessages);
  }
  
  // Bouton effacer debug
  const debugClear = document.getElementById('debug-clear');
  if (debugClear) {
    debugClear.addEventListener('click', clearDebugMessages);
  }
  
  // === BOUTON CHANGER WIFI ===
  const changeWifiBtn = document.getElementById('change-wifi');
  if (changeWifiBtn) {
    changeWifiBtn.addEventListener('click', function() {
      if (confirm('🔄 Changer le réseau WiFi ?\n\nCela va ouvrir le portail de configuration WiFi.')) {
        fetch('/api/change-wifi', { method: 'POST' })
          .then(response => response.json())
          .then(data => {
            if (data.success) {
              alert('✅ Portail WiFi lancé !\n\nConnectez-vous au réseau "KsolotiKontrol-Config" pour choisir un nouveau réseau.');
            } else {
              alert('❌ Erreur: ' + data.message);
            }
          })
          .catch(error => {
            console.error('Erreur:', error);
            alert('❌ Erreur de communication');
          });
      }
    });
  }
});

// Fonction pour changer la waveform (0-6)
function changeWaveform(direction) {
  const currentValue = parseInt(document.getElementById('value-6').textContent);
  const currentWaveform = Math.round(currentValue / 40);
  let newWaveform = currentWaveform + direction;
  
  // Limiter entre 0 et 6
  if (newWaveform < 0) newWaveform = 0;
  if (newWaveform > 6) newWaveform = 6;
  
  const newValue = newWaveform * 40;
  
  // Mettre à jour l'affichage
  document.getElementById('value-6').textContent = newValue;
  document.getElementById('waveform-display').textContent = newWaveform;
  
  // Envoyer la valeur au serveur
  updateParameter(6, newValue);
}

// Fonction unifiée pour définir le mode de filtre (HP/BP/LP/OFF)
function setFilterMode(mode) {
  // Retirer la classe active de tous les boutons
  document.getElementById('filter-hp').classList.remove('active');
  document.getElementById('filter-bp').classList.remove('active');
  document.getElementById('filter-lp').classList.remove('active');
  document.getElementById('filter-off').classList.remove('active');
  
  // Ajouter la classe active au bouton sélectionné
  document.getElementById('filter-' + mode.toLowerCase()).classList.add('active');
  
  // Définir la valeur du mode filtre selon le nouveau système (plage DMX 0-255)
  let filterModeValue;
  
  switch(mode) {
    case 'HP':
      filterModeValue = 85; // High Pass (255/3 * 1)
      break;
    case 'BP':
      filterModeValue = 170; // Band Pass (255/3 * 2)
      break;
    case 'LP':
      filterModeValue = 255; // Low Pass (255/3 * 3)
      break;
    case 'OFF':
      filterModeValue = 0; // Filtre désactivé
      break;
    default:
      filterModeValue = 0;
  }
  
  // Créer/mettre à jour les valeurs cachées pour compatibilité
  if (!document.getElementById('value-20')) {
    const hiddenValue20 = document.createElement('span');
    hiddenValue20.id = 'value-20';
    hiddenValue20.style.display = 'none';
    document.body.appendChild(hiddenValue20);
  }
  if (!document.getElementById('value-23')) {
    const hiddenValue23 = document.createElement('span');
    hiddenValue23.id = 'value-23';
    hiddenValue23.style.display = 'none';
    document.body.appendChild(hiddenValue23);
  }
  
  // Valeur 20: 255 si filtre actif, 0 si OFF
  const filterOnOffValue = (filterModeValue > 0) ? 255 : 0;
  document.getElementById('value-20').textContent = filterOnOffValue;
  document.getElementById('value-23').textContent = filterModeValue;
  
  // Envoyer les valeurs au serveur
  updateParameter(20, filterOnOffValue); // Filter On/Off (compatibilité)
  updateParameter(23, filterModeValue);  // Filter Mode (nouveau système)
  
  console.log('DEBUG: Filter mode set to', mode, '- Mode value:', filterModeValue);
}

function updateCurrentPresetDisplay(presetId) {
  const currentPresetDisplay = document.getElementById('current-preset-display');
  if (currentPresetDisplay) {
    currentPresetDisplay.textContent = 'P' + presetId;
    console.log('DEBUG: Preset actuel mis à jour: P' + presetId);
  }
}

// Variables pour le polling - DÉSACTIVÉ (trop lourd)
// let lastKnownPhysicalChange = 0;
// let statusPollingInterval = null;

// Fonctions de sauvegarde/chargement des presets
function setupBackupButtons() {
  const exportBtn = document.getElementById('export-presets');
  const importBtn = document.getElementById('import-presets');
  const importFile = document.getElementById('import-file');
  
  if (exportBtn) {
    exportBtn.addEventListener('click', exportPresets);
  }
  
  if (importBtn) {
    importBtn.addEventListener('click', () => {
      importFile.click();
    });
  }
  
  if (importFile) {
    importFile.addEventListener('change', (event) => {
      const file = event.target.files[0];
      if (file) {
        importPresets(file);
      }
    });
  }
}

// Fonction pour afficher des messages d'information temporaires
function showTooltip(message, type = 'info') {
  const tooltip = document.createElement('div');
  tooltip.className = 'tooltip ' + type;
  tooltip.textContent = message;
  tooltip.style.cssText = `
    position: fixed;
    top: 20px;
    right: 20px;
    background: ${type === 'success' ? '#4CAF50' : type === 'error' ? '#f44336' : '#2196F3'};
    color: white;
    padding: 12px 20px;
    border-radius: 4px;
    font-size: 14px;
    font-weight: bold;
    z-index: 10000;
    box-shadow: 0 2px 10px rgba(0,0,0,0.3);
    animation: slideIn 0.3s ease-out;
  `;
  
  // Ajouter l'animation CSS si elle n'existe pas déjà
  if (!document.querySelector('#tooltip-styles')) {
    const style = document.createElement('style');
    style.id = 'tooltip-styles';
    style.textContent = `
      @keyframes slideIn {
        from { transform: translateX(100%); opacity: 0; }
        to { transform: translateX(0); opacity: 1; }
      }
      @keyframes slideOut {
        from { transform: translateX(0); opacity: 1; }
        to { transform: translateX(100%); opacity: 0; }
      }
    `;
    document.head.appendChild(style);
  }
  
  document.body.appendChild(tooltip);
  
  // Retirer automatiquement après 3 secondes
  setTimeout(() => {
    tooltip.style.animation = 'slideOut 0.3s ease-in';
    setTimeout(() => {
      if (tooltip.parentNode) {
        tooltip.parentNode.removeChild(tooltip);
      }
    }, 300);
  }, 3000);
}

function exportPresets() {
  console.log('DEBUG: Exportation des presets...');
  
  fetch('/api/export-presets')
    .then(response => {
      if (!response.ok) {
        throw new Error('Erreur lors de l\'exportation');
      }
      
      // Récupérer le nom de fichier depuis les headers
      const contentDisposition = response.headers.get('Content-Disposition');
      let filename = 'ksoloti_presets.json';
      if (contentDisposition) {
        const filenameMatch = contentDisposition.match(/filename="(.+)"/);
        if (filenameMatch) {
          filename = filenameMatch[1];
        }
      }
      
      return response.blob().then(blob => ({ blob, filename }));
    })
    .then(({ blob, filename }) => {
      // Créer un lien de téléchargement
      const url = window.URL.createObjectURL(blob);
      const a = document.createElement('a');
      a.href = url;
      a.download = filename;
      document.body.appendChild(a);
      a.click();
      window.URL.revokeObjectURL(url);
      document.body.removeChild(a);
      
      console.log('DEBUG: Presets exportés:', filename);
      showTooltip('✅ Presets sauvegardés: ' + filename, 'success');
    })
    .catch(error => {
      console.error('ERROR: Échec de l\'exportation:', error);
      showTooltip('❌ Erreur lors de la sauvegarde', 'error');
    });
}

function importPresets(file) {
  console.log('DEBUG: Importation des presets...', file.name);
  
  const reader = new FileReader();
  reader.onload = function(e) {
    try {
      const jsonData = JSON.parse(e.target.result);
      console.log('DEBUG: Données JSON:', jsonData);
      
      // Envoyer les données au serveur
      fetch('/api/import-presets', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json'
        },
        body: JSON.stringify(jsonData)
      })
      .then(response => response.json())
      .then(data => {
        if (data.success) {
          console.log('DEBUG: Import réussi:', data);
          showTooltip('✅ ' + data.imported_count + ' presets importés', 'success');
          
          // Recharger l'interface pour afficher les nouveaux presets
          setTimeout(() => {
            location.reload();
          }, 1500);
        } else {
          console.error('ERROR: Échec de l\'import:', data.message);
          showTooltip('❌ ' + data.message, 'error');
        }
      })
      .catch(error => {
        console.error('ERROR: Erreur réseau lors de l\'import:', error);
        showTooltip('❌ Erreur de communication', 'error');
      });
      
    } catch (error) {
      console.error('ERROR: Fichier JSON invalide:', error);
      showTooltip('❌ Fichier JSON invalide', 'error');
    }
  };
  
  reader.readAsText(file);
  
  // Réinitialiser l'input file
  document.getElementById('import-file').value = '';
};
