// Noms des paramètres
const paramNames = [
  'Autopan Depth', 'Pitch', 'Vibrato Speed', 'Vibrato Depth', 'Delay Time',
  'Delay Feedback', 'OSC Waveform', 'Gate Threshold', 'Portamento Time', 'Scale',
  'Octave Low High', 'OSC 2 Volume', 'OSC 2 Pitch Offset', 'Autopan Frequency', 'Scale Tonic',
  'Volume Drums', 'Reserved 16', 'Reserved 17', 'Reserved 18', 'Master Volume (Inverted)',
  'Filter On-Off', 'Filter Cutoff', 'Filter Reso', 'Filter Type', 'RGB Red', 'RGB Green', 'RGB Blue'
];

// Noms des assignations
const assignmentNames = ['Capteur IR 1', 'Capteur IR 2', 'Fader 2', 'Fader 3'];

// Gestion des vues
let currentView = 'live';

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

function updateAssignment(assignId, value) {
  fetch('/api/assignments', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({index: parseInt(assignId), value: parseInt(value)})
  })
  .then(r => r.json())
  .then(d => {
    // Notification supprimée pour éviter le spam
    if (!d.success) {
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
    const slider = document.getElementById('param-' + i);
    values.push(parseInt(slider.value));
  }
  
  // Paramètres filter (21-23)
  values.push(parseInt(document.getElementById('filter-cutoff').value));
  values.push(parseInt(document.getElementById('filter-reso').value));
  values.push(parseInt(document.getElementById('filter-type').value));
  
  // Paramètres RGB (24-26)
  values.push(parseInt(document.getElementById('rgb-red-value').textContent));
  values.push(parseInt(document.getElementById('rgb-green-value').textContent));
  values.push(parseInt(document.getElementById('rgb-blue-value').textContent));

  fetch('/api/save-preset', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({
      name: 'Preset ' + presetId,
      values: values,
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

function loadPreset(presetId) {
  fetch('/api/load-preset', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({id: parseInt(presetId) - 1}) // Convertir l'ID du bouton (1-8) en index de tableau (0-7)
  })
  .then(r => r.json())
  .then(d => {
    if (d.success) {
      // Appliquer directement les valeurs du preset sans appeler l'API
      applyPresetToInterface(d.parameters);
      
      // Mettre à jour l'affichage de l'octave si présente dans la réponse
      if (typeof d.octave !== 'undefined') {
        const octaveDisplay = document.getElementById('octave-display');
        if (octaveDisplay) {
          octaveDisplay.textContent = d.octave;
        }
      }
      
      // Mettre à jour les assignations si présentes dans la réponse
      if (d.assignments) {
        applyAssignmentsToInterface(d.assignments);
      }
      
      showStatus('✅ Preset ' + presetId + ' chargé');
    } else {
      showStatus('❌ Erreur chargement');
    }
  })
  .catch(e => showStatus('❌ Erreur de connexion'));
}

function loadCurrentParams() {
  fetch('/api/parameters')
  .then(r => r.json())
  .then(d => {
    d.parameters.forEach((value, index) => {
      const slider = document.getElementById('param-' + index);
      const valueDisplay = document.getElementById('value-' + index);
      if (slider && valueDisplay) {
        slider.value = value;
        valueDisplay.textContent = value;
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
  })
  .catch(e => console.error('Erreur chargement assignations:', e));
}

function applyAssignmentsToInterface(assignments) {
  if (!assignments || assignments.length !== 4) {
    console.error('Assignations invalides, attendu 4 assignations, reçu:', assignments?.length);
    return;
  }
  
  // Appliquer les assignations aux sélecteurs
  assignments.forEach((value, index) => {
    const select = document.getElementById('assignment-' + index);
    if (select) {
      select.value = value;
    }
  });
}

// Nouvelle fonction : appliquer un preset directement à l'interface
function applyPresetToInterface(presetValues) {
  if (!presetValues || presetValues.length !== 27) {
    console.error('Valeurs de preset invalides, attendu 27 paramètres, reçu:', presetValues?.length);
    return;
  }
  
  // Appliquer les paramètres principaux (0-20)
  for (let i = 0; i < 21; i++) {
    const slider = document.getElementById('param-' + i);
    const valueDisplay = document.getElementById('value-' + i);
    
    if (slider && valueDisplay) {
      slider.value = presetValues[i];
      valueDisplay.textContent = presetValues[i];
    }
  }
  
  // Appliquer les paramètres filter (21-23)
  const filterCutoff = document.getElementById('filter-cutoff');
  const filterReso = document.getElementById('filter-reso');
  const filterType = document.getElementById('filter-type');
  
  if (filterCutoff) {
    filterCutoff.value = presetValues[21];
    document.getElementById('filter-cutoff-value').textContent = presetValues[21];
    updateParameter(21, presetValues[21]);
  }
  if (filterReso) {
    filterReso.value = presetValues[22];
    document.getElementById('filter-reso-value').textContent = presetValues[22];
    updateParameter(22, presetValues[22]);
  }
  if (filterType) {
    filterType.value = presetValues[23];
    document.getElementById('filter-type-value').textContent = presetValues[23];
    updateParameter(23, presetValues[23]);
  }
  
  // Appliquer les paramètres RGB (24-26)
  const rgbRed = presetValues[24];
  const rgbGreen = presetValues[25];
  const rgbBlue = presetValues[26];
  
  updateRGBDisplay(rgbRed, rgbGreen, rgbBlue);
  updateParameter(24, rgbRed);
  updateParameter(25, rgbGreen);
  updateParameter(26, rgbBlue);
  
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

// Gestion des vues
function switchView(view) {
  currentView = view;
  
  // Masquer toutes les vues
  document.querySelectorAll('.view').forEach(v => v.classList.remove('active'));
  document.querySelectorAll('.view-btn').forEach(btn => btn.classList.remove('active'));
  
  // Afficher la vue sélectionnée
  if (view === 'live') {
    document.getElementById('liveView').classList.add('active');
    document.getElementById('liveViewBtn').classList.add('active');
  } else {
    document.getElementById('configView').classList.add('active');
    document.getElementById('configViewBtn').classList.add('active');
  }
}

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
    
    for (let j = 0; j < 21; j++) {
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
  
  for (let i = 0; i < 21; i++) {
    html += '<div class="param-control">';
    html += '<div class="param-label-wrapper">';
    html += '<span class="param-name">' + paramNames[i] + '</span>';
    html += '<span class="param-value" id="value-' + i + '">0</span>';
    html += '</div>';
    html += '<input type="range" class="slider" id="param-' + i + '" min="0" max="255" value="0" ';
    html += 'oninput="debouncedUpdateParameter(' + i + ', this.value); document.getElementById(\'value-' + i + '\').textContent = this.value">';
    html += '</div>';
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
  
  if (filterType) {
    filterType.addEventListener('input', function() {
      const value = parseInt(this.value);
      document.getElementById('filter-type-value').textContent = value;
      debouncedUpdateParameter(23, value); // Index 23 = filter_type
    });
  }
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

// Initialisation
document.addEventListener('DOMContentLoaded', function() {
  console.log('Interface DMX ESP32 initialisée');
  
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
  
  // Event listeners pour les boutons de vue
  document.getElementById('liveViewBtn').addEventListener('click', () => switchView('live'));
  document.getElementById('configViewBtn').addEventListener('click', () => switchView('config'));
  
  // Démarrer sur la vue LIVE CONTROL
  switchView('live');
});
