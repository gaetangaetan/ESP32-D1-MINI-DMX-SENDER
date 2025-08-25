// Noms des paramètres
const paramNames = [
  'Autopan Depth', 'Pitch', 'Vibrato Speed', 'Vibrato Depth', 'Delay Time',
  'Delay Feedback', 'OSC Waveform', 'Gate Threshold', 'Portamento Time', 'Scale',
  'Octave Low High', 'OSC 2 Volume', 'OSC 2 Pitch Offset', 'Autopan Frequency', 'Scale Tonic',
  'Volume Drums', 'Reserved 16', 'Reserved 17', 'Reserved 18', 'Master Volume (Inverted)',
  'Filter On-Off'
];

// Noms des assignations
const assignmentNames = ['Capteur IR 1', 'Capteur IR 2', 'Fader 2', 'Fader 3'];

// Gestion des vues
let currentView = 'live';

// Fonctions API
function updateParameter(paramId, value) {
  fetch('/api/parameters', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({id: parseInt(paramId), value: parseInt(value)})
  })
  .then(r => r.json())
  .then(d => {
    if (d.success) {
      showStatus('✅ Paramètre mis à jour');
    } else {
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
    if (d.success) {
      showStatus('✅ Assignation mise à jour');
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
  // Récupérer les valeurs actuelles des paramètres
  const values = [];
  for (let i = 0; i < 21; i++) {
    const slider = document.getElementById('param-' + i);
    values.push(parseInt(slider.value));
  }

  fetch('/api/save-preset', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({
      name: 'Preset ' + presetId,
      values: values
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
    body: JSON.stringify({id: parseInt(presetId)})
  })
  .then(r => r.json())
  .then(d => {
    if (d.success) {
      // Appliquer directement les valeurs du preset sans appeler l'API
      applyPresetToInterface(d.parameters);
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

// Nouvelle fonction : appliquer un preset directement à l'interface
function applyPresetToInterface(presetValues) {
  if (!presetValues || presetValues.length !== 21) {
    console.error('Valeurs de preset invalides');
    return;
  }
  
  // Appliquer chaque valeur aux curseurs
  presetValues.forEach((value, index) => {
    const slider = document.getElementById('param-' + index);
    const valueDisplay = document.getElementById('value-' + index);
    
    if (slider && valueDisplay) {
      slider.value = value;
      valueDisplay.textContent = value;
    }
  });
  
  console.log('Preset appliqué à l\'interface:', presetValues);
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
    html += 'oninput="updateParameter(' + i + ', this.value); document.getElementById(\'value-' + i + '\').textContent = this.value">';
    html += '</div>';
  }
  
  container.innerHTML = html;
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
