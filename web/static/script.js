// Global state
let currentBrowserPath = '/home';
let browserCallback = null;
let selectedTopics = [];
let availableTopics = [];
let bagDuration = 0.0;

// Cached DOM elements
const domCache = {
    elements: {},
    get(id) {
        if (!this.elements[id]) {
            this.elements[id] = document.getElementById(id);
        }
        return this.elements[id];
    },
    clear() {
        this.elements = {};
    }
};

// Tab Management
function openTab(tabId) {
    // Hide all tabs
    const tabs = document.querySelectorAll('.tab-content');
    tabs.forEach(tab => tab.classList.remove('active'));

    // Remove active class from all buttons
    const buttons = document.querySelectorAll('.tab-button');
    buttons.forEach(btn => btn.classList.remove('active'));

    // Show selected tab
    domCache.get(tabId).classList.add('active');

    // Activate corresponding button
    event.target.classList.add('active');

    // Scroll to top
    window.scrollTo({ top: 0, behavior: 'smooth' });

    // Select default sub-tab based on main tab
    if (tabId === 'slam-tab') {
        // Default to LiDAR SLAM sub-tab
        openSubTab('lidar-slam-subtab', true);
    } else if (tabId === 'player-tab') {
        // Default to Bag Player sub-tab
        openSubTab('bag-player-subtab', true);
        setTimeout(() => {
            if (typeof initThreeJSDisplay === 'function') {
                initThreeJSDisplay();
            }
        }, 100);
    }
}

// Sub-Tab Management (consolidated function)
function openSubTab(subtabId, skipEvent = false) {
    // Hide all sub-tabs
    const subtabs = document.querySelectorAll('.subtab-content');
    subtabs.forEach(subtab => subtab.classList.remove('active'));

    // Remove active class from all sub-tab buttons
    const buttons = document.querySelectorAll('.subtab-button');
    buttons.forEach(btn => btn.classList.remove('active'));

    // Show selected sub-tab
    const selectedSubtab = domCache.get(subtabId);
    if (selectedSubtab) {
        selectedSubtab.classList.add('active');
    }

    // Activate corresponding button
    if (!skipEvent && event && event.target) {
        event.target.classList.add('active');
    } else {
        // Find and activate corresponding button
        const correspondingButton = Array.from(buttons).find(btn =>
            btn.getAttribute('onclick') && btn.getAttribute('onclick').includes(subtabId)
        );
        if (correspondingButton) {
            correspondingButton.classList.add('active');
        }
    }

    // Scroll to top only if not called internally
    if (!skipEvent) {
        window.scrollTo({ top: 0, behavior: 'smooth' });
    }

    // Initialize or move 3D renderer to active container (for Bag Player/Recorder tabs)
    if (subtabId === 'bag-player-subtab' || subtabId === 'bag-recorder-subtab') {
        setTimeout(() => {
            if (typeof initThreeJSDisplay === 'function') {
                initThreeJSDisplay();
            }
        }, 100);
    }
}

// API Helper Functions
async function apiCall(endpoint, data = null) {
    const options = {
        method: data ? 'POST' : 'GET',
        headers: {
            'Content-Type': 'application/json',
        }
    };

    if (data) {
        options.body = JSON.stringify(data);
    }

    try {
        const response = await fetch(endpoint, options);
        return await response.json();
    } catch (error) {
        console.error('API call failed:', error);
        return { success: false, error: error.message };
    }
}

// File Browser Functions
async function openFileBrowser(callback, startPath = '/home') {
    browserCallback = callback;
    currentBrowserPath = startPath;
    await loadDirectoryList(currentBrowserPath);
    domCache.get('file-browser-modal').style.display = 'block';
}

function closeFileBrowser() {
    domCache.get('file-browser-modal').style.display = 'none';
    browserCallback = null;
}

async function loadDirectoryList(path) {
    try {
        const response = await fetch(`/api/browse?path=${encodeURIComponent(path)}`);
        const result = await response.json();

        if (result.success) {
            currentBrowserPath = result.current_path;
            domCache.get('current-path-display').textContent = result.current_path;

            const listElement = domCache.get('directory-list');
            listElement.innerHTML = '';

            result.entries.forEach(entry => {
                const div = document.createElement('div');
                div.className = 'directory-entry';

                // Add icon for directories and files
                if (entry.is_dir) {
                    div.textContent = '📁 ' + entry.name;
                    div.onclick = () => loadDirectoryList(entry.path);
                } else {
                    div.textContent = '📄 ' + entry.name;
                    div.onclick = () => selectFile(entry.path);
                    div.style.color = '#aaaaaa';
                }

                listElement.appendChild(div);
            });
        } else {
            alert('Failed to load directory: ' + (result.error || 'Unknown error'));
        }
    } catch (error) {
        console.error('Failed to load directory:', error);
        alert('Failed to load directory');
    }
}

function selectFile(filePath) {
    if (browserCallback) {
        browserCallback(filePath);
    }
    closeFileBrowser();
}

function selectCurrentDirectory() {
    if (browserCallback) {
        browserCallback(currentBrowserPath);
    }
    closeFileBrowser();
}

// SLAM GUI Functions
async function loadMap1() {
    openFileBrowser(async (path) => {
        domCache.get('slam-map1').value = path;
        const result = await apiCall('/api/slam/set_map1', { path });
        if (result.success) {
            updateSlamStatus(result.status);
        }
    }, '/home/kkw/localization_ws/src/long_term_mapping');
}

async function loadMap2() {
    openFileBrowser(async (path) => {
        domCache.get('slam-map2').value = path;
        const result = await apiCall('/api/slam/set_map2', { path });
        if (result.success) {
            updateSlamStatus(result.status);
        }
    }, '/home/kkw/localization_ws/src/long_term_mapping');
}

async function setOutput() {
    const outputField = domCache.get('slam-output');
    const directoryName = outputField.value.trim();
    if (!directoryName) {
        alert('Please enter an output directory name');
        return;
    }
    const result = await apiCall('/api/slam/set_output', { path: directoryName });
    if (result.success) {
        updateSlamStatus(result.status);
        // Keep the value in the field after setting
        outputField.value = directoryName;
    }
}

async function runOptimization() {
    const result = await apiCall('/api/slam/optimize', {});
    if (result.success) {
        updateSlamStatus(result.status);
    } else {
        alert('Failed to start optimization: ' + result.status);
    }
}

function updateSlamStatus(status) {
    domCache.get('slam-status').textContent = 'Status: ' + status;
}

async function updateSlamState() {
    const state = await apiCall('/api/slam/state');
    if (state) {
        domCache.get('slam-map1').value = state.map1 || '';
        domCache.get('slam-map2').value = state.map2 || '';

        // Only update output field if it's not currently focused (user is not typing)
        const outputField = domCache.get('slam-output');
        if (document.activeElement !== outputField) {
            outputField.value = state.output || '';
        }

        updateSlamStatus(state.status || 'Ready');
    }
}

// Bag Player Functions
async function loadBagFile() {
    openFileBrowser(async (path) => {
        domCache.get('bag-directory').value = path;
        const result = await apiCall('/api/bag/load', { path });
        if (result.success) {
            console.log('Bag file loaded successfully:', path);
            // Get topics and duration from result
            availableTopics = result.topics || [];
            selectedTopics = [...availableTopics];
            bagDuration = result.duration || 0.0;

            console.log('Loaded topics:', availableTopics);
            console.log('Duration:', bagDuration, 'seconds');

            // Update time label
            updateBagTimeLabel(0, bagDuration);

            // Update selected topics display
            updateSelectedTopicsDisplay();

            if (availableTopics.length === 0) {
                alert('No topics found in the bag file. The bag might be empty or corrupted.');
            }
        } else {
            alert('Failed to load bag file: ' + (result.message || 'Unknown error'));
        }
    }, '/home/kkw/dataset');
}

function formatTime(seconds) {
    const mins = Math.floor(seconds / 60);
    const secs = Math.floor(seconds % 60);
    return `${mins}:${secs.toString().padStart(2, '0')}`;
}

function updateBagTimeLabel(current, total) {
    const label = domCache.get('bag-time-label');
    label.textContent = `${formatTime(current)} / ${formatTime(total)}`;
}

async function selectTopics() {
    const bagPath = domCache.get('bag-directory').value;
    if (!bagPath) {
        alert('Please load a bag file first');
        return;
    }

    if (availableTopics.length === 0) {
        alert('No topics found in the bag file');
        return;
    }

    // Display topic selection modal
    const topicList = domCache.get('topic-list');
    topicList.innerHTML = '';

    availableTopics.forEach(topic => {
        const div = document.createElement('div');
        div.className = 'topic-item';

        const checkbox = document.createElement('input');
        checkbox.type = 'checkbox';
        checkbox.id = `topic-${topic}`;
        checkbox.value = topic;
        checkbox.checked = selectedTopics.includes(topic);

        const label = document.createElement('label');
        label.htmlFor = `topic-${topic}`;
        label.textContent = topic;

        div.appendChild(checkbox);
        div.appendChild(label);
        topicList.appendChild(div);
    });

    domCache.get('topic-selection-modal').style.display = 'block';
}

function closeTopicSelection() {
    domCache.get('topic-selection-modal').style.display = 'none';
}

function confirmTopicSelection() {
    // Get all checked topics
    selectedTopics = [];
    const checkboxes = document.querySelectorAll('#topic-list input[type="checkbox"]:checked');
    checkboxes.forEach(checkbox => {
        selectedTopics.push(checkbox.value);
    });

    console.log('Selected topics:', selectedTopics);

    // Update display
    updateSelectedTopicsDisplay();

    closeTopicSelection();

    if (selectedTopics.length === 0) {
        alert('Please select at least one topic');
    }
}

function updateSelectedTopicsDisplay() {
    const display = domCache.get('bag-selected-topics-display');
    if (!display) return;

    if (selectedTopics.length === 0) {
        display.innerHTML = '<span style="color: #888;">No topics selected</span>';
    } else {
        const topicsHtml = selectedTopics.map(topic =>
            `<div style="display: inline-block; background: #2a5a8a; padding: 3px 8px; margin: 2px; border-radius: 3px; font-size: 0.9em;">${topic}</div>`
        ).join('');
        display.innerHTML = topicsHtml;
    }
}

async function playBag() {
    const bagPath = domCache.get('bag-directory').value;
    if (!bagPath) {
        alert('Please load a bag file first');
        return;
    }

    const result = await apiCall('/api/bag/play', { topics: selectedTopics });
    if (result.success) {
        const button = domCache.get('bag-play-button');
        button.textContent = result.playing ? 'Stop' : 'Play';
        console.log('Bag playback:', result.playing ? 'started' : 'stopped');
    } else {
        alert('Failed to play bag file: ' + (result.message || 'Unknown error'));
    }
}

async function pauseBag() {
    const result = await apiCall('/api/bag/pause', {});
    if (result.success) {
        const button = domCache.get('bag-pause-button');
        button.textContent = result.paused ? 'Resume' : 'Pause';
        console.log('Bag playback:', result.paused ? 'paused' : 'resumed');
    } else {
        console.error('Failed to pause/resume bag');
    }
}

async function setBagPosition(position) {
    console.log('Setting bag position:', position);
    await apiCall('/api/bag/set_position', { position: parseInt(position) });

    // Update time label
    const ratio = position / 10000.0;
    const currentTime = ratio * bagDuration;
    updateBagTimeLabel(currentTime, bagDuration);
}

async function updateBagState() {
    const state = await apiCall('/api/bag/state');
    if (state) {
        // Update slider position based on current time
        if (bagDuration > 0 && state.current_time !== undefined) {
            const ratio = state.current_time / bagDuration;
            const sliderValue = Math.floor(ratio * 10000);

            // Only update slider if user is not dragging it
            const slider = domCache.get('bag-slider');
            if (document.activeElement !== slider) {
                slider.value = sliderValue;
            }

            updateBagTimeLabel(state.current_time, bagDuration);
        }

        // Update play button state
        const playButton = domCache.get('bag-play-button');
        if (state.playing) {
            playButton.textContent = 'Stop';
        } else {
            playButton.textContent = 'Play';
        }

        // Update pause button state
        const pauseButton = domCache.get('bag-pause-button');
        if (state.paused) {
            pauseButton.textContent = 'Resume';
        } else {
            pauseButton.textContent = 'Pause';
        }
    }
}

// File Player Functions
async function loadPlayerPath() {
    openFileBrowser(async (path) => {
        domCache.get('player-path-label').textContent = 'Loading...';
        const result = await apiCall('/api/player/load_data', { path });
        if (result.success) {
            domCache.get('player-path-label').textContent = path;
            console.log('Player data loaded successfully');
        } else {
            domCache.get('player-path-label').textContent = 'Failed to load';
            alert('Failed to load player data: ' + result.message);
        }
    }, '/home');
}

async function playPlayer() {
    const result = await apiCall('/api/player/play', {});
    if (result.success) {
        const button = domCache.get('play-button');
        button.textContent = result.playing ? 'End' : 'Play';
    }
}

async function pausePlayer() {
    const result = await apiCall('/api/player/pause', {});
    if (result.success) {
        const button = domCache.get('pause-button');
        button.textContent = result.paused ? 'Resume' : 'Pause';
    }
}

async function saveBag() {
    const button = event.target;
    const originalText = button.textContent;
    button.textContent = 'Converting...';
    const result = await apiCall('/api/player/save_bag', {});
    setTimeout(() => {
        button.textContent = originalText;
    }, 1000);
}

async function setSpeed(speed) {
    await apiCall('/api/player/set_speed', { speed: parseFloat(speed) });
}

async function setLoop(loop) {
    await apiCall('/api/player/set_loop', { loop });
}

async function setSkipStop(skip_stop) {
    await apiCall('/api/player/set_skip_stop', { skip_stop });
}

async function setAutoStart(auto_start) {
    await apiCall('/api/player/set_auto_start', { auto_start });
}

async function setSliderPosition(position) {
    await apiCall('/api/player/set_slider', { position: parseInt(position) });
}

async function updatePlayerState() {
    const state = await apiCall('/api/player/state');
    if (state) {
        domCache.get('player-path-label').textContent = state.path || '';
        domCache.get('player-loop').checked = state.loop || false;
        domCache.get('player-skip-stop').checked = state.skip_stop !== undefined ? state.skip_stop : true;
        domCache.get('player-auto-start').checked = state.auto_start || false;

        // Only update speed if not currently focused (user is not typing)
        const speedField = domCache.get('player-speed');
        if (document.activeElement !== speedField) {
            speedField.value = state.speed || 1.0;
        }

        domCache.get('player-slider').value = state.slider_pos || 0;
        domCache.get('player-timestamp-label').textContent = state.timestamp || 0;

        // Update button states
        if (state.playing) {
            domCache.get('play-button').textContent = 'End';
        } else {
            domCache.get('play-button').textContent = 'Play';
        }

        if (state.paused) {
            domCache.get('pause-button').textContent = 'Resume';
        } else {
            domCache.get('pause-button').textContent = 'Pause';
        }
    }
}

// Display Topic Selection Functions are now in threejs_display.js

// Bag Recorder Functions
let recorderBagName = '';
let recorderSelectedTopics = [];

async function enterBagName() {
    const bagNameInput = domCache.get('recorder-bag-name');
    const bagName = bagNameInput.value.trim();

    if (!bagName) {
        alert('Please enter a bag name');
        return;
    }

    recorderBagName = bagName;
    console.log('Bag name set:', recorderBagName);

    const result = await apiCall('/api/recorder/set_bag_name', { bag_name: bagName });
    if (result.success) {
        console.log('Bag name confirmed:', bagName);
    } else {
        alert('Failed to set bag name: ' + (result.message || 'Unknown error'));
    }
}

async function selectRecorderTopics() {
    if (!recorderBagName) {
        alert('Please enter bag name first');
        return;
    }

    // Get current ROS2 topics
    const result = await apiCall('/api/recorder/get_topics');

    if (!result.success || !result.topics || result.topics.length === 0) {
        alert('No ROS2 topics found. Make sure ROS2 nodes are running.');
        return;
    }

    // Display topic selection modal
    const topicList = domCache.get('recorder-topic-list');
    topicList.innerHTML = '';

    result.topics.forEach(topic => {
        const div = document.createElement('div');
        div.className = 'topic-item';

        const checkbox = document.createElement('input');
        checkbox.type = 'checkbox';
        checkbox.id = `recorder-topic-${topic}`;
        checkbox.value = topic;
        checkbox.checked = recorderSelectedTopics.includes(topic);

        const label = document.createElement('label');
        label.htmlFor = `recorder-topic-${topic}`;
        label.textContent = topic;

        div.appendChild(checkbox);
        div.appendChild(label);
        topicList.appendChild(div);
    });

    domCache.get('recorder-topic-modal').style.display = 'block';
}

function closeRecorderTopicSelection() {
    domCache.get('recorder-topic-modal').style.display = 'none';
}

function confirmRecorderTopicSelection() {
    // Get all checked topics
    recorderSelectedTopics = [];
    const checkboxes = document.querySelectorAll('#recorder-topic-list input[type="checkbox"]:checked');
    checkboxes.forEach(checkbox => {
        recorderSelectedTopics.push(checkbox.value);
    });

    console.log('Selected topics for recording:', recorderSelectedTopics);

    // Update display
    updateRecorderSelectedTopicsDisplay();

    closeRecorderTopicSelection();

    if (recorderSelectedTopics.length === 0) {
        alert('Please select at least one topic');
    }
}

function updateRecorderSelectedTopicsDisplay() {
    const display = domCache.get('recorder-selected-topics-display');
    if (!display) return;

    if (recorderSelectedTopics.length === 0) {
        display.innerHTML = '<span style="color: #888;">No topics selected</span>';
    } else {
        const topicsHtml = recorderSelectedTopics.map(topic =>
            `<div style="display: inline-block; background: #8a2a2a; padding: 3px 8px; margin: 2px; border-radius: 3px; font-size: 0.9em;">${topic}</div>`
        ).join('');
        display.innerHTML = topicsHtml;
    }
}

async function recordBag() {
    if (!recorderBagName) {
        alert('Please enter bag name first');
        return;
    }

    if (recorderSelectedTopics.length === 0) {
        alert('Please select topics to record');
        return;
    }

    const result = await apiCall('/api/recorder/record', { topics: recorderSelectedTopics });
    if (result.success) {
        const button = domCache.get('recorder-record-button');
        button.textContent = result.recording ? 'Stop' : 'Record';
        console.log('Recording:', result.recording ? 'started' : 'stopped');

        if (result.recording) {
            alert(`Recording started in /home/kkw/${recorderBagName}`);
        } else {
            alert('Recording stopped');
        }
    } else {
        alert('Failed to start/stop recording: ' + (result.message || 'Unknown error'));
    }
}

async function updateRecorderState() {
    const state = await apiCall('/api/recorder/state');
    if (state) {
        // Update button state
        const button = domCache.get('recorder-record-button');
        if (state.recording) {
            button.textContent = 'Stop';
        } else {
            button.textContent = 'Record';
        }
    }
}

// ==============================================================
// Generic Config Manager Class
// ==============================================================
class ConfigManager {
    constructor(name, defaultPath, containerIds, apiEndpoints) {
        this.name = name; // 'slam' or 'localization'
        this.defaultPath = defaultPath;
        this.currentPath = defaultPath;
        this.data = {};
        this.collapsed = true;
        this.containerIds = containerIds; // {parameters, container, toggleBtn}
        this.apiEndpoints = apiEndpoints; // {loadConfig, saveConfig, updateConfig}
    }

    async loadDefault() {
        this.currentPath = this.defaultPath;

        try {
            const result = await apiCall('/api/slam/load_config_file', { path: this.defaultPath });

            if (result.success && result.config) {
                console.log(`Default ${this.name} config loaded successfully`);
                this.data = result.config;
                this.display();

                // Show config container
                domCache.get(this.containerIds.container).style.display = 'block';

                // Set initial collapsed state
                const parametersDiv = domCache.get(this.containerIds.parameters);
                const toggleBtn = domCache.get(this.containerIds.toggleBtn);
                const separators = document.querySelectorAll(`#${this.containerIds.container} .separator`);

                parametersDiv.style.display = 'none';
                separators.forEach(sep => sep.style.display = 'none');
                toggleBtn.textContent = '▼';
            } else {
                console.error(`Failed to load default ${this.name} config:`, result.message);
            }
        } catch (error) {
            console.error(`Error loading default ${this.name} config:`, error);
        }
    }

    async load(startPath) {
        openFileBrowser(async (path) => {
            // Check if file has .yaml or .yml extension
            if (!path.endsWith('.yaml') && !path.endsWith('.yml')) {
                showYamlErrorModal();
                return;
            }

            // Load the selected yaml file
            const result = await apiCall('/api/slam/load_config_file', { path });

            if (result.success && result.config) {
                console.log(`${this.name} config loaded successfully from:`, path);
                this.currentPath = path;
                this.data = result.config;
                this.display();

                // Show config container
                domCache.get(this.containerIds.container).style.display = 'block';
            } else {
                alert('Failed to load config file: ' + (result.message || 'Unknown error'));
            }
        }, startPath);
    }

    async save(targetPath) {
        console.log(`Saving ${this.name} config to:`, targetPath);
        console.log('Config data:', this.data);

        const result = await apiCall('/api/slam/save_config_file', {
            path: targetPath,
            config: this.data
        });

        if (result.success) {
            alert('Config file saved successfully to:\n' + targetPath);
            console.log(`${this.name} config saved to:`, targetPath);
        } else {
            alert('Failed to save config file: ' + (result.message || 'Unknown error'));
        }
    }

    display() {
        const container = domCache.get(this.containerIds.parameters);
        container.innerHTML = '';

        // Separate top-level primitive values and nested objects
        const topLevelParams = [];
        const nestedGroups = [];

        Object.keys(this.data).forEach(key => {
            const value = this.data[key];
            if (typeof value === 'object' && value !== null && !Array.isArray(value)) {
                // This is a nested group
                nestedGroups.push({ key, data: value });
            } else {
                // This is a top-level parameter
                topLevelParams.push({ key, value });
            }
        });

        // Display top-level parameters first
        if (topLevelParams.length > 0) {
            const groupHeader = document.createElement('h4');
            groupHeader.textContent = 'General';
            groupHeader.style.marginTop = '20px';
            groupHeader.style.marginBottom = '10px';
            groupHeader.style.color = '#4a9eff';
            container.appendChild(groupHeader);

            topLevelParams.forEach(param => {
                this.createParameterInput(container, param.key, param.value, param.key);
            });
        }

        // Display nested groups
        nestedGroups.forEach(group => {
            const groupHeader = document.createElement('h4');
            // Convert snake_case to Title Case
            const title = group.key.split('_').map(word =>
                word.charAt(0).toUpperCase() + word.slice(1)
            ).join(' ');
            groupHeader.textContent = title;
            groupHeader.style.marginTop = '20px';
            groupHeader.style.marginBottom = '10px';
            groupHeader.style.color = '#4a9eff';
            container.appendChild(groupHeader);

            Object.keys(group.data).forEach(key => {
                const value = group.data[key];
                const fullKey = `${group.key}.${key}`;
                this.createParameterInput(container, key, value, fullKey);
            });
        });
    }

    createParameterInput(container, label, value, fullKey) {
        const formGroup = document.createElement('div');
        formGroup.className = 'form-group';
        formGroup.style.display = 'grid';
        formGroup.style.gridTemplateColumns = '200px 1fr';
        formGroup.style.alignItems = 'center';
        formGroup.style.marginBottom = '8px';

        const labelElement = document.createElement('label');
        labelElement.textContent = label + ':';
        labelElement.style.fontSize = '0.9em';
        formGroup.appendChild(labelElement);

        let inputElement;

        // Handle different value types
        if (typeof value === 'boolean') {
            const checkboxContainer = document.createElement('div');
            checkboxContainer.style.textAlign = 'right';
            checkboxContainer.style.paddingRight = '10px';

            inputElement = document.createElement('input');
            inputElement.type = 'checkbox';
            inputElement.checked = value;
            inputElement.id = `${this.name}-param-${fullKey}`;
            inputElement.onchange = () => this.updateValue(fullKey, inputElement.checked);

            checkboxContainer.appendChild(inputElement);
            formGroup.appendChild(checkboxContainer);
            container.appendChild(formGroup);
            return;
        } else if (Array.isArray(value)) {
            inputElement = document.createElement('input');
            inputElement.type = 'text';
            // Keep original number precision (don't round)
            const formattedValue = value.map(v => String(v));
            inputElement.value = '[' + formattedValue.join(', ') + ']';
            inputElement.id = `${this.name}-param-${fullKey}`;
            inputElement.style.width = '100%';
            inputElement.onchange = () => {
                try {
                    // Parse the input, removing spaces for flexibility
                    const cleanedInput = inputElement.value.replace(/\s/g, '');
                    const parsedValue = JSON.parse(cleanedInput);
                    this.updateValue(fullKey, parsedValue);
                } catch (e) {
                    alert('Invalid array format. Use format: [1.0, 0.0, 0.0]');
                }
            };
        } else if (typeof value === 'number') {
            inputElement = document.createElement('input');
            inputElement.type = 'number';
            inputElement.value = value;
            inputElement.id = `${this.name}-param-${fullKey}`;
            inputElement.step = 'any';  // Allow any decimal precision
            inputElement.style.width = '100%';
            inputElement.onchange = () => {
                const numValue = parseFloat(inputElement.value);
                this.updateValue(fullKey, numValue);
            };
        } else if (typeof value === 'string') {
            inputElement = document.createElement('input');
            inputElement.type = 'text';
            inputElement.value = value;
            inputElement.id = `${this.name}-param-${fullKey}`;
            inputElement.style.width = '100%';
            inputElement.onchange = () => this.updateValue(fullKey, inputElement.value);
        } else {
            inputElement = document.createElement('span');
            inputElement.textContent = String(value);
        }

        formGroup.appendChild(inputElement);
        container.appendChild(formGroup);
    }

    updateValue(key, value) {
        console.log(`Updated ${this.name} config: ${key} = ${value}`);

        // Update local config data
        const keys = key.split('.');
        let obj = this.data;

        for (let i = 0; i < keys.length - 1; i++) {
            if (!obj[keys[i]]) obj[keys[i]] = {};
            obj = obj[keys[i]];
        }

        obj[keys[keys.length - 1]] = value;

        // Send update to backend
        apiCall('/api/slam/update_config', { key, value });
    }

    toggle() {
        const parametersDiv = domCache.get(this.containerIds.parameters);
        const toggleBtn = domCache.get(this.containerIds.toggleBtn);
        const separators = document.querySelectorAll(`#${this.containerIds.container} .separator`);

        this.collapsed = !this.collapsed;

        if (this.collapsed) {
            // Collapse
            parametersDiv.style.display = 'none';
            separators.forEach(sep => sep.style.display = 'none');
            toggleBtn.textContent = '▼';
        } else {
            // Expand
            parametersDiv.style.display = 'block';
            separators.forEach(sep => sep.style.display = 'block');
            toggleBtn.textContent = '▲';
        }
    }
}

// ==============================================================
// Config Manager Instances
// ==============================================================
const slamConfig = new ConfigManager(
    'slam',
    '/home/kkw/localization_ws/src/FAST_LIO_ROS2/config/mapping_config.yaml',
    {
        parameters: 'slam-config-parameters',
        container: 'slam-config-container',
        toggleBtn: 'slam-config-toggle-btn'
    },
    {
        loadConfig: '/api/slam/load_config_file',
        saveConfig: '/api/slam/save_config_file',
        updateConfig: '/api/slam/update_config'
    }
);

const localizationConfig = new ConfigManager(
    'localization',
    '/home/kkw/localization_ws/src/FAST_LIO_ROS2/config/localization_config.yaml',
    {
        parameters: 'localization-config-parameters',
        container: 'localization-config-container',
        toggleBtn: 'localization-config-toggle-btn'
    },
    {
        loadConfig: '/api/slam/load_config_file',
        saveConfig: '/api/slam/save_config_file',
        updateConfig: '/api/slam/update_config'
    }
);

// ==============================================================
// Config Function Wrappers (for backwards compatibility with HTML)
// ==============================================================
async function loadDefaultSlamConfig() {
    await slamConfig.loadDefault();
}

async function loadSlamConfig() {
    await slamConfig.load('/home/kkw/localization_ws/src/FAST_LIO_ROS2/config');
}

async function saveSlamConfig() {
    await slamConfig.save('/home/kkw/localization_ws/src/FAST_LIO_ROS2/config/mapping_config.yaml');
}

function toggleSlamConfig() {
    slamConfig.toggle();
}

async function loadDefaultLocalizationConfig() {
    await localizationConfig.loadDefault();
}

async function loadLocalizationConfig() {
    await localizationConfig.load('/home/kkw/localization_ws/src/FAST_LIO_ROS2/config');
}

async function saveLocalizationConfig() {
    await localizationConfig.save('/home/kkw/localization_ws/src/FAST_LIO_ROS2/config/localization_config.yaml');
}

function toggleLocalizationConfig() {
    localizationConfig.toggle();
}

// ==============================================================
// SLAM Map Functions
// ==============================================================
async function saveSlamMap() {
    // Open save map modal
    domCache.get('save-map-modal').style.display = 'block';
    domCache.get('save-map-directory').value = '';
    domCache.get('save-map-directory').focus();
}

function closeSaveMapModal() {
    domCache.get('save-map-modal').style.display = 'none';
}

async function confirmSaveMap() {
    const directoryName = domCache.get('save-map-directory').value.trim();

    if (!directoryName) {
        alert('Please enter a directory name');
        return;
    }

    console.log('Saving map to directory:', directoryName);

    // Close modal
    closeSaveMapModal();

    // Call API to save map
    const result = await apiCall('/api/slam/save_map', { directory: directoryName });

    if (result.success) {
        alert('Map save request sent successfully!\nDirectory: ' + directoryName + '\n' + (result.message || ''));
        console.log('Map save result:', result.message);
    } else {
        alert('Failed to save map: ' + (result.message || 'Unknown error'));
    }
}

// ==============================================================
// SLAM Terminal Functions
// ==============================================================
let slamTerminalUpdateInterval = null;

async function startSlamMapping() {
    // Clear terminal output
    const terminalOutput = domCache.get('slam-terminal-output');
    terminalOutput.textContent = 'Starting SLAM mapping...\n';

    const result = await apiCall('/api/slam/start_mapping', {});
    if (result.success) {
        console.log('SLAM mapping started');

        // Start polling for terminal output
        if (slamTerminalUpdateInterval) {
            clearInterval(slamTerminalUpdateInterval);
        }
        slamTerminalUpdateInterval = setInterval(updateSlamTerminalOutput, 500);
    } else {
        terminalOutput.textContent += '\nFailed to start SLAM mapping: ' + (result.message || 'Unknown error');
        console.error('Failed to start SLAM mapping');
    }
}

async function updateSlamTerminalOutput() {
    const result = await apiCall('/api/slam/get_terminal_output');
    if (result.success && result.output) {
        const terminalOutput = domCache.get('slam-terminal-output');
        terminalOutput.textContent = result.output;

        // Auto-scroll to bottom
        terminalOutput.scrollTop = terminalOutput.scrollHeight;
    }
}

async function stopSlamMapping() {
    const result = await apiCall('/api/slam/stop_mapping', {});
    if (result.success) {
        const terminalOutput = domCache.get('slam-terminal-output');
        terminalOutput.textContent += '\n\n[SLAM process stopped]\n';
        console.log('SLAM mapping stopped');

        // Stop polling for terminal output
        if (slamTerminalUpdateInterval) {
            clearInterval(slamTerminalUpdateInterval);
            slamTerminalUpdateInterval = null;
        }
    } else {
        alert('Failed to stop SLAM mapping: ' + (result.message || 'Unknown error'));
        console.error('Failed to stop SLAM mapping');
    }
}

// ==============================================================
// Localization Terminal Functions
// ==============================================================
let localizationTerminalUpdateInterval = null;

async function startLocalizationMapping() {
    // Clear terminal output
    const terminalOutput = domCache.get('localization-terminal-output');
    terminalOutput.textContent = 'Starting Localization mapping...\n';

    const result = await apiCall('/api/localization/start_mapping', {});
    if (result.success) {
        console.log('Localization mapping started');

        // Start polling for terminal output
        if (localizationTerminalUpdateInterval) {
            clearInterval(localizationTerminalUpdateInterval);
        }
        localizationTerminalUpdateInterval = setInterval(updateLocalizationTerminalOutput, 500);
    } else {
        alert('Failed to start Localization mapping: ' + (result.message || 'Unknown error'));
    }
}

async function updateLocalizationTerminalOutput() {
    const result = await apiCall('/api/localization/get_terminal_output', {});

    if (result.success) {
        const terminalDiv = domCache.get('localization-terminal-output');
        terminalDiv.textContent = result.output;
        // Auto-scroll to bottom
        terminalDiv.scrollTop = terminalDiv.scrollHeight;
    }
}

async function stopLocalizationMapping() {
    console.log('Stopping Localization mapping...');
    const result = await apiCall('/api/localization/stop_mapping', {});

    if (result.success) {
        console.log('Localization mapping stopped');

        // Stop polling for terminal output
        if (localizationTerminalUpdateInterval) {
            clearInterval(localizationTerminalUpdateInterval);
            localizationTerminalUpdateInterval = null;
        }
    } else {
        alert('Failed to stop Localization mapping: ' + (result.message || 'Unknown error'));
    }
}

// ==============================================================
// Utility Functions
// ==============================================================
function showYamlErrorModal() {
    // Create modal overlay
    const overlay = document.createElement('div');
    overlay.style.position = 'fixed';
    overlay.style.top = '0';
    overlay.style.left = '0';
    overlay.style.width = '100%';
    overlay.style.height = '100%';
    overlay.style.backgroundColor = 'rgba(0, 0, 0, 0.5)';
    overlay.style.display = 'flex';
    overlay.style.alignItems = 'center';
    overlay.style.justifyContent = 'center';
    overlay.style.zIndex = '10000';

    // Create modal content
    const modal = document.createElement('div');
    modal.style.backgroundColor = '#2a2a2a';
    modal.style.padding = '30px';
    modal.style.borderRadius = '8px';
    modal.style.boxShadow = '0 4px 6px rgba(0, 0, 0, 0.3)';
    modal.style.textAlign = 'center';
    modal.style.minWidth = '300px';

    // Error message
    const message = document.createElement('p');
    message.textContent = 'yaml 파일을 선택하세요.';
    message.style.color = '#ffffff';
    message.style.fontSize = '16px';
    message.style.marginBottom = '20px';

    // OK button
    const okButton = document.createElement('button');
    okButton.textContent = 'OK';
    okButton.style.padding = '8px 30px';
    okButton.style.fontSize = '14px';
    okButton.style.cursor = 'pointer';
    okButton.onclick = () => {
        document.body.removeChild(overlay);
    };

    modal.appendChild(message);
    modal.appendChild(okButton);
    overlay.appendChild(modal);
    document.body.appendChild(overlay);
}

// ==============================================================
// Latency Measurement
// ==============================================================
async function measureLatency() {
    const latencyElement = domCache.get('latency-indicator');
    if (!latencyElement) return;

    try {
        const startTime = performance.now();
        const response = await fetch('/api/ping');
        const endTime = performance.now();

        if (response.ok) {
            const latency = Math.round(endTime - startTime);
            latencyElement.textContent = `latency: ${latency}ms`;

            // Color coding based on latency
            if (latency < 50) {
                latencyElement.style.color = '#4CAF50'; // Green
            } else if (latency < 150) {
                latencyElement.style.color = '#FFC107'; // Yellow
            } else {
                latencyElement.style.color = '#F44336'; // Red
            }
        }
    } catch (error) {
        // Connection error
        latencyElement.textContent = 'latency: N/A';
        latencyElement.style.color = '#888';
    }
}

// ==============================================================
// Initialize and periodic updates
// ==============================================================
window.addEventListener('load', () => {
    // Initial state update
    updateSlamState();
    updatePlayerState();
    updateBagState();

    // Load default configs on startup
    loadDefaultSlamConfig();
    loadDefaultLocalizationConfig();

    // Start latency measurement
    measureLatency();
    setInterval(measureLatency, 2000); // Update every 2 seconds

    // Periodic state updates (every 500ms for smoother updates)
    setInterval(() => {
        const activeTab = document.querySelector('.tab-content.active');
        if (activeTab.id === 'slam-tab') {
            const activeSubTab = document.querySelector('.subtab-content.active');
            if (activeSubTab && activeSubTab.id === 'multi-session-slam-subtab') {
                updateSlamState();
            }
        } else if (activeTab.id === 'player-tab') {
            const activeSubTab = document.querySelector('.subtab-content.active');
            if (activeSubTab && activeSubTab.id === 'bag-player-subtab') {
                updateBagState();
            } else if (activeSubTab && activeSubTab.id === 'file-player-subtab') {
                updatePlayerState();
            }
        }
    }, 500);
});

// Close modal when clicking outside
window.onclick = function(event) {
    const fileBrowserModal = domCache.get('file-browser-modal');
    const topicSelectionModal = domCache.get('topic-selection-modal');
    const displayTopicModal = domCache.get('display-topic-modal');
    const recorderTopicModal = domCache.get('recorder-topic-modal');

    if (event.target === fileBrowserModal) {
        closeFileBrowser();
    }
    if (event.target === topicSelectionModal) {
        closeTopicSelection();
    }
    if (event.target === displayTopicModal) {
        closeDisplayTopicSelection();
    }
    if (event.target === recorderTopicModal) {
        closeRecorderTopicSelection();
    }
}
