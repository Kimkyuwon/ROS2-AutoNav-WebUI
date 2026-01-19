// Global state - grouped by functionality
const fileBrowserState = {
    currentPath: '/home',
    callback: null
};

const bagPlayerState = {
    selectedTopics: [],
    availableTopics: [],
    bagDuration: 0.0
};

const bagRecorderState = {
    bagName: '',
    selectedTopics: []
};

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
    } else if (tabId === 'visualization-tab') {
        openSubTab('plot-subtab', true);
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

    // Initialize Plot subtab
    if (subtabId === 'plot-subtab') {
        console.log('[openSubTab] Initializing Plot subtab');
        initPlotSubtab();
    }
    
    // Initialize 3D Viewer if switching to that subtab
    if (subtabId === '3d-viewer-subtab') {
        // Wait for DOM update, then initialize
        setTimeout(() => {
            if (typeof initialize3DViewer === 'function') {
                console.log('Calling initialize3DViewer from openSubTab');
                initialize3DViewer();
            } else {
                console.warn('initialize3DViewer function not found');
            }
        }, 300);
    }
}

// Plot subtab 초기화
function initPlotSubtab() {
    if (!plotState.tree) {
        console.log('[initPlotSubtab] Initializing PlotJugglerTree');
        initPlotTree();
    }
    
    // PlotTabManager 초기화 (처음 한 번만)
    if (!plotState.plotTabManager) {
        console.log('[initPlotSubtab] Initializing PlotTabManager');
        plotState.plotTabManager = new PlotTabManager('plot-tab-bar-container', 'plot-area-container', 5.0);
        plotState.plotTabManager.init();
        
        // 드롭 존 설정 (PlotTabManager 초기화 후)
        setupPlotDropZone();
    }
    
    if (!plotState.ros) {
        console.log('[initPlotSubtab] Connecting to rosbridge');
        initRosbridge();
    } else if (plotState.ros.isConnected && plotState.topics.length === 0) {
        console.log('[initPlotSubtab] rosbridge already connected, loading topics');
        loadPlotTopics();
    }
    
    // 주기적으로 토픽 목록 갱신 시작
    startTopicRefresh();
}

// 주기적으로 토픽 목록 갱신
function startTopicRefresh() {
    // 기존 인터벌이 있으면 정리
    if (plotState.topicRefreshInterval) {
        clearInterval(plotState.topicRefreshInterval);
    }
    
    plotState.topicRefreshInterval = setInterval(() => {
        if (plotState.ros && plotState.ros.isConnected) {
            console.log('[startTopicRefresh] Refreshing topic list...');
            loadPlotTopics();
        }
    }, plotState.topicRefreshRate);
    
    console.log(`[startTopicRefresh] Started topic refresh every ${plotState.topicRefreshRate}ms`);
}

// 토픽 갱신 중지
function stopTopicRefresh() {
    if (plotState.topicRefreshInterval) {
        clearInterval(plotState.topicRefreshInterval);
        plotState.topicRefreshInterval = null;
        console.log('[stopTopicRefresh] Stopped topic refresh');
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
    fileBrowserState.callback = callback;
    fileBrowserState.currentPath = startPath;
    await loadDirectoryList(fileBrowserState.currentPath);
    domCache.get('file-browser-modal').style.display = 'block';
}

function closeFileBrowser() {
    domCache.get('file-browser-modal').style.display = 'none';
    fileBrowserState.callback = null;
}

async function loadDirectoryList(path) {
    try {
        const response = await fetch(`/api/browse?path=${encodeURIComponent(path)}`);
        const result = await response.json();

        if (result.success) {
            fileBrowserState.currentPath = result.current_path;
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
    if (fileBrowserState.callback) {
        fileBrowserState.callback(filePath);
    }
    closeFileBrowser();
}

function selectCurrentDirectory() {
    if (fileBrowserState.callback) {
        fileBrowserState.callback(fileBrowserState.currentPath);
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

        // Update Multi-Session SLAM status
        updateSlamStatus(state.status || 'Ready');
        
        // Update LiDAR SLAM status (only if LiDAR SLAM tab is active)
        const lidarSlamStatus = domCache.get('lidar-slam-status');
        if (lidarSlamStatus) {
            const lidarSlamTab = document.getElementById('lidar-slam-subtab');
            if (lidarSlamTab && lidarSlamTab.classList.contains('active')) {
                // Determine status based on SLAM state
                let statusText = 'Ready';
                if (state.is_running !== undefined) {
                    if (state.is_running) {
                        statusText = 'Running';
                    } else {
                        statusText = 'Ready';
                    }
                } else if (state.status && state.status !== 'Ready') {
                    statusText = state.status;
                }
                lidarSlamStatus.textContent = 'Status: ' + statusText;
                // Add red color for Stopping status
                if (statusText.includes('Stopping')) {
                    lidarSlamStatus.style.color = '#F44336'; // Red
                } else {
                    lidarSlamStatus.style.color = ''; // Reset to default
                }
            }
        }
        
        // Update Localization status will be handled by updateLocalizationState()
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
            bagPlayerState.availableTopics = result.topics || [];
            bagPlayerState.selectedTopics = [...bagPlayerState.availableTopics];
            bagPlayerState.bagDuration = result.duration || 0.0;

            console.log('Loaded topics:', bagPlayerState.availableTopics);
            console.log('Duration:', bagPlayerState.bagDuration, 'seconds');

            // Update time label
            updateBagTimeLabel(0, bagPlayerState.bagDuration);

            // Update selected topics display
            updateSelectedTopicsDisplay();

            if (bagPlayerState.availableTopics.length === 0) {
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

    if (bagPlayerState.availableTopics.length === 0) {
        alert('No topics found in the bag file');
        return;
    }

    // Display topic selection modal
    const topicList = domCache.get('topic-list');
    topicList.innerHTML = '';

    bagPlayerState.availableTopics.forEach(topic => {
        const div = document.createElement('div');
        div.className = 'topic-item';

        const checkbox = document.createElement('input');
        checkbox.type = 'checkbox';
        checkbox.id = `topic-${topic}`;
        checkbox.value = topic;
        checkbox.checked = bagPlayerState.selectedTopics.includes(topic);

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
    bagPlayerState.selectedTopics = [];
    const checkboxes = document.querySelectorAll('#topic-list input[type="checkbox"]:checked');
    checkboxes.forEach(checkbox => {
        bagPlayerState.selectedTopics.push(checkbox.value);
    });

    console.log('Selected topics:', bagPlayerState.selectedTopics);

    // Update display
    updateSelectedTopicsDisplay();

    closeTopicSelection();

    if (bagPlayerState.selectedTopics.length === 0) {
        alert('Please select at least one topic');
    }
}

function updateSelectedTopicsDisplay() {
    const display = domCache.get('bag-selected-topics-display');
    if (!display) return;

    if (bagPlayerState.selectedTopics.length === 0) {
        display.innerHTML = '<span style="color: #888;">No topics selected</span>';
    } else {
        const topicsHtml = bagPlayerState.selectedTopics.map(topic =>
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

    const result = await apiCall('/api/bag/play', { topics: bagPlayerState.selectedTopics });
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
    const currentTime = ratio * bagPlayerState.bagDuration;
    updateBagTimeLabel(currentTime, bagPlayerState.bagDuration);
}

async function updateBagState() {
    const state = await apiCall('/api/bag/state');
    if (state) {
        // Update slider position based on current time
        if (bagPlayerState.bagDuration > 0 && state.current_time !== undefined) {
            const ratio = state.current_time / bagPlayerState.bagDuration;
            const sliderValue = Math.floor(ratio * 10000);

            // Only update slider if user is not dragging it
            const slider = domCache.get('bag-slider');
            if (document.activeElement !== slider) {
                slider.value = sliderValue;
            }

            updateBagTimeLabel(state.current_time, bagPlayerState.bagDuration);
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

// Bag Recorder Functions

async function enterBagName() {
    const bagNameInput = domCache.get('recorder-bag-name');
    const bagName = bagNameInput.value.trim();

    if (!bagName) {
        alert('Please enter a bag name');
        return;
    }

    bagRecorderState.bagName = bagName;
    console.log('Bag name set:', bagRecorderState.bagName);

    const result = await apiCall('/api/recorder/set_bag_name', { bag_name: bagName });
    if (result.success) {
        console.log('Bag name confirmed:', bagName);
    } else {
        alert('Failed to set bag name: ' + (result.message || 'Unknown error'));
    }
}

async function selectRecorderTopics() {
    if (!bagRecorderState.bagName) {
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
        checkbox.checked = bagRecorderState.selectedTopics.includes(topic);

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
    bagRecorderState.selectedTopics = [];
    const checkboxes = document.querySelectorAll('#recorder-topic-list input[type="checkbox"]:checked');
    checkboxes.forEach(checkbox => {
        bagRecorderState.selectedTopics.push(checkbox.value);
    });

    console.log('Selected topics for recording:', bagRecorderState.selectedTopics);

    // Update display
    updateRecorderSelectedTopicsDisplay();

    closeRecorderTopicSelection();

    if (bagRecorderState.selectedTopics.length === 0) {
        alert('Please select at least one topic');
    }
}

function updateRecorderSelectedTopicsDisplay() {
    const display = domCache.get('recorder-selected-topics-display');
    if (!display) return;

    if (bagRecorderState.selectedTopics.length === 0) {
        display.innerHTML = '<span style="color: #888;">No topics selected</span>';
    } else {
        const topicsHtml = bagRecorderState.selectedTopics.map(topic =>
            `<div style="display: inline-block; background: #8a2a2a; padding: 3px 8px; margin: 2px; border-radius: 3px; font-size: 0.9em;">${topic}</div>`
        ).join('');
        display.innerHTML = topicsHtml;
    }
}

async function recordBag() {
    if (!bagRecorderState.bagName) {
        alert('Please enter bag name first');
        return;
    }

    if (bagRecorderState.selectedTopics.length === 0) {
        alert('Please select topics to record');
        return;
    }

    const result = await apiCall('/api/recorder/record', { topics: bagRecorderState.selectedTopics });
    if (result.success) {
        const button = domCache.get('recorder-record-button');
        button.textContent = result.recording ? 'Stop' : 'Record';
        console.log('Recording:', result.recording ? 'started' : 'stopped');

        if (result.recording) {
            alert(`Recording started in /home/kkw/${bagRecorderState.bagName}`);
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
// SLAM Start/Stop (terminal output removed)
// ==============================================================
async function startSlamMapping() {
    // Immediately update status to Running
    updateLidarSlamStatus('Running');
    
    const result = await apiCall('/api/slam/start_mapping', {});
    if (result.success) {
        console.log('SLAM mapping started');
        // Status will be updated by periodic updateSlamState() calls
    } else {
        alert('Failed to start SLAM mapping: ' + (result.message || 'Unknown error'));
        console.error('Failed to start SLAM mapping');
        updateLidarSlamStatus('Ready');
    }
}

async function stopSlamMapping() {
    // Immediately update status to Stopping
    updateLidarSlamStatus('Stopping...');
    
    const result = await apiCall('/api/slam/stop_mapping', {});
    if (result.success) {
        console.log('SLAM mapping stopped');
        // Wait a bit for process to fully stop, then update to Ready
        setTimeout(() => {
            updateLidarSlamStatus('Ready');
        }, 500);
    } else {
        alert('Failed to stop SLAM mapping: ' + (result.message || 'Unknown error'));
        console.error('Failed to stop SLAM mapping');
        updateLidarSlamStatus('Ready');
    }
}

function updateLidarSlamStatus(status) {
    const lidarSlamStatus = domCache.get('lidar-slam-status');
    const lidarSlamTab = document.getElementById('lidar-slam-subtab');
    if (lidarSlamStatus && lidarSlamTab && lidarSlamTab.classList.contains('active')) {
        lidarSlamStatus.textContent = 'Status: ' + status;
        // Add red color for Stopping status
        if (status.includes('Stopping')) {
            lidarSlamStatus.style.color = '#F44336'; // Red
        } else {
            lidarSlamStatus.style.color = ''; // Reset to default
        }
    }
}

function updateLocalizationStatus(status) {
    const localizationStatus = domCache.get('localization-status');
    const localizationTab = document.getElementById('localization-subtab');
    if (localizationStatus && localizationTab && localizationTab.classList.contains('active')) {
        localizationStatus.textContent = 'Status: ' + status;
        // Add red color for Stopping status
        if (status.includes('Stopping')) {
            localizationStatus.style.color = '#F44336'; // Red
        } else {
            localizationStatus.style.color = ''; // Reset to default
        }
    }
}

async function updateLocalizationState() {
    const state = await apiCall('/api/localization/state');
    if (state) {
        const localizationStatus = domCache.get('localization-status');
        if (localizationStatus) {
            const localizationTab = document.getElementById('localization-subtab');
            if (localizationTab && localizationTab.classList.contains('active')) {
                let statusText = 'Ready';
                if (state.is_running !== undefined) {
                    if (state.is_running) {
                        statusText = 'Running';
                    } else {
                        statusText = 'Ready';
                    }
                }
                localizationStatus.textContent = 'Status: ' + statusText;
                // Add red color for Stopping status
                if (statusText.includes('Stopping')) {
                    localizationStatus.style.color = '#F44336'; // Red
                } else {
                    localizationStatus.style.color = ''; // Reset to default
                }
            }
        }
    }
}

// ==============================================================
// Localization Start/Stop (terminal output removed)
// ==============================================================
async function startLocalizationMapping() {
    // Immediately update status to Running
    updateLocalizationStatus('Running');
    
    const result = await apiCall('/api/localization/start_mapping', {});
    if (result.success) {
        console.log('Localization mapping started');
        // Status will be updated by periodic updateLocalizationState() calls if implemented
    } else {
        alert('Failed to start Localization mapping: ' + (result.message || 'Unknown error'));
        console.error('Failed to start Localization mapping');
        updateLocalizationStatus('Ready');
    }
}

async function stopLocalizationMapping() {
    // Immediately update status to Stopping
    updateLocalizationStatus('Stopping...');
    
    console.log('Stopping Localization mapping...');
    const result = await apiCall('/api/localization/stop_mapping', {});

    if (result.success) {
        console.log('Localization mapping stopped');
        // Wait a bit for process to fully stop, then update to Ready
        setTimeout(() => {
            updateLocalizationStatus('Ready');
        }, 500);
    } else {
        alert('Failed to stop Localization mapping: ' + (result.message || 'Unknown error'));
        console.error('Failed to stop Localization mapping');
        updateLocalizationStatus('Ready');
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
    const latencyElement = document.getElementById('latency-indicator');
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
// Update ROS DOMAIN ID display
async function updateRosDomainId() {
    try {
        const result = await apiCall('/api/ros_domain_id');
        if (result.success && result.domain_id !== undefined) {
            const chip = domCache.get('ros-domain-chip');
            if (chip) {
                chip.textContent = `ROS DOMAIN ID: ${result.domain_id}`;
            }
        }
    } catch (error) {
        console.error('Failed to get ROS DOMAIN ID:', error);
    }
}

window.addEventListener('load', () => {
    // Initial state update
    updateSlamState();
    updateLocalizationState();
    updatePlayerState();
    updateBagState();
    updateRosDomainId(); // Update ROS DOMAIN ID display

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
            if (activeSubTab && (activeSubTab.id === 'multi-session-slam-subtab' || activeSubTab.id === 'lidar-slam-subtab')) {
                updateSlamState();
            }
        } else if (activeTab.id === 'player-tab') {
            const activeSubTab = document.querySelector('.subtab-content.active');
            if (activeSubTab && activeSubTab.id === 'bag-player-subtab') {
                updateBagState();
            } else if (activeSubTab && activeSubTab.id === 'file-player-subtab') {
                updatePlayerState();
            }
        } else if (activeTab.id === 'visualization-tab') {
            // Visualization tab - no periodic updates needed
        }
    }, 500);
});

// Simple status banner updater
// Simple status banner updater (deprecated - status banner removed)
function setRunStatus(message, level = 'success') {
    // Status banner removed - this function is kept for compatibility but does nothing
}
// Close modal when clicking outside
window.onclick = function(event) {
    const fileBrowserModal = domCache.get('file-browser-modal');
    const topicSelectionModal = domCache.get('topic-selection-modal');
    const recorderTopicModal = domCache.get('recorder-topic-modal');

    if (event.target === fileBrowserModal) {
        closeFileBrowser();
    }
    if (event.target === topicSelectionModal) {
        closeTopicSelection();
    }
    if (event.target === recorderTopicModal) {
        closeRecorderTopicSelection();
    }
}

// ==============================================================
// Plot 기능 관련 코드
// ==============================================================

// Plot 상태 관리
const plotState = {
    tree: null,
    ros: null,
    topics: [],
    topicTypes: new Map(), // topic name -> message type (Map)
    selectedTopics: new Set(), // 구독 중인 토픽들
    subscribers: new Map(), // topic -> subscriber
    messageTrees: new Map(), // topic -> message tree data
    topicNodes: new Map(), // topic -> topic node element (최상위 노드)
    topicRefreshInterval: null, // 토픽 목록 갱신 인터벌
    topicRefreshRate: 5000, // 5초마다 토픽 목록 갱신 (타임아웃 방지)
    plotTabManager: null, // PlotTabManager 인스턴스 (탭 관리)
    plottedPaths: [], // 현재 Plot에 표시된 path들 (모든 탭 공유)
    isLoadingTopics: false, // 토픽 로딩 중 플래그
    pathsRestored: false // 저장된 paths 복원 여부 (최초 1회만)
};

// Plot subscriber 키 생성 헬퍼 함수 (setupPlotDataUpdate와 동일한 형식)
function getPlotSubscriberKey(fullPath) {
    // plotState가 초기화되지 않았거나 topicTypes가 없으면 null 반환
    if (!plotState || !plotState.topicTypes) {
        return null;
    }
    
    // 토픽 목록에서 path와 매칭되는 가장 긴 토픽 찾기
    let topic = null;
    let fieldPath = null;
    let maxMatchLength = 0;
    
    for (const [topicName, topicType] of plotState.topicTypes.entries()) {
        // 토픽 이름에서 / 제거하여 비교
        const topicNameWithoutSlash = topicName.startsWith('/') ? topicName.substring(1) : topicName;
        
        // fullPath가 topicNameWithoutSlash로 시작하는지 확인
        if (fullPath.startsWith(topicNameWithoutSlash + '/') || fullPath === topicNameWithoutSlash) {
            const matchLength = topicNameWithoutSlash.length;
            if (matchLength > maxMatchLength) {
                maxMatchLength = matchLength;
                topic = topicName;
                fieldPath = fullPath.substring(matchLength + 1); // +1 for the '/'
            }
        }
    }
    
    if (!topic) {
        // topic을 찾지 못한 경우 null 반환 (setupPlotDataUpdate에서 처리)
        return null;
    }
    
    // setupPlotDataUpdate와 동일한 형식으로 키 생성
    return `${topic}_plot_${fieldPath.replace(/\//g, '_')}`;
}

// PlotJugglerTree 초기화 및 토픽 노드 생성
function initPlotTree() {
    if (!plotState.tree) {
        plotState.tree = new PlotJugglerTree('plot-tree');
        plotState.tree.init();
        console.log('[initPlotTree] PlotJugglerTree initialized');
    }
}

// 토픽 노드를 트리 최상위에 추가 (PlotJuggler 스타일)
function createTopicNodes() {
    initPlotTree();
    
    // 새로운 토픽과 기존 토픽 비교
    const newTopics = new Set(plotState.topics);
    const oldTopics = new Set(plotState.topicNodes.keys());
    
    // 삭제된 토픽 제거
    oldTopics.forEach(topic => {
        if (!newTopics.has(topic)) {
            const node = plotState.topicNodes.get(topic);
            if (node && node.parentElement) {
                node.parentElement.removeChild(node);
            }
            plotState.topicNodes.delete(topic);
            console.log(`[createTopicNodes] Removed topic node: ${topic}`);
        }
    });
    
    // 새로 추가된 토픽만 생성
    plotState.topics.forEach(topic => {
        if (!plotState.topicNodes.has(topic)) {
            // 토픽 이름에서 /를 제거
            const topicName = topic.startsWith('/') ? topic.substring(1) : topic;
            
            // 토픽 노드 생성 (비리프 노드)
            const topicNode = plotState.tree.createNode(topic, topicName, false);
            
            // 토픽 노드 클릭 이벤트 (확장/구독)
            topicNode.addEventListener('click', (e) => {
                // 확장 아이콘 클릭은 제외
                if (e.target.classList.contains('plot-tree-expand-icon')) {
                    return;
                }
                
                e.stopPropagation();
                
                // Ctrl+클릭: 복수 선택 (토글)
                if (e.ctrlKey || e.metaKey) {
                    console.log(`[createTopicNodes] Ctrl+click on topic: ${topic}`);
                    if (plotState.selectedTopics.has(topic)) {
                        unselectPlotTopic(topic);
                    } else {
                        selectPlotTopic(topic);
                    }
                } else {
                    // 일반 클릭: 해당 토픽만 선택, 다른 토픽 선택 해제
                    console.log(`[createTopicNodes] Normal click on topic: ${topic}`);
                    
                    // 이미 선택된 토픽이면 토글 (선택 해제)
                    if (plotState.selectedTopics.has(topic) && plotState.selectedTopics.size === 1) {
                        unselectPlotTopic(topic);
                    } else {
                        // 모든 토픽 선택 해제
                        const selectedTopics = Array.from(plotState.selectedTopics);
                        selectedTopics.forEach(t => unselectPlotTopic(t));
                        
                        // 해당 토픽만 선택
                        selectPlotTopic(topic);
                    }
                }
            });
            
            // 루트에 추가
            plotState.tree.rootNode.childrenContainer.appendChild(topicNode);
            plotState.topicNodes.set(topic, topicNode);
            
            console.log(`[createTopicNodes] Added new topic node: ${topic}`);
        }
    });
    
    const totalNodes = plotState.tree.rootNode.childrenContainer.children.length;
    console.log(`[createTopicNodes] Total nodes in DOM: ${totalNodes}`);
    
    // DOM에 제대로 추가되었는지 확인
    if (totalNodes > 0) {
        console.log('[createTopicNodes] First node:', plotState.tree.rootNode.childrenContainer.children[0]);
    } else {
        console.error('[createTopicNodes] No nodes in DOM! Debug info:', {
            rootNode: plotState.tree.rootNode,
            childrenContainer: plotState.tree.rootNode.childrenContainer,
            topics: plotState.topics
        });
    }
}

// rosbridge 연결
function initRosbridge() {
    if (typeof ROSLIB === 'undefined') {
        console.error('ROSLIB not loaded');
        return;
    }

    try {
        plotState.ros = new ROSLIB.Ros({
            url: 'ws://localhost:9090'
        });

        plotState.ros.on('connection', () => {
            console.log('[rosbridge] Connected to rosbridge');
            loadPlotTopics();
        });

        plotState.ros.on('error', (error) => {
            console.error('[rosbridge] Connection error:', error);
            // 연결 실패 메시지 표시
            const container = domCache.get('plot-tree');
            if (container) {
                container.innerHTML = '<div style="color: var(--warning); padding: 12px; text-align: center;">rosbridge connection failed. Make sure rosbridge is running on port 9090.</div>';
            }
        });

        plotState.ros.on('close', () => {
            console.log('[rosbridge] Connection closed. Attempting to reconnect...');
            // 연결 끊김 메시지 표시
            const container = domCache.get('plot-tree');
            if (container) {
                container.innerHTML = '<div style="color: var(--muted); padding: 12px; text-align: center;">rosbridge disconnected. Reconnecting...</div>';
            }
            setTimeout(() => {
                initRosbridge(); // 재연결 시도
            }, 3000);
        });
    } catch (error) {
        console.error('[rosbridge] Failed to initialize:', error);
    }
}

// 토픽 목록 로드 (rosbridge 사용)
async function loadPlotTopics() {
    console.log('[loadPlotTopics] Loading topics...');
    
    if (!plotState.ros || !plotState.ros.isConnected) {
        console.warn('[loadPlotTopics] rosbridge not connected');
        const container = domCache.get('plot-tree');
        if (container) {
            container.innerHTML = '<div style="color: var(--warning); padding: 12px; text-align: center;">rosbridge not connected. Waiting for connection...</div>';
        }
        return;
    }

    // 이미 로딩 중이면 스킵
    if (plotState.isLoadingTopics) {
        console.log('[loadPlotTopics] Already loading topics, skipping...');
        return;
    }

    plotState.isLoadingTopics = true;

    try {
        // 타임아웃 설정 (10초로 증가)
        const timeout = 10000;
        let timeoutId = null;
        let completed = false;

        // 타임아웃 Promise
        const timeoutPromise = new Promise((_, reject) => {
            timeoutId = setTimeout(() => {
                if (!completed) {
                    reject(new Error('Topic loading timeout'));
                }
            }, timeout);
        });

        // getTopics Promise
        const getTopicsPromise = new Promise((resolve, reject) => {
            try {
                plotState.ros.getTopics((result) => {
                    completed = true;
                    clearTimeout(timeoutId);
                    resolve(result);
                }, (error) => {
                    completed = true;
                    clearTimeout(timeoutId);
                    reject(error);
                });
            } catch (error) {
                completed = true;
                clearTimeout(timeoutId);
                reject(error);
            }
        });

        // 경쟁: getTopics vs timeout
        const result = await Promise.race([getTopicsPromise, timeoutPromise]);

        const topics = result.topics || [];
        const types = result.types || [];
        
        console.log('[loadPlotTopics] Received topics:', topics.length);
        console.log('[loadPlotTopics] Topic list:', topics);
        
        // topics와 types를 Map으로 저장 (별도 저장)
        const topicTypesMap = new Map();
        topics.forEach((name, index) => {
            topicTypesMap.set(name, types[index] || 'unknown');
        });
        plotState.topicTypes = topicTypesMap;
        
        // 이전 토픽 목록과 비교
        const oldTopicsSet = new Set(plotState.topics);
        const newTopicsSet = new Set(topics);
        
        // 추가된 토픽 찾기
        const addedTopics = topics.filter(t => !oldTopicsSet.has(t));
        if (addedTopics.length > 0) {
            console.log('[loadPlotTopics] New topics detected:', addedTopics);
        }
        
        // 제거된 토픽 찾기
        const removedTopics = plotState.topics.filter(t => !newTopicsSet.has(t));
        if (removedTopics.length > 0) {
            console.log('[loadPlotTopics] Removed topics:', removedTopics);
        }
        
        plotState.topics = topics;
        
        if (plotState.topics.length === 0) {
            const container = domCache.get('plot-tree');
            if (container) {
                container.innerHTML = '<div style="color: var(--muted); padding: 12px; text-align: center;">No topics available. Start publishing topics to see them here.</div>';
            }
        } else {
            displayTopicList();
        }
    } catch (error) {
        console.error('[loadPlotTopics] Error:', error);
        
        // 타임아웃이 발생했지만 이미 토픽 목록이 있는 경우 (기존 플롯이 동작 중)
        if (plotState.topics && plotState.topics.length > 0) {
            console.warn('[loadPlotTopics] Timeout occurred, but keeping existing topics');
            // 기존 UI 유지, 에러 메시지는 콘솔에만 출력
            return;
        }
        
        // 토픽 목록이 없는 경우에만 에러 메시지 표시
        const container = domCache.get('plot-tree');
        if (container) {
            container.innerHTML = `<div style="color: var(--danger); padding: 12px; text-align: center;">Failed to load topics: ${error.message}</div>`;
        }
    } finally {
        plotState.isLoadingTopics = false;
        
        // 토픽 로딩 완료 후 저장된 paths 복원 (최초 1회만)
        if (plotState.plotTabManager && !plotState.pathsRestored) {
            console.log('[loadPlotTopics] Restoring saved paths...');
            restoreSavedPaths();
            plotState.pathsRestored = true;
        }
    }
}

// 저장된 paths 복원 (페이지 새로고침 후)
function restoreSavedPaths() {
    if (!plotState.plotTabManager || !plotState.plotTabManager.tabs) {
        console.warn('[restoreSavedPaths] PlotTabManager not initialized');
        return;
    }

    console.log('[restoreSavedPaths] Restoring saved paths for all tabs...');
    
    // 각 탭의 savedPaths 복원
    plotState.plotTabManager.tabs.forEach((tab, tabIndex) => {
        if (tab.savedPaths && tab.savedPaths.length > 0) {
            console.log(`[restoreSavedPaths] Restoring ${tab.savedPaths.length} path(s) for tab ${tab.id}:`, tab.savedPaths);
            
            // 탭을 활성화 (plot 생성을 위해)
            plotState.plotTabManager.switchTab(tab.id);
            
            // Plot 생성
            const success = tab.plotManager.createPlot(tab.savedPaths);
            if (success) {
                console.log(`[restoreSavedPaths] Plot created for tab ${tab.id}`);
                
                // 전역 plottedPaths에 추가 (중복 제거)
                const newPaths = tab.savedPaths.filter(p => !plotState.plottedPaths.includes(p));
                plotState.plottedPaths = plotState.plottedPaths.concat(newPaths);
                console.log(`[restoreSavedPaths] Added ${newPaths.length} new path(s) to global plottedPaths`);
                
                // 각 path에 대해 실시간 데이터 업데이트 설정
                tab.savedPaths.forEach(path => {
                    // 이미 구독 중인지 확인 (setupPlotDataUpdate와 동일한 키 형식 사용)
                    const plotSubscriberKey = getPlotSubscriberKey(path);
                    if (!plotSubscriberKey || !plotState.subscribers.has(plotSubscriberKey)) {
                        console.log(`[restoreSavedPaths] Setting up data update for: ${path}`);
                        setupPlotDataUpdate(path);
                    } else {
                        console.log(`[restoreSavedPaths] Already subscribed to: ${path}`);
                    }
                });
            } else {
                console.error(`[restoreSavedPaths] Failed to create plot for tab ${tab.id}`);
            }
            
            // savedPaths 제거 (이미 복원됨)
            delete tab.savedPaths;
        }
    });
    
    // 첫 번째 탭으로 전환 (또는 활성 탭 복원)
    if (plotState.plotTabManager.tabs.length > 0) {
        const activeTabId = plotState.plotTabManager.activeTabId || plotState.plotTabManager.tabs[0].id;
        plotState.plotTabManager.switchTab(activeTabId);
        console.log(`[restoreSavedPaths] Switched to active tab: ${activeTabId}`);
    }
}

// 토픽 목록 표시 (PlotJuggler 스타일 - 통합 트리)
function displayTopicList() {
    const container = domCache.get('plot-tree');
    if (!container) {
        console.error('[displayTopicList] Container not found');
        return;
    }

    if (plotState.topics.length === 0) {
        container.innerHTML = '<div style="color: var(--muted); padding: 12px; text-align: center;">No topics available</div>';
        return;
    }

    // 기존 에러 메시지나 임시 메시지 제거
    const errorMsg = container.querySelector('div[style*="color"]');
    if (errorMsg && !errorMsg.classList.contains('plot-tree-root')) {
        errorMsg.remove();
        console.log('[displayTopicList] Removed error/info message');
    }

    // 토픽 노드 생성
    createTopicNodes();
    console.log(`[displayTopicList] Created ${plotState.topics.length} topic nodes`);
    console.log('[displayTopicList] Container:', container);
    console.log('[displayTopicList] Root children:', plotState.tree.rootNode.childrenContainer.children.length);
}

// 토픽 선택 및 구독 (PlotJuggler 스타일)
function selectPlotTopic(topic) {
    // 이미 구독 중이면 무시
    if (plotState.selectedTopics.has(topic)) {
        console.log(`[selectPlotTopic] Topic already subscribed: ${topic}`);
        return;
    }

    plotState.selectedTopics.add(topic);
    console.log(`[selectPlotTopic] Subscribing to topic: ${topic}`);

    // 토픽 노드 강조 표시 및 확장
    const topicNode = plotState.topicNodes.get(topic);
    if (topicNode) {
        topicNode.classList.add('plot-tree-topic-subscribed');
        
        // 자동으로 토픽 노드 확장 (메시지 트리 보이도록)
        if (!topicNode.classList.contains('plot-tree-expanded')) {
            plotState.tree.toggleExpand(topicNode);
        }
    }

    // 토픽 구독
    subscribeToTopic(topic);
}

// 토픽 구독 해제
function unselectPlotTopic(topic) {
    if (!plotState.selectedTopics.has(topic)) {
        return;
    }

    plotState.selectedTopics.delete(topic);
    
    // 구독 해제
    if (plotState.subscribers.has(topic)) {
        plotState.subscribers.get(topic).unsubscribe();
        plotState.subscribers.delete(topic);
    }
    
    // 토픽 노드 강조 해제
    const topicNode = plotState.topicNodes.get(topic);
    if (topicNode) {
        topicNode.classList.remove('plot-tree-topic-subscribed');
    }
    
    console.log(`[unselectPlotTopic] Unsubscribed from topic: ${topic}`);
}

// 토픽 구독
function subscribeToTopic(topic) {
    if (!plotState.ros || !plotState.ros.isConnected) {
        console.error('[subscribeToTopic] rosbridge not connected');
        return;
    }

    // 기존 구독 해제
    if (plotState.subscribers.has(topic)) {
        console.log(`[subscribeToTopic] Unsubscribing from existing: ${topic}`);
        plotState.subscribers.get(topic).unsubscribe();
        plotState.subscribers.delete(topic);
    }

    // 토픽 타입 조회 (plotState.topicTypes에서 가져오기)
    const messageType = plotState.topicTypes.get(topic);
    
    if (!messageType) {
        console.error(`[subscribeToTopic] Topic type not found for: ${topic}`);
        console.log('[subscribeToTopic] Available types:', Array.from(plotState.topicTypes.keys()).slice(0, 5));
        return;
    }

    console.log(`[subscribeToTopic] Subscribing to ${topic} (${messageType})`);

    // 메시지 구독
    const listener = new ROSLIB.Topic({
        ros: plotState.ros,
        name: topic,
        messageType: messageType
    });

    listener.subscribe((message) => {
        // 첫 메시지만 로그 출력
        if (!plotState.messageTrees.has(topic)) {
            console.log(`[subscribeToTopic] First message received for ${topic}`);
        }
        updateMessageTree(topic, message);
    });

    plotState.subscribers.set(topic, listener);
    console.log(`[subscribeToTopic] Successfully subscribed to ${topic}`);
}

// 메시지 트리 업데이트 (PlotJuggler 스타일 - 토픽 하위에 추가)
function updateMessageTree(topic, message) {
    if (!plotState.tree) {
        initPlotTree();
    }

    // 토픽 노드 가져오기
    const topicNode = plotState.topicNodes.get(topic);
    if (!topicNode) {
        console.error(`[updateMessageTree] Topic node not found: ${topic}`);
        return;
    }

    // PlotJuggler 스타일로 메시지를 재귀적으로 flatten
    const flattenedData = new Map();
    
    function flattenMessage(obj, prefix = '') {
        if (obj === null || obj === undefined) {
            return;
        }

        if (Array.isArray(obj)) {
            // 배열인 경우: 각 요소를 인덱스로 접근
            if (obj.length > 0) {
                if (typeof obj[0] === 'object' && obj[0] !== null) {
                    // 객체 배열: 첫 번째 요소만 파싱 (PlotJuggler 스타일)
                    flattenMessage(obj[0], prefix ? `${prefix}[0]` : '[0]');
                } else {
                    // 기본 타입 배열: 첫 번째 값만 표시
                    flattenedData.set(prefix, obj[0]);
                }
            }
        } else if (typeof obj === 'object') {
            // 객체인 경우: 각 키를 재귀적으로 처리
            Object.keys(obj).forEach(key => {
                const value = obj[key];
                const newPath = prefix ? `${prefix}/${key}` : key;
                
                if (value === null || value === undefined) {
                    // null/undefined는 건너뛰기
                    return;
                } else if (Array.isArray(value)) {
                    // 배열 필드
                    if (value.length > 0) {
                        if (typeof value[0] === 'object' && value[0] !== null) {
                            // 객체 배열: 첫 번째 요소만 파싱
                            flattenMessage(value[0], `${newPath}[0]`);
                        } else {
                            // 기본 타입 배열: 첫 번째 값만 표시 (리프 노드)
                            flattenedData.set(newPath, value[0]);
                        }
                    } else {
                        // 빈 배열은 건너뛰기
                        return;
                    }
                } else if (typeof value === 'object') {
                    // 중첩 객체: 재귀적으로 처리
                    flattenMessage(value, newPath);
                } else {
                    // 리프 노드 (기본 타입: number, string, boolean)
                    flattenedData.set(newPath, value);
                }
            });
        } else {
            // 기본 타입 (number, string, boolean)
            flattenedData.set(prefix, obj);
        }
    }

    // 메시지 flatten (prefix는 빈 문자열로 시작, 나중에 토픽 이름 추가)
    const topicName = topic.startsWith('/') ? topic.substring(1) : topic;
    flattenMessage(message, '');

    console.log(`[updateMessageTree] Topic: ${topic}, Flattened items: ${flattenedData.size}`);
    if (flattenedData.size === 0) {
        console.warn(`[updateMessageTree] No flattened data for topic: ${topic}`);
        return;
    }

    // 트리 재구성 (첫 메시지인 경우에만)
    const isFirstMessage = plotState.messageTrees.get(topic) === undefined;
    
    if (isFirstMessage) {
        // 첫 메시지: 트리 구조 생성 (토픽 노드 하위에 추가)
        console.log(`[updateMessageTree] First message for ${topic}, building tree structure...`);
        
        flattenedData.forEach((value, path) => {
            // 전체 경로: topic/path
            const fullPath = `${topicName}/${path}`;
            
            // 경로를 /로 분리
            const parts = path.split('/').filter(p => p.length > 0);
            let currentParent = topicNode;
            let currentPath = topicName;

            for (let i = 0; i < parts.length; i++) {
                const part = parts[i];
                const isLeaf = (i === parts.length - 1);
                currentPath = `${currentPath}/${part}`;

                let child = plotState.tree.findChildByName(currentParent, part);

                if (!child) {
                    child = plotState.tree.createNode(part, currentPath, isLeaf);
                    currentParent.childrenContainer.appendChild(child);
                }

                currentParent = child;
            }

            // 리프 노드인 경우 값 업데이트
            if (currentParent && currentParent.valueElement) {
                plotState.tree.updateValue(currentPath, value);
            }
        });
        
        plotState.messageTrees.set(topic, true);
        
        // 토픽 노드 자동 확장
        if (topicNode.childrenContainer.style.display === 'none' || topicNode.childrenContainer.style.display === '') {
            plotState.tree.toggleExpand(topicNode);
        }
        
        // 디버깅: 트리 상태 확인
        console.log(`[updateMessageTree] First message processed for ${topic}`);
    } else {
        // 이후 메시지: 값만 업데이트
        flattenedData.forEach((value, path) => {
            const fullPath = `${topicName}/${path}`;
            plotState.tree.updateValue(fullPath, value);
        });
    }
    
    const leafNodeCount = Array.from(plotState.tree.nodeMap.values()).filter(n => n.dataset.isLeaf === 'true').length;
    console.log(`[updateMessageTree] Tree update complete. Total leaf nodes: ${leafNodeCount}`);
}

// 트리 전체 확장
function expandAllPlotTree() {
    if (plotState.tree) {
        plotState.tree.expandAll();
        console.log('[expandAllPlotTree] All nodes expanded');
    }
}

// 트리 전체 축소
function collapseAllPlotTree() {
    if (plotState.tree) {
        plotState.tree.collapseAll();
        console.log('[collapseAllPlotTree] All nodes collapsed');
    }
}

// 트리 필터링 (검색)

function filterPlotTree(searchText) {
    if (!plotState.tree) {
        return;
    }
    
    const lowerSearch = searchText.toLowerCase();
    
    // 모든 노드를 확인하여 필터링
    plotState.tree.nodeMap.forEach((node, path) => {
        const nodeName = node.querySelector('.plot-tree-label')?.textContent || '';
        const isMatch = nodeName.toLowerCase().includes(lowerSearch) || path.toLowerCase().includes(lowerSearch);
        
        if (searchText === '') {
            // 검색어가 없으면 모두 표시
            node.style.display = '';
        } else if (isMatch) {
            // 매칭되면 표시 및 부모 노드들도 표시
            node.style.display = '';
            
            // 토픽 노드 자체가 매칭된 경우: 하위 노드 확장
            if (plotState.topicNodes && plotState.topicNodes.has(path)) {
                const topicNode = plotState.topicNodes.get(path);
                if (topicNode && topicNode.childrenContainer && 
                    (topicNode.childrenContainer.style.display === 'none' || topicNode.childrenContainer.style.display === '')) {
                    plotState.tree.toggleExpand(topicNode);
                }
            }
            
            // 부모 노드들 표시 및 확장
            let parent = node.parentElement;
            while (parent && parent.classList.contains('plot-tree-children')) {
                parent.style.display = 'block';
                // 부모 노드를 찾아서 표시 및 확장 (children -> node)
                const parentNode = parent.parentElement;
                if (parentNode && parentNode.classList.contains('plot-tree-node')) {
                    // 부모 노드도 표시
                    parentNode.style.display = '';
                    // 부모 노드가 확장되지 않았으면 확장
                    if (parentNode.childrenContainer && 
                        (parentNode.childrenContainer.style.display === 'none' || parentNode.childrenContainer.style.display === '')) {
                        plotState.tree.toggleExpand(parentNode);
                    }
                }
                parent = parentNode?.parentElement; // node -> children
            }
        } else {
            // 매칭되지 않으면 숨김
            node.style.display = 'none';
        }
    });
    
    console.log(`[filterPlotTree] Filtered with: "${searchText}"`);
}

// 버퍼 시간 업데이트
function updateBufferTime(seconds) {
    const bufferTime = parseFloat(seconds);
    
    // 유효성 검사
    if (isNaN(bufferTime) || bufferTime < 1 || bufferTime > 100) {
        console.error('[updateBufferTime] Invalid buffer time:', seconds);
        alert('Buffer time must be between 1 and 100 seconds');
        // 기본값으로 복원
        document.getElementById('buffer-time-input').value = 5;
        return;
    }
    
    console.log(`[updateBufferTime] Setting buffer time to ${bufferTime} seconds`);
    
    // PlotTabManager가 초기화되어 있으면 모든 탭의 버퍼 시간 업데이트
    if (plotState.plotTabManager) {
        plotState.plotTabManager.setBufferTime(bufferTime);
    }
}

// Plot 영역 드롭 이벤트 처리
let isPlotDropZoneSetup = false;  // 중복 등록 방지 플래그

function setupPlotDropZone() {
    const plotAreaContainer = document.getElementById('plot-area-container');
    if (!plotAreaContainer) {
        console.warn('plot-area-container element not found');
        return;
    }

    // 이미 설정되었으면 스킵
    if (isPlotDropZoneSetup) {
        console.log('[setupPlotDropZone] Already setup, skipping...');
        return;
    }

    console.log('[setupPlotDropZone] Setting up drop zone...');
    isPlotDropZoneSetup = true;

    plotAreaContainer.addEventListener('dragover', (e) => {
        e.preventDefault();
        e.stopPropagation();
        e.dataTransfer.dropEffect = 'move';
        plotAreaContainer.style.backgroundColor = 'rgba(74, 214, 255, 0.1)';
        plotAreaContainer.style.border = '2px dashed rgba(74, 214, 255, 0.5)';
    });

    plotAreaContainer.addEventListener('dragleave', (e) => {
        e.preventDefault();
        e.stopPropagation();
        // plot-area-container 내부의 자식 요소로 이동한 경우는 제외
        if (!plotAreaContainer.contains(e.relatedTarget)) {
            plotAreaContainer.style.backgroundColor = 'transparent';
            plotAreaContainer.style.border = 'none';
        }
    });

    plotAreaContainer.addEventListener('drop', (e) => {
        e.preventDefault();
        e.stopPropagation();
        plotAreaContainer.style.backgroundColor = 'transparent';
        plotAreaContainer.style.border = 'none';

        try {
            const data = e.dataTransfer.getData('text/plain');
            if (!data) {
                console.warn('No data in drop event');
                return;
            }

            // JSON 배열로 파싱 시도
            let paths = [];
            try {
                paths = JSON.parse(data);
                if (!Array.isArray(paths)) {
                    paths = [paths]; // 단일 값인 경우 배열로 변환
                }
            } catch (parseError) {
                // JSON이 아닌 경우 단일 문자열로 처리
                paths = [data];
            }

            console.log('[setupPlotDropZone] Dropped paths:', paths);
            console.log('[setupPlotDropZone] Current plotState.plottedPaths BEFORE:', plotState.plottedPaths);

            if (paths.length === 0) {
                console.warn('[setupPlotDropZone] No paths to plot');
                return;
            }

            // PlotTabManager가 초기화되어 있는지 확인
            if (!plotState.plotTabManager) {
                console.error('[setupPlotDropZone] PlotTabManager not initialized');
                return;
            }

            // 활성 탭의 PlotlyPlotManager 가져오기
            const plotManager = plotState.plotTabManager.getActivePlotManager();
            if (!plotManager) {
                console.error('[setupPlotDropZone] No active plot manager');
                return;
            }

            // Plot 생성 (모든 paths 전달 - createPlot이 내부에서 중복 처리)
            const success = plotManager.createPlot(paths);
            if (success) {
                // 기존 paths에 새로운 paths만 추가 (중복 제거)
                const newPaths = paths.filter(p => !plotState.plottedPaths.includes(p));
                console.log('[setupPlotDropZone] New paths to add:', newPaths);
                console.log('[setupPlotDropZone] Filtered out (already exists):', paths.filter(p => plotState.plottedPaths.includes(p)));
                
                plotState.plottedPaths = plotState.plottedPaths.concat(newPaths);
                console.log('[setupPlotDropZone] Plot created/updated. Total paths AFTER:', plotState.plottedPaths);
                
                // 새로운 path에 대해서만 실시간 데이터 업데이트 설정
                newPaths.forEach(path => {
                    // 이미 구독 중인지 확인 (setupPlotDataUpdate와 동일한 키 형식 사용)
                    const plotSubscriberKey = getPlotSubscriberKey(path);
                    if (!plotSubscriberKey || !plotState.subscribers.has(plotSubscriberKey)) {
                        setupPlotDataUpdate(path);
                    } else {
                        console.log(`[setupPlotDropZone] Already subscribed to: ${path}`);
                    }
                });
                
                // 탭 상태 저장
                if (plotState.plotTabManager) {
                    plotState.plotTabManager.saveState();
                }
            } else {
                console.error('[setupPlotDropZone] Failed to create plot');
            }
        } catch (error) {
            console.error('[setupPlotDropZone] Error handling drop event:', error);
        }
    });
}

// Plot 데이터 실시간 업데이트 설정
function setupPlotDataUpdate(fullPath) {
    console.log('[setupPlotDataUpdate] Setting up data update for:', fullPath);
    
    // fullPath에서 토픽과 필드 경로 분리
    // 토픽 목록에서 가장 긴 매칭을 찾음 (예: "imu/data/orientation/x" -> topic: "/imu/data", field: "orientation/x")
    const parts = fullPath.split('/').filter(p => p.length > 0);
    if (parts.length < 2) {
        console.warn('[setupPlotDataUpdate] Invalid path:', fullPath);
        return;
    }
    
    // 토픽 목록에서 path와 매칭되는 가장 긴 토픽 찾기
    let topic = null;
    let fieldPath = null;
    let maxMatchLength = 0;
    
    for (const [topicName, topicType] of plotState.topicTypes.entries()) {
        // 토픽 이름에서 / 제거하여 비교
        const topicNameWithoutSlash = topicName.startsWith('/') ? topicName.substring(1) : topicName;
        
        // fullPath가 topicNameWithoutSlash로 시작하는지 확인
        if (fullPath.startsWith(topicNameWithoutSlash + '/') || fullPath === topicNameWithoutSlash) {
            const matchLength = topicNameWithoutSlash.length;
            if (matchLength > maxMatchLength) {
                maxMatchLength = matchLength;
                topic = topicName;
                fieldPath = fullPath.substring(matchLength + 1); // +1 for the '/'
            }
        }
    }
    
    if (!topic) {
        console.error('[setupPlotDataUpdate] No matching topic found for path:', fullPath);
        console.log('[setupPlotDataUpdate] Available topics:', Array.from(plotState.topicTypes.keys()));
        return;
    }
    
    console.log('[setupPlotDataUpdate] Topic:', topic, 'Field path:', fieldPath);
    
    // Plot 전용 subscriber 키
    const plotSubscriberKey = `${topic}_plot_${fieldPath.replace(/\//g, '_')}`;
    
    if (plotState.subscribers.has(plotSubscriberKey)) {
        console.log('[setupPlotDataUpdate] Plot subscriber already exists for:', plotSubscriberKey);
        return;
    }
    
    // Topic 정보 조회 (메시지 타입 확인)
    const topicType = plotState.topicTypes.get(topic);
    if (!topicType) {
        console.error('[setupPlotDataUpdate] Topic type not found:', topic);
        console.log('[setupPlotDataUpdate] Available topics:', Array.from(plotState.topicTypes.keys()));
        return;
    }
    
    console.log('[setupPlotDataUpdate] Creating subscriber for topic:', topic, 'type:', topicType);
    
    // Plot 전용 subscriber 생성
    const plotSubscriber = new window.ROSLIB.Topic({
        ros: plotState.ros,
        name: topic,
        messageType: topicType
    });
    
    let messageCount = 0;
    
    // Subscribe 전에 연결 상태 확인
    console.log(`[setupPlotDataUpdate] Subscribing to ${topic}... Waiting for messages...`);
    
    plotSubscriber.subscribe((message) => {
        messageCount++;
        
        // 처음 메시지 수신 시 알림
        if (messageCount === 1) {
            console.log(`✅ [setupPlotDataUpdate] First message received from ${topic}!`);
        }
        
        // 필드 경로를 따라가서 값 추출
        const value = extractFieldValue(message, fieldPath);
        
        if (messageCount <= 5) {
            console.log(`[setupPlotDataUpdate] Message #${messageCount} for ${topic}:`, message);
            console.log(`[setupPlotDataUpdate] Extracted value for ${fieldPath}:`, value);
        }
        
        if (value === undefined || value === null) {
            if (messageCount <= 5) {
                console.warn('[setupPlotDataUpdate] Failed to extract value from message');
            }
            return;
        }
        
        // timestamp 추출 (header.stamp 또는 현재 시간)
        let timestamp = Date.now() / 1000.0; // 기본값: 현재 시간 (초 단위)
        
        if (message.header && message.header.stamp) {
            timestamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9;
        }
        
        if (messageCount <= 5) {
            console.log(`[setupPlotDataUpdate] Timestamp:`, timestamp, 'Value:', value);
        }
        
        // PlotlyPlotManager 업데이트 (모든 탭에 전달)
        if (plotState.plotTabManager && plotState.plotTabManager.tabs.length > 0) {
            if (messageCount <= 5) {
                console.log(`[setupPlotDataUpdate] Calling updatePlot("${fullPath}", ${timestamp}, ${value}) on all tabs`);
            }
            
            // 모든 탭의 plotManager에 데이터 추가
            plotState.plotTabManager.tabs.forEach(tab => {
                if (tab.plotManager && tab.plotManager.dataBuffers.has(fullPath)) {
                    tab.plotManager.updatePlot(fullPath, timestamp, value);
                }
            });
        } else {
            if (messageCount === 1) {
                console.warn('[setupPlotDataUpdate] plotTabManager is null or has no tabs!');
            }
        }
    });
    
    plotState.subscribers.set(plotSubscriberKey, plotSubscriber);
    console.log('[setupPlotDataUpdate] Plot subscriber created successfully:', plotSubscriberKey);
}

// 필드 경로를 따라가서 값 추출
function extractFieldValue(obj, fieldPath) {
    const fields = fieldPath.split('/');
    let value = obj;
    
    for (const field of fields) {
        if (value === null || value === undefined) {
            return undefined;
        }
        
        // 배열 인덱스 처리 (예: "covariance[0]")
        const arrayMatch = field.match(/^(\w+)\[(\d+)\]$/);
        if (arrayMatch) {
            const arrayName = arrayMatch[1];
            const index = parseInt(arrayMatch[2], 10);
            value = value[arrayName];
            if (Array.isArray(value)) {
                value = value[index];
            } else {
                return undefined;
            }
        } else {
            value = value[field];
        }
    }
    
    // 숫자 값만 반환 (Plot에 표시 가능)
    if (typeof value === 'number') {
        return value;
    } else if (typeof value === 'boolean') {
        return value ? 1 : 0;
    } else {
        console.warn('[extractFieldValue] Non-numeric value:', value);
        return undefined;
    }
}

// XY Plot 생성 함수 (PlotJugglerTree 컨텍스트 메뉴에서 호출)
function createXYPlot(xPath, yPath) {
    console.log('[createXYPlot] Creating XY Plot:', xPath, 'vs', yPath);
    
    // PlotTabManager가 초기화되어 있는지 확인
    if (!plotState.plotTabManager) {
        console.error('[createXYPlot] PlotTabManager not initialized');
        return;
    }
    
    // 활성 탭의 PlotlyPlotManager 가져오기
    const plotManager = plotState.plotTabManager.getActivePlotManager();
    if (!plotManager) {
        console.error('[createXYPlot] No active plot manager');
        return;
    }
    
    // XY Plot 생성
    const success = plotManager.createXYPlot(xPath, yPath);
    if (success) {
        console.log('[createXYPlot] XY Plot created successfully');
        
        // 전역 plottedPaths에 추가 (중복 제거)
        const paths = [xPath, yPath];
        const newPaths = paths.filter(p => !plotState.plottedPaths.includes(p));
        plotState.plottedPaths = plotState.plottedPaths.concat(newPaths);
        
        // 실시간 데이터 업데이트 설정
        paths.forEach(path => {
            const plotSubscriberKey = getPlotSubscriberKey(path);
            if (!plotSubscriberKey || !plotState.subscribers.has(plotSubscriberKey)) {
                setupPlotDataUpdate(path);
            }
        });
        
        // 탭 상태 저장
        plotState.plotTabManager.saveState();
    } else {
        console.error('[createXYPlot] Failed to create XY Plot');
    }
}

// ==============================================================
// Plot Settings 관련 전역 함수들
// ==============================================================
let currentPlotSettingsPlotId = null;

// Plot Settings 모달 열기
window.openPlotSettings = function(plotId) {
    console.log('[openPlotSettings] Opening settings for plot:', plotId);
    
    currentPlotSettingsPlotId = plotId;
    
    // 현재 플롯의 PlotlyPlotManager 가져오기
    const plotManager = plotState.plotTabManager.getPlotManager(plotId);
    if (!plotManager || !plotManager.isInitialized) {
        console.error('[openPlotSettings] Plot manager not found or not initialized:', plotId);
        return;
    }
    
    // Trace 선택 드롭다운 채우기
    const traceSelect = domCache.get('plot-settings-trace-select');
    if (!traceSelect) {
        console.error('[openPlotSettings] Trace select element not found');
        return;
    }
    
    traceSelect.innerHTML = '';
    plotManager.traces.forEach((trace, index) => {
        const option = document.createElement('option');
        option.value = index;
        option.textContent = trace.name || `Trace ${index + 1}`;
        traceSelect.appendChild(option);
    });
    
    // 첫 번째 trace가 있으면 선택
    if (plotManager.traces.length > 0) {
        traceSelect.value = 0;
        loadTraceSettings(plotManager, 0);
    }
    
    // Trace 선택 변경 시 현재 설정 로드
    traceSelect.onchange = () => {
        const selectedIndex = parseInt(traceSelect.value);
        loadTraceSettings(plotManager, selectedIndex);
    };
    
    // 슬라이더 값 업데이트 이벤트
    const lineWidthSlider = domCache.get('plot-settings-line-width');
    const lineWidthValue = domCache.get('plot-settings-line-width-value');
    if (lineWidthSlider && lineWidthValue) {
        lineWidthSlider.oninput = () => {
            lineWidthValue.textContent = lineWidthSlider.value;
        };
    }
    
    const markerSizeSlider = domCache.get('plot-settings-marker-size');
    const markerSizeValue = domCache.get('plot-settings-marker-size-value');
    if (markerSizeSlider && markerSizeValue) {
        markerSizeSlider.oninput = () => {
            markerSizeValue.textContent = markerSizeSlider.value;
        };
    }
    
    // 모달 표시
    const modal = domCache.get('plot-settings-modal');
    if (modal) {
        modal.style.display = 'flex';
    }
};

// 현재 trace의 설정 로드
function loadTraceSettings(plotManager, traceIndex) {
    const trace = plotManager.traces[traceIndex];
    if (!trace) return;
    
    // 색상
    const colorInput = domCache.get('plot-settings-color');
    if (colorInput && trace.line && trace.line.color) {
        colorInput.value = trace.line.color;
    }
    
    // 선 스타일
    const lineStyleSelect = domCache.get('plot-settings-line-style');
    if (lineStyleSelect && trace.line && trace.line.dash) {
        lineStyleSelect.value = trace.line.dash;
    }
    
    // 마커 스타일
    const markerStyleSelect = domCache.get('plot-settings-marker-style');
    if (markerStyleSelect) {
        if (trace.mode === 'lines') {
            markerStyleSelect.value = 'none';
        } else if (trace.marker && trace.marker.symbol) {
            markerStyleSelect.value = trace.marker.symbol;
        }
    }
    
    // 선 두께
    const lineWidthSlider = domCache.get('plot-settings-line-width');
    const lineWidthValue = domCache.get('plot-settings-line-width-value');
    if (lineWidthSlider && trace.line && trace.line.width) {
        lineWidthSlider.value = trace.line.width;
        if (lineWidthValue) {
            lineWidthValue.textContent = trace.line.width;
        }
    }
    
    // 마커 크기
    const markerSizeSlider = domCache.get('plot-settings-marker-size');
    const markerSizeValue = domCache.get('plot-settings-marker-size-value');
    if (markerSizeSlider && trace.marker && trace.marker.size) {
        markerSizeSlider.value = trace.marker.size;
        if (markerSizeValue) {
            markerSizeValue.textContent = trace.marker.size;
        }
    }
    
    // 그리드 표시 (layout 설정)
    const showGridCheckbox = domCache.get('plot-settings-show-grid');
    if (showGridCheckbox && plotManager.layout) {
        const showGrid = plotManager.layout.xaxis?.showgrid !== false;
        showGridCheckbox.checked = showGrid;
    }
    
    // X축 라벨
    const xaxisLabelInput = domCache.get('plot-settings-xaxis-label');
    if (xaxisLabelInput && plotManager.layout && plotManager.layout.xaxis) {
        xaxisLabelInput.value = plotManager.layout.xaxis.title?.text || '';
    }
    
    // Y축 라벨
    const yaxisLabelInput = domCache.get('plot-settings-yaxis-label');
    if (yaxisLabelInput && plotManager.layout && plotManager.layout.yaxis) {
        yaxisLabelInput.value = plotManager.layout.yaxis.title?.text || '';
    }
}

// Plot Settings 모달 닫기
window.closePlotSettings = function() {
    console.log('[closePlotSettings] Closing settings modal');
    
    const modal = domCache.get('plot-settings-modal');
    if (modal) {
        modal.style.display = 'none';
    }
    
    currentPlotSettingsPlotId = null;
};

// Plot Settings 적용
window.applyPlotSettings = function() {
    console.log('[applyPlotSettings] Applying settings');
    
    if (!currentPlotSettingsPlotId) {
        console.error('[applyPlotSettings] No plot ID set');
        return;
    }
    
    // 현재 플롯의 PlotlyPlotManager 가져오기
    const plotManager = plotState.plotTabManager.getPlotManager(currentPlotSettingsPlotId);
    if (!plotManager || !plotManager.isInitialized) {
        console.error('[applyPlotSettings] Plot manager not found or not initialized');
        return;
    }
    
    // 모든 설정 값 읽기
    const traceIndex = parseInt(domCache.get('plot-settings-trace-select')?.value || 0);
    const color = domCache.get('plot-settings-color')?.value;
    const lineStyle = domCache.get('plot-settings-line-style')?.value;
    const markerStyle = domCache.get('plot-settings-marker-style')?.value;
    const lineWidth = parseFloat(domCache.get('plot-settings-line-width')?.value);
    const markerSize = parseFloat(domCache.get('plot-settings-marker-size')?.value);
    const showGrid = domCache.get('plot-settings-show-grid')?.checked;
    const xaxisLabel = domCache.get('plot-settings-xaxis-label')?.value;
    const yaxisLabel = domCache.get('plot-settings-yaxis-label')?.value;
    
    // 설정 객체 생성
    const settings = {
        traceIndex,
        color,
        lineStyle,
        markerStyle,
        lineWidth,
        markerSize,
        showGrid,
        xaxisLabel,
        yaxisLabel
    };
    
    // PlotlyPlotManager의 applyTraceSettings() 메서드 호출
    plotManager.applyTraceSettings(settings);
    
    // 모달 닫기
    window.closePlotSettings();
};

// 모달 외부 클릭 시 닫기
window.addEventListener('click', (event) => {
    const modal = domCache.get('plot-settings-modal');
    if (event.target === modal) {
        window.closePlotSettings();
    }
});

// ==============================================================
// 페이지 로드 시 초기화
// ==============================================================
document.addEventListener('DOMContentLoaded', () => {
    console.log('[DOMContentLoaded] Page loaded');
    
    // Visualization 탭의 Plot subtab이 기본 활성화되어 있으면 초기화
    setTimeout(() => {
        const visualizationTab = domCache.get('visualization-tab');
        const plotSubtab = domCache.get('plot-subtab');
        
        if (visualizationTab && visualizationTab.classList.contains('active') &&
            plotSubtab && plotSubtab.classList.contains('active')) {
            console.log('[DOMContentLoaded] Plot subtab is active, initializing');
            initPlotSubtab();
        }
    }, 300);
});
