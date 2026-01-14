// Three.js Direct Implementation for PointCloud2 Visualization

// 3D Viewer 상태 - 객체로 그룹화
const viewer3DState = {
    scene: null,
    camera: null,
    renderer: null,
    controls: null,
    ros: null,
    rosConnected: false,
    pointCloudMeshes: {},
    displaySelectedTopics: [],
    currentDisplayContainer: null,
    threeJSInitialized: false
};

// Get the active display container based on current subtab
function getActiveDisplayContainer() {
    const activeSubTab = document.querySelector('.subtab-content.active');
    console.log('getActiveDisplayContainer - activeSubTab:', activeSubTab);
    
    if (!activeSubTab) {
        console.warn('No active subtab found');
        // Fallback: try to get 3d-viewer-subtab directly
        const viewerSubtab = document.getElementById('3d-viewer-subtab');
        if (viewerSubtab) {
            console.log('Using 3d-viewer-subtab as fallback');
            return document.getElementById('3d-viewer-container');
        }
        return null;
    }

    if (activeSubTab.id === '3d-viewer-subtab') {
        const container = document.getElementById('3d-viewer-container');
        console.log('Found 3D viewer container:', container);
        return container;
    }
    
    console.warn('Active subtab is not 3d-viewer-subtab:', activeSubTab.id);
    return null;
}

// Move renderer to active container
function moveRendererToActiveContainer() {
    const newContainer = getActiveDisplayContainer();
    if (!newContainer || !viewer3DState.renderer) return;

    // Don't move if already in the correct container
    if (viewer3DState.currentDisplayContainer === newContainer) return;

    console.log('Moving renderer to:', newContainer.id);

    // Append renderer to new container
    newContainer.appendChild(viewer3DState.renderer.domElement);
    viewer3DState.currentDisplayContainer = newContainer;

    // Resize renderer to fit new container
    if (viewer3DState.camera) {
        viewer3DState.camera.aspect = newContainer.clientWidth / newContainer.clientHeight;
        viewer3DState.camera.updateProjectionMatrix();
    }
    viewer3DState.renderer.setSize(newContainer.clientWidth, newContainer.clientHeight);
}

// Initialize Three.js scene
function initThreeJSDisplay() {
    console.log('=== initThreeJSDisplay called ===');
    console.log('THREE available:', typeof THREE !== 'undefined');
    console.log('OrbitControls available:', typeof window.OrbitControls !== 'undefined');

    // Try multiple ways to get the container
    let container = getActiveDisplayContainer();
    
    // Fallback: directly get the container
    if (!container) {
        console.log('Trying direct container access...');
        container = document.getElementById('3d-viewer-container');
    }
    
    if (!container) {
        console.error('3D Viewer container not found. Make sure 3D Viewer tab is active.');
        console.error('Attempted to find: #3d-viewer-container');
        return;
    }
    
    console.log('Container found:', container.id);
    console.log('Container dimensions:', container.clientWidth, 'x', container.clientHeight);

    // Remove loading message if present
    const loadingMsg = document.getElementById('3d-viewer-loading');
    if (loadingMsg) {
        loadingMsg.remove();
    }

    // If already initialized, just move renderer to new container
    if (viewer3DState.threeJSInitialized) {
        console.log('Three.js already initialized, moving renderer...');
        moveRendererToActiveContainer();
        return;
    }

    // Check if THREE is available
    if (typeof THREE === 'undefined') {
        console.error('Three.js library not loaded!');
        container.innerHTML = '<p style="color: #fff; padding: 20px;">Error: Three.js library failed to load. Please check your internet connection.</p>';
        return;
    }

    viewer3DState.currentDisplayContainer = container;
    viewer3DState.threeJSInitialized = true;

    // Create scene
    viewer3DState.scene = new THREE.Scene();
    viewer3DState.scene.background = new THREE.Color(0x2a2a2a);

    // Get container dimensions
    const containerWidth = container.clientWidth || 800;
    const containerHeight = container.clientHeight || 600;
    const aspect = containerWidth / containerHeight || 1.33;
    
    // Create camera - positioned for top-view (looking down Z-axis)
    viewer3DState.camera = new THREE.PerspectiveCamera(75, aspect, 0.1, 1000);
    viewer3DState.camera.position.set(0, 0, 50);  // Top-view: looking down from +Z
    viewer3DState.camera.lookAt(0, 0, 0);

    // Create renderer
    viewer3DState.renderer = new THREE.WebGLRenderer({ antialias: true });
    
    // Ensure container has valid dimensions
    if (containerWidth === 0 || containerHeight === 0) {
        console.warn('Container has zero dimensions, using default size');
        viewer3DState.renderer.setSize(800, 600);
    } else {
        viewer3DState.renderer.setSize(containerWidth, containerHeight);
    }
    
    container.appendChild(viewer3DState.renderer.domElement);

    // Add OrbitControls (ES6 module import)
    if (typeof window.OrbitControls !== 'undefined') {
        try {
            viewer3DState.controls = new window.OrbitControls(viewer3DState.camera, viewer3DState.renderer.domElement);
            viewer3DState.controls.enableDamping = true;
            viewer3DState.controls.dampingFactor = 0.25;
            console.log('OrbitControls initialized successfully');
        } catch (error) {
            console.error('Failed to initialize OrbitControls:', error);
        }
    } else {
        console.warn('OrbitControls not loaded. 3D navigation will be limited.');
        console.warn('Make sure OrbitControls is imported as ES6 module.');
    }

    // Add grid in XY plane (Z-up coordinate system)
    const gridHelper = new THREE.GridHelper(100, 100, 0x444444, 0x444444);
    gridHelper.rotation.x = Math.PI / 2;  // Rotate to XY plane
    viewer3DState.scene.add(gridHelper);

    // Add axes helper (X=red/forward, Y=green/left, Z=blue/up)
    const axesHelper = new THREE.AxesHelper(5);
    viewer3DState.scene.add(axesHelper);

    // Add lights
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    viewer3DState.scene.add(ambientLight);
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.4);
    directionalLight.position.set(10, 10, 10);
    viewer3DState.scene.add(directionalLight);

    // Handle window resize
    window.addEventListener('resize', onWindowResize);

    // Start animation loop
    animate();

    // Connect to ROS
    connectToROS();

    console.log('=== Three.js display initialized successfully ===');
    console.log('Scene:', viewer3DState.scene);
    console.log('Camera:', viewer3DState.camera);
    console.log('Renderer:', viewer3DState.renderer);
    console.log('Container:', container);
}

function onWindowResize() {
    const container = getActiveDisplayContainer();
    if (!container || !viewer3DState.camera || !viewer3DState.renderer) return;

    viewer3DState.camera.aspect = container.clientWidth / container.clientHeight;
    viewer3DState.camera.updateProjectionMatrix();
    viewer3DState.renderer.setSize(container.clientWidth, container.clientHeight);
}

function animate() {
    requestAnimationFrame(animate);
    if (viewer3DState.controls && viewer3DState.controls.update) {
        viewer3DState.controls.update();
    }
    if (viewer3DState.renderer && viewer3DState.scene && viewer3DState.camera) {
        viewer3DState.renderer.render(viewer3DState.scene, viewer3DState.camera);
    }
}

// Connect to ROS
function connectToROS() {
    // Get hostname dynamically for external access
    const hostname = window.location.hostname || 'localhost';
    const rosbridgeUrl = `ws://${hostname}:9090`;

    console.log('=== ROS Connection Debug ===');
    console.log('Attempting to connect to rosbridge...');
    console.log('WebSocket URL:', rosbridgeUrl);
    console.log('window.location.hostname:', window.location.hostname);
    console.log('window.location.href:', window.location.href);

    viewer3DState.ros = new ROSLIB.Ros({
        url: rosbridgeUrl
    });

    viewer3DState.ros.on('connection', function() {
        console.log('✓ Successfully connected to rosbridge:', rosbridgeUrl);
        viewer3DState.rosConnected = true;
    });

    viewer3DState.ros.on('error', function(error) {
        console.error('✗ Error connecting to rosbridge:', rosbridgeUrl);
        console.error('Error details:', error);
        viewer3DState.rosConnected = false;
    });

    viewer3DState.ros.on('close', function() {
        console.log('✗ Connection to rosbridge closed');
        viewer3DState.rosConnected = false;
        console.log('Will retry in 3 seconds...');
        // Try to reconnect after 3 seconds
        setTimeout(connectToROS, 3000);
    });
}

// Parse PointCloud2 message
function parsePointCloud2(message) {
    const points = [];
    const colors = [];

    // Find field offsets
    let xOffset = -1, yOffset = -1, zOffset = -1, rgbOffset = -1;

    for (let field of message.fields) {
        if (field.name === 'x') xOffset = field.offset;
        if (field.name === 'y') yOffset = field.offset;
        if (field.name === 'z') zOffset = field.offset;
        if (field.name === 'rgb' || field.name === 'rgba') rgbOffset = field.offset;
    }

    if (xOffset === -1 || yOffset === -1 || zOffset === -1) {
        console.error('Missing x, y, or z fields in PointCloud2');
        return { points: new Float32Array(), colors: new Float32Array() };
    }

    // Convert base64 data to binary
    // Optimized base64 decoding (5-10x faster than split+map)
    const binaryString = atob(message.data);
    const len = binaryString.length;
    const data = new Uint8Array(len);
    for (let i = 0; i < len; i++) {
        data[i] = binaryString.charCodeAt(i);
    }
    const view = new DataView(data.buffer);

    const pointStep = message.point_step;
    const numPoints = Math.min(message.width * message.height, 50000); // Limit points for performance

    for (let i = 0; i < numPoints; i++) {
        const offset = i * pointStep;

        // Read x, y, z
        const x = view.getFloat32(offset + xOffset, true);
        const y = view.getFloat32(offset + yOffset, true);
        const z = view.getFloat32(offset + zOffset, true);

        // Skip invalid points
        if (isNaN(x) || isNaN(y) || isNaN(z)) continue;

        points.push(x, y, z);

        // Read color if available
        if (rgbOffset !== -1) {
            const rgb = view.getUint32(offset + rgbOffset, true);
            const r = ((rgb >> 16) & 0xFF) / 255;
            const g = ((rgb >> 8) & 0xFF) / 255;
            const b = (rgb & 0xFF) / 255;
            colors.push(r, g, b);
        } else {
            // Default white color
            colors.push(1, 1, 1);
        }
    }

    return {
        points: new Float32Array(points),
        colors: new Float32Array(colors)
    };
}

// Subscribe to PointCloud2 topic
function subscribeToPointCloud(topicName) {
    if (!viewer3DState.rosConnected) {
        console.warn('Not connected to ROS');
        return;
    }

    console.log('Subscribing to PointCloud2 topic:', topicName);

    // Remove existing mesh if any
    if (viewer3DState.pointCloudMeshes[topicName]) {
        viewer3DState.scene.remove(viewer3DState.pointCloudMeshes[topicName]);
        viewer3DState.pointCloudMeshes[topicName].geometry.dispose();
        viewer3DState.pointCloudMeshes[topicName].material.dispose();
        delete viewer3DState.pointCloudMeshes[topicName];
    }

    // Create ROS topic
    const topic = new ROSLIB.Topic({
        ros: viewer3DState.ros,
        name: topicName,
        messageType: 'sensor_msgs/msg/PointCloud2',
        throttle_rate: 100 // Throttle to 10 Hz
    });

    // Subscribe to topic
    topic.subscribe(function(message) {
        console.log('Received PointCloud2 message from:', topicName);

        // Parse PointCloud2
        const { points, colors } = parsePointCloud2(message);

        if (points.length === 0) {
            console.warn('No valid points in PointCloud2');
            return;
        }

        console.log('Parsed', points.length / 3, 'points');

        // Create or update point cloud mesh
        if (!viewer3DState.pointCloudMeshes[topicName]) {
            const geometry = new THREE.BufferGeometry();
            geometry.setAttribute('position', new THREE.BufferAttribute(points, 3));
            geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
            geometry.computeBoundingSphere();

            const material = new THREE.PointsMaterial({
                size: 0.1,  // 0.1m point size
                vertexColors: true,
                sizeAttenuation: true  // Enable size attenuation for world-space sizing
            });

            const mesh = new THREE.Points(geometry, material);
            viewer3DState.pointCloudMeshes[topicName] = mesh;
            viewer3DState.scene.add(mesh);
            console.log('Added point cloud mesh to scene:', topicName);
            console.log('Mesh position:', mesh.position);
            console.log('Bounding sphere:', geometry.boundingSphere);
        } else {
            viewer3DState.pointCloudMeshes[topicName].geometry.setAttribute('position', new THREE.BufferAttribute(points, 3));
            viewer3DState.pointCloudMeshes[topicName].geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
            viewer3DState.pointCloudMeshes[topicName].geometry.attributes.position.needsUpdate = true;
            viewer3DState.pointCloudMeshes[topicName].geometry.attributes.color.needsUpdate = true;
            viewer3DState.pointCloudMeshes[topicName].geometry.computeBoundingSphere();
        }
    });

    return topic;
}

// Wait for ROS connection with timeout
async function waitForROSConnection(timeoutMs = 5000) {
    const startTime = Date.now();

    while (!viewer3DState.rosConnected && (Date.now() - startTime) < timeoutMs) {
        await new Promise(resolve => setTimeout(resolve, 100));
    }

    return viewer3DState.rosConnected;
}

// Get available PointCloud2 topics
async function getAvailablePointCloudTopics() {
    // Wait for connection if not connected yet
    if (!viewer3DState.rosConnected) {
        console.log('Waiting for ROS connection...');
        const connected = await waitForROSConnection(5000);

        if (!connected) {
            console.warn('Not connected to ROS after timeout');
            return [];
        }
    }

    return new Promise((resolve) => {
        viewer3DState.ros.getTopics(function(topics) {
            const pointCloudTopics = [];

            if (topics.topics && topics.types) {
                topics.topics.forEach((topic, index) => {
                    const type = topics.types[index];
                    if (type === 'sensor_msgs/PointCloud2' || type === 'sensor_msgs/msg/PointCloud2') {
                        pointCloudTopics.push(topic);
                    }
                });
            }

            console.log('Available PointCloud2 topics:', pointCloudTopics);
            resolve(pointCloudTopics);
        }, function(error) {
            console.error('Failed to get topics:', error);
            resolve([]);
        });
    });
}

// Handle topic selection
async function selectDisplayTopics() {
    console.log('Opening topic selection dialog...');

    const topics = await getAvailablePointCloudTopics();

    if (topics.length === 0) {
        if (!viewer3DState.rosConnected) {
            alert('Not connected to ROS. Make sure rosbridge_server is running:\n\nros2 launch rosbridge_server rosbridge_websocket_launch.xml');
        } else {
            alert('No PointCloud2 topics available. Make sure topics are being published.');
        }
        return;
    }

    // Show topic selection modal (reuse existing modal)
    const modal = document.getElementById('display-topic-modal');
    const topicList = document.getElementById('display-topic-list');

    if (!modal || !topicList) {
        console.error('Display topic modal not found');
        return;
    }

    topicList.innerHTML = '';

    topics.forEach(topic => {
        const div = document.createElement('div');
        div.className = 'topic-item';

        const checkbox = document.createElement('input');
        checkbox.type = 'checkbox';
        checkbox.id = `display-topic-${topic}`;
        checkbox.value = topic;
        checkbox.checked = viewer3DState.displaySelectedTopics.includes(topic);

        const label = document.createElement('label');
        label.htmlFor = `display-topic-${topic}`;
        label.textContent = topic;

        div.appendChild(checkbox);
        div.appendChild(label);
        topicList.appendChild(div);
    });

    modal.style.display = 'block';
}

function closeDisplayTopicSelection() {
    const modal = document.getElementById('display-topic-modal');
    if (modal) modal.style.display = 'none';
}

function confirmDisplayTopicSelection() {
    const checkboxes = document.querySelectorAll('#display-topic-list input[type="checkbox"]');
    const newTopics = [];

    checkboxes.forEach(cb => {
        if (cb.checked) {
            newTopics.push(cb.value);
        }
    });

    // Unsubscribe from removed topics
    viewer3DState.displaySelectedTopics.forEach(topic => {
        if (!newTopics.includes(topic)) {
            if (viewer3DState.pointCloudMeshes[topic]) {
                viewer3DState.scene.remove(viewer3DState.pointCloudMeshes[topic]);
                viewer3DState.pointCloudMeshes[topic].geometry.dispose();
                viewer3DState.pointCloudMeshes[topic].material.dispose();
                delete viewer3DState.pointCloudMeshes[topic];
            }
        }
    });

    // Subscribe to new topics
    newTopics.forEach(topic => {
        if (!viewer3DState.displaySelectedTopics.includes(topic)) {
            subscribeToPointCloud(topic);
        }
    });

    viewer3DState.displaySelectedTopics = newTopics;
    closeDisplayTopicSelection();

    console.log('Display selected topics:', viewer3DState.displaySelectedTopics);
}

// Initialize when 3D Viewer tab is opened
function initialize3DViewer() {
    console.log('initialize3DViewer called');
    
    // Check if 3D Viewer subtab is active
    const viewerSubtab = document.getElementById('3d-viewer-subtab');
    if (!viewerSubtab) {
        console.warn('3D Viewer subtab not found');
        return;
    }
    
    // Wait for Three.js to be ready (required)
    const checkThree = (attempts = 0) => {
        if (typeof window.THREE === 'undefined' && !window.threeReady) {
            if (attempts < 50) { // Wait up to 5 seconds
                console.log('Waiting for Three.js...', attempts);
                window.waitingForThree = () => checkThree(attempts + 1);
                setTimeout(() => checkThree(attempts + 1), 100);
                return;
            } else {
                console.error('Three.js failed to load after 5 seconds');
                return;
            }
        }
        
        // Three.js is ready, OrbitControls is optional - proceed with initialization
        console.log('Three.js ready, initializing display...');
        
        // Wait a bit for DOM to update, then initialize
        setTimeout(() => {
            // Double check that subtab is still active
            if (viewerSubtab.classList.contains('active')) {
                console.log('Initializing Three.js display...');
                initThreeJSDisplay();
            } else {
                console.log('3D Viewer subtab is not active, skipping initialization');
            }
        }, 200);
    };
    
    checkThree();
}

// Store initialization function for OrbitControls module
window.threejsDisplayReady = function() {
    const viewerSubtab = document.getElementById('3d-viewer-subtab');
    if (viewerSubtab && viewerSubtab.classList.contains('active')) {
        console.log('OrbitControls ready, initializing 3D Viewer...');
        initialize3DViewer();
    }
};

// Initialize on page load if 3D Viewer is active
window.addEventListener('load', () => {
    setTimeout(() => {
        const viewerSubtab = document.getElementById('3d-viewer-subtab');
        if (viewerSubtab && viewerSubtab.classList.contains('active')) {
            initialize3DViewer();
        }
    }, 500);
});

// Expose functions to global scope for onclick handlers
window.initThreeJSDisplay = initThreeJSDisplay;
window.initialize3DViewer = initialize3DViewer;
window.selectDisplayTopics = selectDisplayTopics;
window.closeDisplayTopicSelection = closeDisplayTopicSelection;
window.confirmDisplayTopicSelection = confirmDisplayTopicSelection;
window.moveRendererToActiveContainer = moveRendererToActiveContainer;

console.log('=== Three.js Display script loaded ===');
console.log('Functions exposed:', {
    initThreeJSDisplay: typeof initThreeJSDisplay,
    initialize3DViewer: typeof initialize3DViewer,
    selectDisplayTopics: typeof selectDisplayTopics
});
