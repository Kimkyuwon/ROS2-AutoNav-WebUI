// Three.js Direct Implementation for PointCloud2 Visualization

let scene, camera, renderer, controls;
let ros, rosConnected = false;
let pointCloudMeshes = {};
let displaySelectedTopics = [];
let currentDisplayContainer = null;
let threeJSInitialized = false;

// Get the active display container based on current subtab
function getActiveDisplayContainer() {
    const activeSubTab = document.querySelector('.subtab-content.active');
    if (!activeSubTab) return null;

    if (activeSubTab.id === 'bag-player-subtab') {
        return document.getElementById('bag-player-display-canvas-container');
    } else if (activeSubTab.id === 'bag-recorder-subtab') {
        return document.getElementById('bag-recorder-display-canvas-container');
    }
    return null;
}

// Move renderer to active container
function moveRendererToActiveContainer() {
    const newContainer = getActiveDisplayContainer();
    if (!newContainer || !renderer) return;

    // Don't move if already in the correct container
    if (currentDisplayContainer === newContainer) return;

    console.log('Moving renderer to:', newContainer.id);

    // Append renderer to new container
    newContainer.appendChild(renderer.domElement);
    currentDisplayContainer = newContainer;

    // Resize renderer to fit new container
    if (camera) {
        camera.aspect = newContainer.clientWidth / newContainer.clientHeight;
        camera.updateProjectionMatrix();
    }
    renderer.setSize(newContainer.clientWidth, newContainer.clientHeight);
}

// Initialize Three.js scene
function initThreeJSDisplay() {
    // Skip if already initialized
    if (threeJSInitialized) {
        moveRendererToActiveContainer();
        return;
    }

    console.log('Initializing Three.js display...');

    const container = getActiveDisplayContainer();
    if (!container) {
        console.log('Display container not found, deferring initialization');
        return;
    }

    currentDisplayContainer = container;
    threeJSInitialized = true;

    // Create scene
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0x2a2a2a);

    // Create camera - positioned for top-view (looking down Z-axis)
    camera = new THREE.PerspectiveCamera(
        75,
        container.clientWidth / container.clientHeight,
        0.1,
        1000
    );
    camera.position.set(0, 0, 50);  // Top-view: looking down from +Z
    camera.lookAt(0, 0, 0);

    // Create renderer
    renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setSize(container.clientWidth, container.clientHeight);
    container.appendChild(renderer.domElement);

    // Add OrbitControls
    controls = new THREE.OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.25;

    // Add grid in XY plane (Z-up coordinate system)
    const gridHelper = new THREE.GridHelper(100, 100, 0x444444, 0x444444);
    gridHelper.rotation.x = Math.PI / 2;  // Rotate to XY plane
    scene.add(gridHelper);

    // Add axes helper (X=red/forward, Y=green/left, Z=blue/up)
    const axesHelper = new THREE.AxesHelper(5);
    scene.add(axesHelper);

    // Add lights
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.4);
    directionalLight.position.set(10, 10, 10);
    scene.add(directionalLight);

    // Handle window resize
    window.addEventListener('resize', onWindowResize);

    // Start animation loop
    animate();

    // Connect to ROS
    connectToROS();

    console.log('Three.js display initialized');
}

function onWindowResize() {
    const container = getActiveDisplayContainer();
    if (!container || !camera || !renderer) return;

    camera.aspect = container.clientWidth / container.clientHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(container.clientWidth, container.clientHeight);
}

function animate() {
    requestAnimationFrame(animate);
    if (controls) controls.update();
    if (renderer && scene && camera) {
        renderer.render(scene, camera);
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

    ros = new ROSLIB.Ros({
        url: rosbridgeUrl
    });

    ros.on('connection', function() {
        console.log('✓ Successfully connected to rosbridge:', rosbridgeUrl);
        rosConnected = true;
    });

    ros.on('error', function(error) {
        console.error('✗ Error connecting to rosbridge:', rosbridgeUrl);
        console.error('Error details:', error);
        rosConnected = false;
    });

    ros.on('close', function() {
        console.log('✗ Connection to rosbridge closed');
        rosConnected = false;
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
    const data = new Uint8Array(atob(message.data).split('').map(c => c.charCodeAt(0)));
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
    if (!rosConnected) {
        console.warn('Not connected to ROS');
        return;
    }

    console.log('Subscribing to PointCloud2 topic:', topicName);

    // Remove existing mesh if any
    if (pointCloudMeshes[topicName]) {
        scene.remove(pointCloudMeshes[topicName]);
        pointCloudMeshes[topicName].geometry.dispose();
        pointCloudMeshes[topicName].material.dispose();
        delete pointCloudMeshes[topicName];
    }

    // Create ROS topic
    const topic = new ROSLIB.Topic({
        ros: ros,
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
        if (!pointCloudMeshes[topicName]) {
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
            pointCloudMeshes[topicName] = mesh;
            scene.add(mesh);
            console.log('Added point cloud mesh to scene:', topicName);
            console.log('Mesh position:', mesh.position);
            console.log('Bounding sphere:', geometry.boundingSphere);
        } else {
            pointCloudMeshes[topicName].geometry.setAttribute('position', new THREE.BufferAttribute(points, 3));
            pointCloudMeshes[topicName].geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
            pointCloudMeshes[topicName].geometry.attributes.position.needsUpdate = true;
            pointCloudMeshes[topicName].geometry.attributes.color.needsUpdate = true;
            pointCloudMeshes[topicName].geometry.computeBoundingSphere();
        }
    });

    return topic;
}

// Wait for ROS connection with timeout
async function waitForROSConnection(timeoutMs = 5000) {
    const startTime = Date.now();

    while (!rosConnected && (Date.now() - startTime) < timeoutMs) {
        await new Promise(resolve => setTimeout(resolve, 100));
    }

    return rosConnected;
}

// Get available PointCloud2 topics
async function getAvailablePointCloudTopics() {
    // Wait for connection if not connected yet
    if (!rosConnected) {
        console.log('Waiting for ROS connection...');
        const connected = await waitForROSConnection(5000);

        if (!connected) {
            console.warn('Not connected to ROS after timeout');
            return [];
        }
    }

    return new Promise((resolve) => {
        ros.getTopics(function(topics) {
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
        if (!rosConnected) {
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
        checkbox.checked = displaySelectedTopics.includes(topic);

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
    displaySelectedTopics.forEach(topic => {
        if (!newTopics.includes(topic)) {
            if (pointCloudMeshes[topic]) {
                scene.remove(pointCloudMeshes[topic]);
                pointCloudMeshes[topic].geometry.dispose();
                pointCloudMeshes[topic].material.dispose();
                delete pointCloudMeshes[topic];
            }
        }
    });

    // Subscribe to new topics
    newTopics.forEach(topic => {
        if (!displaySelectedTopics.includes(topic)) {
            subscribeToPointCloud(topic);
        }
    });

    displaySelectedTopics = newTopics;
    closeDisplayTopicSelection();

    console.log('Display selected topics:', displaySelectedTopics);
}

// Initialize on page load
window.addEventListener('load', () => {
    setTimeout(() => {
        initThreeJSDisplay();
    }, 500);
});

// Expose functions to global scope for onclick handlers
window.initThreeJSDisplay = initThreeJSDisplay;
window.selectDisplayTopics = selectDisplayTopics;
window.closeDisplayTopicSelection = closeDisplayTopicSelection;
window.confirmDisplayTopicSelection = confirmDisplayTopicSelection;
window.moveRendererToActiveContainer = moveRendererToActiveContainer;

console.log('Three.js Display script loaded');
