// Three.js Direct Implementation for PointCloud2 Visualization

// =============================================
// RViz 스타일 색상 팔레트 피커
// =============================================

// 프리셋 색상 팔레트 (9열 × 5행 = 45색)
const SOLID_COLOR_PRESETS = [
    // Row 1 - Light
    '#7FC5F4', '#80E5A0', '#F5F59A', '#F5C580', '#F58080', '#D580F5', '#C5A080', '#F0F0F0', '#A8A8A8',
    // Row 2 - Medium-light
    '#4AA3E8', '#40C060', '#F0E040', '#F0A040', '#E84040', '#B040E8', '#A07040', '#D0D0D0', '#707070',
    // Row 3 - Medium
    '#1E90FF', '#00C000', '#FFD700', '#FF8C00', '#FF2020', '#9400D3', '#8B4513', '#909090', '#404040',
    // Row 4 - Medium-dark
    '#1060D0', '#008000', '#DAA520', '#FF4500', '#CC0000', '#7B00CC', '#6B3410', '#606060', '#202020',
    // Row 5 - Dark
    '#00008B', '#006400', '#B8860B', '#D2691E', '#8B0000', '#4B0082', '#3D1A08', '#303030', '#000000',
];

// 팔레트 피커 전역 상태
const colorPickerPopupState = {
    topicName: null,
    selectedColor: '#ffffff',
    swatchEl: null,
    hexLabel: null,
    onConfirm: null,    // 확정 시 호출할 콜백 (function(color) {}), null이면 PointCloud2 기본 동작
};

/**
 * 팔레트 팝업 DOM 생성 (최초 1회)
 */
function buildColorPickerPopup() {
    if (document.getElementById('solid-color-picker-popup')) return;

    const popup = document.createElement('div');
    popup.id = 'solid-color-picker-popup';
    popup.className = 'solid-color-picker-popup';
    popup.style.display = 'none';

    // 헤더
    const header = document.createElement('div');
    header.className = 'scp-header';

    const cancelBtn = document.createElement('button');
    cancelBtn.className = 'scp-btn scp-cancel';
    cancelBtn.textContent = '취소(C)';
    cancelBtn.addEventListener('click', closeColorPickerPopup);

    const title = document.createElement('span');
    title.className = 'scp-title';
    title.textContent = 'Select Color';

    const selectBtn = document.createElement('button');
    selectBtn.className = 'scp-btn scp-select';
    selectBtn.textContent = '선택(S)';
    selectBtn.addEventListener('click', confirmColorPickerPopup);

    header.appendChild(cancelBtn);
    header.appendChild(title);
    header.appendChild(selectBtn);

    // 색상 그리드
    const grid = document.createElement('div');
    grid.id = 'scp-grid';
    grid.className = 'scp-grid';

    SOLID_COLOR_PRESETS.forEach(color => {
        const cell = document.createElement('button');
        cell.className = 'scp-color-cell';
        cell.style.backgroundColor = color;
        cell.dataset.color = color;
        cell.title = color;
        cell.addEventListener('click', function() {
            selectColorInPicker(color);
        });
        grid.appendChild(cell);
    });

    // 사용자 지정 섹션
    const customSection = document.createElement('div');
    customSection.className = 'scp-custom-section';

    const customLabel = document.createElement('div');
    customLabel.className = 'scp-custom-label';
    customLabel.textContent = '사용자 지정';

    const customRow = document.createElement('div');
    customRow.className = 'scp-custom-row';

    const addBtn = document.createElement('button');
    addBtn.className = 'scp-add-btn';
    addBtn.textContent = '+';
    addBtn.title = '직접 색상 입력';

    const hiddenInput = document.createElement('input');
    hiddenInput.type = 'color';
    hiddenInput.id = 'scp-custom-input';
    hiddenInput.style.cssText = 'position:absolute;opacity:0;pointer-events:none;width:0;height:0;';

    const customSwatch = document.createElement('div');
    customSwatch.id = 'scp-custom-swatch';
    customSwatch.className = 'scp-custom-swatch';

    addBtn.addEventListener('click', function(e) {
        e.stopPropagation();
        hiddenInput.value = colorPickerPopupState.selectedColor;
        hiddenInput.click();
    });

    hiddenInput.addEventListener('input', function() {
        const val = this.value;
        customSwatch.style.backgroundColor = val;
        selectColorInPicker(val, true);  // 커스텀 컬러
    });

    customRow.appendChild(addBtn);
    customRow.appendChild(hiddenInput);
    customRow.appendChild(customSwatch);

    customSection.appendChild(customLabel);
    customSection.appendChild(customRow);

    popup.appendChild(header);
    popup.appendChild(grid);
    popup.appendChild(customSection);

    document.body.appendChild(popup);

    // 외부 클릭 시 닫기
    document.addEventListener('click', function(e) {
        const p = document.getElementById('solid-color-picker-popup');
        if (p && p.style.display !== 'none') {
            if (!p.contains(e.target) && e.target !== colorPickerPopupState.swatchEl) {
                closeColorPickerPopup();
            }
        }
    }, true);
}

/**
 * 팔레트에서 색상 선택 (체크마크 이동)
 * @param {string} color - '#rrggbb' 형태
 * @param {boolean} isCustom - 사용자 지정 색상 여부
 */
function selectColorInPicker(color, isCustom = false) {
    colorPickerPopupState.selectedColor = color;

    // 모든 셀 체크마크 제거
    document.querySelectorAll('#scp-grid .scp-color-cell').forEach(cell => {
        cell.classList.remove('scp-selected');
    });

    if (!isCustom) {
        // 프리셋 셀에 체크마크
        document.querySelectorAll('#scp-grid .scp-color-cell').forEach(cell => {
            if (cell.dataset.color.toLowerCase() === color.toLowerCase()) {
                cell.classList.add('scp-selected');
            }
        });
        // 커스텀 스와치 초기화
        const customSwatch = document.getElementById('scp-custom-swatch');
        if (customSwatch) customSwatch.style.backgroundColor = 'transparent';
    } else {
        // 커스텀 스와치에 반영
        const customSwatch = document.getElementById('scp-custom-swatch');
        if (customSwatch) customSwatch.style.backgroundColor = color;
    }
}

/**
 * 팔레트 팝업 열기
 * @param {string|null} topicName - PointCloud2 토픽명 (Path/Odometry는 null 가능)
 * @param {string} currentColor   - 현재 색상 (#rrggbb)
 * @param {HTMLElement} swatchEl  - 팝업을 열어준 스와치 버튼 요소
 * @param {HTMLElement|null} hexLabel - 색상 hex 텍스트 요소 (없으면 null)
 * @param {Function|null} onConfirm  - 확정 시 콜백 function(color), null이면 PointCloud2 기본 동작
 */
function openColorPickerPopup(topicName, currentColor, swatchEl, hexLabel, onConfirm = null) {
    buildColorPickerPopup();

    colorPickerPopupState.topicName = topicName;
    colorPickerPopupState.selectedColor = currentColor;
    colorPickerPopupState.swatchEl = swatchEl;
    colorPickerPopupState.hexLabel = hexLabel;
    colorPickerPopupState.onConfirm = onConfirm;

    // 커스텀 스와치 초기화
    const customSwatch = document.getElementById('scp-custom-swatch');
    if (customSwatch) customSwatch.style.backgroundColor = 'transparent';

    // 현재 색상 선택 표시
    const isPreset = SOLID_COLOR_PRESETS.some(c => c.toLowerCase() === currentColor.toLowerCase());
    selectColorInPicker(currentColor, !isPreset);
    if (!isPreset && customSwatch) customSwatch.style.backgroundColor = currentColor;

    // 팝업 위치 계산 (스와치 근처)
    const popup = document.getElementById('solid-color-picker-popup');
    popup.style.display = 'block';

    const rect = swatchEl.getBoundingClientRect();
    const pw = popup.offsetWidth || 290;
    const ph = popup.offsetHeight || 260;

    let left = rect.left;
    let top = rect.bottom + 6;

    if (left + pw > window.innerWidth - 8) left = window.innerWidth - pw - 8;
    if (left < 8) left = 8;
    if (top + ph > window.innerHeight - 8) top = rect.top - ph - 6;
    if (top < 8) top = 8;

    popup.style.left = `${left}px`;
    popup.style.top = `${top}px`;
}

/**
 * 팔레트 팝업 닫기 (취소)
 */
function closeColorPickerPopup() {
    const popup = document.getElementById('solid-color-picker-popup');
    if (popup) popup.style.display = 'none';
}

/**
 * 팔레트 팝업 확정 (선택)
 */
function confirmColorPickerPopup() {
    const { topicName, selectedColor, swatchEl, hexLabel, onConfirm } = colorPickerPopupState;

    if (typeof onConfirm === 'function') {
        // Path/Odometry 등 커스텀 콜백
        onConfirm(selectedColor);
        if (swatchEl) swatchEl.style.backgroundColor = selectedColor;
        if (hexLabel) hexLabel.textContent = selectedColor.toUpperCase();
    } else if (topicName) {
        // PointCloud2 기본 동작
        updateTopicSetting(topicName, 'solidColor', selectedColor);
        if (swatchEl) swatchEl.style.backgroundColor = selectedColor;
        if (hexLabel) hexLabel.textContent = selectedColor.toUpperCase();
    }
    closeColorPickerPopup();
}

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
    threeJSInitialized: false,
    topicSubscriptions: new Map(),  // 토픽명 → ROSLIB.Topic 구독 객체
    topicSettings: new Map(),       // 토픽명 → 색상/표시 설정 객체
    pathObjects: new Map(),         // topicName → { line, visible }
    odomObjects: new Map(),         // topicName → { group, arrow, sphere, visible }
    trajectoryObjects: new Map(),   // topicName → { parentGroup, axesGroupList[], visible }
    selectedPathTopics: [],         // 선택된 Path 토픽 목록
    selectedOdomTopics: [],         // 선택된 Odometry 토픽 목록
    trajectoryMaxPoints: 1000,      // 궤적 최대 포인트 수
    decayObjects: new Map()         // topicName → [{ points: THREE.Points, time: DOMHighResTimeStamp }]
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

/**
 * Decay Time: 만료된 PointCloud2 Points mesh를 scene에서 제거
 * animate() 루프에서 매 프레임 호출
 */
function tickDecayObjects() {
    const now = performance.now();
    viewer3DState.decayObjects.forEach((arr, topicName) => {
        if (arr.length === 0) return;
        const settings  = viewer3DState.topicSettings.get(topicName) || {};
        const decayMs   = (settings.decayTime || 0) * 1000;
        if (decayMs <= 0) return;

        let removed = false;
        for (let i = arr.length - 1; i >= 0; i--) {
            if (now - arr[i].time > decayMs) {
                viewer3DState.scene.remove(arr[i].points);
                arr[i].points.geometry.dispose();
                arr[i].points.material.dispose();
                arr.splice(i, 1);
                removed = true;
            }
        }
        if (removed) viewer3DState.decayObjects.set(topicName, arr);
    });
}

function animate() {
    requestAnimationFrame(animate);
    if (viewer3DState.controls && viewer3DState.controls.update) {
        viewer3DState.controls.update();
    }
    tickDecayObjects();
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

// =============================================
// PointCloud2 파싱 성능 최적화: 모듈 레벨 재사용 버퍼
// - Array.push 완전 제거 (GC 압력 제거)
// - 단일 패스 처리 (2패스 → 1패스)
// - 인라인 색상 계산 (함수 호출 / 객체 생성 제거)
// =============================================
const PC2_MAX_POINTS = 50000;
const _pc2PosBuffer = new Float32Array(PC2_MAX_POINTS * 3);
const _pc2ColBuffer = new Float32Array(PC2_MAX_POINTS * 3);

/**
 * PointCloud2 메시지 고성능 파싱 (단일 패스, 재사용 버퍼)
 * @param {Object} message       - rosbridge PointCloud2 메시지
 * @param {Object} [settings]    - 색상 설정 { colorMode, colorField, solidColor, pointSize }
 * @param {number} [prevMin]     - 이전 프레임 rainbow 최솟값 (적응형 범위)
 * @param {number} [prevMax]     - 이전 프레임 rainbow 최댓값
 * @returns {{ count: number, newMin: number, newMax: number }}
 *   결과는 _pc2PosBuffer / _pc2ColBuffer 에 0..count*3-1 범위로 기록됨
 */
function parsePointCloud2(message, settings, prevMin, prevMax) {
    const colorMode = (settings && settings.colorMode) || 'rainbow';
    const colorField = (settings && settings.colorField) || 'intensity';
    const solidColor = (settings && settings.solidColor) || '#ffffff';

    // Field 오프셋 탐색
    let xOffset = -1, yOffset = -1, zOffset = -1, rgbOffset = -1, colorFieldOffset = -1;
    for (const field of message.fields) {
        if (field.name === 'x')                              xOffset = field.offset;
        if (field.name === 'y')                              yOffset = field.offset;
        if (field.name === 'z')                              zOffset = field.offset;
        if (field.name === 'rgb' || field.name === 'rgba')  rgbOffset = field.offset;
        if (field.name === colorField)                       colorFieldOffset = field.offset;
    }

    if (xOffset === -1 || yOffset === -1 || zOffset === -1) {
        console.error('[PC2] Missing x/y/z fields in PointCloud2');
        return { count: 0, newMin: 0, newMax: 1 };
    }

    // base64 → Uint8Array 디코딩
    const binaryString = atob(message.data);
    const len = binaryString.length;
    const data = new Uint8Array(len);
    for (let i = 0; i < len; i++) data[i] = binaryString.charCodeAt(i);
    const view = new DataView(data.buffer);

    const pointStep  = message.point_step;
    const numPoints  = Math.min(message.width * message.height, PC2_MAX_POINTS);

    // ── Solid 색상 사전 파싱 ──
    let solidR = 1, solidG = 1, solidB = 1;
    if (colorMode === 'solid') {
        const hex = solidColor.replace('#', '');
        solidR = parseInt(hex.substring(0, 2), 16) / 255;
        solidG = parseInt(hex.substring(2, 4), 16) / 255;
        solidB = parseInt(hex.substring(4, 6), 16) / 255;
    }

    // ── Rainbow 범위: 이전 프레임 값으로 단일 패스 실현 (적응형) ──
    const rangeMin  = (isFinite(prevMin) ? prevMin : 0);
    const rangeMax  = (isFinite(prevMax) && prevMax > rangeMin) ? prevMax : (rangeMin + 1);
    const rangeSpan = rangeMax - rangeMin;

    let newMin = Infinity, newMax = -Infinity;
    let count = 0;

    // ── 단일 패스: XYZ + 색상 동시 처리 ──
    for (let i = 0; i < numPoints; i++) {
        const base = i * pointStep;

        const x = view.getFloat32(base + xOffset, true);
        const y = view.getFloat32(base + yOffset, true);
        const z = view.getFloat32(base + zOffset, true);
        if (isNaN(x) || isNaN(y) || isNaN(z)) continue;

        const ptr3 = count * 3;
        _pc2PosBuffer[ptr3]     = x;
        _pc2PosBuffer[ptr3 + 1] = y;
        _pc2PosBuffer[ptr3 + 2] = z;

        if (colorMode === 'rainbow') {
            // 색상 필드 값 읽기
            let fv = (colorFieldOffset !== -1)
                ? view.getFloat32(base + colorFieldOffset, true)
                : z;
            if (isNaN(fv)) fv = rangeMin;

            // min/max 갱신 (다음 프레임 적응형 범위)
            if (fv < newMin) newMin = fv;
            if (fv > newMax) newMax = fv;

            // ── HSV Rainbow 인라인 (s=1,v=1 특수화 → 객체/함수 생성 없음) ──
            // t: 0.0(파랑) → 1.0(빨강)
            const t   = rangeSpan > 0 ? Math.min(1.0, Math.max(0.0, (fv - rangeMin) / rangeSpan)) : 0.5;
            const hue = (1.0 - t) * 0.667;          // 파랑(0.667) → 빨강(0.0)
            const h6  = hue * 6;
            const hi  = h6 | 0;                     // Math.floor 대체
            const f   = h6 - hi;
            let cr, cg, cb;
            switch (hi % 6) {
                case 0: cr = 1;     cg = f;     cb = 0;     break;
                case 1: cr = 1 - f; cg = 1;     cb = 0;     break;
                case 2: cr = 0;     cg = 1;     cb = f;     break;
                case 3: cr = 0;     cg = 1 - f; cb = 1;     break;
                case 4: cr = f;     cg = 0;     cb = 1;     break;
                default: cr = 1;   cg = 0;     cb = 1 - f; break;
            }
            _pc2ColBuffer[ptr3]     = cr;
            _pc2ColBuffer[ptr3 + 1] = cg;
            _pc2ColBuffer[ptr3 + 2] = cb;

        } else if (colorMode === 'rgb' && rgbOffset !== -1) {
            const rgb = view.getUint32(base + rgbOffset, true);
            _pc2ColBuffer[ptr3]     = ((rgb >> 16) & 0xFF) / 255;
            _pc2ColBuffer[ptr3 + 1] = ((rgb >>  8) & 0xFF) / 255;
            _pc2ColBuffer[ptr3 + 2] = ( rgb        & 0xFF) / 255;

        } else {
            // solid 또는 fallback
            _pc2ColBuffer[ptr3]     = solidR;
            _pc2ColBuffer[ptr3 + 1] = solidG;
            _pc2ColBuffer[ptr3 + 2] = solidB;
        }

        count++;
    }

    return {
        count,
        newMin: isFinite(newMin) ? newMin : 0,
        newMax: isFinite(newMax) ? newMax : 1,
    };
}

// 토픽 구독 해제 및 리소스 정리
function unsubscribeFromTopic(topicName) {
    // ROSLIB.Topic 구독 해제
    const topic = viewer3DState.topicSubscriptions.get(topicName);
    if (topic) {
        try {
            topic.unsubscribe();
            console.log('Unsubscribed from topic:', topicName);
        } catch (e) {
            console.warn('Failed to unsubscribe from topic:', topicName, e);
        }
        viewer3DState.topicSubscriptions.delete(topicName);
    }

    // Scene에서 PointCloud2 mesh 제거 및 GPU 리소스 dispose
    if (viewer3DState.pointCloudMeshes[topicName]) {
        viewer3DState.scene.remove(viewer3DState.pointCloudMeshes[topicName]);
        viewer3DState.pointCloudMeshes[topicName].geometry.dispose();
        viewer3DState.pointCloudMeshes[topicName].material.dispose();
        delete viewer3DState.pointCloudMeshes[topicName];
        console.log('Removed point cloud mesh for topic:', topicName);
    }
    // Decay 모드 누적 mesh 정리
    const decayArr = viewer3DState.decayObjects.get(topicName);
    if (decayArr && decayArr.length > 0) {
        decayArr.forEach(item => {
            viewer3DState.scene.remove(item.points);
            item.points.geometry.dispose();
            item.points.material.dispose();
        });
        viewer3DState.decayObjects.delete(topicName);
    }

    // Path 객체 정리
    const pathObj = viewer3DState.pathObjects.get(topicName);
    if (pathObj) {
        viewer3DState.scene.remove(pathObj.line);
        pathObj.line.geometry.dispose();
        pathObj.line.material.dispose();
        viewer3DState.pathObjects.delete(topicName);
        console.log('Removed path object for topic:', topicName);
    }

    // Odometry 객체 정리
    const odomObj = viewer3DState.odomObjects.get(topicName);
    if (odomObj) {
        viewer3DState.scene.remove(odomObj.group);
        // axes group 내 모든 Mesh의 geometry/material dispose (GPU 메모리 누수 방지)
        odomObj.group.traverse(function(child) {
            if (child.isMesh) {
                child.geometry.dispose();
                if (Array.isArray(child.material)) {
                    child.material.forEach(m => m.dispose());
                } else if (child.material) {
                    child.material.dispose();
                }
            }
        });
        viewer3DState.odomObjects.delete(topicName);
        console.log('Removed odometry object for topic:', topicName);
    }

    // Trajectory 객체 정리 (parentGroup + 모든 axes group dispose)
    const trajObj = viewer3DState.trajectoryObjects.get(topicName);
    if (trajObj) {
        trajObj.axesGroupList.forEach(ag => {
            trajObj.parentGroup.remove(ag);
            disposeAxesGroup(ag);
        });
        viewer3DState.scene.remove(trajObj.parentGroup);
        viewer3DState.trajectoryObjects.delete(topicName);
        console.log('Removed trajectory object for topic:', topicName);
    }
}

// Subscribe to PointCloud2 topic
function subscribeToPointCloud(topicName) {
    if (!viewer3DState.rosConnected) {
        console.warn('Not connected to ROS');
        return;
    }

    // 재구독 방지: 이미 구독 중인 토픽이면 건너뜀
    if (viewer3DState.topicSubscriptions.has(topicName)) {
        return viewer3DState.topicSubscriptions.get(topicName);
    }

    console.log('[PC2] Subscribing to topic:', topicName);

    // ── 토픽별 적응형 rainbow 범위 상태 (클로저) ──
    // 첫 프레임: [0,1] 으로 초기화, 이후 실제 데이터로 갱신
    let adaptiveMin = 0;
    let adaptiveMax = 1;
    let frameCount  = 0;

    const topic = new ROSLIB.Topic({
        ros: viewer3DState.ros,
        name: topicName,
        messageType: 'sensor_msgs/msg/PointCloud2',
        throttle_rate: 100,   // 최대 10 Hz
        queue_length: 1       // 오래된 메시지 드롭 → 항상 최신 프레임 처리
    });

    viewer3DState.topicSubscriptions.set(topicName, topic);

    topic.subscribe(function(message) {
        // 숨겨진 토픽은 파싱 자체를 건너뜀 (CPU 절약)
        const mesh = viewer3DState.pointCloudMeshes[topicName];
        if (mesh && mesh.visible === false) return;

        const settings   = viewer3DState.topicSettings.get(topicName) || {};
        const pointSize  = settings.pointSize  !== undefined ? settings.pointSize  : 0.1;
        const alpha      = settings.alpha      !== undefined ? settings.alpha      : 1.0;
        const decayTime  = settings.decayTime  !== undefined ? settings.decayTime  : 0;

        // ── 고성능 파싱 (단일 패스, 재사용 버퍼) ──
        const { count, newMin, newMax } = parsePointCloud2(
            message, settings, adaptiveMin, adaptiveMax
        );

        if (count === 0) return;

        // 적응형 범위 갱신 (다음 프레임에 사용)
        adaptiveMin = newMin;
        adaptiveMax = newMax;
        frameCount++;

        // ── Helper: PointsMaterial 생성 ──
        function makeMaterial(sz, a) {
            return new THREE.PointsMaterial({
                size: sz,
                vertexColors: true,
                sizeAttenuation: true,
                transparent: a < 1.0,
                opacity: a,
            });
        }

        if (decayTime > 0) {
            // ═══ DECAY 모드: 매 프레임 독립 Points 객체 생성·누적 ═══
            // 기존 단일 mesh가 있으면 scene에서 제거 (모드 전환)
            if (viewer3DState.pointCloudMeshes[topicName]) {
                viewer3DState.scene.remove(viewer3DState.pointCloudMeshes[topicName]);
                viewer3DState.pointCloudMeshes[topicName].geometry.dispose();
                viewer3DState.pointCloudMeshes[topicName].material.dispose();
                viewer3DState.pointCloudMeshes[topicName] = null;
            }

            // 실제 count만큼만 복사 (GPU 메모리 절약)
            const posArr = _pc2PosBuffer.subarray(0, count * 3).slice();
            const colArr = _pc2ColBuffer.subarray(0, count * 3).slice();

            const geo = new THREE.BufferGeometry();
            geo.setAttribute('position', new THREE.BufferAttribute(posArr, 3));
            geo.setAttribute('color',    new THREE.BufferAttribute(colArr, 3));

            const decayPoints = new THREE.Points(geo, makeMaterial(pointSize, alpha));
            viewer3DState.scene.add(decayPoints);

            const arr = viewer3DState.decayObjects.get(topicName) || [];
            arr.push({ points: decayPoints, time: performance.now() });
            viewer3DState.decayObjects.set(topicName, arr);

        } else {
            // ═══ 일반 모드: 단일 mesh in-place 업데이트 ═══
            // 혹시 decayObjects에 남은 것이 있으면 정리 (모드 전환 시)
            const decayArr = viewer3DState.decayObjects.get(topicName);
            if (decayArr && decayArr.length > 0) {
                decayArr.forEach(item => {
                    viewer3DState.scene.remove(item.points);
                    item.points.geometry.dispose();
                    item.points.material.dispose();
                });
                viewer3DState.decayObjects.set(topicName, []);
            }

            if (!mesh) {
                // ── 첫 메시지: GPU 버퍼를 MAX_POINTS 크기로 미리 할당 ──
                const posArr = new Float32Array(PC2_MAX_POINTS * 3);
                const colArr = new Float32Array(PC2_MAX_POINTS * 3);
                posArr.set(_pc2PosBuffer.subarray(0, count * 3));
                colArr.set(_pc2ColBuffer.subarray(0, count * 3));

                const posAttr = new THREE.BufferAttribute(posArr, 3);
                const colAttr = new THREE.BufferAttribute(colArr, 3);
                posAttr.setUsage(THREE.DynamicDrawUsage);
                colAttr.setUsage(THREE.DynamicDrawUsage);

                const geometry = new THREE.BufferGeometry();
                geometry.setAttribute('position', posAttr);
                geometry.setAttribute('color',    colAttr);
                geometry.setDrawRange(0, count);
                geometry.computeBoundingSphere();

                const newMesh = new THREE.Points(geometry, makeMaterial(pointSize, alpha));
                viewer3DState.pointCloudMeshes[topicName] = newMesh;
                viewer3DState.scene.add(newMesh);
                console.log('[PC2] Mesh added:', topicName, '|', count, 'pts');

            } else {
                // ── 업데이트: 기존 GPU 버퍼를 in-place 갱신 (할당 0회) ──
                const geometry = mesh.geometry;
                const posAttr  = geometry.attributes.position;
                const colAttr  = geometry.attributes.color;

                posAttr.array.set(_pc2PosBuffer.subarray(0, count * 3));
                colAttr.array.set(_pc2ColBuffer.subarray(0, count * 3));
                posAttr.needsUpdate = true;
                colAttr.needsUpdate = true;
                geometry.setDrawRange(0, count);

                // 포인트 사이즈·투명도 실시간 반영
                mesh.material.size        = pointSize;
                mesh.material.opacity     = alpha;
                mesh.material.transparent = alpha < 1.0;
                mesh.material.needsUpdate = true;

                // BoundingSphere 는 30프레임마다 1회만
                if (frameCount % 30 === 0) geometry.computeBoundingSphere();
            }
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

    // rosbridge 미연결 시 즉시 알림 (5초 대기 없이)
    if (!viewer3DState.rosConnected) {
        alert('Not connected to ROS. Make sure rosbridge_server is running:\n\nros2 launch rosbridge_server rosbridge_websocket_launch.xml');
        return;
    }

    // 버튼 로딩 표시
    const btn = document.getElementById('viewer-add-topic-btn');
    const originalText = btn ? btn.textContent : '';
    if (btn) {
        btn.textContent = 'Loading...';
        btn.disabled = true;
    }

    let topics = [];
    try {
        topics = await getAvailablePointCloudTopics();
    } finally {
        if (btn) {
            btn.textContent = originalText;
            btn.disabled = false;
        }
    }

    if (topics.length === 0) {
        alert('No PointCloud2 topics available. Make sure topics are being published.');
        return;
    }

    // Show topic selection modal
    const modal = document.getElementById('display-topic-modal');
    const topicList = document.getElementById('display-topic-list');

    if (!modal || !topicList) {
        console.error('Display topic modal not found in HTML');
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

    // 제거된 토픽: 구독 해제 + mesh dispose (unsubscribeFromTopic으로 통합 처리)
    viewer3DState.displaySelectedTopics.forEach(topic => {
        if (!newTopics.includes(topic)) {
            unsubscribeFromTopic(topic);
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

    // Display 패널 업데이트
    renderDisplayPanel();

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

/**
 * 토픽 표시/숨기기 토글
 * @param {string} topicName - 토픽명
 * @param {boolean} visible - 표시 여부
 */
function toggleTopicVisible(topicName, visible) {
    if (viewer3DState.pointCloudMeshes[topicName]) {
        viewer3DState.pointCloudMeshes[topicName].visible = visible;
        console.log(`Topic ${topicName} visibility: ${visible}`);
    }
}

/**
 * 토픽별 색상/크기 설정 업데이트
 * 다음 메시지 수신 시 새 설정이 자동 적용됨
 * @param {string} topicName - 토픽명
 * @param {string} key - 설정 키 (colorMode | colorField | solidColor | pointSize)
 * @param {*} value - 설정 값
 */
function updateTopicSetting(topicName, key, value) {
    const current = viewer3DState.topicSettings.get(topicName) || {};
    current[key] = value;
    viewer3DState.topicSettings.set(topicName, current);
    console.log(`Updated setting for ${topicName}: ${key} = ${value}`);
}

/**
 * Display 패널 동적 렌더링
 * 구독 중인 PointCloud2 / Path / Odometry 토픽 목록을 #viewer-display-panel-content 영역에 렌더링
 */
function renderDisplayPanel() {
    const container = document.getElementById('viewer-display-panel-content');
    if (!container) return;

    const pcTopics   = viewer3DState.displaySelectedTopics;
    const pathTopics = viewer3DState.selectedPathTopics;
    const odomTopics = viewer3DState.selectedOdomTopics;

    const hasAny = pcTopics.length > 0 || pathTopics.length > 0 || odomTopics.length > 0;

    if (!hasAny) {
        container.innerHTML = '<div style="font-size:12px; color: var(--muted); padding: 4px 2px;">구독 중인 토픽 없음</div>';
        return;
    }

    container.innerHTML = '';

    // ── PointCloud2 섹션 ──
    if (pcTopics.length > 0) {
        const pcLabel = document.createElement('div');
        pcLabel.className = 'display-panel-section-label';
        pcLabel.textContent = 'PointCloud2';
        container.appendChild(pcLabel);

        pcTopics.forEach(topicName => {
            const settings   = viewer3DState.topicSettings.get(topicName) || {};
            const colorMode  = settings.colorMode  || 'rainbow';
            const colorField = settings.colorField || 'intensity';
            const solidColor = settings.solidColor || '#ffffff';
            const pointSize  = settings.pointSize  !== undefined ? settings.pointSize  : 0.1;
            const alpha      = settings.alpha      !== undefined ? settings.alpha      : 1.0;
            const decayTime  = settings.decayTime  !== undefined ? settings.decayTime  : 0;
            const visible = viewer3DState.pointCloudMeshes[topicName]
                ? viewer3DState.pointCloudMeshes[topicName].visible !== false
                : true;

            // 토픽 항목 컨테이너
            const item = document.createElement('div');
            item.className = 'display-topic-item';
            item.dataset.topicName = topicName;

            // 헤더: 체크박스 + 토픽명
            const header = document.createElement('div');
            header.className = 'display-topic-item-header';

            const checkbox = document.createElement('input');
            checkbox.type = 'checkbox';
            checkbox.checked = visible;
            checkbox.title = '표시/숨기기';
            checkbox.addEventListener('change', function() {
                toggleTopicVisible(topicName, this.checked);
            });

            const nameSpan = document.createElement('span');
            nameSpan.className = 'display-topic-item-name';
            const shortName = topicName.split('/').pop() || topicName;
            nameSpan.textContent = shortName;
            nameSpan.title = topicName;

            header.appendChild(checkbox);
            header.appendChild(nameSpan);

            // 설정 영역: 색상 모드 + 필드 선택
            const settingsDiv = document.createElement('div');
            settingsDiv.className = 'display-topic-item-settings';

            // 색상 모드 선택
            const colorModeLabel = document.createElement('label');
            colorModeLabel.textContent = '색상 모드';

            const colorModeSelect = document.createElement('select');
            colorModeSelect.title = '색상 모드 선택';
            [
                { value: 'rainbow', label: 'Rainbow' },
                { value: 'rgb',     label: 'RGB' },
                { value: 'solid',   label: 'Solid' }
            ].forEach(opt => {
                const option = document.createElement('option');
                option.value = opt.value;
                option.textContent = opt.label;
                if (opt.value === colorMode) option.selected = true;
                colorModeSelect.appendChild(option);
            });
            colorModeSelect.addEventListener('change', function() {
                updateTopicSetting(topicName, 'colorMode', this.value);
                colorFieldRow.style.display = this.value === 'rainbow' ? 'flex' : 'none';
                solidColorRow.style.display = this.value === 'solid'   ? 'flex' : 'none';
            });

            // 색상 필드 선택 (Rainbow 모드)
            const colorFieldRow = document.createElement('div');
            colorFieldRow.style.display = colorMode === 'rainbow' ? 'flex' : 'none';
            colorFieldRow.style.flexDirection = 'column';
            colorFieldRow.style.gap = '2px';

            const colorFieldLabel = document.createElement('label');
            colorFieldLabel.textContent = '색상 필드';

            const colorFieldSelect = document.createElement('select');
            colorFieldSelect.title = '색상 매핑 필드 선택';
            ['intensity', 'z', 'x', 'y'].forEach(fieldName => {
                const option = document.createElement('option');
                option.value = fieldName;
                option.textContent = fieldName;
                if (fieldName === colorField) option.selected = true;
                colorFieldSelect.appendChild(option);
            });
            colorFieldSelect.addEventListener('change', function() {
                updateTopicSetting(topicName, 'colorField', this.value);
            });

            colorFieldRow.appendChild(colorFieldLabel);
            colorFieldRow.appendChild(colorFieldSelect);

            // 단색 색상 선택 (Solid 모드) - RViz 스타일 팔레트 팝업
            const solidColorRow = document.createElement('div');
            solidColorRow.style.display = colorMode === 'solid' ? 'flex' : 'none';
            solidColorRow.style.flexDirection = 'column';
            solidColorRow.style.gap = '2px';

            const solidColorLabel = document.createElement('label');
            solidColorLabel.textContent = '색상';

            const solidColorPickerRow = document.createElement('div');
            solidColorPickerRow.className = 'solid-color-picker-row';

            const colorSwatch = document.createElement('button');
            colorSwatch.className = 'solid-color-swatch';
            colorSwatch.title = '색상 선택';
            colorSwatch.style.backgroundColor = solidColor;

            const colorHexLabel = document.createElement('span');
            colorHexLabel.className = 'solid-color-hex';
            colorHexLabel.textContent = solidColor.toUpperCase();

            colorSwatch.addEventListener('click', function(e) {
                e.stopPropagation();
                openColorPickerPopup(topicName, solidColor, colorSwatch, colorHexLabel);
            });

            solidColorPickerRow.appendChild(colorSwatch);
            solidColorPickerRow.appendChild(colorHexLabel);

            solidColorRow.appendChild(solidColorLabel);
            solidColorRow.appendChild(solidColorPickerRow);

            settingsDiv.appendChild(colorModeLabel);
            settingsDiv.appendChild(colorModeSelect);
            settingsDiv.appendChild(colorFieldRow);
            settingsDiv.appendChild(solidColorRow);

            // ── Point Size 슬라이더 ──
            const sizeRow = document.createElement('div');
            sizeRow.className = 'pc2-setting-row';

            const sizeLabelEl = document.createElement('label');
            sizeLabelEl.textContent = `Size: ${pointSize.toFixed(2)}`;

            const sizeSlider = document.createElement('input');
            sizeSlider.type  = 'range';
            sizeSlider.min   = '0.01';
            sizeSlider.max   = '1.0';
            sizeSlider.step  = '0.01';
            sizeSlider.value = pointSize;
            sizeSlider.className = 'display-item-slider';
            sizeSlider.title = '포인트 크기 (m)';
            sizeSlider.addEventListener('input', function() {
                const v = parseFloat(this.value);
                sizeLabelEl.textContent = `Size: ${v.toFixed(2)}`;
                updatePointSize(topicName, v);
            });

            sizeRow.appendChild(sizeLabelEl);
            sizeRow.appendChild(sizeSlider);
            settingsDiv.appendChild(sizeRow);

            // ── Alpha (투명도) 슬라이더 ──
            const alphaRow = document.createElement('div');
            alphaRow.className = 'pc2-setting-row';

            const alphaLabelEl = document.createElement('label');
            alphaLabelEl.textContent = `Alpha: ${alpha.toFixed(2)}`;

            const alphaSlider = document.createElement('input');
            alphaSlider.type  = 'range';
            alphaSlider.min   = '0.0';
            alphaSlider.max   = '1.0';
            alphaSlider.step  = '0.05';
            alphaSlider.value = alpha;
            alphaSlider.className = 'display-item-slider';
            alphaSlider.title = '투명도 (0=완전투명, 1=불투명)';
            alphaSlider.addEventListener('input', function() {
                const v = parseFloat(this.value);
                alphaLabelEl.textContent = `Alpha: ${v.toFixed(2)}`;
                updatePointAlpha(topicName, v);
            });

            alphaRow.appendChild(alphaLabelEl);
            alphaRow.appendChild(alphaSlider);
            settingsDiv.appendChild(alphaRow);

            // ── Decay Time 슬라이더 ──
            const decayRow = document.createElement('div');
            decayRow.className = 'pc2-setting-row';

            const decayLabelEl = document.createElement('label');
            decayLabelEl.textContent = `Decay Time: ${decayTime.toFixed(1)}s`;

            const decaySlider = document.createElement('input');
            decaySlider.type  = 'range';
            decaySlider.min   = '0';
            decaySlider.max   = '30';
            decaySlider.step  = '0.5';
            decaySlider.value = decayTime;
            decaySlider.className = 'display-item-slider';
            decaySlider.title = 'Decay Time (0=최신 프레임만, >0=n초간 누적 표시)';
            decaySlider.addEventListener('input', function() {
                const v = parseFloat(this.value);
                decayLabelEl.textContent = `Decay Time: ${v.toFixed(1)}s`;
                updateTopicSetting(topicName, 'decayTime', v);
                // decay time을 0으로 낮추면 기존 누적 mesh 즉시 정리
                if (v === 0) clearDecayObjects(topicName);
            });

            decayRow.appendChild(decayLabelEl);
            decayRow.appendChild(decaySlider);
            settingsDiv.appendChild(decayRow);

            item.appendChild(header);
            item.appendChild(settingsDiv);
            container.appendChild(item);
        });
    }

    // ── Path 섹션 ──
    if (pathTopics.length > 0) {
        const pathLabel = document.createElement('div');
        pathLabel.className = 'display-panel-section-label';
        pathLabel.textContent = 'Path';
        container.appendChild(pathLabel);

        pathTopics.forEach(topicName => {
            const settings      = viewer3DState.topicSettings.get(topicName) || {};
            const pathColor     = settings.pathColor     || '#00ff00';
            const pathLineWidth = settings.pathLineWidth || 2;
            const pathAlpha     = settings.pathAlpha     !== undefined ? settings.pathAlpha : 1.0;
            const pathObj       = viewer3DState.pathObjects.get(topicName);
            const visible       = pathObj ? pathObj.line.visible !== false : true;

            const item = document.createElement('div');
            item.className = 'display-topic-item display-path-item';
            item.dataset.topicName = topicName;

            // 헤더: 체크박스 + 토픽명
            const header = document.createElement('div');
            header.className = 'display-topic-item-header';

            const checkbox = document.createElement('input');
            checkbox.type = 'checkbox';
            checkbox.checked = visible;
            checkbox.title = '경로 표시/숨기기';
            checkbox.addEventListener('change', function() {
                togglePathVisible(topicName, this.checked);
            });

            const nameSpan = document.createElement('span');
            nameSpan.className = 'display-topic-item-name';
            const shortName = topicName.split('/').pop() || topicName;
            nameSpan.textContent = shortName;
            nameSpan.title = topicName;

            header.appendChild(checkbox);
            header.appendChild(nameSpan);

            // 설정: 경로 색상
            const settingsDiv = document.createElement('div');
            settingsDiv.className = 'display-topic-item-settings';

            const colorRow = document.createElement('div');
            colorRow.className = 'display-item-color-row';

            const colorLabel = document.createElement('label');
            colorLabel.textContent = '색상';

            // RViz 스타일 팔레트 팝업 (PointCloud2 Solid와 동일한 UI)
            const colorPickerRow = document.createElement('div');
            colorPickerRow.className = 'solid-color-picker-row';

            const colorSwatch = document.createElement('button');
            colorSwatch.className = 'solid-color-swatch';
            colorSwatch.title = '경로 색상 선택';
            colorSwatch.style.backgroundColor = pathColor;

            const colorHexLabel = document.createElement('span');
            colorHexLabel.className = 'solid-color-hex';
            colorHexLabel.textContent = pathColor.toUpperCase();

            colorSwatch.addEventListener('click', function(e) {
                e.stopPropagation();
                const curColor = viewer3DState.topicSettings.get(topicName)?.pathColor || pathColor;
                openColorPickerPopup(null, curColor, colorSwatch, colorHexLabel, function(newColor) {
                    updateTopicSetting(topicName, 'pathColor', newColor);
                    updatePathColor(topicName, newColor);
                });
            });

            colorPickerRow.appendChild(colorSwatch);
            colorPickerRow.appendChild(colorHexLabel);

            colorRow.appendChild(colorLabel);
            colorRow.appendChild(colorPickerRow);
            settingsDiv.appendChild(colorRow);

            // Line Width 슬라이더
            const lineWidthRow = document.createElement('div');
            lineWidthRow.className = 'display-item-maxpts-row';

            const lineWidthLabel = document.createElement('label');
            lineWidthLabel.textContent = `Line Width: ${pathLineWidth}`;

            const lineWidthSlider = document.createElement('input');
            lineWidthSlider.type = 'range';
            lineWidthSlider.min = '1';
            lineWidthSlider.max = '10';
            lineWidthSlider.step = '1';
            lineWidthSlider.value = pathLineWidth;
            lineWidthSlider.className = 'display-item-slider';
            lineWidthSlider.title = '경로 선 굵기';
            lineWidthSlider.addEventListener('input', function() {
                const w = parseInt(this.value);
                lineWidthLabel.textContent = `Line Width: ${w}`;
                updatePathLineWidth(topicName, w);
            });

            lineWidthRow.appendChild(lineWidthLabel);
            lineWidthRow.appendChild(lineWidthSlider);
            settingsDiv.appendChild(lineWidthRow);

            // Alpha (투명도) 슬라이더
            const alphaRow = document.createElement('div');
            alphaRow.className = 'pc2-setting-row';

            const alphaLabel = document.createElement('label');
            alphaLabel.textContent = `Alpha: ${pathAlpha.toFixed(2)}`;

            const alphaSlider = document.createElement('input');
            alphaSlider.type  = 'range';
            alphaSlider.min   = '0.0';
            alphaSlider.max   = '1.0';
            alphaSlider.step  = '0.05';
            alphaSlider.value = pathAlpha;
            alphaSlider.className = 'display-item-slider';
            alphaSlider.title = '경로 투명도 (0=완전투명, 1=불투명)';
            alphaSlider.addEventListener('input', function() {
                const v = parseFloat(this.value);
                alphaLabel.textContent = `Alpha: ${v.toFixed(2)}`;
                updatePathAlpha(topicName, v);
            });

            alphaRow.appendChild(alphaLabel);
            alphaRow.appendChild(alphaSlider);
            settingsDiv.appendChild(alphaRow);

            item.appendChild(header);
            item.appendChild(settingsDiv);
            container.appendChild(item);
        });
    }

    // ── Odometry 섹션 ──
    if (odomTopics.length > 0) {
        const odomLabel = document.createElement('div');
        odomLabel.className = 'display-panel-section-label';
        odomLabel.textContent = 'Odometry';
        container.appendChild(odomLabel);

        odomTopics.forEach(topicName => {
            const settings    = viewer3DState.topicSettings.get(topicName) || {};
            const odomObj     = viewer3DState.odomObjects.get(topicName);
            const visible     = odomObj ? odomObj.group.visible !== false : true;
            const axesLength  = settings.axesLength  !== undefined ? settings.axesLength  : 0.5;
            const axesRadius  = settings.axesRadius  !== undefined ? settings.axesRadius  : 0.01;
            const maxPts      = settings.trajectoryMaxPoints !== undefined
                                    ? settings.trajectoryMaxPoints
                                    : viewer3DState.trajectoryMaxPoints;

            const item = document.createElement('div');
            item.className = 'display-topic-item display-odom-item';
            item.dataset.topicName = topicName;

            // 헤더: 체크박스 + 토픽명
            const header = document.createElement('div');
            header.className = 'display-topic-item-header';

            const checkbox = document.createElement('input');
            checkbox.type = 'checkbox';
            checkbox.checked = visible;
            checkbox.title = '로봇 마커 표시/숨기기';
            checkbox.addEventListener('change', function() {
                toggleOdomVisible(topicName, this.checked);
            });

            const nameSpan = document.createElement('span');
            nameSpan.className = 'display-topic-item-name';
            const shortName = topicName.split('/').pop() || topicName;
            nameSpan.textContent = shortName;
            nameSpan.title = topicName;

            header.appendChild(checkbox);
            header.appendChild(nameSpan);

            // 설정 영역
            const settingsDiv = document.createElement('div');
            settingsDiv.className = 'display-topic-item-settings';

            // ── Axes Length ──
            const axesLenRow = document.createElement('div');
            axesLenRow.className = 'display-item-maxpts-row';

            const axesLenLabel = document.createElement('label');
            axesLenLabel.textContent = 'Axes Length';

            const axesLenInput = document.createElement('input');
            axesLenInput.type  = 'number';
            axesLenInput.min   = '0.01';
            axesLenInput.max   = '10';
            axesLenInput.step  = '0.01';
            axesLenInput.value = axesLength;
            axesLenInput.className = 'display-item-number-input';
            axesLenInput.title = 'Axes 길이 (m)';
            axesLenInput.addEventListener('change', function() {
                const val = Math.max(0.01, parseFloat(this.value) || 0.5);
                this.value = val;
                updateTopicSetting(topicName, 'axesLength', val);
                const rVal = parseFloat(axesRadInput.value) || 0.01;
                updateOdomAxes(topicName, val, rVal);
                rebuildTrajectoryAxes(topicName);   // 기존 궤적 axes도 즉시 재생성
            });

            axesLenRow.appendChild(axesLenLabel);
            axesLenRow.appendChild(axesLenInput);

            // ── Axes Radius ──
            const axesRadRow = document.createElement('div');
            axesRadRow.className = 'display-item-maxpts-row';

            const axesRadLabel = document.createElement('label');
            axesRadLabel.textContent = 'Axes Radius';

            const axesRadInput = document.createElement('input');
            axesRadInput.type  = 'number';
            axesRadInput.min   = '0.001';
            axesRadInput.max   = '1';
            axesRadInput.step  = '0.001';
            axesRadInput.value = axesRadius;
            axesRadInput.className = 'display-item-number-input';
            axesRadInput.title = 'Axes 반경 (m)';
            axesRadInput.addEventListener('change', function() {
                const val = Math.max(0.001, parseFloat(this.value) || 0.01);
                this.value = val;
                updateTopicSetting(topicName, 'axesRadius', val);
                const lVal = parseFloat(axesLenInput.value) || 0.5;
                updateOdomAxes(topicName, lVal, val);
                rebuildTrajectoryAxes(topicName);   // 기존 궤적 axes도 즉시 재생성
            });

            axesRadRow.appendChild(axesRadLabel);
            axesRadRow.appendChild(axesRadInput);

            // ── 궤적 최대 포인트 ──
            const maxPtsRow = document.createElement('div');
            maxPtsRow.className = 'display-item-maxpts-row';

            const maxPtsLabelEl = document.createElement('label');
            maxPtsLabelEl.textContent = `Max Points: ${maxPts}`;

            const maxPtsSlider = document.createElement('input');
            maxPtsSlider.type  = 'range';
            maxPtsSlider.min   = '1';
            maxPtsSlider.max   = '1000';
            maxPtsSlider.step  = '1';
            maxPtsSlider.value = maxPts;
            maxPtsSlider.className = 'display-item-slider';
            maxPtsSlider.title = '궤적 axes 최대 표시 개수 (1~1000)';
            maxPtsSlider.addEventListener('input', function() {
                const v = parseInt(this.value);
                maxPtsLabelEl.textContent = `Max Points: ${v}`;
                updateTopicSetting(topicName, 'trajectoryMaxPoints', v);
                // 현재 궤적 버퍼가 새 한도를 초과하면 앞부분 FIFO 제거
                const trajObj = viewer3DState.trajectoryObjects.get(topicName);
                if (trajObj) {
                    while (trajObj.axesGroupList.length > v) {
                        const oldest = trajObj.axesGroupList.shift();
                        trajObj.parentGroup.remove(oldest);
                        disposeAxesGroup(oldest);
                    }
                }
            });

            maxPtsRow.appendChild(maxPtsLabelEl);
            maxPtsRow.appendChild(maxPtsSlider);

            settingsDiv.appendChild(axesLenRow);
            settingsDiv.appendChild(axesRadRow);
            settingsDiv.appendChild(maxPtsRow);

            item.appendChild(header);
            item.appendChild(settingsDiv);
            container.appendChild(item);
        });
    }
}

// =============================================
// PathRenderer: nav_msgs/Path → THREE.Line (녹색)
// =============================================

/**
 * Path 토픽 구독 및 THREE.Line으로 시각화
 * @param {string} topicName - nav_msgs/msg/Path 토픽명
 * @returns {ROSLIB.Topic} 구독 객체
 */
function subscribeToPath(topicName) {
    if (!viewer3DState.rosConnected) {
        console.warn('[Path] Not connected to ROS');
        return;
    }

    // 재구독 방지
    if (viewer3DState.topicSubscriptions.has(topicName)) {
        return viewer3DState.topicSubscriptions.get(topicName);
    }

    console.log('[Path] Subscribing to topic:', topicName);

    const topic = new ROSLIB.Topic({
        ros: viewer3DState.ros,
        name: topicName,
        messageType: 'nav_msgs/msg/Path',
        throttle_rate: 500,   // 2Hz
        queue_length: 1
    });

    viewer3DState.topicSubscriptions.set(topicName, topic);

    topic.subscribe(function(message) {
        if (!message.poses || message.poses.length === 0) return;

        // 숨겨진 경우 스킵
        const existingObj = viewer3DState.pathObjects.get(topicName);
        if (existingObj && existingObj.visible === false) return;

        // poses → THREE.Vector3 배열 변환
        const positions = message.poses.map(poseStamped => {
            const p = poseStamped.pose.position;
            return new THREE.Vector3(p.x, p.y, p.z);
        });

        if (existingObj) {
            // 기존 라인 geometry 업데이트
            existingObj.line.geometry.dispose();
            existingObj.line.geometry = new THREE.BufferGeometry().setFromPoints(positions);
        } else {
            // 신규 THREE.Line 생성 (저장된 설정 반영)
            const s       = viewer3DState.topicSettings.get(topicName) || {};
            const color   = s.pathColor     ? parseInt(s.pathColor.replace('#', ''), 16) : 0x00ff00;
            const width   = s.pathLineWidth || 2;
            const a       = s.pathAlpha     !== undefined ? s.pathAlpha : 1.0;
            const geometry = new THREE.BufferGeometry().setFromPoints(positions);
            const material = new THREE.LineBasicMaterial({
                color, linewidth: width,
                transparent: a < 1.0, opacity: a,
            });
            const line = new THREE.Line(geometry, material);
            viewer3DState.scene.add(line);
            viewer3DState.pathObjects.set(topicName, { line, visible: true });
            console.log('[Path] Line added for topic:', topicName);
        }
    });

    return topic;
}

// =============================================
// TrajectoryTracker: Odometry 기반 궤적 추적
// =============================================

/**
 * Odometry 위치를 궤적 버퍼에 추가하고 THREE.Line 업데이트
 * @param {string} topicName - Odometry 토픽명 (궤적 키)
 * @param {number} x
 * @param {number} y
 * @param {number} z
 */
/**
 * 단일 axes group을 dispose (geometry + material)
 * @param {THREE.Group} axesGroup
 */
function disposeAxesGroup(axesGroup) {
    axesGroup.traverse(function(child) {
        if (child.isMesh) {
            child.geometry.dispose();
            if (child.material) child.material.dispose();
        }
    });
}

/**
 * Odometry 위치/자세를 궤적 버퍼에 추가하고 axes group으로 시각화 (FIFO)
 * 각 포인트마다 독립적인 axes group을 생성하여 parentGroup에 추가.
 * maxPoints 초과 시 가장 오래된 axes group 제거(FIFO).
 *
 * @param {string} topicName
 * @param {number} x
 * @param {number} y
 * @param {number} z
 * @param {number} qx - Quaternion x
 * @param {number} qy - Quaternion y
 * @param {number} qz - Quaternion z
 * @param {number} qw - Quaternion w
 */
function updateTrajectory(topicName, x, y, z, qx, qy, qz, qw) {
    // per-topic 설정
    const settings  = viewer3DState.topicSettings.get(topicName) || {};
    const maxPoints = Math.min(
        settings.trajectoryMaxPoints !== undefined
            ? settings.trajectoryMaxPoints
            : viewer3DState.trajectoryMaxPoints,
        1000   // 하드 상한
    );
    const axesLength = settings.axesLength !== undefined ? settings.axesLength : 0.5;
    const axesRadius = settings.axesRadius !== undefined ? settings.axesRadius : 0.01;

    let trajObj = viewer3DState.trajectoryObjects.get(topicName);

    if (!trajObj) {
        // 최초 생성: parentGroup을 scene에 추가
        const parentGroup = new THREE.Group();
        viewer3DState.scene.add(parentGroup);
        trajObj = { parentGroup, axesGroupList: [], visible: true };
        viewer3DState.trajectoryObjects.set(topicName, trajObj);
        console.log('[Trajectory] Axes parent group created for topic:', topicName);
    }

    // 새 axes group 생성 (position + quaternion 적용)
    const axesGroup = createOdomAxesGroup(axesLength, axesRadius);
    axesGroup.position.set(x, y, z);
    axesGroup.setRotationFromQuaternion(new THREE.Quaternion(qx, qy, qz, qw));
    trajObj.parentGroup.add(axesGroup);
    trajObj.axesGroupList.push(axesGroup);

    // 최대 포인트 초과 시 FIFO: 가장 오래된 axes group 제거
    while (trajObj.axesGroupList.length > maxPoints) {
        const oldest = trajObj.axesGroupList.shift();
        trajObj.parentGroup.remove(oldest);
        disposeAxesGroup(oldest);
    }
}

// =============================================
// OdometryRenderer: nav_msgs/Odometry → THREE.Group (RViz 스타일 Axes)
// =============================================

/**
 * RViz 스타일 Axes THREE.Group 생성
 * X축: 빨강, Y축: 초록, Z축: 파랑 (cylinder + cone 화살표)
 * @param {number} length - 전체 축 길이 (m)
 * @param {number} radius - cylinder 반경 (m)
 * @returns {THREE.Group}
 */
function createOdomAxesGroup(length, radius) {
    const group = new THREE.Group();
    const radSegs  = 6;
    const stemLen  = length * 0.8;
    const headLen  = length * 0.2;
    const headRad  = radius * 2.5;

    // [색상, axisGroup.rotation(x,z)] – CylinderGeometry는 Y방향 기본
    const axesDefs = [
        { color: 0xff2020, rx: 0,           rz: -Math.PI / 2 },  // X축: 빨강
        { color: 0x20ff20, rx: 0,           rz: 0            },  // Y축: 초록
        { color: 0x2020ff, rx: Math.PI / 2, rz: 0            },  // Z축: 파랑
    ];

    axesDefs.forEach(({ color, rx, rz }) => {
        const axisGroup = new THREE.Group();
        axisGroup.rotation.set(rx, 0, rz);

        const mat = new THREE.MeshLambertMaterial({ color });

        // stem: Y축 방향 cylinder, 중심을 stemLen/2 위치로 이동
        const stemGeo = new THREE.CylinderGeometry(radius, radius, stemLen, radSegs);
        const stem    = new THREE.Mesh(stemGeo, mat);
        stem.position.y = stemLen / 2;
        axisGroup.add(stem);

        // cone (화살촉): stem 끝에서 headLen/2 추가
        const coneGeo = new THREE.ConeGeometry(headRad, headLen, radSegs);
        const cone    = new THREE.Mesh(coneGeo, mat);
        cone.position.y = stemLen + headLen / 2;
        axisGroup.add(cone);

        group.add(axisGroup);
    });

    return group;
}

/**
 * Odometry 토픽 구독 및 THREE.Group으로 로봇 위치/방향 시각화
 * 수신마다 updateTrajectory() 호출
 * @param {string} topicName - nav_msgs/msg/Odometry 토픽명
 * @returns {ROSLIB.Topic} 구독 객체
 */
function subscribeToOdometry(topicName) {
    if (!viewer3DState.rosConnected) {
        console.warn('[Odom] Not connected to ROS');
        return;
    }

    // 재구독 방지
    if (viewer3DState.topicSubscriptions.has(topicName)) {
        return viewer3DState.topicSubscriptions.get(topicName);
    }

    console.log('[Odom] Subscribing to topic:', topicName);

    const topic = new ROSLIB.Topic({
        ros: viewer3DState.ros,
        name: topicName,
        messageType: 'nav_msgs/msg/Odometry',
        throttle_rate: 100,   // 10Hz
        queue_length: 1
    });

    viewer3DState.topicSubscriptions.set(topicName, topic);

    topic.subscribe(function(message) {
        const pos = message.pose.pose.position;
        const ori = message.pose.pose.orientation;

        const x = pos.x;
        const y = pos.y;
        const z = pos.z;

        let existingObj = viewer3DState.odomObjects.get(topicName);

        if (!existingObj) {
            // RViz 스타일 axes 그룹 최초 생성
            const group = new THREE.Group();
            const settings    = viewer3DState.topicSettings.get(topicName) || {};
            const axesLength  = settings.axesLength !== undefined ? settings.axesLength : 0.5;
            const axesRadius  = settings.axesRadius !== undefined ? settings.axesRadius : 0.01;

            const axesGroup = createOdomAxesGroup(axesLength, axesRadius);
            group.add(axesGroup);

            group.position.set(x, y, z);
            viewer3DState.scene.add(group);

            existingObj = { group, axesGroup, visible: true };
            viewer3DState.odomObjects.set(topicName, existingObj);
            console.log('[Odom] Axes group added for topic:', topicName);
        } else {
            // 숨겨진 경우 스킵
            if (existingObj.visible === false) return;
        }

        // 위치 업데이트
        existingObj.group.position.set(x, y, z);

        // Quaternion으로 그룹 회전 업데이트
        const q = new THREE.Quaternion(ori.x, ori.y, ori.z, ori.w);
        existingObj.group.setRotationFromQuaternion(q);

        // 궤적 업데이트 (quaternion 전달 → 각 포인트에 axes 방향 적용)
        updateTrajectory(topicName, x, y, z, ori.x, ori.y, ori.z, ori.w);
    });

    return topic;
}

// =============================================
// 토픽 조회 함수: Path / Odometry
// =============================================

/**
 * 사용 가능한 nav_msgs/Path 토픽 목록 조회
 * @returns {Promise<string[]>} Path 토픽 이름 배열
 */
async function getAvailablePathTopics() {
    if (!viewer3DState.rosConnected) {
        console.log('[Path] Waiting for ROS connection...');
        const connected = await waitForROSConnection(5000);
        if (!connected) {
            console.warn('[Path] Not connected to ROS after timeout');
            return [];
        }
    }

    return new Promise((resolve) => {
        viewer3DState.ros.getTopics(function(topics) {
            const pathTopics = [];
            if (topics.topics && topics.types) {
                topics.topics.forEach((topic, index) => {
                    const type = topics.types[index];
                    if (type === 'nav_msgs/Path' || type === 'nav_msgs/msg/Path') {
                        pathTopics.push(topic);
                    }
                });
            }
            console.log('[Path] Available Path topics:', pathTopics);
            resolve(pathTopics);
        }, function(error) {
            console.error('[Path] Failed to get topics:', error);
            resolve([]);
        });
    });
}

/**
 * 사용 가능한 nav_msgs/Odometry 토픽 목록 조회
 * @returns {Promise<string[]>} Odometry 토픽 이름 배열
 */
async function getAvailableOdometryTopics() {
    if (!viewer3DState.rosConnected) {
        console.log('[Odom] Waiting for ROS connection...');
        const connected = await waitForROSConnection(5000);
        if (!connected) {
            console.warn('[Odom] Not connected to ROS after timeout');
            return [];
        }
    }

    return new Promise((resolve) => {
        viewer3DState.ros.getTopics(function(topics) {
            const odomTopics = [];
            if (topics.topics && topics.types) {
                topics.topics.forEach((topic, index) => {
                    const type = topics.types[index];
                    if (type === 'nav_msgs/Odometry' || type === 'nav_msgs/msg/Odometry') {
                        odomTopics.push(topic);
                    }
                });
            }
            console.log('[Odom] Available Odometry topics:', odomTopics);
            resolve(odomTopics);
        }, function(error) {
            console.error('[Odom] Failed to get topics:', error);
            resolve([]);
        });
    });
}

// =============================================
// Path / Odometry 가시성 토글 및 색상 업데이트
// =============================================

/**
 * Path 라인 표시/숨기기
 * @param {string} topicName
 * @param {boolean} visible
 */
function togglePathVisible(topicName, visible) {
    const pathObj = viewer3DState.pathObjects.get(topicName);
    if (pathObj) {
        pathObj.line.visible = visible;
        pathObj.visible = visible;
        console.log(`[Path] ${topicName} visibility: ${visible}`);
    }
}

/**
 * Odometry 로봇 마커 그룹 표시/숨기기
 * @param {string} topicName
 * @param {boolean} visible
 */
function toggleOdomVisible(topicName, visible) {
    const odomObj = viewer3DState.odomObjects.get(topicName);
    if (odomObj) {
        odomObj.group.visible = visible;
        odomObj.visible = visible;
        console.log(`[Odom] ${topicName} visibility: ${visible}`);
    }
    // 궤적 axes parentGroup도 함께 show/hide
    const trajObj = viewer3DState.trajectoryObjects.get(topicName);
    if (trajObj) {
        trajObj.parentGroup.visible = visible;
        trajObj.visible = visible;
    }
}

/**
 * Path 라인 선 굵기 업데이트 및 설정 저장
 * @param {string} topicName
 * @param {number} width - 1~10
 */
function updatePathLineWidth(topicName, width) {
    const pathObj = viewer3DState.pathObjects.get(topicName);
    if (pathObj) {
        pathObj.line.material.linewidth = width;
        pathObj.line.material.needsUpdate = true;
    }
    updateTopicSetting(topicName, 'pathLineWidth', width);
}

/**
 * Path 라인 투명도(Alpha) 즉시 업데이트
 * @param {string} topicName
 * @param {number} alpha - 0.0(완전투명) ~ 1.0(불투명)
 */
function updatePathAlpha(topicName, alpha) {
    const pathObj = viewer3DState.pathObjects.get(topicName);
    if (pathObj) {
        pathObj.line.material.opacity     = alpha;
        pathObj.line.material.transparent = alpha < 1.0;
        pathObj.line.material.needsUpdate = true;
    }
    updateTopicSetting(topicName, 'pathAlpha', alpha);
}

/**
 * Odometry axes 크기(length, radius) 업데이트
 * 기존 axesGroup을 dispose하고 새로운 크기로 재생성
 * @param {string} topicName
 * @param {number} length
 * @param {number} radius
 */
function updateOdomAxes(topicName, length, radius) {
    const odomObj = viewer3DState.odomObjects.get(topicName);
    if (!odomObj) return;

    // 기존 axesGroup 제거 및 dispose
    if (odomObj.axesGroup) {
        odomObj.group.remove(odomObj.axesGroup);
        odomObj.axesGroup.traverse(function(child) {
            if (child.isMesh) {
                child.geometry.dispose();
                if (child.material) child.material.dispose();
            }
        });
    }

    // 새 axesGroup 생성 및 추가
    const newAxes = createOdomAxesGroup(length, radius);
    odomObj.group.add(newAxes);
    odomObj.axesGroup = newAxes;

    console.log(`[Odom] Axes updated for ${topicName}: length=${length}, radius=${radius}`);
}

/**
 * 기존 궤적 axes group 전체를 현재 설정(axesLength/axesRadius)으로 재생성
 * Axes Length 또는 Axes Radius 변경 시 호출하여 즉시 반영
 * @param {string} topicName
 */
function rebuildTrajectoryAxes(topicName) {
    const trajObj = viewer3DState.trajectoryObjects.get(topicName);
    if (!trajObj || trajObj.axesGroupList.length === 0) return;

    const settings   = viewer3DState.topicSettings.get(topicName) || {};
    const axesLength = settings.axesLength !== undefined ? settings.axesLength : 0.5;
    const axesRadius = settings.axesRadius !== undefined ? settings.axesRadius : 0.01;

    // 각 axes group의 position/quaternion 보존하며 재생성
    const rebuilt = trajObj.axesGroupList.map(oldGroup => {
        const pos  = oldGroup.position.clone();
        const quat = oldGroup.quaternion.clone();

        trajObj.parentGroup.remove(oldGroup);
        disposeAxesGroup(oldGroup);

        const newGroup = createOdomAxesGroup(axesLength, axesRadius);
        newGroup.position.copy(pos);
        newGroup.setRotationFromQuaternion(quat);
        trajObj.parentGroup.add(newGroup);
        return newGroup;
    });

    trajObj.axesGroupList = rebuilt;
    console.log(`[Trajectory] Axes rebuilt for ${topicName}: length=${axesLength}, radius=${axesRadius}, count=${rebuilt.length}`);
}

/**
 * Path 라인 색상 업데이트 및 설정 저장
 * @param {string} topicName
 * @param {string} colorHex - '#rrggbb' 형태
 */
function updatePathColor(topicName, colorHex) {
    const pathObj = viewer3DState.pathObjects.get(topicName);
    if (pathObj) {
        const colorInt = parseInt(colorHex.replace('#', ''), 16);
        pathObj.line.material.color.setHex(colorInt);
    }
    updateTopicSetting(topicName, 'pathColor', colorHex);
}

// updateTrajectoryColor 제거: axes 방식에서는 X/Y/Z 고정 색상 사용 (RViz 스타일)

// =============================================
// PointCloud2 런타임 속성 업데이트
// =============================================

/**
 * PointCloud2 포인트 사이즈 즉시 업데이트
 * @param {string} topicName
 * @param {number} size - 포인트 크기(m)
 */
function updatePointSize(topicName, size) {
    updateTopicSetting(topicName, 'pointSize', size);
    const mesh = viewer3DState.pointCloudMeshes[topicName];
    if (mesh) {
        mesh.material.size = size;
        mesh.material.needsUpdate = true;
    }
    // decay 모드의 누적 mesh들도 즉시 반영
    const arr = viewer3DState.decayObjects.get(topicName);
    if (arr) arr.forEach(item => {
        item.points.material.size = size;
        item.points.material.needsUpdate = true;
    });
}

/**
 * PointCloud2 투명도(Alpha) 즉시 업데이트
 * @param {string} topicName
 * @param {number} alpha - 0.0(완전투명) ~ 1.0(불투명)
 */
function updatePointAlpha(topicName, alpha) {
    updateTopicSetting(topicName, 'alpha', alpha);
    const mesh = viewer3DState.pointCloudMeshes[topicName];
    if (mesh) {
        mesh.material.opacity     = alpha;
        mesh.material.transparent = alpha < 1.0;
        mesh.material.needsUpdate = true;
    }
    const arr = viewer3DState.decayObjects.get(topicName);
    if (arr) arr.forEach(item => {
        item.points.material.opacity     = alpha;
        item.points.material.transparent = alpha < 1.0;
        item.points.material.needsUpdate = true;
    });
}

/**
 * Decay Time 변경 시 해당 토픽의 누적 decay mesh 전체 제거
 * @param {string} topicName
 */
function clearDecayObjects(topicName) {
    const arr = viewer3DState.decayObjects.get(topicName);
    if (arr) {
        arr.forEach(item => {
            viewer3DState.scene.remove(item.points);
            item.points.geometry.dispose();
            item.points.material.dispose();
        });
        viewer3DState.decayObjects.set(topicName, []);
    }
}

// =============================================
// Path 토픽 선택 UI (selectPathTopics)
// =============================================

/**
 * Path 토픽 선택 다이얼로그 열기
 */
async function selectPathTopics() {
    if (!viewer3DState.rosConnected) {
        alert('Not connected to ROS. Make sure rosbridge_server is running:\n\nros2 launch rosbridge_server rosbridge_websocket_launch.xml');
        return;
    }

    const btn = document.getElementById('viewer-add-path-btn');
    const originalText = btn ? btn.textContent : '';
    if (btn) { btn.textContent = 'Loading...'; btn.disabled = true; }

    let topics = [];
    try {
        topics = await getAvailablePathTopics();
    } finally {
        if (btn) { btn.textContent = originalText; btn.disabled = false; }
    }

    if (topics.length === 0) {
        alert('No nav_msgs/Path topics available.\nMake sure path topics are being published.');
        return;
    }

    const modal     = document.getElementById('path-topic-modal');
    const topicList = document.getElementById('path-topic-list');
    if (!modal || !topicList) {
        console.error('[Path] Topic modal element not found in HTML');
        return;
    }

    topicList.innerHTML = '';
    topics.forEach(topic => {
        const div = document.createElement('div');
        div.className = 'topic-item';

        const checkbox = document.createElement('input');
        checkbox.type    = 'checkbox';
        checkbox.id      = `path-topic-cb-${topic}`;
        checkbox.value   = topic;
        checkbox.checked = viewer3DState.selectedPathTopics.includes(topic);

        const label = document.createElement('label');
        label.htmlFor     = `path-topic-cb-${topic}`;
        label.textContent = topic;

        div.appendChild(checkbox);
        div.appendChild(label);
        topicList.appendChild(div);
    });

    modal.style.display = 'block';
}

/**
 * Path 토픽 선택 모달 닫기
 */
function closePathTopicSelection() {
    const modal = document.getElementById('path-topic-modal');
    if (modal) modal.style.display = 'none';
}

/**
 * Path 토픽 선택 확인: 체크된 토픽 구독, 해제된 토픽 정리
 */
function confirmPathTopicSelection() {
    const checkboxes = document.querySelectorAll('#path-topic-list input[type="checkbox"]');
    const newTopics  = [];
    checkboxes.forEach(cb => { if (cb.checked) newTopics.push(cb.value); });

    // 제거된 토픽: 구독 해제 및 Three.js 리소스 정리
    viewer3DState.selectedPathTopics.forEach(topic => {
        if (!newTopics.includes(topic)) {
            unsubscribeFromTopic(topic);
        }
    });

    // 신규 토픽: 구독 시작
    newTopics.forEach(topic => {
        if (!viewer3DState.selectedPathTopics.includes(topic)) {
            subscribeToPath(topic);
        }
    });

    viewer3DState.selectedPathTopics = newTopics;
    closePathTopicSelection();
    renderDisplayPanel();
    console.log('[Path] Selected topics:', viewer3DState.selectedPathTopics);
}

// =============================================
// Odometry 토픽 선택 UI (selectOdometryTopics)
// =============================================

/**
 * Odometry 토픽 선택 다이얼로그 열기
 */
async function selectOdometryTopics() {
    if (!viewer3DState.rosConnected) {
        alert('Not connected to ROS. Make sure rosbridge_server is running:\n\nros2 launch rosbridge_server rosbridge_websocket_launch.xml');
        return;
    }

    const btn = document.getElementById('viewer-add-odom-btn');
    const originalText = btn ? btn.textContent : '';
    if (btn) { btn.textContent = 'Loading...'; btn.disabled = true; }

    let topics = [];
    try {
        topics = await getAvailableOdometryTopics();
    } finally {
        if (btn) { btn.textContent = originalText; btn.disabled = false; }
    }

    if (topics.length === 0) {
        alert('No nav_msgs/Odometry topics available.\nMake sure odometry topics are being published.');
        return;
    }

    const modal     = document.getElementById('odom-topic-modal');
    const topicList = document.getElementById('odom-topic-list');
    if (!modal || !topicList) {
        console.error('[Odom] Topic modal element not found in HTML');
        return;
    }

    topicList.innerHTML = '';
    topics.forEach(topic => {
        const div = document.createElement('div');
        div.className = 'topic-item';

        const checkbox = document.createElement('input');
        checkbox.type    = 'checkbox';
        checkbox.id      = `odom-topic-cb-${topic}`;
        checkbox.value   = topic;
        checkbox.checked = viewer3DState.selectedOdomTopics.includes(topic);

        const label = document.createElement('label');
        label.htmlFor     = `odom-topic-cb-${topic}`;
        label.textContent = topic;

        div.appendChild(checkbox);
        div.appendChild(label);
        topicList.appendChild(div);
    });

    modal.style.display = 'block';
}

/**
 * Odometry 토픽 선택 모달 닫기
 */
function closeOdometryTopicSelection() {
    const modal = document.getElementById('odom-topic-modal');
    if (modal) modal.style.display = 'none';
}

/**
 * Odometry 토픽 선택 확인: 체크된 토픽 구독, 해제된 토픽 정리
 */
function confirmOdometryTopicSelection() {
    const checkboxes = document.querySelectorAll('#odom-topic-list input[type="checkbox"]');
    const newTopics  = [];
    checkboxes.forEach(cb => { if (cb.checked) newTopics.push(cb.value); });

    // 제거된 토픽: 구독 해제 및 Three.js 리소스 정리
    viewer3DState.selectedOdomTopics.forEach(topic => {
        if (!newTopics.includes(topic)) {
            unsubscribeFromTopic(topic);
        }
    });

    // 신규 토픽: 구독 시작
    newTopics.forEach(topic => {
        if (!viewer3DState.selectedOdomTopics.includes(topic)) {
            subscribeToOdometry(topic);
        }
    });

    viewer3DState.selectedOdomTopics = newTopics;
    closeOdometryTopicSelection();
    renderDisplayPanel();
    console.log('[Odom] Selected topics:', viewer3DState.selectedOdomTopics);
}

// Expose functions to global scope for onclick handlers
window.initThreeJSDisplay = initThreeJSDisplay;
window.initialize3DViewer = initialize3DViewer;
window.selectDisplayTopics = selectDisplayTopics;
window.closeDisplayTopicSelection = closeDisplayTopicSelection;
window.confirmDisplayTopicSelection = confirmDisplayTopicSelection;
window.moveRendererToActiveContainer = moveRendererToActiveContainer;
window.renderDisplayPanel = renderDisplayPanel;
window.toggleTopicVisible = toggleTopicVisible;
window.updateTopicSetting = updateTopicSetting;
window.closeColorPickerPopup = closeColorPickerPopup;
window.confirmColorPickerPopup = confirmColorPickerPopup;
window.subscribeToPath = subscribeToPath;
window.subscribeToOdometry = subscribeToOdometry;
window.updateTrajectory = updateTrajectory;
window.getAvailablePathTopics = getAvailablePathTopics;
window.getAvailableOdometryTopics = getAvailableOdometryTopics;
// Path / Odometry 가시성 토글 및 색상 업데이트
window.togglePathVisible = togglePathVisible;
window.toggleOdomVisible = toggleOdomVisible;
window.updatePathColor = updatePathColor;
// Path / Odometry 토픽 선택 UI
window.selectPathTopics = selectPathTopics;
window.closePathTopicSelection = closePathTopicSelection;
window.confirmPathTopicSelection = confirmPathTopicSelection;
window.selectOdometryTopics = selectOdometryTopics;
window.closeOdometryTopicSelection = closeOdometryTopicSelection;
window.confirmOdometryTopicSelection = confirmOdometryTopicSelection;

console.log('=== Three.js Display script loaded ===');
console.log('Functions exposed:', {
    initThreeJSDisplay: typeof initThreeJSDisplay,
    initialize3DViewer: typeof initialize3DViewer,
    selectDisplayTopics: typeof selectDisplayTopics
});
