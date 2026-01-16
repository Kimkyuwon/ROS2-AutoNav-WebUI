// DataBuffer 클래스 - 시계열 데이터 버퍼 관리 (ROS time 기준)
class DataBuffer {
    constructor(bufferTime = 5.0) {
        this.bufferTime = bufferTime;  // 버퍼 시간 (초 단위)
        this.timestamps = [];          // X축 데이터 (ROS time)
        this.values = [];              // Y축 데이터
    }

    addData(timestamp, value) {
        this.timestamps.push(timestamp);
        this.values.push(value);

        // ROS time 기준: 현재 timestamp - bufferTime보다 오래된 데이터 삭제
        const cutoffTime = timestamp - this.bufferTime;
        while (this.timestamps.length > 0 && this.timestamps[0] < cutoffTime) {
            this.timestamps.shift();
            this.values.shift();
        }
    }

    getData() {
        return {
            timestamps: [...this.timestamps],
            values: [...this.values]
        };
    }

    getLength() {
        return this.timestamps.length;
    }

    clear() {
        this.timestamps = [];
        this.values = [];
    }

    isEmpty() {
        return this.timestamps.length === 0;
    }

    setBufferTime(bufferTime) {
        this.bufferTime = bufferTime;
        
        // 현재 가장 최신 timestamp 기준으로 오래된 데이터 삭제
        if (this.timestamps.length > 0) {
            const latestTimestamp = this.timestamps[this.timestamps.length - 1];
            const cutoffTime = latestTimestamp - this.bufferTime;
            
            while (this.timestamps.length > 0 && this.timestamps[0] < cutoffTime) {
                this.timestamps.shift();
                this.values.shift();
            }
        }
    }
}

// PlotlyPlotManager 클래스 - Plotly.js를 이용한 Plot 생성 및 업데이트
class PlotlyPlotManager {
    constructor(containerId, bufferTime = 5.0) {  // 기본 5초 (ROS time 기준)
        this.containerId = containerId;
        this.container = null;
        this.bufferTime = bufferTime;  // 버퍼 시간 (초 단위, ROS time 기준)
        this.dataBuffers = new Map();  // path -> DataBuffer 매핑
        this.traces = [];              // Plotly traces
        this.isInitialized = false;
        this.updateThrottleMs = 100;   // 10Hz throttling
        this.lastUpdateTime = 0;
        this.pendingUpdate = false;
        this.firstTimestamp = null;    // 첫 데이터 포인트의 timestamp (t0 기준)
        this.t0Mode = false;           // t0 모드 (상대 시간 표시)
        this.isPaused = false;         // 일시정지 상태
        this.autoScaleRange = null;    // 오토 스케일 범위 (줌 제한용)
    }

    init() {
        this.container = document.getElementById(this.containerId);
        if (!this.container) {
            console.error(`[PlotlyPlotManager] Container ${this.containerId} not found`);
            return false;
        }

        // Plotly.js 로드 확인
        if (typeof Plotly === 'undefined') {
            console.error('[PlotlyPlotManager] Plotly.js not loaded');
            return false;
        }

        console.log('[PlotlyPlotManager] Initialized');
        return true;
    }

    createPlot(paths) {
        if (!this.init()) {
            return false;
        }

        // paths가 배열이 아니면 배열로 변환
        if (!Array.isArray(paths)) {
            paths = [paths];
        }

        if (paths.length === 0) {
            console.warn('[PlotlyPlotManager] No paths provided');
            return false;
        }

        console.log('[PlotlyPlotManager] Creating plot for paths:', paths);

        // 이미 초기화된 경우 trace만 추가
        if (this.isInitialized) {
            console.log('[PlotlyPlotManager] Plot already exists, adding traces...');
            return this.addTraces(paths);
        }

        // 기존 Plot 초기화 (처음 생성 시에만)
        this.clear();

        // 각 path별로 DataBuffer와 trace 생성
        paths.forEach((path, index) => {
            // DataBuffer 생성 (ROS time 기준 버퍼)
            const buffer = new DataBuffer(this.bufferTime);
            this.dataBuffers.set(path, buffer);
            console.log(`[PlotlyPlotManager] Created buffer for path: "${path}" (bufferTime: ${this.bufferTime}s)`);

            // Plotly trace 생성
            const trace = {
                x: [],
                y: [],
                mode: 'lines',
                name: path,
                type: 'scatter',
                line: {
                    width: 2
                }
            };
            this.traces.push(trace);
        });

        // Plotly layout 설정 (흰색 배경, 검은색 격자선)
        const layout = {
            title: {
                text: paths.length === 1 ? `Plot: ${paths[0]}` : `Plot: ${paths.length} items`,
                font: {
                    color: '#000000',
                    size: 14
                }
            },
            xaxis: {
                title: {
                    text: 'Time (seconds)',
                    font: { color: '#000000' }
                },
                showgrid: true,
                gridcolor: '#cccccc',
                gridwidth: 1,
                zeroline: true,
                zerolinecolor: '#000000',
                zerolinewidth: 1,
                tickfont: { color: '#000000' },
                exponentformat: 'e',  // 과학적 표기법 (1.23e+08)
                showexponent: 'all'
            },
            yaxis: {
                title: {
                    text: 'Value',
                    font: { color: '#000000' }
                },
                showgrid: true,
                gridcolor: '#cccccc',
                gridwidth: 1,
                zeroline: true,
                zerolinecolor: '#000000',
                zerolinewidth: 1,
                tickfont: { color: '#000000' },
                exponentformat: 'e',  // 과학적 표기법 (1.23e+08)
                showexponent: 'all'
            },
            showlegend: true,
            legend: {
                x: 1,
                xanchor: 'right',
                y: 1,
                yanchor: 'top',
                bgcolor: 'rgba(255, 255, 255, 0.9)',
                bordercolor: '#000000',
                borderwidth: 1,
                font: { color: '#000000' }
            },
            margin: {
                l: 60,
                r: 120,
                b: 50,
                t: 50
            },
            paper_bgcolor: '#ffffff',
            plot_bgcolor: '#ffffff',
            font: {
                color: '#000000',
                family: 'Arial, sans-serif'
            }
        };

        // Plotly config 설정
        const config = {
            responsive: true,
            displayModeBar: true,
            modeBarButtonsToRemove: ['lasso2d', 'select2d', 'zoomIn2d', 'zoomOut2d', 'autoScale2d', 'resetScale2d'],  // +/-, reset axes 버튼 제거
            displaylogo: false,
            scrollZoom: true,  // 마우스 휠 줌 활성화 (일시정지 시에만 작동, dragmode로 제어)
            modeBarButtonsToAdd: [
                {
                    name: 'Pause/Play',
                    icon: {
                        width: 1000,
                        height: 1000,
                        // 일시정지 아이콘 (두 개의 수직 막대)
                        path: 'M300,200 L300,800 L400,800 L400,200 Z M600,200 L600,800 L700,800 L700,200 Z',
                        transform: 'matrix(1 0 0 -1 0 1000)'
                    },
                    click: (gd) => {
                        this.togglePause();
                    }
                },
                {
                    name: 't0 (Relative Time)',
                    icon: {
                        width: 1000,
                        height: 1000,
                        // 시계 모양 아이콘
                        path: 'M500,100 A400,400 0 1,1 500,900 A400,400 0 1,1 500,100 M500,300 L500,500 L650,650',
                        transform: 'matrix(1 0 0 -1 0 1000)'
                    },
                    click: (gd) => {
                        this.toggleT0Mode();
                    }
                },
                {
                    name: 'Zoom Out (Auto Scale)',
                    icon: {
                        width: 1000,
                        height: 1000,
                        // 돋보기 - (줌아웃) 아이콘
                        path: 'M450,200 A250,250 0 1,1 450,700 A250,250 0 1,1 450,200 M350,450 L550,450 M600,650 L800,850',
                        transform: 'matrix(1 0 0 -1 0 1000)'
                    },
                    click: (gd) => {
                        this.zoomOutAutoScale();
                    }
                },
                {
                    name: 'Clear Plot (Reset)',
                    icon: {
                        width: 1000,
                        height: 1000,
                        // 쓰레기통 모양 아이콘
                        path: 'M300,200 L300,800 L700,800 L700,200 Z M250,200 L750,200 M350,150 L650,150 M400,350 L400,700 M500,350 L500,700 M600,350 L600,700',
                        transform: 'matrix(1 0 0 -1 0 1000)'
                    },
                    click: (gd) => {
                        this.clearPlot();
                    }
                }
            ]
        };

        // Plotly Plot 생성
        try {
            Plotly.newPlot(this.containerId, this.traces, layout, config);
            this.isInitialized = true;
            
            // 타이틀 더블클릭 이벤트 추가
            this.setupTitleEditor();
            
            // 커스텀 컨텍스트 메뉴 추가
            this.setupContextMenu();
            
            // 줌 제한 설정
            this.setupZoomLimiter();
            
            // 마우스 휠 제어 (일시정지 시에만 줌 가능)
            this.setupWheelControl();

            // 모드바(Plotly 툴바) 가드: 재생 중에는 Zoom/Pan 관련 버튼이 동작하지 않도록 차단
            this.setupModeBarGuards();
            // 초기 상태(재생)에서는 버튼 비활성화 상태로 표시
            setTimeout(() => this.updateModeBarButtonStates(), 200);
            
            console.log('[PlotlyPlotManager] Plot created successfully');
            return true;
        } catch (error) {
            console.error('[PlotlyPlotManager] Failed to create plot:', error);
            return false;
        }
    }

    setupContextMenu() {
        const plotDiv = document.getElementById(this.containerId);
        let contextMenuPosition = { x: 0, y: 0 };  // 마우스 클릭 위치 저장
        
        // 기존 컨텍스트 메뉴 제거
        plotDiv.addEventListener('contextmenu', (e) => {
            e.preventDefault();
            
            // 마우스 위치 저장 (plot 영역 기준)
            const rect = plotDiv.getBoundingClientRect();
            contextMenuPosition.x = e.clientX - rect.left;
            contextMenuPosition.y = e.clientY - rect.top;
            
            // 기존 메뉴 제거
            const existingMenu = document.getElementById('plot-context-menu');
            if (existingMenu) {
                existingMenu.remove();
            }
            
            // 커스텀 메뉴 생성
            const menu = document.createElement('div');
            menu.id = 'plot-context-menu';
            menu.style.cssText = `
                position: absolute;
                left: ${e.pageX}px;
                top: ${e.pageY}px;
                background: #ffffff;
                border: 1px solid #ccc;
                border-radius: 4px;
                box-shadow: 0 2px 10px rgba(0,0,0,0.2);
                z-index: 10000;
                font-size: 13px;
                min-width: 180px;
            `;
            
            const menuItems = [
                { label: '📷 Save plot to file', action: () => this.savePlotToFile() },
                { label: '↔️ Auto Scale', action: () => this.zoomOutAutoScale() },
                { separator: true },
                { label: '➗ Split Horizontally (Coming soon)', action: () => console.log('Coming soon'), disabled: true },
                { label: '➗ Split Vertically (Coming soon)', action: () => console.log('Coming soon'), disabled: true }
            ];
            
            menuItems.forEach(item => {
                if (item.separator) {
                    const separator = document.createElement('div');
                    separator.style.cssText = 'height: 1px; background: #ddd; margin: 4px 0;';
                    menu.appendChild(separator);
                } else {
                    const menuItem = document.createElement('div');
                    menuItem.textContent = item.label;
                    menuItem.style.cssText = `
                        padding: 8px 16px;
                        cursor: ${item.disabled ? 'not-allowed' : 'pointer'};
                        color: ${item.disabled ? '#999' : '#000'};
                        background: transparent;
                    `;
                    
                    if (!item.disabled) {
                        menuItem.onmouseenter = () => {
                            menuItem.style.background = '#f0f0f0';
                        };
                        menuItem.onmouseleave = () => {
                            menuItem.style.background = 'transparent';
                        };
                        menuItem.onclick = () => {
                            item.action();
                            menu.remove();
                        };
                    }
                    
                    menu.appendChild(menuItem);
                }
            });
            
            document.body.appendChild(menu);
            
            // 외부 클릭 시 메뉴 닫기
            const closeMenu = (event) => {
                if (!menu.contains(event.target)) {
                    menu.remove();
                    document.removeEventListener('click', closeMenu);
                }
            };
            setTimeout(() => {
                document.addEventListener('click', closeMenu);
            }, 100);
        });
    }

    savePlotToFile() {
        console.log('[PlotlyPlotManager] Saving plot to file...');
        
        Plotly.downloadImage(this.containerId, {
            format: 'png',
            width: 1200,
            height: 800,
            filename: `plot_${Date.now()}`
        });
        
        console.log('[PlotlyPlotManager] Plot saved.');
    }

    setupZoomLimiter() {
        const plotDiv = document.getElementById(this.containerId);
        
        // relayout 이벤트 감지
        plotDiv.on('plotly_relayout', (eventData) => {
            // 오토스케일 버튼이나 더블클릭 시
            if (eventData['xaxis.autorange'] || eventData['yaxis.autorange']) {
                // autorange가 true가 되면 Plotly가 알아서 범위를 맞춤 -> 우리가 저장할 필요는 없음 (updatePlot에서 계속 저장 중)
                return;
            }
            
            // 일시정지 상태가 아니면 줌 제한하지 않음
            if (!this.isPaused || !this.autoScaleRange) {
                return;
            }
            
            // 현재 범위 확인
            let currentXRange = null;
            let currentYRange = null;
            
            if (eventData['xaxis.range[0]'] !== undefined && eventData['xaxis.range[1]'] !== undefined) {
                currentXRange = [eventData['xaxis.range[0]'], eventData['xaxis.range[1]']];
            } else if (eventData['xaxis.range']) {
                currentXRange = eventData['xaxis.range'];
            }
            
            if (eventData['yaxis.range[0]'] !== undefined && eventData['yaxis.range[1]'] !== undefined) {
                currentYRange = [eventData['yaxis.range[0]'], eventData['yaxis.range[1]']];
            } else if (eventData['yaxis.range']) {
                currentYRange = eventData['yaxis.range'];
            }
            
            // 범위 제한 체크
            const needsCorrection = {};
            
            if (currentXRange) {
                const xSpan = currentXRange[1] - currentXRange[0];
                const autoXSpan = this.autoScaleRange.x[1] - this.autoScaleRange.x[0];
                
                // 오토스케일 범위보다 5% 이상 더 커지면 (줌 아웃 과도)
                if (xSpan > autoXSpan * 1.05) {
                    needsCorrection['xaxis.range'] = this.autoScaleRange.x;
                    console.log('[PlotlyPlotManager] X-axis zoom limited to auto scale');
                }
            }
            
            if (currentYRange) {
                const ySpan = currentYRange[1] - currentYRange[0];
                const autoYSpan = this.autoScaleRange.y[1] - this.autoScaleRange.y[0];
                
                if (ySpan > autoYSpan * 1.05) {
                    needsCorrection['yaxis.range'] = this.autoScaleRange.y;
                    console.log('[PlotlyPlotManager] Y-axis zoom limited to auto scale');
                }
            }
            
            if (Object.keys(needsCorrection).length > 0) {
                Plotly.relayout(this.containerId, needsCorrection);
            }
        });
    }

    setupWheelControl() {
        const plotDiv = document.getElementById(this.containerId);
        
        // wheel 이벤트를 가로채서 일시정지 상태에서만 허용
        plotDiv.addEventListener('wheel', (e) => {
            if (!this.isPaused) {
                // 재생 중에는 줌 방지
                e.preventDefault();
                e.stopPropagation();
                return false;
            }
            // 일시정지 상태에서는 기본 줌 동작 허용
        }, { passive: false, capture: true });
        
        console.log('[PlotlyPlotManager] Wheel control setup (paused mode only)');
    }

    _isZoomPanModeBarButtonTitle(title) {
        if (!title) return false;
        const t = String(title).toLowerCase();
        // Plotly 기본 버튼 타이틀들 (브라우저/버전에 따라 약간 다를 수 있음)
        // - Zoom / Pan / Box Select / Lasso Select / Zoom in / Zoom out 등
        // + 우리가 추가한 'Zoom Out (Auto Scale)' 도 재생 중에는 막는다.
        return (
            t.includes('zoom') ||
            t.includes('pan') ||
            t.includes('box select') ||
            t.includes('lasso')
        );
    }

    setupModeBarGuards() {
        const plotDiv = document.getElementById(this.containerId);
        if (!plotDiv) return;

        // 캡처 단계에서 클릭을 막아 Plotly 내부 핸들러보다 먼저 처리
        plotDiv.addEventListener('click', (e) => {
            const btn = e.target && e.target.closest ? e.target.closest('.modebar-btn') : null;
            if (!btn) return;

            const title = btn.getAttribute('data-title') || '';
            if (!this.isPaused && this._isZoomPanModeBarButtonTitle(title)) {
                e.preventDefault();
                e.stopPropagation();

                // 재생 중에는 dragmode가 바뀌지 않도록 즉시 복원
                Plotly.relayout(this.containerId, { dragmode: false });
                return false;
            }
        }, true);
    }

    updateModeBarButtonStates() {
        const plotDiv = document.getElementById(this.containerId);
        const modeBar = plotDiv ? plotDiv.querySelector('.modebar') : null;
        if (!modeBar) return;

        const buttons = modeBar.querySelectorAll('.modebar-btn[data-title]');
        buttons.forEach((btn) => {
            const title = btn.getAttribute('data-title') || '';
            if (!this._isZoomPanModeBarButtonTitle(title)) return;

            // 재생 중: 비활성화(클릭 불가) + 흐리게 표시
            if (!this.isPaused) {
                btn.style.pointerEvents = 'none';
                btn.style.opacity = '0.35';
                btn.style.filter = 'grayscale(1)';
            } else {
                // 일시정지: 활성화
                btn.style.pointerEvents = 'auto';
                btn.style.opacity = '1';
                btn.style.filter = 'none';
            }
        });
    }

    setupTitleEditor() {
        // Plot 타이틀 요소 찾기 (Plotly의 title text 요소)
        // 약간의 지연을 두고 DOM이 완전히 생성된 후 찾기
        setTimeout(() => {
            const titleElement = this.container.querySelector('.gtitle') || 
                                 this.container.querySelector('text.gtitle') ||
                                 this.container.querySelector('.g-gtitle');
            
            if (!titleElement) {
                console.warn('[PlotlyPlotManager] Title element not found, trying SVG text...');
                // SVG text 요소 중 타이틀 찾기
                const svgTexts = this.container.querySelectorAll('text');
                for (let text of svgTexts) {
                    if (text.textContent && text.textContent.startsWith('Plot:')) {
                        this.attachTitleClickHandler(text);
                        console.log('[PlotlyPlotManager] Title editor setup complete (via SVG text)');
                        return;
                    }
                }
                console.warn('[PlotlyPlotManager] Could not find title element');
                return;
            }

            this.attachTitleClickHandler(titleElement);
            console.log('[PlotlyPlotManager] Title editor setup complete');
        }, 100);
    }

    attachTitleClickHandler(titleElement) {
        titleElement.style.cursor = 'pointer';
        titleElement.addEventListener('dblclick', () => {
            const currentTitle = titleElement.textContent;
            this.showTitleEditDialog(currentTitle);
        });
    }

    showTitleEditDialog(currentTitle) {
        // 다이얼로그 생성
        const dialog = document.createElement('div');
        dialog.style.cssText = `
            position: fixed;
            top: 50%;
            left: 50%;
            transform: translate(-50%, -50%);
            background: white;
            padding: 20px;
            border: 2px solid #333;
            border-radius: 8px;
            box-shadow: 0 4px 12px rgba(0,0,0,0.3);
            z-index: 10000;
            min-width: 300px;
        `;

        // 제목
        const dialogTitle = document.createElement('h3');
        dialogTitle.textContent = 'Edit Plot Title';
        dialogTitle.style.cssText = 'margin: 0 0 15px 0; color: #333;';
        dialog.appendChild(dialogTitle);

        // 입력창
        const input = document.createElement('input');
        input.type = 'text';
        input.value = currentTitle;
        input.style.cssText = `
            width: 100%;
            padding: 8px;
            margin-bottom: 15px;
            border: 1px solid #ccc;
            border-radius: 4px;
            font-size: 14px;
            box-sizing: border-box;
        `;
        dialog.appendChild(input);

        // 버튼 컨테이너
        const buttonContainer = document.createElement('div');
        buttonContainer.style.cssText = 'display: flex; gap: 10px; justify-content: flex-end;';

        // OK 버튼
        const okButton = document.createElement('button');
        okButton.textContent = 'OK';
        okButton.style.cssText = `
            padding: 8px 16px;
            background: #007bff;
            color: white;
            border: none;
            border-radius: 4px;
            cursor: pointer;
            font-size: 14px;
        `;
        okButton.onmouseover = () => okButton.style.background = '#0056b3';
        okButton.onmouseout = () => okButton.style.background = '#007bff';
        okButton.onclick = () => {
            const newTitle = input.value.trim();
            if (newTitle) {
                Plotly.relayout(this.containerId, {
                    'title.text': newTitle
                });
            }
            document.body.removeChild(overlay);
            document.body.removeChild(dialog);
        };

        // Cancel 버튼
        const cancelButton = document.createElement('button');
        cancelButton.textContent = 'Cancel';
        cancelButton.style.cssText = `
            padding: 8px 16px;
            background: #6c757d;
            color: white;
            border: none;
            border-radius: 4px;
            cursor: pointer;
            font-size: 14px;
        `;
        cancelButton.onmouseover = () => cancelButton.style.background = '#5a6268';
        cancelButton.onmouseout = () => cancelButton.style.background = '#6c757d';
        cancelButton.onclick = () => {
            document.body.removeChild(overlay);
            document.body.removeChild(dialog);
        };

        buttonContainer.appendChild(cancelButton);
        buttonContainer.appendChild(okButton);
        dialog.appendChild(buttonContainer);

        // 배경 오버레이
        const overlay = document.createElement('div');
        overlay.style.cssText = `
            position: fixed;
            top: 0;
            left: 0;
            width: 100%;
            height: 100%;
            background: rgba(0, 0, 0, 0.5);
            z-index: 9999;
        `;
        overlay.onclick = () => {
            document.body.removeChild(overlay);
            document.body.removeChild(dialog);
        };

        // DOM에 추가
        document.body.appendChild(overlay);
        document.body.appendChild(dialog);

        // 입력창에 포커스
        input.focus();
        input.select();

        // Enter 키로 OK
        input.onkeydown = (e) => {
            if (e.key === 'Enter') {
                okButton.click();
            } else if (e.key === 'Escape') {
                cancelButton.click();
            }
        };
    }

    updatePlot(path, timestamp, value) {
        if (!this.isInitialized) {
            console.warn('[PlotlyPlotManager] Plot not initialized');
            return;
        }

        // DataBuffer에 데이터 추가
        const buffer = this.dataBuffers.get(path);
        if (!buffer) {
            console.warn(`[PlotlyPlotManager] Buffer not found for path: ${path}`);
            console.warn('[PlotlyPlotManager] Available buffers:', Array.from(this.dataBuffers.keys()));
            return;
        }

        buffer.addData(timestamp, value);
        
        // 첫 timestamp 저장 (t0 기준)
        if (this.firstTimestamp === null) {
            this.firstTimestamp = timestamp;
            console.log(`[PlotlyPlotManager] First timestamp set: ${this.firstTimestamp}`);
        }
        
        // 첫 10개 데이터만 로그
        if (buffer.getLength() <= 10) {
            console.log(`[PlotlyPlotManager] Data added for ${path}: t=${timestamp}, v=${value}, buffer size=${buffer.getLength()}`);
        }

        // 일시정지 상태면 plot 업데이트 하지 않음 (데이터는 버퍼에 계속 저장됨)
        if (this.isPaused) {
            return;
        }

        // Throttling: 10Hz로 업데이트 제한
        const now = Date.now();
        if (now - this.lastUpdateTime < this.updateThrottleMs) {
            // 이미 pending update가 있으면 스킵
            if (!this.pendingUpdate) {
                this.pendingUpdate = true;
                setTimeout(() => {
                    if (!this.isPaused) {  // 일시정지 상태 재확인
                        this._updatePlotly();
                    }
                    this.pendingUpdate = false;
                }, this.updateThrottleMs - (now - this.lastUpdateTime));
            }
            return;
        }

        this._updatePlotly();
    }

    _updatePlotly() {
        if (!this.isInitialized) {
            return;
        }

        try {
            // 각 trace의 데이터 업데이트
            const updateData = {
                x: [],
                y: []
            };

            let totalPoints = 0;
            this.traces.forEach((trace, index) => {
                // trace.name을 사용하여 올바른 buffer 찾기 (인덱스 매칭 대신)
                const path = trace.name;
                const buffer = this.dataBuffers.get(path);
                
                if (buffer && !buffer.isEmpty()) {
                    const data = buffer.getData();
                    
                    // t0 모드: 첫 timestamp를 0으로 만들어 상대 시간 표시
                    let timestamps = data.timestamps;
                    if (this.t0Mode && this.firstTimestamp !== null) {
                        timestamps = data.timestamps.map(t => t - this.firstTimestamp);
                    }
                    
                    updateData.x.push(timestamps);
                    updateData.y.push(data.values);
                    totalPoints += data.timestamps.length;
                } else {
                    // 버퍼가 없거나 비어있으면 빈 배열
                    updateData.x.push([]);
                    updateData.y.push([]);
                }
            });

            // 첫 5번만 상세 로그
            if (this.lastUpdateTime === 0 || Date.now() - this.lastUpdateTime > 5000) {
                console.log(`[PlotlyPlotManager] Updating plot: ${totalPoints} total points`);
                console.log('[PlotlyPlotManager] Update data:', {
                    traces: this.traces.length,
                    xArrays: updateData.x.length,
                    yArrays: updateData.y.length,
                    firstXSample: updateData.x[0] ? updateData.x[0].slice(0, 3) : [],
                    firstYSample: updateData.y[0] ? updateData.y[0].slice(0, 3) : []
                });
            }

            // Plotly update - restyle 사용 (더 안전)
            // update 대신 각 trace별로 x, y 데이터 업데이트
            updateData.x.forEach((xData, index) => {
                Plotly.restyle(this.containerId, {
                    x: [xData],
                    y: [updateData.y[index]]
                }, [index]);
            });
            
            this.lastUpdateTime = Date.now();

            // 오토스케일 범위 업데이트 (줌 제한을 위해 데이터 범위 저장)
            this._updateAutoScaleRange(updateData);

        } catch (error) {
            console.error('[PlotlyPlotManager] Failed to update plot:', error);
        }
    }

    _updateAutoScaleRange(updateData) {
        if (!updateData.x || updateData.x.length === 0) return;

        let minX = Infinity, maxX = -Infinity;
        let minY = Infinity, maxY = -Infinity;

        updateData.x.forEach((xArr, i) => {
            if (!xArr || xArr.length === 0) return;
            const yArr = updateData.y[i];
            
            // X 범위
            const xMinLocal = Math.min(...xArr);
            const xMaxLocal = Math.max(...xArr);
            if (xMinLocal < minX) minX = xMinLocal;
            if (xMaxLocal > maxX) maxX = xMaxLocal;

            // Y 범위
            if (yArr && yArr.length > 0) {
                const yMinLocal = Math.min(...yArr);
                const yMaxLocal = Math.max(...yArr);
                if (yMinLocal < minY) minY = yMinLocal;
                if (yMaxLocal > maxY) maxY = yMaxLocal;
            }
        });

        if (minX !== Infinity && maxX !== -Infinity) {
            // 여유분 5% 추가
            const xPadding = (maxX - minX) * 0.05;
            const yPadding = (maxY - minY) * 0.05;

            this.autoScaleRange = {
                x: [minX - xPadding, maxX + xPadding],
                y: [minY - yPadding, maxY + yPadding]
            };
        }
    }

    clear() {
        // 모든 이벤트 리스너 제거
        this.eventListeners.forEach(listener => {
            listener.element.removeEventListener(listener.event, listener.handler);
        });
        this.eventListeners = [];
        
        // legendHoverTimeout 취소 (향후 setupLegendHover() 추가 시 사용)
        if (this.legendHoverTimeout !== null) {
            clearTimeout(this.legendHoverTimeout);
            this.legendHoverTimeout = null;
        }
        
        this.dataBuffers.clear();
        this.traces = [];
        this.isInitialized = false;
        this.lastUpdateTime = 0;
        this.pendingUpdate = false;

        if (this.container) {
            this.container.innerHTML = '';
        }
    }

    addTraces(paths) {
        if (!this.isInitialized) {
            console.warn('[PlotlyPlotManager] Plot not initialized, cannot add traces');
            return false;
        }

        // paths가 배열이 아니면 배열로 변환
        if (!Array.isArray(paths)) {
            paths = [paths];
        }

        console.log('[PlotlyPlotManager] Adding traces to existing plot:', paths);

        // 새로운 trace 생성
        const newTraces = [];
        paths.forEach((path) => {
            // 이미 존재하는 path면 스킵
            if (this.dataBuffers.has(path)) {
                console.log(`[PlotlyPlotManager] Path already exists: ${path}`);
                return;
            }

            // DataBuffer 생성 (ROS time 기준 버퍼)
            const buffer = new DataBuffer(this.bufferTime);
            this.dataBuffers.set(path, buffer);
            console.log(`[PlotlyPlotManager] Created buffer for path: "${path}" (bufferTime: ${this.bufferTime}s)`);

            // Plotly trace 생성
            const trace = {
                x: [],
                y: [],
                mode: 'lines',
                name: path,
                type: 'scatter',
                line: {
                    width: 2
                }
            };
            // this.traces에는 나중에 추가 (Plotly.addTraces 성공 후)
            newTraces.push(trace);
        });

        if (newTraces.length === 0) {
            console.log('[PlotlyPlotManager] No new traces to add');
            return true;
        }

        // Plotly에 trace 추가
        try {
            const plotDiv = document.getElementById(this.containerId);
            const beforeCount = plotDiv.data ? plotDiv.data.length : 0;
            
            console.log(`[PlotlyPlotManager] BEFORE adding: Plotly has ${beforeCount} traces`);
            console.log(`[PlotlyPlotManager] this.traces.length = ${this.traces.length}`);
            console.log(`[PlotlyPlotManager] Adding traces:`, newTraces.map(t => t.name));
            
            // this.traces에 추가
            newTraces.forEach(trace => {
                this.traces.push(trace);
            });
            
            // ⚠️ 중요: Plotly.react를 사용하여 전체 traces를 다시 렌더링
            // Plotly.addTraces는 내부적으로 이상한 동작을 하므로 사용하지 않음
            const currentLayout = plotDiv.layout;
            Plotly.react(this.containerId, this.traces, currentLayout);
            
            const afterCount = plotDiv.data ? plotDiv.data.length : 0;
            console.log(`[PlotlyPlotManager] AFTER adding: Plotly has ${afterCount} traces (expected ${this.traces.length})`);
            console.log(`[PlotlyPlotManager] Added ${newTraces.length} traces successfully`);
            return true;
        } catch (error) {
            console.error('[PlotlyPlotManager] Failed to add traces:', error);
            return false;
        }
    }

    toggleT0Mode() {
        this.t0Mode = !this.t0Mode;
        console.log(`[PlotlyPlotManager] t0 mode: ${this.t0Mode ? 'ON' : 'OFF'}`);
        
        if (this.t0Mode && this.firstTimestamp === null) {
            console.warn('[PlotlyPlotManager] No data yet, t0 mode will activate when data arrives');
            return;
        }
        
        // X축 라벨 변경
        const xAxisTitle = this.t0Mode ? 'Time (seconds, relative to t0)' : 'Time (seconds)';
        
        const plotDiv = document.getElementById(this.containerId);
        const layout = plotDiv && plotDiv.layout ? plotDiv.layout : null;

        // 재생 중: autorange 유지 (실시간 데이터 따라감)
        if (!this.isPaused) {
            Plotly.relayout(this.containerId, {
                'xaxis.title.text': xAxisTitle,
                'xaxis.autorange': true
            });
            // 즉시 업데이트
            this._updatePlotly();
        } 
        // 일시정지 중: 현재 보고 있는 뷰 유지하면서 좌표 변환
        else if (layout && layout.xaxis && Array.isArray(layout.xaxis.range)) {
            const currentXRange = layout.xaxis.range;
            let newXRange;
            
            if (this.firstTimestamp !== null) {
                if (this.t0Mode) {
                    // 절대 -> 상대: 값에서 firstTimestamp 빼기
                    // 단, 이미 상대값인 경우(버그 방지) 체크는 어렵지만, 
                    // toggle이므로 이전 상태가 절대값이면 현재값들은 절대값임.
                    newXRange = [currentXRange[0] - this.firstTimestamp, currentXRange[1] - this.firstTimestamp];
                } else {
                    // 상대 -> 절대: 값에 firstTimestamp 더하기
                    newXRange = [currentXRange[0] + this.firstTimestamp, currentXRange[1] + this.firstTimestamp];
                }
            }

            const relayoutArgs = { 'xaxis.title.text': xAxisTitle };
            if (newXRange) {
                relayoutArgs['xaxis.autorange'] = false; // 줌 상태 유지
                relayoutArgs['xaxis.range'] = newXRange;
            }
            
            // 뷰 업데이트
            Plotly.relayout(this.containerId, relayoutArgs).then(() => {
                // 데이터도 변환된 좌표계로 다시 그려야 함
                // 일시정지 상태에서 _updatePlotly() 호출 허용을 위해 잠시 플래그 조작 또는 강제 실행
                // 하지만 _updatePlotly는 this.isPaused 체크를 하지 않으므로 그냥 호출하면 됨
                // 단, updatePlot 메서드 내부에서만 체크함. _updatePlotly 자체는 체크 안함.
                this._updatePlotly();
            });
        } else {
            // 레이아웃 정보 없으면 라벨만 변경
            Plotly.relayout(this.containerId, { 'xaxis.title.text': xAxisTitle });
            this._updatePlotly();
        }
        
        console.log(`[PlotlyPlotManager] t0 mode toggled, X-axis ${this.t0Mode ? 'relative' : 'absolute'}`);
    }

    togglePause() {
        this.isPaused = !this.isPaused;
        console.log(`[PlotlyPlotManager] Plot ${this.isPaused ? 'PAUSED' : 'RESUMED'}`);
        
        const plotDiv = document.getElementById(this.containerId);
        
        if (this.isPaused) {
            // 일시정지: 줌 및 팬 모드 활성화
            Plotly.relayout(this.containerId, {
                'dragmode': 'zoom'  // 박스 선택으로 줌, 팬은 shift+드래그
            });
            
            console.log('[PlotlyPlotManager] Zoom/Pan enabled (paused mode)');
        } else {
            // 재생: 자동 오토스케일 후 모든 상호작용 비활성화
            this.zoomOutAutoScale();  // 자동 오토스케일
            
            // dragmode를 false로 설정하여 드래그 비활성화
            Plotly.relayout(this.containerId, {
                'dragmode': false
            });
            
            console.log('[PlotlyPlotManager] All interactions disabled (playing mode), auto scaled');
        }

        // 재생/일시정지 상태에 따라 모드바 버튼 활성/비활성 업데이트
        setTimeout(() => this.updateModeBarButtonStates(), 50);
        
        // 버튼 아이콘 업데이트
        const modeBar = plotDiv.querySelector('.modebar');
        
        if (modeBar) {
            // 모든 버튼을 순회하여 Pause/Play 버튼 찾기
            const buttons = modeBar.querySelectorAll('[data-title]');
            let pauseButton = null;
            
            for (const btn of buttons) {
                const title = btn.getAttribute('data-title');
                if (title && (title.includes('Pause') || title.includes('Play'))) {
                    pauseButton = btn;
                    break;
                }
            }
            
            if (pauseButton) {
                const path = pauseButton.querySelector('path');
                if (path) {
                    if (this.isPaused) {
                        // 재생 아이콘 (삼각형)
                        path.setAttribute('d', 'M300,200 L300,800 L800,500 Z');
                        pauseButton.setAttribute('data-title', 'Play (Zoom enabled)');
                        console.log('[PlotlyPlotManager] Button changed to PLAY icon');
                    } else {
                        // 일시정지 아이콘 (두 개의 수직 막대)
                        path.setAttribute('d', 'M300,200 L300,800 L400,800 L400,200 Z M600,200 L600,800 L700,800 L700,200 Z');
                        pauseButton.setAttribute('data-title', 'Pause/Play');
                        console.log('[PlotlyPlotManager] Button changed to PAUSE icon');
                    }
                }
            } else {
                console.warn('[PlotlyPlotManager] Pause button not found');
            }
        }
        
        // 일시정지 해제 시 즉시 plot 업데이트
        if (!this.isPaused) {
            this._updatePlotly();
        }
    }

    zoomOutAutoScale() {
        console.log('[PlotlyPlotManager] Zoom out (auto scale)...');
        
        // Plot을 오토 스케일 (축 범위를 데이터에 맞게 자동 조정)
        Plotly.relayout(this.containerId, {
            'xaxis.autorange': true,
            'yaxis.autorange': true
        });
        
        console.log('[PlotlyPlotManager] Plot auto scaled.');
    }

    clearPlot() {
        console.log('[PlotlyPlotManager] Clearing plot data...');
        
        // 모든 버퍼 초기화 (버퍼 객체는 유지, 데이터만 삭제)
        this.dataBuffers.forEach((buffer, path) => {
            buffer.clear();
            console.log(`[PlotlyPlotManager] Cleared buffer for: ${path}`);
        });
        
        // firstTimestamp 리셋
        this.firstTimestamp = null;
        
        // t0 모드는 유지 (사용자 요청)
        
        // Plot 리셋 (autoscale)
        Plotly.relayout(this.containerId, {
            'xaxis.autorange': true,
            'yaxis.autorange': true
        });
        
        // 즉시 plot 업데이트 (빈 데이터로)
        this._updatePlotly();
        
        console.log('[PlotlyPlotManager] Plot cleared. New messages will be plotted from now on. t0 mode preserved.');
    }

    setBufferTime(seconds) {
        console.log(`[PlotlyPlotManager] Setting buffer time to ${seconds} seconds (ROS time based)`);
        
        // PlotManager의 bufferTime 업데이트
        this.bufferTime = seconds;
        
        // 모든 DataBuffer의 bufferTime 업데이트 (기존 데이터는 시간 기준으로 유지/삭제)
        this.dataBuffers.forEach((buffer, path) => {
            const beforeLength = buffer.getLength();
            buffer.setBufferTime(seconds);
            const afterLength = buffer.getLength();
            console.log(`[PlotlyPlotManager] Updated buffer for ${path}: ${beforeLength} → ${afterLength} points`);
        });
        
        console.log(`[PlotlyPlotManager] Buffer time updated. All buffers now use ${seconds}s window (ROS time based)`);
    }

    destroy() {
        if (this.isInitialized && this.container) {
            try {
                Plotly.purge(this.containerId);
            } catch (error) {
                console.error('[PlotlyPlotManager] Failed to purge plot:', error);
            }
        }
        this.clear();
    }
}