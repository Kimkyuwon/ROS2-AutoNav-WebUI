// PlotJugglerTree 클래스 - PlotJuggler 스타일의 메시지 트리 구조
class PlotJugglerTree {
    constructor(containerId) {
        this.containerId = containerId;
        this.container = null;
        this.rootNode = null;
        this.selectedItems = new Set(); // 선택된 항목 관리 (Set 사용)
        this.nodeMap = new Map(); // path -> node 매핑
        this.messageData = new Map(); // path -> value 매핑 (최신 값 저장)
    }

    init() {
        this.container = document.getElementById(this.containerId);
        if (!this.container) {
            console.error(`[PlotJugglerTree] Container ${this.containerId} not found`);
            return;
        }

        // 이미 초기화되었으면 스킵
        if (this.rootNode && this.container.contains(this.rootNode)) {
            console.log('[PlotJugglerTree] Already initialized, skipping...');
            return;
        }

        // 트리 컨테이너 초기화 - 기존 메시지만 제거
        const existingMessages = this.container.querySelectorAll('div[style*="color"]');
        existingMessages.forEach(msg => {
            if (!msg.classList.contains('plot-tree-root') && !msg.classList.contains('plot-tree-node')) {
                msg.remove();
            }
        });

        // 루트 노드 생성 - 실제로는 childrenContainer만 사용
        this.rootNode = document.createElement('div');
        this.rootNode.className = 'plot-tree-root';
        this.rootNode.style.display = 'block';
        
        // childrenContainer 직접 생성
        this.rootNode.childrenContainer = document.createElement('div');
        this.rootNode.childrenContainer.className = 'plot-tree-root-children';
        this.rootNode.childrenContainer.style.display = 'block';
        this.rootNode.childrenContainer.style.marginLeft = '0';
        this.rootNode.childrenContainer.style.borderLeft = 'none';
        
        this.rootNode.appendChild(this.rootNode.childrenContainer);
        this.container.appendChild(this.rootNode);
        
        console.log('[PlotJugglerTree] Initialized, container:', this.container, 'rootNode:', this.rootNode);
    }

    createNode(label, path, isLeaf) {
        const node = document.createElement('div');
        node.className = 'plot-tree-node';
        node.dataset.path = path;
        node.dataset.isLeaf = String(isLeaf);

        // 노드 내용 (2열 레이아웃: 아이콘 | 이름 | 값)
        const content = document.createElement('div');
        content.className = 'plot-tree-node-content';

        // 확장/축소 아이콘 (리프 노드가 아닐 때만)
        if (!isLeaf) {
            const expandIcon = document.createElement('span');
            expandIcon.className = 'plot-tree-expand-icon';
            expandIcon.textContent = '▶';
            expandIcon.onclick = (e) => {
                e.stopPropagation();
                e.preventDefault();
                this.toggleExpand(node);
            };
            expandIcon.onmousedown = (e) => {
                e.stopPropagation();
            };
            content.appendChild(expandIcon);
        } else {
            // 리프 노드일 때는 공간 확보를 위한 빈 아이콘
            const spacer = document.createElement('span');
            spacer.className = 'plot-tree-expand-icon';
            spacer.style.visibility = 'hidden';
            spacer.style.width = '16px';
            spacer.style.display = 'inline-block';
            content.appendChild(spacer);
        }

        // 라벨 (이름 컬럼)
        const labelElement = document.createElement('span');
        labelElement.className = 'plot-tree-label';
        labelElement.textContent = label;
        content.appendChild(labelElement);

        // 값 표시 (리프 노드만, 값 컬럼)
        const valueElement = document.createElement('span');
        valueElement.className = 'plot-tree-value';
        if (isLeaf) {
            valueElement.textContent = '-';
            node.valueElement = valueElement;
        } else {
            valueElement.textContent = '';
        }
        content.appendChild(valueElement);

        node.appendChild(content);

        // 자식 노드 컨테이너
        const childrenContainer = document.createElement('div');
        childrenContainer.className = 'plot-tree-children';
        childrenContainer.style.display = 'none';
        node.childrenContainer = childrenContainer;
        node.appendChild(childrenContainer);

        // 리프 노드만 드래그 가능 및 선택 가능
        if (isLeaf) {
            node.draggable = true;
            node.classList.add('plot-tree-leaf');

            // 드래그 이벤트
            node.addEventListener('dragstart', (e) => {
                // 선택된 모든 항목의 path를 배열로 전달
                const selectedPaths = this.getSelectedPaths();
                const pathsToDrag = selectedPaths.length > 0 && selectedPaths.includes(path)
                    ? selectedPaths
                    : [path]; // 선택된 항목이 없거나 현재 항목이 선택되지 않았으면 현재 항목만
                
                // JSON 형식으로 배열 전달
                e.dataTransfer.setData('text/plain', JSON.stringify(pathsToDrag));
                e.dataTransfer.effectAllowed = 'move';
                
                // 드래그 중인 모든 선택된 노드에 스타일 적용
                pathsToDrag.forEach(p => {
                    const n = this.nodeMap.get(p);
                    if (n) {
                        n.classList.add('plot-tree-dragging');
                    }
                });
                
                console.log('[dragstart] Dragging paths:', pathsToDrag);
            });

            node.addEventListener('dragend', (e) => {
                // 모든 드래그 중인 노드의 스타일 제거
                const allNodes = this.container.querySelectorAll('.plot-tree-dragging');
                allNodes.forEach(n => n.classList.remove('plot-tree-dragging'));
            });

            // 클릭 이벤트 (복수 선택)
            node.addEventListener('click', (e) => {
                // 확장 아이콘 클릭은 제외
                if (e.target.classList.contains('plot-tree-expand-icon')) {
                    return;
                }
                
                e.stopPropagation();
                
                if (e.ctrlKey || e.metaKey) {
                    // Ctrl/Cmd + 클릭: 복수 선택
                    this.toggleSelection(node);
                } else {
                    // 일반 클릭: 단일 선택
                    this.setSelection(node);
                }
            });
            
            // 우클릭 컨텍스트 메뉴 이벤트 (리프 노드만)
            node.addEventListener('contextmenu', (e) => {
                e.preventDefault();
                e.stopPropagation();
                
                const selectedPaths = this.getSelectedPaths();
                if (selectedPaths.length === 2) {
                    this.showContextMenu(e.clientX, e.clientY, selectedPaths);
                }
            });
        } else {
            // 비리프 노드는 더블클릭으로 재귀적 확장/축소
            node.addEventListener('dblclick', (e) => {
                e.stopPropagation();
                this.toggleExpandRecursive(node);
            });
        }

        // 노드 맵에 추가
        if (path) {
            this.nodeMap.set(path, node);
        }

        return node;
    }

    findChildByName(parent, name) {
        if (!parent || !parent.childrenContainer) {
            return null;
        }

        const children = parent.childrenContainer.children;
        for (let i = 0; i < children.length; i++) {
            const child = children[i];
            const labelElement = child.querySelector('.plot-tree-label');
            if (labelElement && labelElement.textContent === name) {
                return child;
            }
        }
        return null;
    }

    addItem(fullPath, value = null) {
        if (!fullPath) {
            console.warn('[addItem] Empty path');
            return;
        }

        // 경로를 /로 분리
        // 예: "rosout/stamp/sec" -> ["rosout", "stamp", "sec"]
        // 배열 인덱스 표기 [0]는 이름의 일부로 유지
        const parts = fullPath.split('/').filter(p => p.length > 0);
        
        if (parts.length === 0) {
            console.warn('[addItem] No valid parts in path:', fullPath);
            return;
        }

        let currentParent = this.rootNode;
        let currentPath = '';

        for (let i = 0; i < parts.length; i++) {
            const part = parts[i];
            const isLeaf = (i === parts.length - 1);
            currentPath = currentPath ? `${currentPath}/${part}` : part;

            let child = this.findChildByName(currentParent, part);

            if (!child) {
                child = this.createNode(part, currentPath, isLeaf);
                currentParent.childrenContainer.appendChild(child);
                console.log(`[addItem] Created ${isLeaf ? 'leaf' : 'branch'} node: ${currentPath}`);
            }

            currentParent = child;
        }

        // 리프 노드인 경우 값 업데이트
        if (value !== null && currentParent && currentParent.valueElement) {
            this.updateValue(currentParent.dataset.path, value);
        }
    }

    updateValue(path, value) {
        // 최신 값 저장
        this.messageData.set(path, value);
        
        const node = this.nodeMap.get(path);
        if (node && node.valueElement) {
            // 숫자인 경우 포맷팅
            if (typeof value === 'number') {
                node.valueElement.textContent = value.toFixed(3);
            } else if (typeof value === 'boolean') {
                node.valueElement.textContent = value ? 'true' : 'false';
            } else if (value === null || value === undefined) {
                node.valueElement.textContent = '-';
            } else {
                const strValue = String(value);
                // 너무 긴 문자열은 잘라서 표시
                if (strValue.length > 20) {
                    node.valueElement.textContent = strValue.substring(0, 17) + '...';
                } else {
                    node.valueElement.textContent = strValue;
                }
            }
        }
    }

    toggleExpand(node) {
        const childrenContainer = node.childrenContainer;
        const expandIcon = node.querySelector('.plot-tree-expand-icon');

        if (childrenContainer.style.display === 'none' || childrenContainer.style.display === '') {
            childrenContainer.style.display = 'block';
            if (expandIcon) {
                expandIcon.textContent = '▼';
            }
            node.classList.add('plot-tree-expanded');
        } else {
            childrenContainer.style.display = 'none';
            if (expandIcon) {
                expandIcon.textContent = '▶';
            }
            node.classList.remove('plot-tree-expanded');
        }
    }

    toggleExpandRecursive(node) {
        const childrenContainer = node.childrenContainer;
        const expandIcon = node.querySelector('.plot-tree-expand-icon');
        const isExpanded = childrenContainer.style.display !== 'none' && childrenContainer.style.display !== '';

        // 현재 노드 확장/축소
        if (isExpanded) {
            childrenContainer.style.display = 'none';
            if (expandIcon) {
                expandIcon.textContent = '▶';
            }
            node.classList.remove('plot-tree-expanded');
        } else {
            childrenContainer.style.display = 'block';
            if (expandIcon) {
                expandIcon.textContent = '▼';
            }
            node.classList.add('plot-tree-expanded');
        }

        // 모든 자식 노드도 재귀적으로 확장/축소
        const children = childrenContainer.children;
        for (let i = 0; i < children.length; i++) {
            const child = children[i];
            if (child.dataset.isLeaf === 'false') {
                this.toggleExpandRecursive(child);
            }
        }
    }

    expandAll() {
        const allNodes = this.container.querySelectorAll('.plot-tree-node');
        allNodes.forEach(node => {
            if (node.childrenContainer && node.childrenContainer.children.length > 0) {
                const childrenContainer = node.childrenContainer;
                const expandIcon = node.querySelector('.plot-tree-expand-icon');
                childrenContainer.style.display = 'block';
                if (expandIcon) {
                    expandIcon.textContent = '▼';
                }
                node.classList.add('plot-tree-expanded');
            }
        });
    }

    collapseAll() {
        const allNodes = this.container.querySelectorAll('.plot-tree-node');
        allNodes.forEach(node => {
            if (node.childrenContainer) {
                const childrenContainer = node.childrenContainer;
                const expandIcon = node.querySelector('.plot-tree-expand-icon');
                childrenContainer.style.display = 'none';
                if (expandIcon) {
                    expandIcon.textContent = '▶';
                }
                node.classList.remove('plot-tree-expanded');
            }
        });
    }

    toggleSelection(node) {
        if (node.dataset.isLeaf !== 'true') {
            return;
        }

        const path = node.dataset.path;
        if (this.selectedItems.has(path)) {
            this.selectedItems.delete(path);
            node.classList.remove('plot-tree-selected');
            console.log('[toggleSelection] Deselected:', path);
        } else {
            this.selectedItems.add(path);
            node.classList.add('plot-tree-selected');
            console.log('[toggleSelection] Selected:', path);
        }
        console.log('[toggleSelection] Total selected:', this.selectedItems.size);
    }

    setSelection(node) {
        // 기존 선택 해제
        this.selectedItems.forEach(path => {
            const selectedNode = this.nodeMap.get(path);
            if (selectedNode) {
                selectedNode.classList.remove('plot-tree-selected');
            }
        });
        this.selectedItems.clear();

        // 새 선택
        if (node && node.dataset.isLeaf === 'true') {
            const path = node.dataset.path;
            this.selectedItems.add(path);
            node.classList.add('plot-tree-selected');
            console.log('[setSelection] Selected:', path);
        }
    }

    getSelectedPaths() {
        return Array.from(this.selectedItems);
    }

    clearSelection() {
        this.selectedItems.forEach(path => {
            const node = this.nodeMap.get(path);
            if (node) {
                node.classList.remove('plot-tree-selected');
            }
        });
        this.selectedItems.clear();
    }

    showContextMenu(x, y, paths) {
        // 기존 컨텍스트 메뉴 제거
        this.hideContextMenu();
        
        // 컨텍스트 메뉴 생성
        const menu = document.createElement('div');
        menu.id = 'plot-tree-context-menu';
        menu.style.cssText = `
            position: fixed;
            left: ${x}px;
            top: ${y}px;
            background: #ffffff;
            border: 1px solid #ccc;
            border-radius: 4px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.2);
            z-index: 10000;
            font-size: 13px;
            min-width: 180px;
            padding: 4px 0;
        `;
        
        // 메뉴 항목: Create XY Plot
        const menuItem = document.createElement('div');
        menuItem.textContent = '📊 Create XY Plot';
        menuItem.style.cssText = `
            padding: 8px 16px;
            cursor: pointer;
            color: #000;
            background: transparent;
        `;
        
        menuItem.onmouseenter = () => {
            menuItem.style.background = '#f0f0f0';
        };
        menuItem.onmouseleave = () => {
            menuItem.style.background = 'transparent';
        };
        menuItem.onclick = () => {
            this.createXYPlotFromSelection(paths);
            this.hideContextMenu();
        };
        
        menu.appendChild(menuItem);
        document.body.appendChild(menu);
        
        // 외부 클릭 시 메뉴 닫기
        const closeMenu = (event) => {
            if (!menu.contains(event.target)) {
                this.hideContextMenu();
                document.removeEventListener('click', closeMenu);
            }
        };
        setTimeout(() => {
            document.addEventListener('click', closeMenu);
        }, 100);
        
        console.log('[PlotJugglerTree] Context menu shown for paths:', paths);
    }

    hideContextMenu() {
        const existingMenu = document.getElementById('plot-tree-context-menu');
        if (existingMenu) {
            existingMenu.remove();
        }
    }

    createXYPlotFromSelection(paths) {
        if (paths.length !== 2) {
            console.warn('[PlotJugglerTree] XY Plot requires exactly 2 paths');
            return;
        }
        
        console.log('[PlotJugglerTree] Creating XY Plot:', paths);
        
        // script.js의 createXYPlot 함수 호출
        if (typeof createXYPlot === 'function') {
            createXYPlot(paths[0], paths[1]);
        } else {
            console.error('[PlotJugglerTree] createXYPlot function not found');
        }
    }

    clear() {
        // 트리 구조는 유지하고 값만 초기화
        this.messageData.clear();
        this.selectedItems.clear();
        
        // 모든 리프 노드의 값을 '-'로 초기화
        this.nodeMap.forEach((node, path) => {
            if (node.dataset.isLeaf === 'true' && node.valueElement) {
                node.valueElement.textContent = '-';
            }
        });
    }

    rebuildTree(messageData) {
        // 트리 완전히 재구성
        if (this.container) {
            this.container.innerHTML = '';
        }
        this.rootNode = null;
        this.nodeMap.clear();
        this.messageData.clear();
        this.selectedItems.clear();
        this.init();
        
        // 메시지 데이터로 트리 재구성
        if (messageData) {
            messageData.forEach((value, path) => {
                this.addItem(path, value);
            });
        }
    }
    
    // 디버깅용: 트리 구조 출력
    debugTree() {
        console.log('=== PlotJugglerTree Debug ===');
        console.log(`Total nodes: ${this.nodeMap.size}`);
        console.log(`Selected items: ${this.selectedItems.size}`);
        
        const leafNodes = Array.from(this.nodeMap.values()).filter(n => n.dataset.isLeaf === 'true');
        console.log(`Leaf nodes: ${leafNodes.length}`);
        if (leafNodes.length > 0) {
            console.log('Leaf nodes:');
            leafNodes.slice(0, 10).forEach(node => {
                console.log(`  - ${node.dataset.path}: ${node.valueElement ? node.valueElement.textContent : 'N/A'}`);
            });
            if (leafNodes.length > 10) {
                console.log(`  ... and ${leafNodes.length - 10} more`);
            }
        } else {
            console.warn('  No leaf nodes found!');
        }
    }
}
