/**
 * pc2_stream_worker.js
 * PC2 전용 rosbridge 직접 WebSocket Worker
 *
 * ┌─────────────────────────────────────────────────────────────┐
 * │  기존 문제:                                                   │
 * │  ROSLIB(메인 스레드) → JSON.parse(10MB) = 30~50ms 블로킹     │
 * │                                                               │
 * │  이 Worker:                                                   │
 * │  Worker 전용 WebSocket → JSON.parse(Worker) → PC2 파싱       │
 * │  → postMessage(Float32Array) → 메인 스레드 GPU 업로드 (~3ms) │
 * │                                                               │
 * │  메인 스레드: GPU 업로드만 담당 → render loop 60fps 유지      │
 * └─────────────────────────────────────────────────────────────┘
 *
 * Commands from main thread:
 *   { cmd: 'connect',          url: string }
 *   { cmd: 'subscribe',        topicName, throttle_rate, colorMode, colorField, solidColor }
 *   { cmd: 'unsubscribe',      topicName }
 *   { cmd: 'updateSettings',   topicName, colorMode?, colorField?, solidColor? }
 *
 * Messages to main thread:
 *   { type: 'pc2frame', topicName, frameId, pos, col, count, newMin, newMax }
 *   { type: 'connected' }
 *   { type: 'disconnected' }
 */

const PC2_MAX_POINTS = 50000;

// ── 사전 할당 버퍼 (프레임당 new Float32Array 방지 → GC 압력 제거) ──
// 토픽별로 버퍼를 관리하면 좋지만, 대부분 1-2개 토픽을 구독하므로
// 간단하게 2세트 ping-pong 버퍼 사용
// 각 세트: pos(PC2_MAX_POINTS*3), col(PC2_MAX_POINTS*3)
const _bufPos = [
    new Float32Array(PC2_MAX_POINTS * 3),
    new Float32Array(PC2_MAX_POINTS * 3),
];
const _bufCol = [
    new Float32Array(PC2_MAX_POINTS * 3),
    new Float32Array(PC2_MAX_POINTS * 3),
];
let _bufIdx = 0;  // 현재 사용 중인 버퍼 인덱스 (0 또는 1)

// decode 작업용 임시 Uint8Array (최대 크기 사전 할당)
// PC2_MAX_POINTS × max_point_step(64 bytes) = 3.2MB
const _rawDataBuf = new Uint8Array(PC2_MAX_POINTS * 64);

let ws        = null;   // Worker 전용 WebSocket
let wsUrl     = null;   // rosbridge WebSocket URL
let wsReady   = false;  // WebSocket 연결 완료 여부

// topicName → 토픽별 상태
// { colorMode, colorField, solidColor, solidR, solidG, solidB,
//   adaptiveMin, adaptiveMax, throttle_rate }
const subscriptions = new Map();

// ──────────────────────────────────────────
// 메인 스레드 → Worker 명령 처리
// ──────────────────────────────────────────
self.onmessage = function (e) {
    const { cmd } = e.data;

    if (cmd === 'connect') {
        wsUrl = e.data.url;
        _connectWs();

    } else if (cmd === 'subscribe') {
        const { topicName, throttle_rate, colorMode, colorField, solidColor } = e.data;
        const solid = solidColor || '#ffffff';
        const hex   = solid.replace('#', '');
        subscriptions.set(topicName, {
            colorMode:   colorMode   || 'rainbow',
            colorField:  colorField  || 'intensity',
            solidColor:  solid,
            solidR: parseInt(hex.substring(0, 2), 16) / 255,
            solidG: parseInt(hex.substring(2, 4), 16) / 255,
            solidB: parseInt(hex.substring(4, 6), 16) / 255,
            adaptiveMin:   0,
            adaptiveMax:   1,
            throttle_rate: throttle_rate || 500,
        });
        if (wsReady) _sendSubscribe(topicName);

    } else if (cmd === 'unsubscribe') {
        const { topicName } = e.data;
        subscriptions.delete(topicName);
        if (wsReady) {
            ws.send(JSON.stringify({ op: 'unsubscribe', topic: topicName }));
        }

    } else if (cmd === 'updateSettings') {
        const { topicName, colorMode, colorField, solidColor } = e.data;
        const entry = subscriptions.get(topicName);
        if (!entry) return;
        if (colorMode  !== undefined) entry.colorMode  = colorMode;
        if (colorField !== undefined) entry.colorField = colorField;
        if (solidColor !== undefined) {
            const hex   = solidColor.replace('#', '');
            entry.solidColor = solidColor;
            entry.solidR = parseInt(hex.substring(0, 2), 16) / 255;
            entry.solidG = parseInt(hex.substring(2, 4), 16) / 255;
            entry.solidB = parseInt(hex.substring(4, 6), 16) / 255;
        }
    }
};

// ──────────────────────────────────────────
// WebSocket 관리
// ──────────────────────────────────────────
function _connectWs() {
    ws = new WebSocket(wsUrl);

    ws.onopen = function () {
        wsReady = true;
        self.postMessage({ type: 'connected' });
        // 연결 전에 요청된 구독 모두 전송
        for (const topicName of subscriptions.keys()) {
            _sendSubscribe(topicName);
        }
    };

    ws.onmessage = _handleWsMessage;

    ws.onerror = function () {
        self.postMessage({ type: 'error' });
    };

    ws.onclose = function () {
        wsReady = false;
        self.postMessage({ type: 'disconnected' });
        // 3초 후 재연결
        setTimeout(_connectWs, 3000);
    };
}

function _sendSubscribe(topicName) {
    const entry = subscriptions.get(topicName);
    ws.send(JSON.stringify({
        op:            'subscribe',
        topic:         topicName,
        type:          'sensor_msgs/msg/PointCloud2',
        throttle_rate: entry ? entry.throttle_rate : 500,
        queue_length:  1,
    }));
}

// ──────────────────────────────────────────
// WebSocket 메시지 처리 (Worker 스레드에서 실행)
// JSON.parse가 여기서 수행되므로 메인 스레드 블로킹 없음!
// ──────────────────────────────────────────
function _handleWsMessage(evt) {
    // ── 핵심: JSON.parse가 Worker 스레드에서 실행됨 ──
    let data;
    try { data = JSON.parse(evt.data); } catch (e) { return; }

    if (data.op !== 'publish') return;

    const topicName = data.topic;
    const entry     = subscriptions.get(topicName);
    if (!entry) return;

    _parseAndPost(topicName, data.msg, entry);
}

// ──────────────────────────────────────────
// PC2 파싱 + 결과 전송
// ──────────────────────────────────────────
function _parseAndPost(topicName, message, entry) {
    if (!message || !message.data || !message.fields) return;

    // Field 오프셋 탐색
    let xOff = -1, yOff = -1, zOff = -1, rgbOff = -1, cfOff = -1;
    const colorField = entry.colorField || 'intensity';
    for (const f of message.fields) {
        if (f.name === 'x')                         xOff   = f.offset;
        if (f.name === 'y')                         yOff   = f.offset;
        if (f.name === 'z')                         zOff   = f.offset;
        if (f.name === 'rgb' || f.name === 'rgba')  rgbOff = f.offset;
        if (f.name === colorField)                  cfOff  = f.offset;
    }
    if (xOff === -1 || yOff === -1 || zOff === -1) return;

    // base64 decode (필요한 바이트만)
    const pointStep   = message.point_step;
    const numPoints   = Math.min(message.width * message.height, PC2_MAX_POINTS);
    const bytesNeeded = numPoints * pointStep;
    const charsNeeded = Math.ceil(bytesNeeded / 3) * 4;
    const b64Slice    = (charsNeeded < message.data.length)
        ? message.data.substring(0, charsNeeded)
        : message.data;

    const binaryString = atob(b64Slice);
    const len          = binaryString.length;
    // 사전 할당 버퍼 재사용 (new Uint8Array 방지)
    const rawData = (len <= _rawDataBuf.length) ? _rawDataBuf.subarray(0, len)
                                                 : new Uint8Array(len);
    // 4× 루프 언롤링
    let j = 0;
    for (; j + 3 < len; j += 4) {
        rawData[j]     = binaryString.charCodeAt(j);
        rawData[j + 1] = binaryString.charCodeAt(j + 1);
        rawData[j + 2] = binaryString.charCodeAt(j + 2);
        rawData[j + 3] = binaryString.charCodeAt(j + 3);
    }
    for (; j < len; j++) rawData[j] = binaryString.charCodeAt(j);
    const view = new DataView(rawData.buffer, rawData.byteOffset, rawData.byteLength);

    // 사전 할당 ping-pong 버퍼 사용 (new Float32Array 방지 → GC 압력 제거)
    const bufIdx = _bufIdx;
    _bufIdx      = 1 - _bufIdx;   // 다음 프레임은 반대편 버퍼 사용
    const pos = _bufPos[bufIdx];
    const col = _bufCol[bufIdx];

    const { colorMode, solidR, solidG, solidB } = entry;
    const rangeMin  = entry.adaptiveMin;
    const rangeMax  = entry.adaptiveMax;
    const rangeSpan = (rangeMax > rangeMin) ? (rangeMax - rangeMin) : 1.0;
    let newMin = Infinity, newMax = -Infinity;
    let count  = 0;

    for (let i = 0; i < numPoints; i++) {
        const base = i * pointStep;
        const x = view.getFloat32(base + xOff, true);
        const y = view.getFloat32(base + yOff, true);
        const z = view.getFloat32(base + zOff, true);
        if (!isFinite(x) || !isFinite(y) || !isFinite(z)) continue;

        const ptr3 = count * 3;
        pos[ptr3]     = x;
        pos[ptr3 + 1] = y;
        pos[ptr3 + 2] = z;

        if (colorMode === 'rainbow') {
            let fv = (cfOff !== -1) ? view.getFloat32(base + cfOff, true) : z;
            if (!isFinite(fv)) fv = rangeMin;
            if (fv < newMin) newMin = fv;
            if (fv > newMax) newMax = fv;
            const t   = Math.min(1.0, Math.max(0.0, (fv - rangeMin) / rangeSpan));
            const hue = (1.0 - t) * 0.667;
            const h6  = hue * 6.0;
            const hi  = h6 | 0;
            const f_  = h6 - hi;
            let cr, cg, cb;
            switch (hi % 6) {
                case 0: cr = 1;      cg = f_;     cb = 0;      break;
                case 1: cr = 1 - f_; cg = 1;      cb = 0;      break;
                case 2: cr = 0;      cg = 1;      cb = f_;     break;
                case 3: cr = 0;      cg = 1 - f_; cb = 1;      break;
                case 4: cr = f_;     cg = 0;      cb = 1;      break;
                default: cr = 1;    cg = 0;      cb = 1 - f_; break;
            }
            col[ptr3] = cr; col[ptr3 + 1] = cg; col[ptr3 + 2] = cb;

        } else if (colorMode === 'rgb' && rgbOff !== -1) {
            const rgb   = view.getUint32(base + rgbOff, true);
            col[ptr3]   = ((rgb >> 16) & 0xFF) / 255;
            col[ptr3+1] = ((rgb >>  8) & 0xFF) / 255;
            col[ptr3+2] = ( rgb        & 0xFF) / 255;

        } else {
            col[ptr3] = solidR; col[ptr3+1] = solidG; col[ptr3+2] = solidB;
        }
        count++;
    }

    // adaptive 범위 갱신
    if (isFinite(newMin)) entry.adaptiveMin = newMin;
    if (isFinite(newMax)) entry.adaptiveMax = newMax;

    const frameId = (message.header && message.header.frame_id)
        ? message.header.frame_id : '';

    // 사전 할당 버퍼를 사용하므로 Transferable 미사용
    // postMessage가 pos.subarray(0, count*3)를 복사해서 전송 (600KB 이하)
    // → Worker는 버퍼 소유권을 유지, GC 할당 없음
    self.postMessage({
        type: 'pc2frame',
        topicName,
        frameId,
        pos: pos.subarray(0, count * 3),
        col: col.subarray(0, count * 3),
        count,
        newMin: isFinite(newMin) ? newMin : rangeMin,
        newMax: isFinite(newMax) ? newMax : rangeMax,
    });
}
