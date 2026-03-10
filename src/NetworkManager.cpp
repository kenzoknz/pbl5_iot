/**
 * NetworkManager.cpp
 * Triển khai WiFi, HTTP và WebSocket cho ESP32.
 */

#include "NetworkManager.h"

// ── Static member definitions ──────────────────────────────────
WebSocketsClient NetworkManager::_ws;
volatile bool    NetworkManager::_wsConnected          = false;
WsMessageCb      NetworkManager::_wsCallback           = nullptr;
uint32_t         NetworkManager::_backoffMs            = 1000;
uint32_t         NetworkManager::_lastReconnectAttempt = 0;

// ══════════════════════════════════════════
//  WiFi
// ══════════════════════════════════════════

bool NetworkManager::initWiFi() {
    Serial.printf("[NET] Kết nối WiFi: %s\n", WIFI_SSID);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_TIMEOUT_MS) {
        vTaskDelay(pdMS_TO_TICKS(500));
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        _backoffMs = 1000;  // Reset backoff khi thành công
        Serial.printf("\n[NET] WiFi OK  IP: %s  RSSI: %d dBm\n",
                      WiFi.localIP().toString().c_str(), WiFi.RSSI());
        return true;
    }

    Serial.println("\n[NET] WiFi FAILED!");
    return false;
}

bool NetworkManager::isWiFiConnected() {
    return WiFi.status() == WL_CONNECTED;
}

void NetworkManager::reconnectIfNeeded() {
    if (isWiFiConnected()) {
        _backoffMs = 1000;  // Reset nếu đang kết nối
        return;
    }

    uint32_t now = millis();
    if (now - _lastReconnectAttempt < _backoffMs) return;

    _lastReconnectAttempt = now;
    Serial.printf("[NET] WiFi mất kết nối — thử lại (backoff=%u ms)...\n", _backoffMs);
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    // Exponential backoff: 1s → 2s → 4s → ... → 60s
    _backoffMs = min(_backoffMs * 2, (uint32_t)MAX_BACKOFF_MS);
}

// ══════════════════════════════════════════
//  WebSocket
// ══════════════════════════════════════════

void NetworkManager::initWebSocket(WsMessageCb callback) {
    _wsCallback = callback;

    _ws.begin(SERVER_HOST, SERVER_PORT, WS_PATH);
    _ws.onEvent(_onWsEvent);
    _ws.setReconnectInterval(5000);            // Tự reconnect sau 5s
    _ws.enableHeartbeat(15000, 3000, 2);       // ping 15s, pong timeout 3s, retry 2

    Serial.printf("[WS] Khởi tạo: ws://%s:%d%s\n", SERVER_HOST, SERVER_PORT, WS_PATH);
}

void NetworkManager::wsLoop() {
    _ws.loop();
}

bool NetworkManager::wsConnected() {
    return _wsConnected;
}

void NetworkManager::wsSend(const String& jsonStr) {
    if (_wsConnected) {
        String payload = jsonStr;
        _ws.sendTXT(payload);
        Serial.printf("[WS] Sent: %s\n", jsonStr.c_str());
    } else {
        Serial.println("[WS] Skip send — chưa kết nối");
    }
}

/** Static handler — WebSocketsClient yêu cầu plain function pointer */
void NetworkManager::_onWsEvent(WStype_t type, uint8_t* payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            _wsConnected = false;
            Serial.println("[WS] Ngắt kết nối");
            break;

        case WStype_CONNECTED:
            _wsConnected = true;
            Serial.printf("[WS] Kết nối thành công: %s\n", (char*)payload);
            {
                // Gửi thông điệp đăng ký ngay khi kết nối
                StaticJsonDocument<128> reg;
                reg["type"]              = "REGISTER";
                reg["data"]["device"]    = "esp32-robot";
                reg["data"]["firmware"]  = "1.0.0";
                String msg;
                serializeJson(reg, msg);
                _ws.sendTXT(msg);
            }
            break;

        case WStype_TEXT:
            if (_wsCallback && length > 0) {
                _wsCallback(String((char*)payload));
            }
            break;

        case WStype_BIN:
            Serial.printf("[WS] Binary %u bytes (bỏ qua)\n", length);
            break;

        case WStype_PING:
            Serial.println("[WS] Ping nhận được");
            break;

        case WStype_PONG:
            Serial.println("[WS] Pong nhận được");
            break;

        case WStype_ERROR:
            Serial.printf("[WS] Lỗi: %s\n", (char*)payload);
            break;

        default:
            break;
    }
}

// ══════════════════════════════════════════
//  HTTP Helpers
// ══════════════════════════════════════════

String NetworkManager::_url(const String& path) {
    return String(BASE_URL) + path;
}

void NetworkManager::_addHeaders(HTTPClient& client) {
    client.addHeader("Content-Type", "application/json");
    client.addHeader("Accept", "application/json");
#ifdef AUTH_TOKEN
    client.addHeader("Authorization", "Bearer " AUTH_TOKEN);
#endif
}

String NetworkManager::httpGet(const String& path) {
    if (!isWiFiConnected()) {
        Serial.println("[HTTP] GET skip — WiFi disconnected");
        return "";
    }

    HTTPClient http;
    http.setTimeout(HTTP_TIMEOUT_MS);
    http.begin(_url(path));
    _addHeaders(http);

    int code = http.GET();
    String response;

    if (code == HTTP_CODE_OK) {
        response = http.getString();
    } else if (code > 0) {
        Serial.printf("[HTTP] GET %s → %d\n", path.c_str(), code);
    } else {
        Serial.printf("[HTTP] GET %s → Lỗi: %s\n",
                      path.c_str(), http.errorToString(code).c_str());
    }

    http.end();
    return response;
}

int NetworkManager::httpPost(const String& path, const String& jsonBody) {
    if (!isWiFiConnected()) {
        Serial.println("[HTTP] POST skip — WiFi disconnected");
        return -1;
    }

    HTTPClient http;
    http.setTimeout(HTTP_TIMEOUT_MS);
    http.begin(_url(path));
    _addHeaders(http);

    int code = http.POST(jsonBody);
    if (code < 0) {
        Serial.printf("[HTTP] POST %s → Lỗi: %s\n",
                      path.c_str(), http.errorToString(code).c_str());
    } else {
        Serial.printf("[HTTP] POST %s → %d\n", path.c_str(), code);
    }

    http.end();
    return code;
}

int NetworkManager::httpPut(const String& path, const String& jsonBody) {
    if (!isWiFiConnected()) {
        Serial.println("[HTTP] PUT skip — WiFi disconnected");
        return -1;
    }

    HTTPClient http;
    http.setTimeout(HTTP_TIMEOUT_MS);
    http.begin(_url(path));
    _addHeaders(http);

    int code = http.PUT(jsonBody);
    if (code < 0) {
        Serial.printf("[HTTP] PUT %s → Lỗi: %s\n",
                      path.c_str(), http.errorToString(code).c_str());
    } else {
        Serial.printf("[HTTP] PUT %s → %d\n", path.c_str(), code);
    }

    http.end();
    return code;
}
