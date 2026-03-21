/**
 * NetworkManager.cpp
 * Triển khai WiFi, HTTP và WebSocket cho ESP32.
 */

#include "NetworkManager.h"
#include <Preferences.h>
#include <WebServer.h>

namespace {
constexpr uint8_t RECONNECT_FAILS_BEFORE_AP = 5;
constexpr uint16_t PORTAL_RECONNECT_DELAY_MS = 1200;
WebServer gPortalServer(80);
}

// ── Static member definitions ──────────────────────────────────
WebSocketsClient NetworkManager::_ws;
volatile bool    NetworkManager::_wsConnected          = false;
WsMessageCb      NetworkManager::_wsCallback           = nullptr;
uint32_t         NetworkManager::_backoffMs            = 1000;
uint32_t         NetworkManager::_lastReconnectAttempt = 0;
String           NetworkManager::_wifiSsid;
String           NetworkManager::_wifiPass;
bool             NetworkManager::_wifiLoaded           = false;
uint8_t          NetworkManager::_failedReconnects     = 0;
bool             NetworkManager::_provisioningMode     = false;
bool             NetworkManager::_portalReconnectPending = false;
uint32_t         NetworkManager::_portalReconnectAt    = 0;

void NetworkManager::_loadWiFiCredentials() {
    if (_wifiLoaded) return;

    Preferences prefs;
    prefs.begin("netcfg", true);
    _wifiSsid = prefs.getString("ssid", WIFI_SSID_DEFAULT);
    _wifiPass = prefs.getString("pass", WIFI_PASS_DEFAULT);
    prefs.end();

    _wifiLoaded = true;
}

void NetworkManager::_saveWiFiCredentials(const String& ssid, const String& pass) {
    Preferences prefs;
    prefs.begin("netcfg", false);
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.end();
}

bool NetworkManager::setWiFiCredentials(const String& ssid, const String& pass, bool reconnectNow) {
    if (ssid.isEmpty()) {
        Serial.println("[NET] SET_WIFI rejected: ssid rỗng");
        return false;
    }

    _wifiSsid = ssid;
    _wifiPass = pass;
    _wifiLoaded = true;
    _saveWiFiCredentials(ssid, pass);

    Serial.printf("[NET] Đã lưu WiFi mới: %s\n", _wifiSsid.c_str());
    _failedReconnects = 0;

    if (reconnectNow) {
        Serial.println("[NET] Reconnect WiFi với credentials mới...");
        _stopProvisionPortal();
        WiFi.disconnect(true);
        vTaskDelay(pdMS_TO_TICKS(250));
        return initWiFi();
    }

    return true;
}

String NetworkManager::getCurrentWiFiSsid() {
    _loadWiFiCredentials();
    return _wifiSsid;
}

void NetworkManager::_startProvisionPortal() {
    if (_provisioningMode) return;

    WiFi.mode(WIFI_AP_STA);
    const bool apOk = WiFi.softAP("KaliVega-Setup", "12345678");
    if (!apOk) {
        Serial.println("[AP] Không thể khởi động SoftAP");
        return;
    }

    gPortalServer.on("/", HTTP_GET, []() {
        const String page =
            "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
            "<title>KaliVega WiFi Setup</title><style>body{font-family:Arial,sans-serif;margin:24px;background:#0f172a;color:#e2e8f0;}"
            "form{max-width:420px;padding:16px;border:1px solid #334155;border-radius:10px;background:#111827;}"
            "label{display:block;margin:10px 0 6px;}input{width:100%;padding:10px;border-radius:8px;border:1px solid #334155;background:#0b1220;color:#fff;box-sizing:border-box;}"
            "button{margin-top:12px;padding:10px 14px;border-radius:8px;border:none;background:#2563eb;color:#fff;cursor:pointer;width:100%;}</style></head>"
            "<body><h2>KaliVega WiFi Setup</h2><p>Nhap SSID va password de ESP32 ket noi.</p>"
            "<form method='POST' action='/save'><label>SSID</label><input name='ssid' required maxlength='64'/>"
            "<label>Password</label><input name='password' type='password' maxlength='64'/>"
            "<button type='submit'>Save and Connect</button></form></body></html>";
        gPortalServer.send(200, "text/html", page);
    });

    gPortalServer.on("/save", HTTP_POST, []() {
        const String ssid = gPortalServer.arg("ssid");
        const String pass = gPortalServer.arg("password");

        if (ssid.isEmpty()) {
            gPortalServer.send(400, "text/plain", "SSID is required");
            return;
        }

        NetworkManager::setWiFiCredentials(ssid, pass, false);
        NetworkManager::_portalReconnectPending = true;
        NetworkManager::_portalReconnectAt = millis() + PORTAL_RECONNECT_DELAY_MS;
        gPortalServer.send(200, "text/plain", "Saved. ESP32 will reconnect shortly.");
    });

    gPortalServer.onNotFound([]() {
        gPortalServer.send(404, "text/plain", "Not found");
    });

    gPortalServer.begin();
    _provisioningMode = true;
    _portalReconnectPending = false;
    Serial.printf("[AP] SoftAP ready: SSID=KaliVega-Setup IP=%s\n", WiFi.softAPIP().toString().c_str());
}

void NetworkManager::_stopProvisionPortal() {
    if (!_provisioningMode) return;

    gPortalServer.stop();
    WiFi.softAPdisconnect(true);
    _provisioningMode = false;
    _portalReconnectPending = false;
    Serial.println("[AP] SoftAP stopped");
}

void NetworkManager::portalLoop() {
    if (!_provisioningMode) return;

    gPortalServer.handleClient();

    if (_portalReconnectPending && millis() >= _portalReconnectAt) {
        _portalReconnectPending = false;
        _stopProvisionPortal();
        if (!initWiFi()) {
            _startProvisionPortal();
        }
    }
}

bool NetworkManager::isProvisioningMode() {
    return _provisioningMode;
}

// ══════════════════════════════════════════
//  WiFi
// ══════════════════════════════════════════

bool NetworkManager::initWiFi() {
    _loadWiFiCredentials();

    if (_wifiSsid.isEmpty()) {
        Serial.println("[NET] Không có SSID để kết nối");
        return false;
    }

    Serial.printf("[NET] Kết nối WiFi: %s\n", _wifiSsid.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.begin(_wifiSsid.c_str(), _wifiPass.c_str());

    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_TIMEOUT_MS) {
        vTaskDelay(pdMS_TO_TICKS(500));
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        _backoffMs = 1000;  // Reset backoff khi thành công
        _failedReconnects = 0;
        _stopProvisionPortal();
        Serial.printf("\n[NET] WiFi OK  IP: %s  RSSI: %d dBm\n",
                      WiFi.localIP().toString().c_str(), WiFi.RSSI());
        return true;
    }

    Serial.println("\n[NET] WiFi FAILED!");
    _failedReconnects++;
    _startProvisionPortal();
    return false;
}

bool NetworkManager::isWiFiConnected() {
    return WiFi.status() == WL_CONNECTED;
}

void NetworkManager::reconnectIfNeeded() {
    if (isWiFiConnected()) {
        _backoffMs = 1000;  // Reset nếu đang kết nối
        _failedReconnects = 0;
        if (_provisioningMode) _stopProvisionPortal();
        return;
    }

    if (_provisioningMode) return;

    uint32_t now = millis();
    if (now - _lastReconnectAttempt < _backoffMs) return;

    _lastReconnectAttempt = now;
    Serial.printf("[NET] WiFi mất kết nối — thử lại (backoff=%u ms)...\n", _backoffMs);
    _loadWiFiCredentials();
    WiFi.disconnect();
    WiFi.begin(_wifiSsid.c_str(), _wifiPass.c_str());
    _failedReconnects = min<uint8_t>(_failedReconnects + 1, 250);
    if (_failedReconnects >= RECONNECT_FAILS_BEFORE_AP) {
        Serial.println("[NET] Reconnect thất bại nhiều lần — chuyển sang SoftAP provisioning");
        _startProvisionPortal();
        return;
    }

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
