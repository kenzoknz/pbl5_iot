#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

/**
 * NetworkManager.h
 * Quản lý kết nối WiFi, HTTP (GET/POST/PUT), và WebSocket client.
 * WebSocket dùng thư viện: Links2004/WebSockets (platformio: links2004/WebSockets)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>

// ══════════════════════════════════════════
//  CẤU HÌNH — Chỉnh sửa theo môi trường
// ══════════════════════════════════════════
#define WIFI_SSID_DEFAULT     "ITF-DUT"     // Tên WiFi mặc định
#define WIFI_PASS_DEFAULT     "Haduckien1709"

#define SERVER_HOST          "pbl5.ddns.net"   // IP server (không có http://)
#define SERVER_PORT          5000
#define BASE_URL             "http://" SERVER_HOST ":" "5000"

// WebSocket endpoint (server sẽ lắng nghe tại ws://SERVER_HOST:PORT/ws/robot)
#define WS_PATH              "/ws/robot"

// Bỏ comment dòng dưới nếu server yêu cầu Bearer token
// #define AUTH_TOKEN        "your-token-here"

// ══════════════════════════════════════════
//  TIMING
// ══════════════════════════════════════════
#define POLL_INTERVAL_MANUAL   500      // ms — polling nhanh khi MANUAL
#define POLL_INTERVAL_AUTO    2000      // ms — polling chậm khi AUTONOMOUS
#define MAX_BACKOFF_MS       60000      // ms — backoff tối đa khi mất WiFi
#define WIFI_TIMEOUT_MS      15000      // ms — timeout kết nối WiFi ban đầu
#define HTTP_TIMEOUT_MS       8000      // ms — timeout HTTP request

// Kiểu callback nhận message từ WebSocket
typedef void (*WsMessageCb)(const String& message);

// ══════════════════════════════════════════
class NetworkManager {
public:
    // ── WiFi ──
    static bool initWiFi();
    static bool isWiFiConnected();
    static void reconnectIfNeeded();
    static bool setWiFiCredentials(const String& ssid, const String& pass, bool reconnectNow = true);
    static String getCurrentWiFiSsid();
    static void portalLoop();
    static bool isProvisioningMode();

    // ── WebSocket ──
    static void initWebSocket(WsMessageCb callback);
    static void wsLoop();                          // Gọi liên tục trong AppTask
    static bool wsConnected();
    static void wsSend(const String& jsonStr);     // Gửi JSON string qua WS
    static void wsSendSerialLog(const String& level, const String& message);

    // ── HTTP ──
    /** Trả về body string (rỗng nếu lỗi) */
    static String httpGet(const String& path);

    /** Trả về HTTP status code (-1 nếu network error) */
    static int    httpPost(const String& path, const String& jsonBody);

    /** Trả về HTTP status code (-1 nếu network error) */
    static int    httpPut(const String& path, const String& jsonBody = "{}");

private:
    static WebSocketsClient _ws;
    static volatile bool    _wsConnected;
    static WsMessageCb      _wsCallback;

    // Backoff WiFi reconnect
    static uint32_t _backoffMs;
    static uint32_t _lastReconnectAttempt;
    static uint8_t  _failedReconnects;

    static String _wifiSsid;
    static String _wifiPass;
    static bool   _wifiLoaded;

    static bool    _provisioningMode;
    static bool    _portalReconnectPending;
    static uint32_t _portalReconnectAt;

    // Handler nội bộ — phải là static plain function để truyền vào WebSocketsClient
    static void _onWsEvent(WStype_t type, uint8_t* payload, size_t length);

    // Helpers
    static String _url(const String& path);
    static void   _addHeaders(HTTPClient& client);
    static void   _loadWiFiCredentials();
    static void   _saveWiFiCredentials(const String& ssid, const String& pass);
    static void   _startProvisionPortal();
    static void   _stopProvisionPortal();
};

#endif // NETWORK_MANAGER_H
