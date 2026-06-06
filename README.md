# PBL5 IoT — Robot Phát Hiện Đồ Vật Để Quên

Hệ thống robot tự hành kết hợp trí tuệ nhân tạo (AI) nhận diện đồ vật, hỗ trợ giám sát và phát hiện đồ vật bỏ quên thông qua IoT.

---

## 1. Giới thiệu

Dự án PBL5 xây dựng một robot tự hành có khả năng di chuyển trong khu vực giám sát, phát hiện và cảnh báo đồ vật bỏ quên. Hệ thống hoạt động dựa trên sự phối hợp giữa ba thành phần chính:

- **ESP32 (Bộ điều khiển chính):** Vi điều khiển chạy FreeRTOS với kiến trúc dual-core. Core 0 đảm nhận xử lý mạng (WiFi, WebSocket, GPS, UART Jetson), Core 1 đảm nhận xử lý logic điều khiển (state machine, motor, cảm biến siêu âm). ESP32 quản lý toàn bộ phần cứng: 4 cảm biến siêu âm HC-SR04, cảm biến quán tính MPU6050, mô-đun GPS NEO-7N, encoder quang 600PPR, motor DC thông qua driver BTS7960, và servo lái.

- **Jetson Nano (Xử lý AI):** Máy tính nhúng của NVIDIA chạy mô hình YOLO (You Only Look Once) để nhận diện đồ vật qua camera thời gian thực. Khi phát hiện vật thể nguy hiểm hoặc đồ vật bỏ quên, Jetson gửi lệnh STOP cho ESP32 qua giao thức UART (JSON, 115200 baud). Jetson hoạt động độc lập với WiFi, đảm bảo robot phản ứng ngay cả khi mất mạng.

- **Web Server (Dashboard giám sát):** Giao diện web cho phép người dùng giám sát trạng thái robot theo thời gian thực, điều khiển robot từ xa qua joystick ảo, xem vị trí GPS trên bản đồ, và chuyển đổi giữa chế độ tự hành (AUTONOMOUS) và điều khiển thủ công (MANUAL). Giao tiếp với ESP32 qua WebSocket cho độ trễ thấp, có fallback sang HTTP polling khi WebSocket ngắt kết nối. Mã nguồn web server: https://github.com/nguyenhieu126/Robot_Control_Web

Robot có hai chế độ hoạt động chính:
- **AUTONOMOUS (Tự hành):** Robot tự di chuyển, tránh vật cản bằng state machine với 11 trạng thái (NORMAL, SLOW, AVOID_LEFT, AVOID_RIGHT, TURN_LEFT, TURN_RIGHT, BACKING, STOP, EMERGENCY, MANUAL_CONTROL, ESCAPE). Nhận lệnh STOP từ Jetson khi phát hiện vật nguy hiểm.
- **MANUAL (Thủ công):** Người dùng điều khiển robot trực tiếp qua joystick trên web dashboard, motor phản hồi realtime.

---

## 2. Cấu trúc dự án

```
pbl5_iot/
├── platformio.ini                       # Cấu hình PlatformIO (board, thư viện, build flags)
├── README.md
├── docs/
│   ├── guides/
│   │   ├── QUICK_REFERENCE.md           # Bảng tra cứu nhanh
│   │   └── UART_DATA_MODEL.md           # Mô hình dữ liệu UART
│   └── integration/
│       └── JETSON_ESP32_INTEGRATION.md  # Hướng dẫn tích hợp Jetson - ESP32
├── src/
│   ├── main.cpp                         # Entry point — khởi tạo phần cứng, tạo 3 FreeRTOS tasks
│   ├── core/
│   │   ├── Config.h                     # Định nghĩa chân GPIO, hằng số, tốc độ, ngưỡng, enum State
│   │   └── ConfigStorage.h/.cpp         # Lưu trữ cấu hình vào NVS (non-volatile storage)
│   ├── sensors/
│   │   ├── UltrasonicSensor.h/.cpp      # 4 cảm biến HC-SR04 (trước/phải/trái/sau)
│   │   ├── MPUSensor.h/.cpp             # MPU6050 gia tốc kế — phát hiện va chạm, nghiêng
│   │   ├── GPSSensor.h/.cpp             # NEO-7N GPS qua UART2 — đọc tọa độ
│   │   ├── EncoderSensor.h/.cpp         # Encoder quang 600PPR qua PCNT phần cứng
│   │   └── GpsQueue.h/.cpp              # Hàng đợi GPS offline (buffer khi mất mạng)
│   ├── control/
│   │   ├── VehicleStateMachine.h/.cpp   # State machine điều hướng (11 trạng thái)
│   │   └── MotorController.h/.cpp       # Điều khiển motor BTS7960 + servo lái + PID tốc độ
│   └── communication/
│       ├── NetworkManager.h/.cpp        # Quản lý WiFi, WebSocket, Captive Portal
│       ├── CommandProcessor.h/.cpp      # Xử lý lệnh từ web (joystick, mode change, status)
│       └── JetsonUART.h/.cpp            # Giao tiếp UART với Jetson Nano (JSON protocol)
├── include/                             # Header files chung
├── lib/                                 # Thư viện riêng
└── test/                                # Unit tests
```

---

## 3. Yêu cầu phần cứng

| Thành phần | Thông số |
|------------|----------|
| Vi điều khiển | ESP32 DevKit (dual-core, WiFi + Bluetooth) |
| Driver motor | BTS7960 (2 kênh PWM: RPWM, LPWM) |
| Motor DC | Motor DC có hộp số + bánh xe đường kính 6.5 cm |
| Servo lái | Servo PWM điều khiển góc lái bánh trước |
| Cảm biến siêu âm | 4x HC-SR04 (trước, phải, trái, sau) |
| Cảm biến quán tính | MPU6050 (I2C: SDA GPIO32, SCL GPIO33) |
| Mô-đun GPS | NEO-7N (UART2: RX GPIO27, TX GPIO5, 9600 baud) |
| Encoder quang | 600 PPR, x4 quadrature = 2400 CPR (GPIO34, GPIO35) |
| Máy tính AI | NVIDIA Jetson Nano (tùy chọn) |
| Camera | USB hoặc CSI camera cho Jetson (tùy chọn) |
| Nguồn | Pin 12V cho motor, 5V cho ESP32 và cảm biến |

---

## 4. Yêu cầu phần mềm

- **PlatformIO IDE** (extension VS Code) hoặc PlatformIO CLI
- **USB cable** (micro-USB hoặc USB-C tùy board) để nạp firmware ESP32
- **Python 3.8+** trên Jetson Nano (nếu sử dụng Jetson)
- Các thư viện PlatformIO được tự động tải khi build lần đầu:
  - `ArduinoJson` — Parse/serialize JSON
  - `WebSockets` — Giao tiếp WebSocket realtime
  - `TinyGPSPlus` — Parse dữ liệu NMEA từ GPS
  - `MPU6050_light` — Đọc dữ liệu MPU6050
  - `ESP32Servo` — Điều khiển servo trên ESP32

---

## 5. Hướng dẫn cài đặt và chạy

### Bước 1: Clone và build firmware

```bash
git clone https://github.com/kenzoknz/pbl5_iot.git
cd pbl5_iot

# Build firmware (PlatformIO tự động tải thư viện)
pio run

# Nạp firmware lên ESP32
pio run --target upload

# Mở terminal theo dõi serial log
pio device monitor
```

### Bước 2: Cấu hình WiFi (lần đầu khởi động)

Khi ESP32 khởi động lần đầu hoặc không tìm thấy WiFi đã lưu, thiết bị sẽ phát **Captive Portal** (điểm truy cập riêng):

1. Dùng điện thoại hoặc laptop kết nối WiFi tên **`ESP32-Setup`** (không có mật khẩu).
2. Trình duyệt sẽ tự động mở trang cấu hình WiFi.
3. Nhập **SSID** và **Password** mạng WiFi muốn kết nối.
4. Nhấn lưu — ESP32 sẽ khởi động lại và kết nối vào mạng WiFi.

Thông tin WiFi được lưu vào NVS, không cần cấu hình lại khi mất điện.

### Bước 3: Truy cập Web Dashboard

Sau khi ESP32 kết nối WiFi thành công:

1. Mở serial monitor để xem địa chỉ IP của ESP32 (ví dụ: `192.168.1.100`).
2. Mở trình duyệt, truy cập địa chỉ IP đó.
3. Dashboard hiển thị: trạng thái robot, vị trí GPS, chế độ hoạt động, joystick điều khiển.
4. Sử dụng joystick ảo để điều khiển robot ở chế độ MANUAL, hoặc chuyển sang chế độ AUTONOMOUS để robot tự hành.

Server WebSocket chính: `ws://pbl5.ddns.net:5000/ws/robot`

### Bước 4: Tích hợp Jetson Nano (tùy chọn)

```bash
# Trên Jetson Nano — cài đặt thư viện Python
pip install pyserial ultralytics opencv-python

# Chạy chương trình nhận diện và điều khiển
python3 main_yolo_control.py
```

**Kết nối phần cứng Jetson với ESP32:**

| Jetson Nano (Header J41) | ESP32 | Ghi chú |
|--------------------------|-------|---------|
| Pin 8 (TX)               | GPIO16 (ROS_RX_PIN) | Đấu chéo TX ↔ RX |
| Pin 10 (RX)              | GPIO17 (ROS_TX_PIN) | Đấu chéo TX ↔ RX |
| Pin 6 (GND)              | GND   | Bắt buộc nối chung mass |

**Lưu ý quan trọng:**
- Phải đấu chéo TX/RX (TX của Jetson nối RX của ESP32 và ngược lại).
- Baudrate mặc định: **115200**.
- Jetson gửi lệnh STOP dạng JSON qua UART, ESP32 phản hồi trạng thái lại Jetson.
- Watchdog timeout: 3 giây — nếu Jetson không gửi lệnh trong 3 giây, ESP32 tự động thoát chế độ STOP.

Hướng dẫn chi tiết tích hợp Jetson: xem `docs/integration/JETSON_ESP32_INTEGRATION.md`.

---

## 6. Cấu hình chi tiết

### 6.1. WiFi và Server

Cấu hình trong file `src/communication/NetworkManager.h`:

```cpp
#define WIFI_SSID_DEFAULT     "ITF-DUT"       // SSID mặc định
#define WIFI_PASS_DEFAULT     "********"       // Mật khẩu mặc định
#define SERVER_HOST           "pbl5.ddns.net"  // Địa chỉ server
#define SERVER_PORT           5000             // Cổng server
#define BASE_URL              "http://pbl5.ddns.net:5000"
#define WS_PATH               "/ws/robot"      // Đường dẫn WebSocket
```

### 6.2. Pin Mapping ESP32

| GPIO | Chức năng | Ghi chú |
|------|-----------|---------|
| 12, 39 | Cảm biến siêu âm trước (Trig, Echo) | Front center |
| 13, 14 | Cảm biến siêu âm phải (Trig, Echo) | Front right |
| 4, 2 | Cảm biến siêu âm trái (Trig, Echo) | Front left |
| 25, 26 | Cảm biến siêu âm sau (Trig, Echo) | Back center |
| 18, 19 | Motor PWM (RPWM, LPWM) | Driver BTS7960 |
| 21, 22 | Motor Enable (REN, LEN) | BTS7960 enable |
| 23 | Servo lái | PWM điều khiển góc lái |
| 34, 35 | Encoder quang (Pin A, Pin B) | Input only, 600PPR |
| 32, 33 | MPU6050 (SDA, SCL) | Giao tiếp I2C |
| 27, 5 | GPS NEO-7N (RX, TX) | UART2, 9600 baud |
| 16, 17 | Jetson UART (ROS_RX, ROS_TX) | Serial1, 115200 baud |

### 6.3. Tốc độ motor (PWM 8-bit, 0-255)

Cấu hình trong `src/core/Config.h`:

| Tham số | Giá trị | Mô tả |
|---------|---------|-------|
| MIN_RUN_SPEED | 150 | Tốc độ tối thiểu để thắng ma sát |
| CRUISE_SPEED | 70 | Tốc độ hành trình (2 tầng) |
| FAST_SPEED | 90 | Tốc độ nhanh (2 tầng) |
| BACK_SPEED | 170 | Tốc độ lùi |
| ESCAPE_SPEED | 200 | Tốc độ xoay thoát bẫy |
| SHARP_TURN_BOOST | 220 | Cua gắt (góc 100-130 độ) |
| MEDIUM_TURN_BOOST | 180 | Cua vừa (góc 60-80 độ) |
| TURN_BOOST | 160 | Cua vừa nhẹ |
| LIGHT_TURN_BOOST | 145 | Cua nhẹ (góc 15-60 độ) |

### 6.4. Ngưỡng khoảng cách (cm)

| Tham số | Giá trị | Mô tả |
|---------|---------|-------|
| EMERGENCY_DIST | 35 | Dừng khẩn cấp |
| STOP_DISTANCE | 40 | Dừng lại |
| SLOW_DISTANCE | 50 | Bắt đầu giảm tốc |
| TURN_DISTANCE | 65 | Quyết định rẽ |
| PREPARE_DISTANCE | 70 | Chuẩn bị rẽ |
| SIDE_DANGER_DIST | 45 | Ngưỡng nguy hiểm cảm biến bên |
| BACK_DANGER_DISTANCE | 45 | Ngưỡng nguy hiểm cảm biến sau |
| DIRECTION_HYSTERESIS | 8 | Chênh lệch tối thiểu để ưu tiên 1 bên (tránh dao động) |

### 6.5. FreeRTOS Tasks

| Task | Core | Ưu tiên | Tần suất | Nhiệm vụ |
|------|------|---------|----------|----------|
| LogicTask | Core 1 | 5 | 20Hz (50ms) | State machine, điều khiển motor, xử lý lệnh Jetson |
| AppTask | Core 0 | 6 | 20Hz (50ms) | WiFi, WebSocket, HTTP, GPS, heartbeat |
| JetsonTask | Core 0 | 6 | 100Hz (10ms) | Đọc UART Jetson, parse JSON, enqueue lệnh |

---

## 7. Giao thức truyền thông

### 7.1. WebSocket (ESP32 ↔ Web Server)

**Điều khiển joystick từ web:**
```json
{
  "type": "JOYSTICK",
  "data": {
    "throttle": 150,
    "steering": 90
  }
}
```

**Đổi chế độ hoạt động:**
```json
{
  "type": "MODE_CHANGE",
  "data": {
    "mode": "MANUAL"
  }
}
```

**Gửi trạng thái robot lên server (mỗi 5 giây):**
```json
{
  "type": "STATUS",
  "data": {
    "mode": "AUTONOMOUS",
    "state": "NORMAL",
    "speed": 150,
    "steering": 90
  }
}
```

### 7.2. UART (Jetson Nano ↔ ESP32, 115200 baud)

**Lệnh STOP từ Jetson:**
```json
{
  "type": "COMMAND",
  "cmd": "STOP",
  "source": "jetson_yolo",
  "confidence": 0.95
}
```

**Phản hồi trạng thái từ ESP32 sang Jetson (mỗi 500ms):**
```json
{
  "type": "STATUS",
  "state": "STOPPED",
  "mode": "MANUAL",
  "speed": 0
}
```

**Lưu ý:**
- Tin nhắn UART phải kết thúc bằng ký tự newline (`\n`).
- Kích thước buffer JSON tối đa: 512 byte.
- Watchdog timeout: 3 giây — nếu Jetson ngừng gửi lệnh, ESP32 tự động thoát chế độ STOP.

---

## 8. Xử lý sự cố

| Vấn đề | Nguyên nhân có thể | Giải pháp |
|--------|-------------------|-----------|
| ESP32 không kết nối WiFi | Sai SSID/password; WiFi quá xa | Kiểm tra lại thông tin; thử lại Captive Portal |
| Không thấy GPS fix | Chưa đủ vệ tinh; đang trong nhà | Ra ngoài trời, chờ 1-2 phút để module bắt sóng |
| Motor không quay | Mất nguồn 12V; hỏng driver BTS7960 | Kiểm tra nguồn điện và dây nối driver |
| Servo không hoạt động | Sai GPIO; thiếu nguồn 5V | Kiểm tra chân GPIO23 và nguồn servo |
| Cảm biến siêu âm đo sai | Góc lắp không phù hợp; bề mặt nghiêng | Lắp cảm biến vuông góc, tránh bề mặt phản xạ kém |
| Encoder không đếm | Sai GPIO; hỏng encoder | Kiểm tra GPIO34/35, kiểm tra encoder quang |
| WebSocket ngắt liên tục | Tín hiệu WiFi yếu; server quá tải | Kiểm tra tín hiệu WiFi; hệ thống tự fallback sang HTTP |
| Dữ liệu UART bị lỗi (garbled) | TX/RX nối thẳng thay vì đấu chéo; thiếu GND chung | Đấu chéo TX ↔ RX; nối GND giữa Jetson và ESP32 |
| Robot dao động khi tránh vật cản | Ngưỡng DIRECTION_HYSTERESIS quá nhỏ | Tăng giá trị DIRECTION_HYSTERESIS trong Config.h (ví dụ: 12-15 cm) |
| Jetson gửi lệnh nhưng ESP32 không nhận | Baudrate không khớp; dây lỏng | Kiểm tra baud = 115200; kiểm tra kết nối vật lý |

---

## 9. Tài liệu tham khảo

- [PlatformIO Documentation](https://docs.platformio.org/)
- [ESP32 Arduino Core](https://github.com/espressif/arduino-esp32)
- [ArduinoJson](https://arduinojson.org/)
- [WebSockets for Arduino](https://github.com/Links2004/arduinoWebSockets)
- [TinyGPS++](http://arduiniana.org/libraries/tinygpsplus/)
- [Ultralytics YOLOv8](https://docs.ultralytics.com/)
- [NVIDIA Jetson Nano](https://developer.nvidia.com/embedded/jetson-nano-developer-kit)

---

## 10. License

Dự án phục vụ mục đích học tập — môn PBL5, Trường Đại học Bách Khoa, Đại học Đà Nẵng.