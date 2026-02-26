# **PBL5**

## **1. Dual Core Execution:**

- Core 0: Xử lý kết nối (App, WiFi/Bluetooth) và các tác vụ tính toán nặng.

- Core 1: Xử lý Real-time (Đọc cảm biến, State Machine, Điều khiển Motor/Servo).

(
*Core 0: Xử lý Cảm biến & Kết nối (Input/Output Core)*
SensorTask: Đọc Ultrasonic và MPU6050. Dữ liệu được đẩy vào một Global Struct hoặc dùng Queue để truyền sang Core 1.

AppTask (Dự phòng): Đây là nơi bạn sẽ bổ sung code nhận dữ liệu từ App (WiFi/Bluetooth) sau này.

*Core 1: Xử lý Logic & Điều khiển (Logic Core)*
StateMachineTask: Tính toán trạng thái (NORMAL, TURN, BACKING,...) dựa trên dữ liệu cảm biến.

MotorTask: Thực hiện làm mượt (smooth transition) tốc độ và góc lái.
)

Bước 1: Cấu trúc lại Task và ưu tiên (Priority)
Dưới đây là sơ đồ phân chia Task:

TaskSensor: Priority 3 (Cao) - Đọc MPU6050 và Ultrasonic.

TaskControl: Priority 2 (Trung bình) - Chạy State Machine hoặc nhận lệnh từ App.

TaskApp: Priority 1 (Thấp) - Nhận dữ liệu điều khiển (giả định qua Serial/Bluetooth/Wifi).