# Khắc phục lỗi ESP32 Reset liên tục

## 🔴 Vấn đề
ESP32 bị reset liên tục với thông báo: `rst:0xc (SW_CPU_RESET)`

## ✅ Đã sửa trong code

1. **Thêm Watchdog Feed**: `yield()` trong loop để tránh watchdog timeout
2. **Tăng delay**: Từ 50ms → 100ms để giảm tải CPU
3. **Filter Ultrasonic**: Lọc giá trị nhiễu từ cảm biến
4. **Timeout ngắn hơn**: pulseIn từ 30ms → 25ms
5. **Giới hạn khoảng cách**: Chỉ chấp nhận 2-400cm

## 🔧 Cần kiểm tra phần cứng

### 1. NGUỒN ĐIỆN (Quan trọng nhất!)
**Hiện tượng**: Motor + Servo hoạt động cùng lúc → điện áp sụt → ESP32 reset

**Giải pháp**:
```
ESP32 -----> Nguồn 5V riêng (từ USB hoặc adapter 5V/2A)
             
Motor -----> Nguồn 12V riêng (qua BTS7960)
             
Servo -----> Có thể dùng chung nguồn 5V với ESP32 
             NHƯNG cần capacitor 1000µF gần chân nguồn servo
```

**Nếu dùng chung nguồn**: Thêm tụ điện 1000µF giữa VCC và GND của servo

### 2. KẾT NỐI GND
- **Bắt buộc**: Nối GND của ESP32, Motor power, và Servo lại với nhau
- Không nối GND → nhiễu điện → reset

### 3. KIỂM TRA PIN
Đảm bảo các pin sau ĐÚNG:
- GPIO23: Servo signal
- GPIO18: RPWM (Motor tiến)
- GPIO19: LPWM (Motor lùi)
- GPIO21: REN (Enable phải)
- GPIO22: LEN (Enable trái)

### 4. THÊM CAPACITOR (Khuyến nghị)
```
[ESP32 5V] ----[+]---- 100µF ----[-]---- [GND]
[Servo 5V] ----[+]---- 1000µF ---[-]---- [GND]
```

## 🧪 Test từng bước

1. **Test chỉ Servo** (tắt motor):
   ```cpp
   // Trong loop(), comment phần motor:
   // moveForward(currentSpeed);
   ```

2. **Test chỉ Motor** (tắt servo):
   ```cpp
   // Trong loop(), comment phần servo:
   // myServo.write(servoAngle);
   ```

3. **Nếu riêng rẽ OK nhưng chung bị reset** → Vấn đề nguồn điện!

## 📊 Đo điện áp
Dùng multimeter đo điện áp 5V khi motor chạy:
- Nếu < 4.5V → Nguồn yếu, cần nguồn mạnh hơn
- Nếu >= 4.5V → OK

## 🚨 Giải pháp tạm thời
Nếu không thể tách nguồn ngay, giảm tốc độ motor:
```cpp
#define AUTO_SPEED 100  // Thay vì 150
```
