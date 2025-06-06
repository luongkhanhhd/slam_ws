#include <SoftwareSerial.h>

// UART: giao tiếp với ROS 2
SoftwareSerial mySerial(PA10, PA9);  // RX = PA10, TX = PA9

// Động cơ: PWM và hướng cho 4 bánh mecanum
#define M1_PWM PB6   // Motor 1 PWM
#define M1_DIR PB7   // Motor 1 Direction
#define M2_PWM PB8   // Motor 2 PWM
#define M2_DIR PB9   // Motor 2 Direction
#define M3_PWM PA8   // Motor 3 PWM
#define M3_DIR PA9   // Motor 3 Direction
#define M4_PWM PA10  // Motor 4 PWM
#define M4_DIR PA11  // Motor 4 Direction

// Biến odometry (giả lập)
float x = 0.0, y = 0.0, theta = 0.0;
float vx = 0.0, vy = 0.0, wz = 0.0;
unsigned long last_time = 0;

// Thông số robot
const float LX = 0.15, LY = 0.15; // Khoảng cách tâm robot đến bánh (m)

void setup() {
    mySerial.begin(115200);

    // Cấu hình chân động cơ
    pinMode(M1_PWM, OUTPUT); pinMode(M1_DIR, OUTPUT);
    pinMode(M2_PWM, OUTPUT); pinMode(M2_DIR, OUTPUT);
    pinMode(M3_PWM, OUTPUT); pinMode(M3_DIR, OUTPUT);
    pinMode(M4_PWM, OUTPUT); pinMode(M4_DIR, OUTPUT);

    // Khởi tạo động cơ dừng
    analogWrite(M1_PWM, 0);
    analogWrite(M2_PWM, 0);
    analogWrite(M3_PWM, 0);
    analogWrite(M4_PWM, 0);
}

void loop() {
    unsigned long now = millis();
    
    // Cập nhật odometry mỗi 100ms
    if (now - last_time >= 100) {
        float dt = (now - last_time) / 1000.0;
        last_time = now;

        // Giả lập odometry: tích phân vận tốc
        x += (vx * cos(theta) - vy * sin(theta)) * dt;
        y += (vx * sin(theta) + vy * cos(theta)) * dt;
        theta += wz * dt;

        // Gửi ODOM qua UART
        mySerial.print("ODOM:");
        mySerial.print(x, 3); mySerial.print(",");
        mySerial.print(y, 3); mySerial.print(",");
        mySerial.print(theta, 3); mySerial.print(",");
        mySerial.print(vx, 3); mySerial.print(",");
        mySerial.print(vy, 3); mySerial.print(",");
        mySerial.println(wz, 3);
    }

    // Nhận lệnh vận tốc từ ROS qua UART
    if (mySerial.available()) {
        String cmd = mySerial.readStringUntil('\n');
        cmd.trim();
        
        // Định dạng lệnh: VEL:vx,vy,wz (m/s, rad/s)
        if (cmd.startsWith("VEL:")) {
            float cmd_vx = 0.0, cmd_vy = 0.0, cmd_wz = 0.0;
            int comma1 = cmd.indexOf(',', 4);
            int comma2 = cmd.indexOf(',', comma1 + 1);
            if (comma1 != -1 && comma2 != -1) {
                cmd_vx = cmd.substring(4, comma1).toFloat();
                cmd_vy = cmd.substring(comma1 + 1, comma2).toFloat();
                cmd_wz = cmd.substring(comma2 + 1).toFloat();
            }

            // Cập nhật vận tốc toàn cục
            vx = cmd_vx;
            vy = cmd_vy;
            wz = cmd_wz;

            // Tính tốc độ mỗi bánh (kinematics mecanum)
            float v1 = cmd_vx - cmd_vy - cmd_wz * (LX + LY);  // Motor 1
            float v2 = cmd_vx + cmd_vy + cmd_wz * (LX + LY);  // Motor 2
            float v3 = cmd_vx + cmd_vy - cmd_wz * (LX + LY);  // Motor 3
            float v4 = cmd_vx - cmd_vy + cmd_wz * (LX + LY);  // Motor 4

            // Chuyển tốc độ thành PWM (0-255)
            int pwm1 = constrain(abs(v1 / 0.5 * 255), 0, 255);  // 0.5 m/s max
            int pwm2 = constrain(abs(v2 / 0.5 * 255), 0, 255);
            int pwm3 = constrain(abs(v3 / 0.5 * 255), 0, 255);
            int pwm4 = constrain(abs(v4 / 0.5 * 255), 0, 255);

            // Đặt hướng và PWM
            digitalWrite(M1_DIR, v1 >= 0 ? HIGH : LOW); analogWrite(M1_PWM, pwm1);
            digitalWrite(M2_DIR, v2 >= 0 ? HIGH : LOW); analogWrite(M2_PWM, pwm2);
            digitalWrite(M3_DIR, v3 >= 0 ? HIGH : LOW); analogWrite(M3_PWM, pwm3);
            digitalWrite(M4_DIR, v4 >= 0 ? HIGH : LOW); analogWrite(M4_PWM, pwm4);

            mySerial.println("OK");
        } else {
            // Dừng nếu lệnh không hợp lệ
            analogWrite(M1_PWM, 0);
            analogWrite(M2_PWM, 0);
            analogWrite(M3_PWM, 0);
            analogWrite(M4_PWM, 0);
            vx = vy = wz = 0.0;  // Reset vận tốc
            mySerial.println("OK");
        }
    }
}
