// ============================================================
//  Mobile Robot — LOW LEVEL CONTROLLER with RPi Communication
//  ESP32 ↔ Raspberry Pi via UART (Serial, 115200 baud)
//
//  Wheel diameter : 65 mm
//  Wheel base     : 372 mm
//
//  ── RPi → ESP32 (commands, 10Hz) ────────────────────────────
//  "V <linear_m_s> <angular_rad_s>\n"
//      Set velocity. RPi sends this every control cycle.
//      Example: "V 0.150 0.300\n"
//
//  "Y <yaw_deg>\n"
//      Set heading setpoint (yaw hold mode).
//      Example: "Y 90.0\n"
//
//  "M H\n"   — enable yaw HOLD at current heading
//  "M F\n"   — free angular mode (use angular from V command)
//  "M S\n"   — stop all motion immediately
//  "M R\n"   — reset yaw to 0 and encoders to 0
//
//  ── ESP32 → RPi (odometry, 25Hz) ────────────────────────────
//  "O <dist_L_m> <dist_R_m> <yaw_deg> <vel_L_ms> <vel_R_ms>\n"
//      Sent every 40ms.
//      RPi serial_node.py parses this and publishes /odom + /imu.
//      Example: "O 1.2341 1.2298 89.7 0.149 0.151\n"
//
//  ── RPi serial_node.py (pseudocode) ─────────────────────────
//  Receives "O ..." → computes x,y via diff-drive kinematics
//                   → publishes /odom (nav_msgs/Odometry)
//                   → publishes /imu  (sensor_msgs/Imu)
//  Receives /cmd_vel from robot_navigator.py
//                   → formats "V linear angular\n"
//                   → writes to serial port
//
//  Wiring:
//    MPU6050   SDA→21  SCL→22  VCC 3.3V  GND
//    Motor L   IN1→25  IN2→26  ENA→27
//    Motor R   IN3→32  IN4→33  ENB→14
//    Enc L     A→34    B→35
//    Enc R     A→36    B→39
//    UART RPi  TX→GPIO16  RX→GPIO17  (Serial2, 115200)
//              !! Use 3.3V logic — RPi GPIO is 3.3V !!
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Encoder.h>

// ── UART ports ───────────────────────────────────────────────
// Serial  (USB)    — debug monitor only
// Serial2 (GPIO16/17) — RPi communication
#define RPI_SERIAL      Serial2
#define RPI_BAUD        115200
#define RPI_TX_PIN      17
#define RPI_RX_PIN      16

// ── Pins ─────────────────────────────────────────────────────
#define L_IN1    25
#define L_IN2    26
#define L_ENA    27
#define R_IN3    32
#define R_IN4    33
#define R_ENB    14
#define ENC_L_A  34
#define ENC_L_B  35
#define ENC_R_A  36
#define ENC_R_B  39

// ── PWM ──────────────────────────────────────────────────────
#define PWM_FREQ   20000
#define PWM_RES    8
#define PWM_CH_L   0
#define PWM_CH_R   1

// ── Robot geometry ───────────────────────────────────────────
const float WHEEL_DIAM_M    = 0.065f;
const float WHEEL_BASE_M    = 0.372f;
const float COUNTS_PER_REV  = 1440.0f;
const float COUNTS_PER_M    = COUNTS_PER_REV / (PI * WHEEL_DIAM_M);
// ≈ 7053 counts/m for 65mm wheel + 1440 CPR

// ── Timing ───────────────────────────────────────────────────
const float TS_S    = 0.02f;   // 20ms control loop (50Hz)
const int   TS_MS   = 20;
const int   ODOM_PERIOD_MS = 40;   // send odom every 40ms (25Hz)

// ── Watchdog: if no V command received within this time, stop ─
const unsigned long CMD_TIMEOUT_MS = 500;

// ── Wheel PID gains ──────────────────────────────────────────
const float WHL_KP_VEL   = 0.20f;
const float WHL_KI_VEL   = 2.50f;
const float WHL_MAX_VEL  = 2000.0f;   // counts/s
const int   WHL_DEADZONE = 55;

// ── Yaw PID gains ────────────────────────────────────────────
const float YAW_KP      = 0.80f;
const float YAW_KI      = 0.05f;
const float YAW_KD      = 0.05f;
const float YAW_I_MAX   = 0.30f;
const float YAW_OUT_MAX = 0.50f;

// ── MPU6050 ──────────────────────────────────────────────────
#define MPU_ADDR        0x68
#define MPU_PWR_MGMT_1  0x6B
#define MPU_GYRO_CONFIG 0x1B
#define MPU_GYRO_ZOUT_H 0x47
const float GYRO_SCALE = (1.0f / 131.0f) * (PI / 180.0f);

// ============================================================
//  PID
// ============================================================
struct PID {
    float kp, ki, kd, i_max, out_max;
    float integral, prev_error;
    bool  first;

    void init(float _kp, float _ki, float _kd,
              float _i_max, float _out_max) {
        kp = _kp; ki = _ki; kd = _kd;
        i_max = _i_max; out_max = _out_max;
        reset();
    }

    void reset() {
        integral = 0.0f; prev_error = 0.0f; first = true;
    }

    float compute(float error, float dt) {
        if (first) { prev_error = error; first = false; }
        float p = kp * error;
        integral = constrain(integral + error * dt, -i_max, i_max);
        float i  = ki * integral;
        float d  = kd * (error - prev_error) / dt;
        prev_error = error;
        return constrain(p + i + d, -out_max, out_max);
    }
};

// ============================================================
//  Globals
// ============================================================
ESP32Encoder encL, encR;
PID yawPID;

// IMU state
float yaw_rad       = 0.0f;
float gyro_z_bias   = 0.0f;

// Command state (written by parser, read by control_step)
volatile float cmd_linear   = 0.0f;   // m/s from RPi
volatile float cmd_angular  = 0.0f;   // rad/s from RPi
float yaw_setpoint          = 0.0f;
bool  yaw_hold              = false;

// Watchdog
unsigned long last_cmd_ms = 0;

// Encoder state
long  prev_cnt_L  = 0;
long  prev_cnt_R  = 0;
float vel_int_L   = 0.0f;
float vel_int_R   = 0.0f;

// Odometry accumulators (reset on "M R")
float odom_dist_L = 0.0f;   // metres
float odom_dist_R = 0.0f;

// Last measured wheel velocities (for odom packet)
float cur_vel_L   = 0.0f;
float cur_vel_R   = 0.0f;

// ============================================================
//  Helpers
// ============================================================
float norm_angle(float a) {
    while (a >  PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

// ============================================================
//  MPU6050
// ============================================================
void mpu_init() {
    Wire.begin(21, 22);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_PWR_MGMT_1);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_GYRO_CONFIG);
    Wire.write(0x00);
    Wire.endTransmission();
    Serial.println("[IMU] MPU6050 OK");
}

int16_t read_gyro_z_raw() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_GYRO_ZOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);
    return (Wire.read() << 8) | Wire.read();
}

void calibrate_gyro(int n = 300) {
    Serial.println("[CAL] Gyro calibration — keep still...");
    long sum = 0;
    for (int i = 0; i < n; i++) { sum += read_gyro_z_raw(); delay(5); }
    gyro_z_bias = (float)sum / n;
    Serial.printf("[CAL] Bias = %.2f LSB\n", gyro_z_bias);
}

void update_yaw() {
    float gz = (read_gyro_z_raw() - gyro_z_bias) * GYRO_SCALE;
    yaw_rad   = norm_angle(yaw_rad + gz * TS_S);
}

// ============================================================
//  Motor driver
// ============================================================
void motor_init() {
    pinMode(L_IN1, OUTPUT); pinMode(L_IN2, OUTPUT);
    pinMode(R_IN3, OUTPUT); pinMode(R_IN4, OUTPUT);
    ledcSetup(PWM_CH_L, PWM_FREQ, PWM_RES);
    ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES);
    ledcAttachPin(L_ENA, PWM_CH_L);
    ledcAttachPin(R_ENB, PWM_CH_R);
}

void set_motor_L(int pwm) {
    pwm = constrain(pwm, -255, 255);
    digitalWrite(L_IN1, pwm >= 0 ? HIGH : LOW);
    digitalWrite(L_IN2, pwm >= 0 ? LOW  : HIGH);
    ledcWrite(PWM_CH_L, abs(pwm));
}

void set_motor_R(int pwm) {
    pwm = constrain(pwm, -255, 255);
    digitalWrite(R_IN3, pwm >= 0 ? HIGH : LOW);
    digitalWrite(R_IN4, pwm >= 0 ? LOW  : HIGH);
    ledcWrite(PWM_CH_R, abs(pwm));
}

void stop_motors() {
    set_motor_L(0); set_motor_R(0);
    vel_int_L = 0.0f; vel_int_R = 0.0f;
}

// ============================================================
//  Wheel velocity PI
// ============================================================
int wheel_pid(float vel_ref, float vel_meas, float &vel_int) {
    float err   = vel_ref - vel_meas;
    float unsat = WHL_KP_VEL * err + WHL_KI_VEL * vel_int;
    int   cmd   = constrain((int)unsat, -255, 255);
    if (abs(cmd) < 255)
        vel_int = constrain(vel_int + err * TS_S, -800.0f, 800.0f);
    if (abs(vel_ref) > 10.0f && abs(cmd) < WHL_DEADZONE)
        cmd = (vel_ref > 0) ? WHL_DEADZONE : -WHL_DEADZONE;
    return cmd;
}

// ============================================================
//  Control step (50Hz)
// ============================================================
void control_step() {
    // ── Read encoders ─────────────────────────────────────────
    long cnt_L = encL.getCount();
    long cnt_R = encR.getCount();
    cur_vel_L  = (cnt_L - prev_cnt_L) / TS_S;   // counts/s
    cur_vel_R  = (cnt_R - prev_cnt_R) / TS_S;
    prev_cnt_L = cnt_L;
    prev_cnt_R = cnt_R;

    // Accumulate odometry distances (metres)
    odom_dist_L = cnt_L / COUNTS_PER_M;
    odom_dist_R = cnt_R / COUNTS_PER_M;

    // ── Update yaw ────────────────────────────────────────────
    update_yaw();

    // ── Watchdog: stop if RPi silent too long ─────────────────
    if (millis() - last_cmd_ms > CMD_TIMEOUT_MS) {
        if (cmd_linear != 0.0f || cmd_angular != 0.0f) {
            cmd_linear  = 0.0f;
            cmd_angular = 0.0f;
            stop_motors();
            Serial.println("[WARN] CMD timeout — stopping");
        }
        return;
    }

    // ── Steering bias from yaw PID or direct angular ──────────
    float steer_bias = 0.0f;

    if (yaw_hold) {
        // Hold heading setpoint — corrects external disturbances
        float err  = norm_angle(yaw_setpoint - yaw_rad);
        steer_bias = yawPID.compute(err, TS_S);
    } else {
        // Direct angular rate from RPi cmd_vel
        // Convert angular (rad/s) → wheel speed difference bias
        float half_diff = cmd_angular * WHEEL_BASE_M / 2.0f;
        float diff_cnts = half_diff / (PI * WHEEL_DIAM_M / COUNTS_PER_REV);
        steer_bias = constrain(diff_cnts / WHL_MAX_VEL,
                               -YAW_OUT_MAX, YAW_OUT_MAX);
        yawPID.reset();
    }

    // ── Per-wheel velocity references ─────────────────────────
    float base_vel  = cmd_linear * COUNTS_PER_M;
    float vel_ref_L = constrain(base_vel * (1.0f - steer_bias),
                                -WHL_MAX_VEL, WHL_MAX_VEL);
    float vel_ref_R = constrain(base_vel * (1.0f + steer_bias),
                                -WHL_MAX_VEL, WHL_MAX_VEL);

    // ── Inner velocity PID ────────────────────────────────────
    set_motor_L(wheel_pid(vel_ref_L, cur_vel_L, vel_int_L));
    set_motor_R(wheel_pid(vel_ref_R, cur_vel_R, vel_int_R));
}

// ============================================================
//  Send odometry to RPi (25Hz)
//  Format: "O <dist_L_m> <dist_R_m> <yaw_deg> <vel_L_ms> <vel_R_ms>"
//
//  RPi serial_node.py parses this and computes:
//    dc  = (dist_L + dist_R) / 2          centre distance
//    dyaw = yaw_deg (use directly — IMU is authoritative)
//    x  += dc * cos(yaw)
//    y  += dc * sin(yaw)
//    publishes nav_msgs/Odometry and sensor_msgs/Imu
// ============================================================
void send_odom() {
    float vel_L_ms = (cur_vel_L / COUNTS_PER_M);   // counts/s → m/s
    float vel_R_ms = (cur_vel_R / COUNTS_PER_M);
    RPI_SERIAL.printf("O %.4f %.4f %.3f %.4f %.4f\n",
                      odom_dist_L,
                      odom_dist_R,
                      yaw_rad * 180.0f / PI,
                      vel_L_ms,
                      vel_R_ms);
}

// ============================================================
//  Parse command from RPi (Serial2)
//
//  V <linear> <angular>   — velocity setpoint from robot_navigator
//  Y <deg>                — heading setpoint (yaw hold mode)
//  M H                    — hold current heading
//  M F                    — free angular (use V angular)
//  M S                    — stop
//  M R                    — reset encoders and yaw
// ============================================================
void parse_rpi(String line) {
    line.trim();
    if (line.length() == 0) return;

    char type = line.charAt(0);

    if (type == 'V') {
        // "V 0.150 0.300"
        int sp = line.indexOf(' ', 2);
        if (sp > 0) {
            cmd_linear  = line.substring(2, sp).toFloat();
            cmd_angular = line.substring(sp + 1).toFloat();
            last_cmd_ms = millis();
        }

    } else if (type == 'Y') {
        // "Y 90.0" — set heading setpoint in degrees
        float deg      = line.substring(2).toFloat();
        yaw_setpoint   = deg * PI / 180.0f;
        yaw_hold       = true;
        yawPID.reset();
        last_cmd_ms    = millis();
        Serial.printf("[RPI] Yaw setpoint = %.1f°\n", deg);

    } else if (type == 'M') {
        char mode = (line.length() > 2) ? line.charAt(2) : ' ';
        last_cmd_ms = millis();

        if (mode == 'H') {
            // Hold current heading
            yaw_setpoint = yaw_rad;
            yaw_hold     = true;
            yawPID.reset();
            Serial.println("[RPI] Yaw HOLD enabled");

        } else if (mode == 'F') {
            // Free angular — use angular from V command
            yaw_hold = false;
            yawPID.reset();
            Serial.println("[RPI] Free angular mode");

        } else if (mode == 'S') {
            // Emergency stop
            cmd_linear  = 0.0f;
            cmd_angular = 0.0f;
            stop_motors();
            Serial.println("[RPI] STOP");

        } else if (mode == 'R') {
            // Full reset
            cmd_linear  = 0.0f;
            cmd_angular = 0.0f;
            stop_motors();
            yaw_rad      = 0.0f;
            yaw_setpoint = 0.0f;
            yawPID.reset();
            encL.setCount(0);
            encR.setCount(0);
            prev_cnt_L   = 0;
            prev_cnt_R   = 0;
            odom_dist_L  = 0.0f;
            odom_dist_R  = 0.0f;
            vel_int_L    = 0.0f;
            vel_int_R    = 0.0f;
            Serial.println("[RPI] Full reset — yaw=0, encoders=0");
        }
    }
}

// ============================================================
//  Arduino entry points
// ============================================================
void setup() {
    // USB debug monitor
    Serial.begin(115200);
    delay(300);
    Serial.println("============================================");
    Serial.println(" Mobile Robot — Low Level + RPi Comms");
    Serial.printf("  COUNTS_PER_M : %.1f\n", COUNTS_PER_M);
    Serial.println("  Serial2 (GPIO16/17) ← RPi UART");
    Serial.println("============================================");

    // RPi UART
    RPI_SERIAL.begin(RPI_BAUD, SERIAL_8N1, RPI_RX_PIN, RPI_TX_PIN);

    motor_init();
    stop_motors();

    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encL.attachHalfQuad(ENC_L_A, ENC_L_B);
    encR.attachHalfQuad(ENC_R_A, ENC_R_B);
    encL.setCount(0);
    encR.setCount(0);

    mpu_init();
    delay(200);
    calibrate_gyro(300);

    yawPID.init(YAW_KP, YAW_KI, YAW_KD, YAW_I_MAX, YAW_OUT_MAX);

    last_cmd_ms = millis();   // start watchdog from boot
    Serial.println("[BOOT] Ready — waiting for RPi commands on Serial2");
}

void loop() {
    unsigned long t0 = millis();

    // ── Read RPi commands ─────────────────────────────────────
    while (RPI_SERIAL.available()) {
        String line = RPI_SERIAL.readStringUntil('\n');
        parse_rpi(line);
    }

    // ── Control step ──────────────────────────────────────────
    control_step();

    // ── Send odometry to RPi at 25Hz ──────────────────────────
    static unsigned long last_odom_ms = 0;
    if (millis() - last_odom_ms >= ODOM_PERIOD_MS) {
        last_odom_ms = millis();
        send_odom();
    }

    // ── USB debug every 1s ────────────────────────────────────
    static unsigned long last_dbg = 0;
    if (millis() - last_dbg >= 1000) {
        last_dbg = millis();
        Serial.printf("[DBG] yaw=%.1f° lin=%.3f ang=%.3f "
                      "encL=%ld encR=%ld hold=%d wdog=%lums\n",
                      yaw_rad * 180.0f / PI,
                      cmd_linear, cmd_angular,
                      encL.getCount(), encR.getCount(),
                      yaw_hold ? 1 : 0,
                      millis() - last_cmd_ms);
    }

    // ── Maintain 20ms loop ────────────────────────────────────
    unsigned long el = millis() - t0;
    if (el < (unsigned long)TS_MS) delay(TS_MS - el);
}
