// ============================================================
//  Mobile Robot — LOW LEVEL TEST CODE (no RPi communication)
//  Purpose: Tune PID gains, verify motors, encoders, IMU yaw
//           All test commands sent via Arduino Serial Monitor
//
//  Wheel diameter : 65 mm
//  Wheel base     : 372 mm (from simulation calibration)
//  Encoder        : adjust COUNTS_PER_REV to your motor spec
//
//  Wiring:
//    MPU6050   SDA→21  SCL→22  VCC 3.3V  GND
//    Motor L   IN1→25  IN2→26  ENA→27
//    Motor R   IN3→32  IN4→33  ENB→14
//    Enc L     A→34    B→35
//    Enc R     A→36    B→39
//
//  Serial Monitor: 115200 baud, Newline endings
//  Commands:
//    fw <sec>          — move forward N seconds then stop
//    bw <sec>          — move backward N seconds then stop
//    lt <deg>          — turn left N degrees in place
//    rt <deg>          — turn right N degrees in place
//    sp <m/s>          — set linear speed (default 0.15)
//    yh                — yaw hold ON  (hold current heading)
//    yf                — yaw hold OFF (free spin)
//    yr                — reset yaw to zero
//    st                — stop immediately
//    info              — print current state
//    cal               — re-calibrate gyro (keep still)
// ============================================================

#include <Arduino.h>
#include <Wire.h>
#include <ESP32Encoder.h>

// ── Pins ────────────────────────────────────────────────────
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

// ── PWM (LEDC) ───────────────────────────────────────────────
#define PWM_FREQ   20000
#define PWM_RES    8
#define PWM_CH_L   0
#define PWM_CH_R   1

// ── Robot geometry ───────────────────────────────────────────
const float WHEEL_DIAM_M    = 0.065f;
const float WHEEL_BASE_M    = 0.372f;
const float COUNTS_PER_REV  = 1440.0f;   // ← adjust to your encoder spec
const float COUNTS_PER_M    = COUNTS_PER_REV / (PI * WHEEL_DIAM_M);
// COUNTS_PER_M ≈ 7053 counts per metre for 65mm wheel + 1440 CPR

// ── Control loop timing ──────────────────────────────────────
const float TS_S  = 0.02f;   // 20 ms
const int   TS_MS = 20;

// ── Wheel PID gains ──────────────────────────────────────────
// Outer position loop: P only → gives velocity reference
// Inner velocity loop: PI with anti-windup
const float WHL_KP_POS   = 0.30f;
const float WHL_KP_VEL   = 0.20f;    // ← tune first: increase until wheel tracks without oscillating
const float WHL_KI_VEL   = 2.50f;    // ← tune second: increase until steady-state error = 0
const float WHL_MAX_VEL  = 2000.0f;  // counts/s max reference
const int   WHL_DEADZONE = 55;       // ← tune: PWM below which motor stalls

// ── Yaw PID gains ────────────────────────────────────────────
const float YAW_KP      = 0.80f;
const float YAW_KI      = 0.05f;
const float YAW_KD      = 0.05f;
const float YAW_I_MAX   = 0.30f;
const float YAW_OUT_MAX = 0.50f;   // ±50% of base speed bias

// ── MPU6050 ─────────────────────────────────────────────────
#define MPU_ADDR        0x68
#define MPU_PWR_MGMT_1  0x6B
#define MPU_GYRO_CONFIG 0x1B
#define MPU_GYRO_ZOUT_H 0x47
const float GYRO_SCALE = (1.0f / 131.0f) * (PI / 180.0f); // LSB → rad/s

// ============================================================
//  PID structure
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
        integral   = 0.0f;
        prev_error = 0.0f;
        first      = true;
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

float yaw_rad       = 0.0f;
float yaw_setpoint  = 0.0f;
bool  yaw_hold      = false;   // disabled by default during testing
float cmd_linear    = 0.0f;    // m/s, set by test commands
float cmd_angular   = 0.0f;    // rad/s, set by test commands

float gyro_z_bias   = 0.0f;
float vel_int_L     = 0.0f;
float vel_int_R     = 0.0f;
long  prev_cnt_L    = 0;
long  prev_cnt_R    = 0;

// Test state machine
enum TestState { IDLE, MOVING_FWD, MOVING_BWD, TURNING };
TestState test_state      = IDLE;
unsigned long test_end_ms = 0;   // millis() when timed move ends
float target_yaw_rad      = 0.0f; // for turn commands

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
    Wire.write(0x00);   // ±250°/s
    Wire.endTransmission();
    Serial.println("[IMU] MPU6050 init OK");
}

int16_t read_gyro_z_raw() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_GYRO_ZOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 2, true);
    return (Wire.read() << 8) | Wire.read();
}

void calibrate_gyro(int samples = 300) {
    Serial.println("[CAL] Keep robot still for gyro calibration...");
    long sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += read_gyro_z_raw();
        delay(5);
    }
    gyro_z_bias = (float)sum / samples;
    Serial.printf("[CAL] Gyro Z bias = %.2f LSB  done\n", gyro_z_bias);
}

void update_yaw() {
    int16_t raw  = read_gyro_z_raw();
    float gyro_z = (raw - gyro_z_bias) * GYRO_SCALE;
    yaw_rad     += gyro_z * TS_S;
    while (yaw_rad >  PI) yaw_rad -= 2.0f * PI;
    while (yaw_rad < -PI) yaw_rad += 2.0f * PI;
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
    cmd_linear = 0.0f; cmd_angular = 0.0f;
}

// ============================================================
//  Per-wheel velocity PI (inner loop)
// ============================================================
int wheel_pid(float vel_ref, float vel_meas, float &vel_int) {
    float err     = vel_ref - vel_meas;
    float unsat   = WHL_KP_VEL * err + WHL_KI_VEL * vel_int;
    int   pwm_cmd = constrain((int)unsat, -255, 255);
    if (abs(pwm_cmd) < 255)
        vel_int = constrain(vel_int + err * TS_S, -800.0f, 800.0f);
    if (abs(vel_ref) > 10.0f && abs(pwm_cmd) < WHL_DEADZONE)
        pwm_cmd = (vel_ref > 0) ? WHL_DEADZONE : -WHL_DEADZONE;
    return pwm_cmd;
}

// ============================================================
//  Normalise angle to [-π, π]
// ============================================================
float norm_angle(float a) {
    while (a >  PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

// ============================================================
//  Control step — runs every 20ms
// ============================================================
void control_step() {
    // Read encoders
    long cnt_L = encL.getCount();
    long cnt_R = encR.getCount();
    float vel_L = (cnt_L - prev_cnt_L) / TS_S;
    float vel_R = (cnt_R - prev_cnt_R) / TS_S;
    prev_cnt_L  = cnt_L;
    prev_cnt_R  = cnt_R;

    // Update IMU yaw
    update_yaw();

    // ── Test state machine ───────────────────────────────────
    // Handles timed forward/backward moves and turns
    if (test_state == MOVING_FWD || test_state == MOVING_BWD) {
        if (millis() >= test_end_ms) {
            stop_motors();
            test_state = IDLE;
            Serial.println("[TEST] Move complete");
        }
    } else if (test_state == TURNING) {
        float err = norm_angle(target_yaw_rad - yaw_rad);
        if (abs(err) < 0.035f) {   // ~2 degrees tolerance
            stop_motors();
            yaw_setpoint = yaw_rad;
            yaw_hold     = true;
            test_state   = IDLE;
            Serial.printf("[TEST] Turn complete — yaw=%.1f°\n",
                          yaw_rad * 180.0f / PI);
            return;
        }
        // Rotate in place toward target
        float angular = constrain(err * 2.0f, -0.5f, 0.5f);
        cmd_angular   = angular;
        cmd_linear    = 0.0f;
    }

    // ── Yaw correction ───────────────────────────────────────
    float steer_bias = 0.0f;
    if (yaw_hold && test_state != TURNING) {
        float yaw_err = norm_angle(yaw_setpoint - yaw_rad);
        steer_bias    = yawPID.compute(yaw_err, TS_S);
    } else if (!yaw_hold && cmd_angular != 0.0f) {
        // Convert rad/s → wheel speed bias fraction
        float spd_diff = cmd_angular * WHEEL_BASE_M / 2.0f;
        steer_bias = (spd_diff / (PI * WHEEL_DIAM_M / COUNTS_PER_REV)) / WHL_MAX_VEL;
        steer_bias = constrain(steer_bias, -YAW_OUT_MAX, YAW_OUT_MAX);
        yawPID.reset();
    }

    // ── Compute per-wheel references ─────────────────────────
    float base_vel  = cmd_linear * COUNTS_PER_M;
    float vel_ref_L = constrain(base_vel * (1.0f - steer_bias),
                                -WHL_MAX_VEL, WHL_MAX_VEL);
    float vel_ref_R = constrain(base_vel * (1.0f + steer_bias),
                                -WHL_MAX_VEL, WHL_MAX_VEL);

    // ── Inner velocity PID ───────────────────────────────────
    int pwm_L = wheel_pid(vel_ref_L, vel_L, vel_int_L);
    int pwm_R = wheel_pid(vel_ref_R, vel_R, vel_int_R);

    set_motor_L(pwm_L);
    set_motor_R(pwm_R);
}

// ============================================================
//  Serial command parser (test commands only)
// ============================================================
void parse_command(String line) {
    line.trim();
    if (line.length() == 0) return;

    // ── fw <seconds> — move forward ──────────────────────────
    if (line.startsWith("fw ")) {
        float sec    = line.substring(3).toFloat();
        cmd_linear   = 0.15f;   // default speed
        cmd_angular  = 0.0f;
        yaw_hold     = true;
        yaw_setpoint = yaw_rad;  // lock current heading
        yawPID.reset();
        test_state   = MOVING_FWD;
        test_end_ms  = millis() + (unsigned long)(sec * 1000);
        Serial.printf("[TEST] Moving forward %.1f s at %.2f m/s\n", sec, cmd_linear);

    // ── bw <seconds> — move backward ─────────────────────────
    } else if (line.startsWith("bw ")) {
        float sec    = line.substring(3).toFloat();
        cmd_linear   = -0.15f;
        cmd_angular  = 0.0f;
        yaw_hold     = true;
        yaw_setpoint = yaw_rad;
        yawPID.reset();
        test_state   = MOVING_BWD;
        test_end_ms  = millis() + (unsigned long)(sec * 1000);
        Serial.printf("[TEST] Moving backward %.1f s\n", sec);

    // ── lt <degrees> — turn left in place ────────────────────
    } else if (line.startsWith("lt ")) {
        float deg      = line.substring(3).toFloat();
        target_yaw_rad = norm_angle(yaw_rad + deg * PI / 180.0f);
        cmd_linear     = 0.0f;
        yaw_hold       = false;
        test_state     = TURNING;
        Serial.printf("[TEST] Turning left %.1f°  target yaw=%.1f°\n",
                      deg, target_yaw_rad * 180.0f / PI);

    // ── rt <degrees> — turn right in place ───────────────────
    } else if (line.startsWith("rt ")) {
        float deg      = line.substring(3).toFloat();
        target_yaw_rad = norm_angle(yaw_rad - deg * PI / 180.0f);
        cmd_linear     = 0.0f;
        yaw_hold       = false;
        test_state     = TURNING;
        Serial.printf("[TEST] Turning right %.1f°  target yaw=%.1f°\n",
                      deg, target_yaw_rad * 180.0f / PI);

    // ── sp <m/s> — set speed for future moves ────────────────
    } else if (line.startsWith("sp ")) {
        float spd = line.substring(3).toFloat();
        // This stores the value — next fw/bw uses it
        // For live speed change while moving:
        if (test_state == MOVING_FWD)  cmd_linear =  spd;
        if (test_state == MOVING_BWD)  cmd_linear = -spd;
        Serial.printf("[TEST] Speed set to %.3f m/s\n", spd);

    // ── yh — enable yaw hold at current heading ───────────────
    } else if (line == "yh") {
        yaw_setpoint = yaw_rad;
        yaw_hold     = true;
        yawPID.reset();
        Serial.printf("[TEST] Yaw HOLD ON — locking %.1f°\n",
                      yaw_setpoint * 180.0f / PI);

    // ── yf — free yaw (disable hold) ─────────────────────────
    } else if (line == "yf") {
        yaw_hold = false;
        yawPID.reset();
        Serial.println("[TEST] Yaw HOLD OFF — free rotation");

    // ── yr — reset yaw to zero ────────────────────────────────
    } else if (line == "yr") {
        yaw_rad      = 0.0f;
        yaw_setpoint = 0.0f;
        yawPID.reset();
        Serial.println("[TEST] Yaw reset to 0°");

    // ── st — emergency stop ───────────────────────────────────
    } else if (line == "st") {
        stop_motors();
        test_state = IDLE;
        Serial.println("[TEST] STOP");

    // ── cal — re-calibrate gyro ───────────────────────────────
    } else if (line == "cal") {
        stop_motors();
        test_state = IDLE;
        calibrate_gyro(300);

    // ── info — print current state ────────────────────────────
    } else if (line == "info") {
        float dist_L = encL.getCount() / COUNTS_PER_M;
        float dist_R = encR.getCount() / COUNTS_PER_M;
        Serial.println("──────────── ROBOT STATE ────────────");
        Serial.printf("  Yaw        : %.2f°  (setpoint %.2f°)\n",
                      yaw_rad * 180.0f / PI,
                      yaw_setpoint * 180.0f / PI);
        Serial.printf("  Yaw hold   : %s\n", yaw_hold ? "ON" : "OFF");
        Serial.printf("  Enc L dist : %.4f m  (%ld counts)\n",
                      dist_L, encL.getCount());
        Serial.printf("  Enc R dist : %.4f m  (%ld counts)\n",
                      dist_R, encR.getCount());
        Serial.printf("  cmd_linear : %.3f m/s\n", cmd_linear);
        Serial.printf("  cmd_angular: %.3f rad/s\n", cmd_angular);
        Serial.printf("  Test state : %d\n", test_state);
        Serial.printf("  YAW PID    : I=%.3f\n", yawPID.integral);
        Serial.println("─────────────────────────────────────");

    } else {
        Serial.println("[ERR] Unknown command — valid: fw bw lt rt sp yh yf yr st cal info");
    }
}

// ============================================================
//  Arduino entry points
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(300);
    Serial.println("================================================");
    Serial.println(" Mobile Robot Test Code — no RPi communication");
    Serial.println("  Wheel diam : 65 mm");
    Serial.printf("  COUNTS_PER_M: %.1f\n", COUNTS_PER_M);
    Serial.println("================================================");

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

    Serial.println("\nReady. Commands:");
    Serial.println("  fw <sec>   bw <sec>   lt <deg>   rt <deg>");
    Serial.println("  sp <m/s>   yh   yf   yr   st   cal   info\n");
}

void loop() {
    unsigned long t0 = millis();

    // Parse incoming test command
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        parse_command(cmd);
    }

    // Run control
    control_step();

    // Debug print every 500ms during motion
    static unsigned long last_dbg = 0;
    if (test_state != IDLE && millis() - last_dbg > 500) {
        last_dbg = millis();
        Serial.printf("[DBG] yaw=%.1f° encL=%ld encR=%ld lin=%.2f bias=%.2f\n",
                      yaw_rad * 180.0f / PI,
                      encL.getCount(), encR.getCount(),
                      cmd_linear, yawPID.integral);
    }

    // Maintain 20ms loop
    unsigned long el = millis() - t0;
    if (el < (unsigned long)TS_MS) delay(TS_MS - el);
}
