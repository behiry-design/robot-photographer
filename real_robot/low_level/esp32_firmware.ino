#include <Arduino.h>
#include <Wire.h>
#include <ESP32Encoder.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/pose2_d.h>
#include <nav_msgs/msg/odometry.h>

// ── Pins ────────────────────────────────────────────────────
#define L_IN1    25
#define L_IN2    26
#define L_ENA    27
#define R_IN1    32
#define R_IN2    33
#define R_ENA    14
#define ENC_L_A  34
#define ENC_L_B  35
#define ENC_R_A  18
#define ENC_R_B  19
#define LED_G    23
#define LED_Y    12
#define LED_R    5
#define US_L_TRIG  4
#define US_L_ECHO  13
#define US_R_TRIG  2
#define US_R_ECHO  15
#define SERVO_PIN  17

// ── MicroROS ─────────────────────────────────────────────────
rcl_publisher_t odom_pub;
rcl_publisher_t wp_reached_pub;
rcl_publisher_t cam_ready_pub;
rcl_publisher_t cam_done_pub;
rcl_publisher_t obstacle_pub;

rcl_subscription_t cmd_pos_sub;
rcl_subscription_t robot_mode_sub;
rcl_subscription_t cam_next_sub;
rcl_subscription_t cmd_dir_sub;

nav_msgs__msg__Odometry     odom_msg;
std_msgs__msg__Bool         wp_reached_msg;
std_msgs__msg__Int32        cam_ready_msg;
std_msgs__msg__Bool         cam_done_msg;
std_msgs__msg__String       obstacle_msg;
geometry_msgs__msg__Pose2D  cmd_pos_msg;
std_msgs__msg__String       robot_mode_msg;
std_msgs__msg__Bool         cam_next_msg;
std_msgs__msg__String       cmd_dir_msg;

rclc_support_t   support;
rcl_allocator_t  allocator;
rcl_node_t       node;
rclc_executor_t  executor;

char mode_buf[20]     = "STOP";
char dir_buf[5]       = "S";
char obstacle_buf[20] = "CLEAR";

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// ── State Enumerations ───────────────────────────────────────
enum TestState {
    STATE_IDLE,
    STATE_MOVING,
    STATE_WAITING_FOR_CLEARANCE,
    STATE_DETOUR,
    STATE_MANUAL
};
volatile TestState test_state = STATE_IDLE;

enum MechState {
    MECH_IDLE,
    MECH_MOVING,
    MECH_DONE
};
volatile MechState mech_state = MECH_IDLE;

// ── Camera sequence state ────────────────────────────────────
int cam_angles[3]    = {0, 45, 90};
int cam_angle_idx    = 0;
bool cam_sequence_active = false;
bool cam_next_flag   = false;

// ── Spatial tracking ─────────────────────────────────────────
float robot_X = 0.0, robot_Y = 0.0;
float target_X = 0.0, target_Y = 0.0, target_Theta = 0.0;
int nav_phase = 0;

int target_servo_angle  = 0;
int current_servo_angle = 0;

bool obstacle_flag = false;
unsigned long obstacle_detected_time = 0;
const unsigned long CLEARANCE_WAIT_MS = 5000;

int detourStep = 0;
float saved_target_dist = 0.0;
float saved_yaw_setpoint = 0.0;
int saved_nav_phase = 0;

// Detour stabilization / timeout
unsigned long path_clear_time = 0;
bool waiting_for_clear_delay = false;
const unsigned long POST_CLEAR_DELAY_MS = 2000;

const float D_SIDE  = 0.25;
const float D_FRONT = 1.2;
const float W_OBSTACLE = 0.30;

// ── Robot geometry ───────────────────────────────────────────
const float WHEEL_DIAM_M   = 0.065;
const float WHEEL_BASE_M   = 0.35;
const float COUNTS_PER_REV = 1955;
const float COUNTS_PER_M   = COUNTS_PER_REV / (PI * WHEEL_DIAM_M);
const float TS_S  = 0.02;
const int   TS_MS = 20;
const float MAX_LINEAR_VEL = 0.25;
const int   L_DEADZONE   = 180;
const float L_VEL_TO_PWM = 0.158;
const int   R_DEADZONE   = 142;
const float R_VEL_TO_PWM = 0.135;
const float WHL_KP_VEL = 0.20;
const float WHL_KI_VEL = 0.6;
const float YAW_KP = 100, YAW_KI = 0, YAW_KD = 0;
const float YAW_I_MAX = 0.30, YAW_OUT_MAX = 0.50;

// ── Globals ───────────────────────────────────────────────────
Adafruit_MPU6050 mpu;
ESP32Encoder encL, encR;
float gx_off = 0, gz_off = 0;
float yaw_rad = 0.0, yaw_setpoint = 0.0;
bool  yaw_hold = false;
float cmd_linear = 0.0, cmd_angular = 0.0;
float vel_int_L = 0.0, vel_int_R = 0.0;
long  prev_cnt_L = 0, prev_cnt_R = 0;
long  start_cnt_L = 0, start_cnt_R = 0;
float target_dist_m = 0.0;
unsigned long last_imu_micros = 0;

QueueHandle_t     cmdQueue;
SemaphoreHandle_t yawMutex;
SemaphoreHandle_t telemMutex;
SemaphoreHandle_t odomMutex;

struct TelemetryData {
    float yaw_deg, yaw_setpoint_deg, dist;
    long encL, encR;
    int pwmL, pwmR;
    TestState state;
    MechState m_state;
    float odom_x, odom_y;
} telemetry;

struct PID {
    float kp, ki, kd, i_max, out_max, integral, prev_error;
    bool first = true;
    void init(float p, float i, float d, float im, float om) {
        kp=p; ki=i; kd=d; i_max=im; out_max=om; reset();
    }
    void reset() { integral=0; prev_error=0; first=true; }
    float compute(float error, float dt) {
        if(first){ prev_error=error; first=false; }
        integral = constrain(integral + error*dt, -i_max, i_max);
        float d = (error - prev_error) / dt;
        prev_error = error;
        return constrain(kp*error + ki*integral + kd*d, -out_max, out_max);
    }
} yawPID;

// ── Helpers ───────────────────────────────────────────────────
float norm_angle(float a) {
    while(a >  PI) a -= 2.0*PI;
    while(a < -PI) a += 2.0*PI;
    return a;
}

void error_loop(){
    while(1){
        digitalWrite(LED_R, !digitalRead(LED_R));
        delay(100);
    }
}

void set_motors(int pL, int pR) {
    digitalWrite(L_IN1, pL>=0?HIGH:LOW); digitalWrite(L_IN2, pL>=0?LOW:HIGH);
    analogWrite(L_ENA, abs(pL));
    digitalWrite(R_IN1, pR>=0?HIGH:LOW); digitalWrite(R_IN2, pR>=0?LOW:HIGH);
    analogWrite(R_ENA, abs(pR));
}

void stop_motors() {
    set_motors(0,0);
    vel_int_L=0; vel_int_R=0;
    cmd_linear=0; cmd_angular=0;
}

void init_segment_move(float distance) {
    start_cnt_L = encL.getCount();
    start_cnt_R = encR.getCount();
    target_dist_m = distance;
    cmd_linear = (distance>=0) ? 0.15 : -0.15;
    yaw_hold = true;
    yaw_setpoint = yaw_rad;
}

void init_segment_turn(float target_angle_rad) {
    yaw_setpoint = norm_angle(target_angle_rad);
    yaw_hold = true;
    cmd_linear = 0.0;
    yawPID.reset();
}

int wheel_pid_custom(float vel_ref, float vel_meas, float &vel_int, float feed_forward, int deadzone) {
    float err = vel_ref - vel_meas;
    float unsat = (WHL_KP_VEL*err + WHL_KI_VEL*vel_int) * feed_forward;
    int pwm_cmd = constrain((int)unsat, -255, 255);
    if(abs(pwm_cmd)<255) vel_int = constrain(vel_int+err*TS_S, -800.0, 800.0);
    if(abs(vel_ref)>5.0 && abs(pwm_cmd)<deadzone)
        pwm_cmd = (vel_ref>0) ? deadzone : -deadzone;
    return pwm_cmd;
}

void calibrateMPU() {
    gz_off = 0;
    for(int i=0; i<1000; i++){
        sensors_event_t a,g,t; mpu.getEvent(&a,&g,&t);
        gz_off += g.gyro.z; delay(2);
    }
    gz_off /= 1000.0;
}

float read_ultrasonic_distance(int trig, int echo) {
    digitalWrite(trig, LOW); delayMicroseconds(2);
    digitalWrite(trig, HIGH); delayMicroseconds(10);
    digitalWrite(trig, LOW);
    long duration = pulseIn(echo, HIGH, 15000);
    if(duration==0) return 400.0;
    return (duration*0.0343)/2.0/100.0;
}

// ── Publish obstacle state ────────────────────────────────────
void publish_obstacle(const char* state_str){
    strncpy(obstacle_buf, state_str, 19);
    obstacle_msg.data.size = strlen(obstacle_buf);
    RCSOFTCHECK(rcl_publish(&obstacle_pub, &obstacle_msg, NULL));
}

// ── Publish odometry ──────────────────────────────────────────
void publish_odom(float x, float y, float yaw){
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.orientation.z = sin(yaw/2.0);
    odom_msg.pose.pose.orientation.w = cos(yaw/2.0);
    RCSOFTCHECK(rcl_publish(&odom_pub, &odom_msg, NULL));
}

// ── MicroROS Callbacks ────────────────────────────────────────
void cmd_pos_callback(const void* msg_in){
    const geometry_msgs__msg__Pose2D* msg = (const geometry_msgs__msg__Pose2D*)msg_in;
    target_X     = msg->x;
    target_Y     = msg->y;
    target_Theta = msg->theta;
    nav_phase    = 1;
    test_state   = STATE_MOVING;
    obstacle_flag = false;
    detourStep   = 0;
    waiting_for_clear_delay = false;
}

void robot_mode_callback(const void* msg_in){
    const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msg_in;
    char prev_mode[20];
    strcpy(prev_mode, mode_buf);
    strncpy(mode_buf, msg->data.data, 19);
    mode_buf[19] = '\0';

    if(strcmp(mode_buf, "MANUAL") == 0){
        test_state = STATE_MANUAL;
        stop_motors();
    }
    else if(strcmp(mode_buf, "STOP") == 0){
        stop_motors();
        test_state = STATE_IDLE;
        nav_phase = 0;
        if(strcmp(prev_mode, "MANUAL") == 0){
            target_X = 0.0; target_Y = 0.0; target_Theta = 0.0;
            nav_phase = 1;
            test_state = STATE_MOVING;
        }
    }
    else if(strcmp(mode_buf, "AUTO") == 0){
        if(strcmp(prev_mode, "MANUAL") == 0){
            target_X = 0.0; target_Y = 0.0; target_Theta = 0.0;
            nav_phase = 1;
            test_state = STATE_MOVING;
        }
    }
    else if(strcmp(mode_buf, "RESET") == 0){
        stop_motors();
        robot_X = 0; robot_Y = 0; yaw_rad = 0;
        test_state = STATE_IDLE;
        nav_phase = 0;
    }
}

void cam_next_callback(const void* msg_in){
    const std_msgs__msg__Bool* msg = (const std_msgs__msg__Bool*)msg_in;
    if(msg->data) cam_next_flag = true;
}

void cmd_dir_callback(const void* msg_in){
    const std_msgs__msg__String* msg = (const std_msgs__msg__String*)msg_in;
    strncpy(dir_buf, msg->data.data, 4);
    dir_buf[4] = '\0';

    if(test_state != STATE_MANUAL) return;

    if(strcmp(dir_buf, "F") == 0){
        cmd_linear = 0.15; cmd_angular = 0.0;
    } else if(strcmp(dir_buf, "B") == 0){
        cmd_linear = -0.15; cmd_angular = 0.0;
    } else if(strcmp(dir_buf, "L") == 0){
        cmd_linear = 0.0; cmd_angular = 1.0;
    } else if(strcmp(dir_buf, "R") == 0){
        cmd_linear = 0.0; cmd_angular = -1.0;
    } else {
        cmd_linear = 0.0; cmd_angular = 0.0;
        stop_motors();
    }
}

// ════════════════════════════════════════════════════════════
// TASK 1 — IMU
// ════════════════════════════════════════════════════════════
void imuTask(void *pv){
    for(;;){
        sensors_event_t a,g,t; mpu.getEvent(&a,&g,&t);
        unsigned long now = micros();
        float dt_imu = (now - last_imu_micros) / 1000000.0;
        last_imu_micros = now;
        float delta_yaw = (g.gyro.z - gz_off) * dt_imu;
        if(xSemaphoreTake(yawMutex, portMAX_DELAY)){
            yaw_rad += delta_yaw;
            xSemaphoreGive(yawMutex);
        }
        vTaskDelay(1);
    }
}

// ════════════════════════════════════════════════════════════
// TASK 2 — Control Loop
// ════════════════════════════════════════════════════════════
void controlTask(void *pv){
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(TS_MS);
    bool phase_init_done = false;

    for(;;){
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        long cL = encL.getCount(); long cR = encR.getCount();
        float delta_dist_L = (cL - prev_cnt_L) / COUNTS_PER_M;
        float delta_dist_R = (cR - prev_cnt_R) / COUNTS_PER_M;
        float delta_robot_dist = (delta_dist_L + delta_dist_R) / 2.0;

        float local_yaw;
        if(xSemaphoreTake(yawMutex, pdMS_TO_TICKS(2))){
            local_yaw = yaw_rad; xSemaphoreGive(yawMutex);
        } else { local_yaw = yaw_rad; }

        if(xSemaphoreTake(odomMutex, pdMS_TO_TICKS(2))){
            robot_X += delta_robot_dist * cos(local_yaw);
            robot_Y += delta_robot_dist * sin(local_yaw);
            xSemaphoreGive(odomMutex);
        }

        float vel_L = (cL - prev_cnt_L) / TS_S;
        float vel_R = (cR - prev_cnt_R) / TS_S;
        prev_cnt_L = cL; prev_cnt_R = cR;

        if(test_state == STATE_WAITING_FOR_CLEARANCE || test_state == STATE_IDLE){
            set_motors(0,0); phase_init_done = false; continue;
        }

        if(test_state == STATE_MANUAL){
            float vRefL = (cmd_linear - (cmd_angular * WHEEL_BASE_M/2.0)) * COUNTS_PER_M;
            float vRefR = (cmd_linear + (cmd_angular * WHEEL_BASE_M/2.0)) * COUNTS_PER_M;
            int pwmL = wheel_pid_custom(vRefL, vel_L, vel_int_L, L_VEL_TO_PWM, L_DEADZONE);
            int pwmR = wheel_pid_custom(vRefR, vel_R, vel_int_R, R_VEL_TO_PWM, R_DEADZONE);
            if(cmd_linear==0 && cmd_angular==0){ pwmL=0; pwmR=0; vel_int_L=0; vel_int_R=0; }
            set_motors(pwmL, pwmR);
            continue;
        }

        if(test_state == STATE_MOVING && detourStep == 0){
            float dx = target_X - robot_X;
            float dy = target_Y - robot_Y;

            if(nav_phase == 1){
                if(abs(dx) < 0.02){ nav_phase=2; phase_init_done=false; }
                else {
                    if(!phase_init_done){
                        float target_heading = 0.0;
                        float heading_err = norm_angle(target_heading - local_yaw);
                        if(abs(heading_err) > (2.0*PI/180.0)) init_segment_turn(target_heading);
                        else { init_segment_move(dx); phase_init_done=true; }
                    }
                }
            }
            else if(nav_phase == 2){
                if(abs(dy) < 0.02){ nav_phase=3; phase_init_done=false; }
                else {
                    if(!phase_init_done){
                        float target_heading = PI/2.0;
                        float heading_err = norm_angle(target_heading - local_yaw);
                        if(abs(heading_err) > (2.0*PI/180.0)) init_segment_turn(target_heading);
                        else { init_segment_move(dy); phase_init_done=true; }
                    }
                }
            }
            else if(nav_phase == 3){
                if(isnan(target_Theta)){
                    stop_motors();
                    test_state = STATE_IDLE;
                    nav_phase = 0;
                    phase_init_done = false;
                    wp_reached_msg.data = true;
                    RCSOFTCHECK(rcl_publish(&wp_reached_pub, &wp_reached_msg, NULL));
                    cam_sequence_active = true;
                    cam_angle_idx = 0;
                    cam_next_flag = false;
                    continue;
                }

                if(!phase_init_done){ init_segment_turn(target_Theta); phase_init_done=true; }
                else {
                    float th_err = norm_angle(target_Theta - local_yaw);
                    if(abs(th_err) < (1.5*PI/180.0)){
                        stop_motors();
                        test_state = STATE_IDLE;
                        nav_phase = 0;
                        phase_init_done = false;
                        wp_reached_msg.data = true;
                        RCSOFTCHECK(rcl_publish(&wp_reached_pub, &wp_reached_msg, NULL));
                        cam_sequence_active = true;
                        cam_angle_idx = 0;
                        cam_next_flag = false;
                        continue;
                    }
                }
            }
        }

        if (test_state == STATE_DETOUR && detourStep > 0) {
            float running_dist = abs(((float)(cL - start_cnt_L) + (float)(cR - start_cnt_R)) / (2.0 * COUNTS_PER_M));
            float current_yaw_err = norm_angle(yaw_setpoint - local_yaw);

            bool step_complete = false;
            if ((cmd_linear != 0) && (abs(target_dist_m) - running_dist < 0.02)) step_complete = true;
            if ((cmd_linear == 0) && (abs(current_yaw_err) < (1.5 * PI / 180.0))) step_complete = true;

            if (step_complete || detourStep == 1) {
                static float bypass_dist = 0.0;
                
                if (detourStep == 1) {
                    bypass_dist = D_SIDE + (W_OBSTACLE / 2.0);
                }

                switch (detourStep) {
                    case 1:
                        yaw_setpoint = norm_angle(saved_yaw_setpoint + (PI / 2.0));
                        yaw_hold = true; cmd_linear = 0; yawPID.reset();
                        detourStep = 2; break;
                    case 2:
                        start_cnt_L = cL; start_cnt_R = cR;
                        target_dist_m = bypass_dist; cmd_linear = 0.15;
                        yaw_hold = true;
                        detourStep = 3; break;
                    case 3:
                        yaw_setpoint = norm_angle(saved_yaw_setpoint);
                        yaw_hold = true; cmd_linear = 0; yawPID.reset();
                        detourStep = 4; break;
                    case 4:
                        start_cnt_L = cL; start_cnt_R = cR;
                        target_dist_m = D_FRONT; cmd_linear = 0.15;
                        yaw_hold = true;
                        detourStep = 5; break;
                    case 5:
                        yaw_setpoint = norm_angle(saved_yaw_setpoint - (PI / 2.0));
                        yaw_hold = true; cmd_linear = 0; yawPID.reset();
                        detourStep = 6; break;
                    case 6:
                        start_cnt_L = cL; start_cnt_R = cR;
                        target_dist_m = bypass_dist; cmd_linear = 0.15;
                        yaw_hold = true;
                        detourStep = 7; break;
                    case 7:
                        yaw_setpoint = saved_yaw_setpoint;
                        yaw_hold = true; cmd_linear = 0; yawPID.reset();
                        detourStep = 8; break;
                    case 8:
                        start_cnt_L = cL; start_cnt_R = cR;
                        target_dist_m = saved_target_dist;
                        yaw_setpoint = saved_yaw_setpoint;
                        nav_phase = saved_nav_phase;
                        yaw_hold = true; cmd_linear = (target_dist_m >= 0) ? 0.15 : -0.15;
                        test_state = STATE_MOVING;
                        obstacle_flag = false;
                        detourStep = 0;
                        phase_init_done = true;
                        waiting_for_clear_delay = false;
                        break;
                }
            }
        }

        float dist = abs(((float)(cL-start_cnt_L)+(float)(cR-start_cnt_R))/(2.0*COUNTS_PER_M));
        float steer_bias = 0.0;
        bool is_driving = (cmd_linear != 0.0);
        bool is_turning = (cmd_linear == 0.0 && yaw_hold);

        if(is_driving){
            float dist_error = abs(target_dist_m) - dist;
            if(test_state==STATE_MOVING && detourStep==0 && dist_error<0.02){
                stop_motors(); phase_init_done=false;
                float dy = target_Y - robot_Y;
                if((nav_phase==2 && abs(dy)<0.02 && abs(target_Theta)<0.02)||(nav_phase==0)){
                    test_state=STATE_IDLE; nav_phase=0;
                    wp_reached_msg.data = true;
                    RCSOFTCHECK(rcl_publish(&wp_reached_pub, &wp_reached_msg, NULL));
                    cam_sequence_active = true;
                    cam_angle_idx = 0;
                    cam_next_flag = false;
                }
                continue;
            }
            float yaw_err = norm_angle(yaw_setpoint - local_yaw);
            steer_bias = yawPID.compute(yaw_err, TS_S);
            if(cmd_linear < 0) steer_bias = -steer_bias;
        }
        else if(is_turning){
            float yaw_err = norm_angle(yaw_setpoint - local_yaw);
            steer_bias = constrain(yaw_err*2.5, -0.5, 0.5);
            if(test_state==STATE_MOVING && detourStep==0 && abs(yaw_err)<(1.5*PI/180.0)){
                stop_motors(); phase_init_done=false;
                if(nav_phase==3){
                    test_state=STATE_IDLE; nav_phase=0;
                    wp_reached_msg.data = true;
                    RCSOFTCHECK(rcl_publish(&wp_reached_pub, &wp_reached_msg, NULL));
                    cam_sequence_active = true;
                    cam_angle_idx = 0;
                    cam_next_flag = false;
                }
                continue;
            }
        }

        float base_vel = cmd_linear * COUNTS_PER_M;
        float vRefL=0, vRefR=0;
        if(is_turning){
            float turn_ff = (steer_bias>=0) ? 500.0 : -500.0;
            vRefL = (-turn_ff) - (steer_bias*800.0);
            vRefR = (turn_ff)  + (steer_bias*800.0);
        } else {
            float differential = steer_bias * 400.0;
            vRefL = base_vel - differential;
            vRefR = base_vel + differential;
        }

        int pwmL = wheel_pid_custom(vRefL, vel_L, vel_int_L, L_VEL_TO_PWM, L_DEADZONE);
        int pwmR = wheel_pid_custom(vRefR, vel_R, vel_int_R, R_VEL_TO_PWM, R_DEADZONE);
        set_motors(constrain(pwmL,-255,255), constrain(pwmR,-255,255));

        if(xSemaphoreTake(telemMutex, 0)){
            telemetry.yaw_deg = local_yaw*180.0/PI;
            telemetry.dist = dist;
            telemetry.encL = cL; telemetry.encR = cR;
            telemetry.pwmL = pwmL; telemetry.pwmR = pwmR;
            telemetry.state = test_state;
            telemetry.m_state = mech_state;
            if(xSemaphoreTake(odomMutex, 0)){
                telemetry.odom_x = robot_X; telemetry.odom_y = robot_Y;
                xSemaphoreGive(odomMutex);
            }
            xSemaphoreGive(telemMutex);
        }
    }
}

// ════════════════════════════════════════════════════════════
// TASK 3 — MicroROS Task
// ════════════════════════════════════════════════════════════
void microsRosTask(void *pv){
    vTaskDelay(pdMS_TO_TICKS(2000));

    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "esp32_robot_node", "", &support));

    // Publishers
    RCCHECK(rclc_publisher_init_default(&odom_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom"));
    RCCHECK(rclc_publisher_init_default(&wp_reached_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/waypoint_reached"));
    RCCHECK(rclc_publisher_init_default(&cam_ready_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "/cam_ready"));
    RCCHECK(rclc_publisher_init_default(&cam_done_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/cam_done"));
    RCCHECK(rclc_publisher_init_default(&obstacle_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/obstacle_state"));

    // Subscribers
    RCCHECK(rclc_subscription_init_default(&cmd_pos_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D), "/cmd_position"));
    RCCHECK(rclc_subscription_init_default(&robot_mode_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/robot_mode"));
    RCCHECK(rclc_subscription_init_default(&cam_next_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "/cam_next"));
    RCCHECK(rclc_subscription_init_default(&cmd_dir_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/cmd_direction"));

    // Message buffers
    robot_mode_msg.data.data = mode_buf;
    robot_mode_msg.data.capacity = 20;
    robot_mode_msg.data.size = 0;
    cmd_dir_msg.data.data = dir_buf;
    cmd_dir_msg.data.capacity = 5;
    cmd_dir_msg.data.size = 0;
    obstacle_msg.data.data = obstacle_buf;
    obstacle_msg.data.capacity = 20;
    obstacle_msg.data.size = 0;

    // Executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_pos_sub,
        &cmd_pos_msg, &cmd_pos_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &robot_mode_sub,
        &robot_mode_msg, &robot_mode_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &cam_next_sub,
        &cam_next_msg, &cam_next_callback, ON_NEW_DATA));
    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_dir_sub,
        &cmd_dir_msg, &cmd_dir_callback, ON_NEW_DATA));

    unsigned long last_odom_time = 0;

    for(;;){
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        unsigned long now = millis();
        if(now - last_odom_time > 100){
            last_odom_time = now;
            float local_x, local_y, local_yaw;
            if(xSemaphoreTake(odomMutex, pdMS_TO_TICKS(2))){
                local_x = robot_X; local_y = robot_Y;
                xSemaphoreGive(odomMutex);
            }
            if(xSemaphoreTake(yawMutex, pdMS_TO_TICKS(2))){
                local_yaw = yaw_rad;
                xSemaphoreGive(yawMutex);
            }
            publish_odom(local_x, local_y, local_yaw);
        }

        if(cam_sequence_active){
            if(cam_angle_idx < 3){
                if(cam_next_flag || cam_angle_idx == 0){
                    cam_next_flag = false;
                    target_servo_angle = cam_angles[cam_angle_idx];
                    mech_state = MECH_MOVING;
                    vTaskDelay(pdMS_TO_TICKS(500));
                    cam_ready_msg.data = cam_angles[cam_angle_idx];
                    RCSOFTCHECK(rcl_publish(&cam_ready_pub, &cam_ready_msg, NULL));
                    cam_angle_idx++;
                }
            } else {
                cam_done_msg.data = true;
                RCSOFTCHECK(rcl_publish(&cam_done_pub, &cam_done_msg, NULL));
                cam_sequence_active = false;
                cam_angle_idx = 0;
                target_servo_angle = 0;
                mech_state = MECH_MOVING;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ════════════════════════════════════════════════════════════
// TASK 4 — Ultrasonic
// ════════════════════════════════════════════════════════════
void ultrasonicTask(void *pv){
    pinMode(US_L_TRIG, OUTPUT); pinMode(US_L_ECHO, INPUT);
    pinMode(US_R_TRIG, OUTPUT); pinMode(US_R_ECHO, INPUT);

    for(;;){
        vTaskDelay(pdMS_TO_TICKS(60));
        TestState current_state = test_state;
        float distL = read_ultrasonic_distance(US_L_TRIG, US_L_ECHO);
        float distR = read_ultrasonic_distance(US_R_TRIG, US_R_ECHO);
        bool obstacle_sensed = ((distL<0.25&&distL>0.02)||(distR<0.25&&distR>0.02));

        if(!obstacle_flag && current_state==STATE_MOVING && cmd_linear!=0 && obstacle_sensed){
            obstacle_flag = true;
            waiting_for_clear_delay = false;
            obstacle_detected_time = millis();
            long cL=encL.getCount(); long cR=encR.getCount();
            float completed_m = abs(((float)(cL-start_cnt_L)+(float)(cR-start_cnt_R))/(2.0*COUNTS_PER_M));
            saved_target_dist = target_dist_m - completed_m;
            saved_yaw_setpoint = yaw_setpoint;
            saved_nav_phase = nav_phase;
            set_motors(0,0);
            test_state = STATE_WAITING_FOR_CLEARANCE;
            publish_obstacle("STATIC");
        }
        else if(obstacle_flag && current_state==STATE_WAITING_FOR_CLEARANCE){
            if(!obstacle_sensed){
                if (!waiting_for_clear_delay) {
                    waiting_for_clear_delay = true;
                    path_clear_time = millis();
                    Serial.println("[INFO] Obstacle cleared, stabilizing before resume");
                } else if (millis() - path_clear_time >= POST_CLEAR_DELAY_MS) {
                    long cL=encL.getCount(); long cR=encR.getCount();
                    start_cnt_L=cL; start_cnt_R=cR;
                    target_dist_m=saved_target_dist;
                    yaw_setpoint=saved_yaw_setpoint;
                    nav_phase=saved_nav_phase;
                    yaw_hold=true; cmd_linear=(target_dist_m>=0)?0.15:-0.15;
                    test_state=STATE_MOVING;
                    obstacle_flag=false;
                    waiting_for_clear_delay=false;
                    publish_obstacle("CLEAR");
                    Serial.println("[INFO] Obstacle cleared, resuming path");
                }
            }
            else {
                if (waiting_for_clear_delay) {
                    waiting_for_clear_delay = false;
                    Serial.println("[WARN] False alarm. Obstacle detected again during stabilization window.");
                }

                if (millis() - obstacle_detected_time >= CLEARANCE_WAIT_MS) {
                    detourStep = 1;
                    test_state = STATE_DETOUR;
                    Serial.println("ALERT,DETOUR_START");
                }
            }
        }
        else {
            publish_obstacle("CLEAR");
        }
    }
}

// ════════════════════════════════════════════════════════════
// TASK 5 — LED
// ════════════════════════════════════════════════════════════
void ledTask(void *pv){
    pinMode(LED_G,OUTPUT); pinMode(LED_Y,OUTPUT); pinMode(LED_R,OUTPUT);
    bool toggle=false;
    for(;;){
        TestState s = test_state;
        if(s==STATE_MANUAL){ digitalWrite(LED_G,HIGH); digitalWrite(LED_Y,HIGH); digitalWrite(LED_R,LOW); }
        else if(s==STATE_IDLE){ digitalWrite(LED_G,HIGH); digitalWrite(LED_Y,LOW); digitalWrite(LED_R,LOW); }
        else if(s==STATE_MOVING){ digitalWrite(LED_G,LOW); digitalWrite(LED_Y,HIGH); digitalWrite(LED_R,LOW); }
        else if(s==STATE_WAITING_FOR_CLEARANCE){ digitalWrite(LED_G,LOW); digitalWrite(LED_Y,LOW); digitalWrite(LED_R,HIGH); }
        else if(s==STATE_DETOUR){ toggle=!toggle; digitalWrite(LED_G,LOW); digitalWrite(LED_Y,LOW); digitalWrite(LED_R,toggle?HIGH:LOW); vTaskDelay(pdMS_TO_TICKS(150)); continue; }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ════════════════════════════════════════════════════════════
// TASK 6 — Mechanism/Servo
// ════════════════════════════════════════════════════
void mechanismTask(void *pv){
    ledcAttach(SERVO_PIN, 50, 16);
    for(;;){
        if(mech_state==MECH_MOVING){
            int duty = map(target_servo_angle, 0, 180, 1638, 7864);
            ledcWrite(SERVO_PIN, duty);
            int angle_delta = abs(target_servo_angle - current_servo_angle);
            int step_delay = max(angle_delta*6, 150);
            vTaskDelay(pdMS_TO_TICKS(step_delay));
            current_servo_angle = target_servo_angle;
            mech_state = MECH_DONE;
        }
        else if(mech_state==MECH_DONE){
            vTaskDelay(pdMS_TO_TICKS(200));
            mech_state = MECH_IDLE;
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

// ════════════════════════════════════════════════════════════
// SETUP
// ════════════════════════════════════════════════════
void setup(){
    Serial.begin(115200);
    set_microros_transports();

    Wire.begin(21, 22);
    if(!mpu.begin()){ 
        gz_off = 0; // MPU not found — continue anyway
    } else {
        calibrateMPU();
    }

    pinMode(L_IN1,OUTPUT); pinMode(L_IN2,OUTPUT); pinMode(L_ENA,OUTPUT);
    pinMode(R_IN1,OUTPUT); pinMode(R_IN2,OUTPUT); pinMode(R_ENA,OUTPUT);

    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    encL.attachFullQuad(ENC_L_A, ENC_L_B);
    encR.attachFullQuad(ENC_R_B, ENC_R_A);

    yawPID.init(YAW_KP, YAW_KI, YAW_KD, YAW_I_MAX, YAW_OUT_MAX);
    last_imu_micros = micros();

    cmdQueue   = xQueueCreate(5, sizeof(String*));
    yawMutex   = xSemaphoreCreateMutex();
    telemMutex = xSemaphoreCreateMutex();
    odomMutex  = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(imuTask,       "IMU",    4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(controlTask,   "CTRL",   8192, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(microsRosTask, "UROS",   8192, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(ultrasonicTask,"USONIC", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(ledTask,       "LED",    2048, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(mechanismTask, "MECH",   2048, NULL, 1, NULL, 0);
}

void loop(){
    vTaskDelay(pdMS_TO_TICKS(1000));
}
