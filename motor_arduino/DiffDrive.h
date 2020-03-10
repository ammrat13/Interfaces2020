/**********************************************************************************************
 * Arduino Differential Drive Velocity Control Library - Version 1.0.0
 * by Binit shah <bshah@ieee.org> binitshah.com
 *
 * Full duplex control over serial of a differential drive robot through velocity commands,
 * returning integrated SE2 odometry for motion updates in your localization filter.
 *
 * This library is licensed under the MIT License
 **********************************************************************************************/

#ifndef DiffDrive_h
#define DiffDrive_h

#include "Arduino.h"
#include <PID_v1.h>

#define MICROSECONDS_PER_SECOND (1e6)
#define MAX_STR_LEN (200)

template<int pwm_left, int dir1_left, int dir2_left, int enca_left, int encb_left,
         int pwm_right, int dir1_right, int dir2_right, int enca_right, int encb_right>
class DiffDrive {
    public:
        PID motor_velctrl_left;
        PID motor_velctrl_right;

        DiffDrive(double cpr, double R, double D, double L,
                  double kp_conservative, double ki_conservative, double kd_conservative,
                  double kp_aggressive, double ki_aggressive, double kd_aggressive);

        /*
         * Set the rate at which the PID controller runs.
         *
         * @param sample_time_microsec period between samples in microseconds
         */
        void setSampleTime(int sample_time_microsec);

        /*
         * Set the conservative and aggressive PID constants. The aggressive
         * constants are used where the robot can afford to gain speed quicker,
         * such as in wide open areas.
         *
         * @param kp_conservative controller p constant used in tighter areas
         * @param ki_conservative controller i constant used in tighter areas
         * @param kd_conservative controller d constant used in tighter areas
         * @param kp_aggressive   controller p constant used in open areas
         * @param ki_aggressive   controller i constant used in open areas
         * @param kd_aggressive   controller d constant used in open areas
         */
        void setPIDConstants(double kp_conservative, double ki_conservative, double kd_conservative,
                             double kp_aggressive, double ki_aggressive, double kd_aggressive);

        /*
         * Set the bounds of the PID's output. This is typically the resolution
         * of the microcontroller's PWM. 8-bit PWM leads to 256 steps. Param
         * min_pwm is negative max_pwm because that's how we encode wheel
         * direction.
         *
         * @param min_pwm usually equal to negative max_pwm
         * @param max_pwm usually equal to PWM's steps
         */
        void setOutputLimits(double min_pwm, double max_pwm);

        /*
         * Set the bounds of the PID's output. This is typically the resolution
         * of the microcontroller's PWM. 8-bit PWM leads to 256 steps. Param
         * min_pwm is negative max_pwm because that's how we encode wheel
         * direction.
         *
         * @param min_pwm usually equal to negative max_pwm
         * @param max_pwm usually equal to PWM's steps
         */
        void setRobotParams(double cpr, double R, double D, double L);

        /*
         * Run this method once every Arduino main loop.
         */
        void loop();

    private:
        double x, y, theta = 0.0f; // unit: x,y -> meters, theta -> rad/s
        double gearbox_cpr; // unit: counts per revolution of geared shaft
        double wheel_radius, wheel_base, single_intept_dist; // unit: meters

        static volatile int32_t encoder_count_left, encoder_count_right; // unit: counts
        static uint8_t enc_val_left, enc_val_right;
        static void encoder_isr_left(void);
        static void encoder_isr_right(void);

        double wtarget_left, wtarget_right, wcurr_left, wcurr_right = 0.0f; // unit: rad/s
        double cmd_left, cmd_right = 0; // unit: pwm

        double kp_cons, ki_cons, kd_cons, kp_agg, ki_agg, kd_agg;
        bool is_conservative = false;
        uint32_t sample_time = 10000; // unit: microseconds
        uint32_t prev_time_odom, prev_time_pid = 0; // unit: microseconds
        uint32_t prev_encoder_count_left, prev_encoder_count_right = 0; // unit: counts

        String input = "";
        const char start_marker = '<';
        const char end_marker = '>';
        void processAndReply();
};

// -------- Private Methods -------- //

const int8_t encoder_table[] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};

template<int pwm_left, int dir1_left, int dir2_left, int enca_left, int encb_left,
         int pwm_right, int dir1_right, int dir2_right, int enca_right, int encb_right>
volatile int32_t DiffDrive<pwm_left, dir1_left, dir2_left, enca_left, encb_left,
                           pwm_right, dir1_right, dir2_right, enca_right, encb_right>::encoder_count_left = 0;

template<int pwm_left, int dir1_left, int dir2_left, int enca_left, int encb_left,
         int pwm_right, int dir1_right, int dir2_right, int enca_right, int encb_right>
volatile int32_t DiffDrive<pwm_left, dir1_left, dir2_left, enca_left, encb_left,
                           pwm_right, dir1_right, dir2_right, enca_right, encb_right>::encoder_count_right = 0;

template<int pwm_left, int dir1_left, int dir2_left, int enca_left, int encb_left,
         int pwm_right, int dir1_right, int dir2_right, int enca_right, int encb_right>
uint8_t DiffDrive<pwm_left, dir1_left, dir2_left, enca_left, encb_left,
                           pwm_right, dir1_right, dir2_right, enca_right, encb_right>::enc_val_left = 0;

template<int pwm_left, int dir1_left, int dir2_left, int enca_left, int encb_left,
         int pwm_right, int dir1_right, int dir2_right, int enca_right, int encb_right>
uint8_t DiffDrive<pwm_left, dir1_left, dir2_left, enca_left, encb_left,
                           pwm_right, dir1_right, dir2_right, enca_right, encb_right>::enc_val_right = 0;

template<int pwm_left, int dir1_left, int dir2_left, int enca_left, int encb_left,
         int pwm_right, int dir1_right, int dir2_right, int enca_right, int encb_right>
void DiffDrive<pwm_left, dir1_left, dir2_left, enca_left, encb_left,
               pwm_right, dir1_right, dir2_right, enca_right, encb_right>::encoder_isr_left() {
    enc_val_left = (enc_val_left << 2) | (digitalRead(enca_left) << 1) | digitalRead(encb_left);

    encoder_count_left += encoder_table[enc_val_left & 0b1111];
}

template<int pwm_left, int dir1_left, int dir2_left, int enca_left, int encb_left,
         int pwm_right, int dir1_right, int dir2_right, int enca_right, int encb_right>
void DiffDrive<pwm_left, dir1_left, dir2_left, enca_left, encb_left,
               pwm_right, dir1_right, dir2_right, enca_right, encb_right>::encoder_isr_right() {
    enc_val_right = (enc_val_right << 2) | (digitalRead(enca_right) << 1) | digitalRead(encb_right);

    encoder_count_right += encoder_table[enc_val_right & 0b1111];
}

template<int pwm_left, int dir1_left, int dir2_left, int enca_left, int encb_left,
         int pwm_right, int dir1_right, int dir2_right, int enca_right, int encb_right>
void DiffDrive<pwm_left, dir1_left, dir2_left, enca_left, encb_left,
               pwm_right, dir1_right, dir2_right, enca_right, encb_right>::processAndReply() {
    char input_arr[MAX_STR_LEN];
    input.toCharArray(input_arr, MAX_STR_LEN);

    char* wcommand_left = strtok(input_arr, ",");
    wtarget_left = atof(wcommand_left);

    char* wcommand_right = strtok(NULL, ",");
    wtarget_right = atof(wcommand_right);

    char* cons_command = strtok(NULL, ",");
    is_conservative = atoi(cons_command);

    Serial.print("<");
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.print(",");
    Serial.print(theta);
    Serial.println(">");

    x = 0;
    y = 0;
    theta = 0;
    input = "";
}

// -------- Public Methods -------- //

template<int pwm_left, int dir1_left, int dir2_left, int enca_left, int encb_left,
         int pwm_right, int dir1_right, int dir2_right, int enca_right, int encb_right>
DiffDrive<pwm_left, dir1_left, dir2_left, enca_left, encb_left,
          pwm_right, dir1_right, dir2_right, enca_right, encb_right>::DiffDrive(
              double cpr, double R, double D, double L,
              double kp_conservative, double ki_conservative, double kd_conservative,
              double kp_aggressive, double ki_aggressive, double kd_aggressive) :
              gearbox_cpr(cpr), wheel_radius(R), wheel_base(D), single_intept_dist(L),
              kp_cons(kp_conservative), ki_cons(ki_conservative), kd_cons(kd_conservative),
              kp_agg(kp_aggressive), ki_agg(ki_aggressive), kd_agg(kd_aggressive),
              motor_velctrl_left(&wcurr_left, &cmd_left, &wtarget_left, kp_aggressive, ki_aggressive, kd_aggressive, P_ON_M, DIRECT),motor_velctrl_right(&wcurr_right, &cmd_right, &wtarget_right, kp_aggressive, ki_aggressive, kd_aggressive, P_ON_M, DIRECT) {
    Serial.begin(115200);
    input.reserve(MAX_STR_LEN);

    pinMode(pwm_left, OUTPUT);
    pinMode(dir1_left, OUTPUT);
    pinMode(dir2_left, OUTPUT);
    pinMode(enca_left, INPUT);
    pinMode(encb_left, INPUT);
    pinMode(pwm_right, OUTPUT);
    pinMode(dir1_right, OUTPUT);
    pinMode(dir2_right, OUTPUT);
    pinMode(enca_right, INPUT);
    pinMode(encb_right, INPUT);

    attachInterrupt(enca_left, encoder_isr_left, CHANGE);
    attachInterrupt(encb_left, encoder_isr_left, CHANGE);
    attachInterrupt(enca_right, encoder_isr_right, CHANGE);
    attachInterrupt(encb_right, encoder_isr_right, CHANGE);

    motor_velctrl_left.SetSampleTime(sample_time / 1000);
    motor_velctrl_right.SetSampleTime(sample_time / 1000);

    motor_velctrl_left.SetOutputLimits(-255, 255);
    motor_velctrl_right.SetOutputLimits(-255, 255);

    motor_velctrl_left.SetMode(AUTOMATIC);
    motor_velctrl_right.SetMode(AUTOMATIC);

    prev_time_pid = micros();
    prev_time_odom = micros();

    Serial.println("motor");
}

template<int pwm_left, int dir1_left, int dir2_left, int enca_left, int encb_left,
         int pwm_right, int dir1_right, int dir2_right, int enca_right, int encb_right>
void DiffDrive<pwm_left, dir1_left, dir2_left, enca_left, encb_left,
          pwm_right, dir1_right, dir2_right, enca_right, encb_right>::setSampleTime(int sample_time_microsec) {
    sample_time = sample_time_microsec;
    motor_velctrl_left.SetSampleTime(sample_time / 1000);
    motor_velctrl_right.SetSampleTime(sample_time / 1000);
}

template<int pwm_left, int dir1_left, int dir2_left, int enca_left, int encb_left,
         int pwm_right, int dir1_right, int dir2_right, int enca_right, int encb_right>
void DiffDrive<pwm_left, dir1_left, dir2_left, enca_left, encb_left,
          pwm_right, dir1_right, dir2_right, enca_right, encb_right>::setPIDConstants(
              double kp_conservative, double ki_conservative, double kd_conservative,
              double kp_aggressive, double ki_aggressive, double kd_aggressive) {
    kp_cons = kp_conservative;
    ki_cons = ki_conservative;
    kd_cons = kd_conservative;
    kp_agg = kp_aggressive;
    ki_agg = ki_aggressive;
    kd_agg = kd_aggressive;
    if (is_conservative) {
        motor_velctrl_left.SetTunings(kp_cons, ki_cons, kd_cons, P_ON_M);
        motor_velctrl_right.SetTunings(kp_cons, ki_cons, kd_cons, P_ON_M);
    } else {
        motor_velctrl_left.SetTunings(kp_agg, ki_agg, kd_agg, P_ON_M);
        motor_velctrl_right.SetTunings(kp_agg, ki_agg, kd_agg, P_ON_M);
    }
}

template<int pwm_left, int dir1_left, int dir2_left, int enca_left, int encb_left,
         int pwm_right, int dir1_right, int dir2_right, int enca_right, int encb_right>
void DiffDrive<pwm_left, dir1_left, dir2_left, enca_left, encb_left,
          pwm_right, dir1_right, dir2_right, enca_right, encb_right>::setOutputLimits(double min_pwm, double max_pwm) {
    motor_velctrl_left.SetOutputLimits(min_pwm, max_pwm);
    motor_velctrl_right.SetOutputLimits(min_pwm, max_pwm);
}

template<int pwm_left, int dir1_left, int dir2_left, int enca_left, int encb_left,
         int pwm_right, int dir1_right, int dir2_right, int enca_right, int encb_right>
void DiffDrive<pwm_left, dir1_left, dir2_left, enca_left, encb_left,
          pwm_right, dir1_right, dir2_right, enca_right, encb_right>::setRobotParams(
              double cpr, double R, double D, double L) {
    gearbox_cpr = cpr;
    wheel_radius = R;
    wheel_base = D;
    single_intept_dist = L;
}

template<int pwm_left, int dir1_left, int dir2_left, int enca_left, int encb_left,
         int pwm_right, int dir1_right, int dir2_right, int enca_right, int encb_right>
void DiffDrive<pwm_left, dir1_left, dir2_left, enca_left, encb_left,
          pwm_right, dir1_right, dir2_right, enca_right, encb_right>::loop() {
    noInterrupts();
    double dcount_left = prev_encoder_count_left - encoder_count_left;
    prev_encoder_count_left = encoder_count_left;
    double dcount_right = prev_encoder_count_right - encoder_count_right;
    prev_encoder_count_right = encoder_count_right;
    interrupts();
    uint32_t curr_time = micros();
    double dt = curr_time - prev_time_odom;
    if (dt > 0.0) {
        wcurr_left = ((dcount_left / dt) / gearbox_cpr) * MICROSECONDS_PER_SECOND * TWO_PI;
        wcurr_right = ((dcount_right / dt) / gearbox_cpr) * MICROSECONDS_PER_SECOND * TWO_PI;

        double xdot = wcurr_left * ((wheel_radius / 2 * cos(theta)) + (wheel_radius * (single_intept_dist / wheel_base) * sin(theta))) +
                      wcurr_right * ((wheel_radius / 2 * cos(theta)) - (wheel_radius * (single_intept_dist / wheel_base) * sin(theta)));
        double ydot = wcurr_left * ((wheel_radius / 2 * sin(theta)) - (wheel_radius * (single_intept_dist / wheel_base) * cos(theta))) +
                      wcurr_right * ((wheel_radius / 2 * sin(theta)) + (wheel_radius * (single_intept_dist / wheel_base) * cos(theta)));
        double thetadot = wcurr_right * (wheel_radius / wheel_base) - wcurr_left * (wheel_radius / wheel_base);

        x += xdot * dt;
        y += ydot * dt;
        theta += thetadot * dt;
        theta = fmod(theta, TWO_PI);
    }
    prev_time_odom = curr_time;
    
    if (curr_time - prev_time_pid > sample_time) {
        motor_velctrl_left.Compute();
        motor_velctrl_right.Compute();

        if (is_conservative) {
            motor_velctrl_left.SetTunings(kp_cons, ki_cons, kd_cons, P_ON_M);
            motor_velctrl_right.SetTunings(kp_cons, ki_cons, kd_cons, P_ON_M);
        } else {
            motor_velctrl_left.SetTunings(kp_agg, ki_agg, kd_agg, P_ON_M);
            motor_velctrl_right.SetTunings(kp_agg, ki_agg, kd_agg, P_ON_M);
        }

        if (cmd_left > 0) {
            digitalWrite(dir1_left, HIGH);
            digitalWrite(dir2_left, LOW);
            analogWrite(pwm_left, abs(cmd_left));
        } else if (cmd_left == 0) {
            digitalWrite(dir1_left, LOW);
            digitalWrite(dir2_left, LOW);
            analogWrite(pwm_left, 0);
        } else if (cmd_left < 0) {
            digitalWrite(dir1_left, LOW);
            digitalWrite(dir2_left, HIGH);
            analogWrite(pwm_left, abs(cmd_left));
        }

        if (cmd_right > 0) {
            digitalWrite(dir1_right, HIGH);
            digitalWrite(dir2_right, LOW);
            analogWrite(pwm_right, abs(cmd_right));
        } else if (cmd_right == 0) {
            digitalWrite(dir1_right, LOW);
            digitalWrite(dir2_right, LOW);
            analogWrite(pwm_right, 0);
        } else if (cmd_right < 0) {
            digitalWrite(dir1_right, LOW);
            digitalWrite(dir2_right, HIGH);
            analogWrite(pwm_right, abs(cmd_right));
        }

        prev_time_pid = curr_time;
    }

    while (Serial.available() > 0) {
        char input_char = (char) Serial.read();

        if (input_char == end_marker) {
            processAndReply();
        } else if (input_char == start_marker) {
            input = "";
        } else {
            input += input_char;
        }
    }
}

#endif
