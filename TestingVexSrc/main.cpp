#include "main.h"
#include "pos.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include <cmath>

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
    // static bool pressed = false;
    // pressed = !pressed;
    // if (pressed) {
    // 	pros::lcd::set_text(2, "I was pressed!");
    // } else {
    // 	pros::lcd::clear_line(2);
    // }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    // pros::lcd::initialize();
    // pros::lcd::set_text(1, "Tuning Opmode");
    //
    // pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

/* Experimentally tuned BotValues:
 * Distance unit = tiles
 * leftRatio: -0.00043777
 * rightRatio: 0.00043170
 * trackwidth: 0.43322
 * */

const float leftRatio = -0.00043777;
const float rightRatio = 0.00043170;
const float trackWidth = 0.43322;

const std::uint8_t LEFT_PORT = 20;
const std::uint8_t RIGHT_PORT = 10;
const std::uint8_t IMU_PORT = 5;

void opcontrol() {
    // Hardware Setup
    pros::IMU imu(IMU_PORT); // Port 5
    pros::Motor left_motor(LEFT_PORT, pros::MotorGears::green);
    pros::Motor right_motor(RIGHT_PORT, pros::MotorGears::green);
    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    bool is_running_pid = false;
    float target_x = 0, target_y = 0, target_head = 0;
    bool mode_linear = true; // true for linear, false for angular

    // PID Gains - TUNING STARTS HERE
    // Start with just kP, keep kI and kD at 0 until kP is stable
    PID linPID(4.0f, 0.0f, 0.1f);
    PID angPID(2.0f, 0.0f, 0.05f);
    int valTuning = 0;

    imu.reset();
    while (imu.is_calibrating())
        pros::delay(10); // Arch/PROS safety: wait for IMU

    // Initial placeholder values - we are here to find these!
    BotValues tuning_vals(rightRatio, leftRatio, trackWidth);
    Pos position = Pos(0.f, 0.f, 0.f);

    float now, last;
    last = pros::millis();

    while (true) {
        now = pros::millis();
        // dt is in millis
        float dt = (now - last) * 0.001;
        last = now;

        // Convert RPM to deg/sec
        float l_vel = left_motor.get_actual_velocity() * 360 / 60;
        float r_vel = right_motor.get_actual_velocity() * 360 / 60;
        // This assumes the rotation is in the z-axis. Change this if you
        // reorient the IMU!
        // THIS IS IN RADIANS
        float imu_heading = imu.get_rotation() * (M_PI / 180.f);

        Vel curr_vel = Vel::from_encoders(tuning_vals, l_vel, r_vel);
        position.apply_with_imu(curr_vel, imu_heading, dt);

        pros::screen::print(pros::E_TEXT_MEDIUM, 1,
                            "x: %.2f | y: %.2f | head: %2.f", position.x,
                            position.y, position.heading * (180.f / M_PI));

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            valTuning += 1;
            valTuning %= 6;
        }
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            valTuning -= 1;
            valTuning %= 6;
        }
        float change = 0.f;
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            change -= 0.1f;
        }
        if (controller.get_digital_new_press(
                pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            change += 0.1f;
        }

        std::string name;
        float val;
        switch (valTuning) {
        case 0:
            name = "lin_kp";
            linPID.kp += change;
            val = linPID.kp;
            break;
        case 1:
            name = "lin_ki";
            linPID.ki += change;
            val = linPID.ki;
            break;
        case 2:
            name = "lin_kd";
            linPID.kd += change;
            val = linPID.kd;
            break;
        case 3:
            name = "ang_kp";
            angPID.kp += change;
            val = angPID.kp;
            break;
        case 4:
            name = "ang_ki";
            angPID.ki += change;
            val = angPID.ki;
            break;
        case 5:
            name = "ang_kd";
            angPID.kd += change;
            val = angPID.kd;
            break;
        }
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "val: %s = %2.f ", name,
                            val);
        pros::screen::print(
            pros::E_TEXT_MEDIUM, 3,
            "lin: kp %.2f | ki %2.f | kd %2.f, target = %2.f, err = %2.f",
            linPID.kp, linPID.ki, linPID.kd, target_x, linPID.error);
        pros::screen::print(
            pros::E_TEXT_MEDIUM, 4,
            "ang: kp %.2f | ki %2.f | kd %2.f, target = %2.f, err = %2.f",
            angPID.kp, angPID.ki, angPID.kd, target_head, angPID.error);

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
            is_running_pid = true;
            mode_linear = true;
            // target_x = position.x + 1.0f; // Target 1 meter forward
            target_x = 1.0f;
            target_y = position.y;
            linPID.reset();
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            if (target_x == 1.0f)
                target_x = 0.0f;
            else
                target_x = 1.0f;
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            is_running_pid = true;
            mode_linear = false;
            target_head =
                position.heading + (M_PI / 2.0f); // Target +90 degrees
            angPID.reset();
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B))
            is_running_pid = false;

        if (is_running_pid) {
            float l_out = 0, r_out = 0;

            if (mode_linear) {
                // Linear Tuning: Move to X target while maintaining heading
                float dist_error = target_x - position.x;
                float head_error = 0 - position.heading; // Keep facing forward

                float drive = linPID.calculate(target_x, position.x, dt);
                float steer = angPID.calculate(0, position.heading, dt);

                l_out = drive - steer;
                r_out = drive + steer;
            } else {
                // Angular Tuning: Spin in place
                float steer = angPID.calculate(target_head, position.heading,
                                               dt / 1000.f);
                l_out = -steer;
                r_out = steer;
            }

            // Apply motor voltages (clamped to +/- 127)
            left_motor.move(-std::clamp((int)(l_out * 127.f), -127, 127));
            right_motor.move(std::clamp((int)(r_out * 127.f), -127, 127));
        } else {
            // Tank drive for testing
            float throttle = (float)(controller.get_analog(
                                 pros::E_CONTROLLER_ANALOG_LEFT_Y)) /
                             127.f;
            float rotation = -(float)(controller.get_analog(
                                 pros::E_CONTROLLER_ANALOG_RIGHT_X)) /
                             127.f;
            float final_left = throttle - rotation;
            float final_right = throttle + rotation;
            left_motor.move(-(std::int32_t)(final_left * 127.f));
            right_motor.move((std::int32_t)(final_right * 127.f));
        }

        pros::delay(50);
    }
}
