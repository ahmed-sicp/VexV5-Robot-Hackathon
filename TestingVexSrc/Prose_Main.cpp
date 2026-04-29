#include "main.h"
#include "pos.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motors.h"

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
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Tuning Opmode");

  pros::lcd::register_btn1_cb(on_center_button);
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

const std::uint8_t LEFT_PORT = 1;
const std::uint8_t RIGHT_PORT = 1;

void opcontrol() {
  // 1. Hardware Setup
  pros::IMU imu(5); // Port 5
  pros::Motor left_motor(LEFT_PORT, pros::MotorGears::green);
  pros::Motor right_motor(10, pros::MotorGears::green);

  imu.reset();
  while (imu.is_calibrating())
    pros::delay(10); // Arch/PROS safety: wait for IMU

  // Initial placeholder values - we are here to find these!
  BotValues tuning_vals(1.0f, 1.0f, 12.5f);

  std::cout << "--- IMU & ENCODER TUNER ---" << std::endl;
  std::cout << "Press X to Reset. Rotate 10 times." << std::endl;

  while (true) {
    float l_deg = left_motor.get_position();
    float r_deg = right_motor.get_position();
    float imu_deg =
        -imu.get_rotation(); // PROS IMU is clockwise positive by default

    // Calculate "Encoder Heading" based on current trackWidth guess
    // (RightDist - LeftDist) / trackWidth = Radians
    float l_dist = l_deg * tuning_vals.leftRatio;
    float r_dist = r_deg * tuning_vals.rightRatio;
    float enc_rad = (r_dist - l_dist) / tuning_vals.trackWidth;
    float enc_deg = enc_rad * (180.0f / M_PI);

    // Print Comparison to Terminal
    // If Enc Deg > IMU Deg, your trackWidth is too SMALL.
    // If Enc Deg < IMU Deg, your trackWidth is too LARGE.
    printf("IMU: %.2f | ENC: %.2f | DIFF: %.2f\n", imu_deg, enc_deg,
           imu_deg - enc_deg);

    // Screen Output
    pros::screen::print(pros::E_TEXT_MEDIUM, 1, "IMU Deg: %.2f", imu_deg);
    pros::screen::print(pros::E_TEXT_MEDIUM, 2, "ENC Deg: %.2f", enc_deg);

    if (pros::Controller(pros::E_CONTROLLER_MASTER).get_digital(DIGITAL_X)) {
      left_motor.tare_position();
      right_motor.tare_position();
      imu.reset();
    }

    pros::delay(50);
  }
}
