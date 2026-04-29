#include "main.h"
#include "color.hpp"
#include "pos.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motors.h"
#include "pros/optical.h"
#include "pros/optical.hpp"
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
void autonomous() {
    while (true) {
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Hello!");
        pros::delay(500);
    }
}

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

// Unit of measure is two tiles now
const float leftRatio = -0.00053777;
const float rightRatio = 0.00053170;
const float trackWidth = 0.43322;

const std::uint8_t LEFT_PORT = 20;
const std::uint8_t RIGHT_PORT = 10;
const std::uint8_t IMU_PORT = 5;
const std::uint8_t OPTICAL_PORT = 6;

const std::vector<PathPoint> final_path = {
    {2, -0, false},     {2.5, -0.5, false}, {2, -6, false},
    {1.5, -4.5, false}, {1.5, -0.5, false}, {1.5, -4.5, true},
    {2, -6, true},      {1.5, -6, false},   {1, -4.5, false},
    {1, -0.5, false},   {1, -4.5, true},    {1.5, -6, true},
    {1, -6, false},     {0.5, -4.5, false}, {0.5, -0.5, false},
    {0.5, -4.5, true},  {1, -6, true},      {0.5, -6, false},
    {0, -4.5, false},   {0, -0.5, false},   {0, -4.5, true},
    {0.5, -6, true},    {0, -4.5, false},   {2.5, -0.5, false},
    {2, -0, false},     {1, -0, false}};
// const std::vector<PathPoint> final_path = {
//     {2, 0, false},
//     {2.5, -0.5, false},
//     {2, -5, false},
// };

void do_tank_drive(pros::Motor &left_motor, pros::Motor &right_motor,
                   pros::Controller &controller) {
    float throttle =
        (float)(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) /
        127.f;
    float rotation =
        -(float)(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) /
        127.f;
    float final_left = throttle - rotation;
    float final_right = throttle + rotation;
    left_motor.move(-(std::int32_t)(final_left * 127.f));
    right_motor.move((std::int32_t)(final_right * 127.f));
}

void update_position(pros::IMU &imu, pros::Motor &left_motor,
                     pros::Motor &right_motor, BotValues &tuning_vals,
                     Pos &position, float &last_l_deg, float &last_r_deg,
                     float &dt) {
    float curr_l_deg = left_motor.get_position();
    float curr_r_deg = right_motor.get_position();
    // THIS IS IN DEGREES (must convert), AND NOW CCW POSITIVE
    auto raw_imu = -imu.get_rotation();
    // --- SANITIZE HARDWARE ERRORS ---
    // Prevent PROS_ERR_F (Infinity) from poisoning the math if a wire wiggles
    // loose
    if (curr_l_deg == PROS_ERR_F)
        curr_l_deg = last_l_deg;
    if (curr_r_deg == PROS_ERR_F)
        curr_r_deg = last_r_deg;
    if (raw_imu == PROS_ERR_F)
        raw_imu = position.heading * (180.f / M_PI); // Use last known heading

    // 2. Calculate DELTA degrees (This is much more accurate than Velocity)
    float d_left_deg = curr_l_deg - last_l_deg;
    float d_right_deg = curr_r_deg - last_r_deg;
    // Update the "last" values for the next iteration
    last_l_deg = curr_l_deg;
    last_r_deg = curr_r_deg;

    // 3. Convert Delta Degrees to Velocity (Units: Tiles per Second)
    // Since Vel::from_encoders expects deg/sec:
    float l_vel = d_left_deg / dt;
    float r_vel = d_right_deg / dt;

    // This assumes the rotation is in the z-axis. Change this if you
    // reorient the IMU!
    // THIS IS IN RADIANS
    float imu_heading = raw_imu * (M_PI / 180.f);

    Vel curr_vel = Vel::from_encoders(tuning_vals, l_vel, r_vel);
    position.apply_with_imu(curr_vel, imu_heading, dt);
}

void opcontrol() {
    // Hardware Setup
    pros::IMU imu(IMU_PORT);
    pros::Motor left_motor(LEFT_PORT, pros::MotorGears::green);
    pros::Motor right_motor(RIGHT_PORT, pros::MotorGears::green);
    pros::Optical optical_sensor(OPTICAL_PORT);
    optical_sensor.set_led_pwm(100);

    left_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    right_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    BotValues tuning_vals(rightRatio, leftRatio, trackWidth);

    // Position Tracking variables (Assumes you have the delta-position loop set
    // up)
    Pos position(0.f, 0.f, 0.f);

    // --- PATH FOLLOWER SETUP ---
    // Start with a small P, 0 I, and some D to prevent overshoot.
    static PID distancePID(40.00f, 0.0f, 2.5f);
    static PathFollower follower(trackWidth, distancePID);

    // Ping-Pong Paths
    std::vector<PathPoint> forward_path = {
        {1.5f, 0.0f, false}}; // 1.5 tiles forward
    std::vector<PathPoint> reverse_path = {
        {0.0f, 0.0f, true}}; // 0 tiles (return to start), reverse

    std::vector<PathPoint> rectangle = {{1.f, 0.f, false},
                                        {1.f, 1.f, false},
                                        {0.f, 1.f, false},
                                        {0.f, 0.f, false}};

    std::vector<PathPoint> circle = {{0.5f, 0.f, false},
                                     {1.f, .5f, false},
                                     {0.5f, 1.f, false},
                                     {0.f, 0.5f, false}};

    std::vector<PathPoint> arc_and_back = {{0.5f, 0.f, false},
                                           {1.f, .5f, false},
                                           {0.5f, 1.f, false},
                                           {1.f, .5f, true},
                                           {0.5f, 0.f, true}};

    bool going_forward = true;
    bool autonmode = true;

    // --- LIVE TUNING UI SETUP ---
    int selected_idx = 0;
    const int NUM_PARAMS = 4;
    float *params[NUM_PARAMS] = {&distancePID.kp, &distancePID.kd,
                                 &distancePID.ki, &follower.lookaheadDist};
    const char *names[NUM_PARAMS] = {"Dist kP", "Dist kD", "Dist kI",
                                     "Lookahead"};
    float steps[NUM_PARAMS] = {5.0f, 1.0f, 0.1f, 0.05f};

    imu.reset();
    while (imu.is_calibrating())
        pros::delay(10); // PROS safety: wait for IMU

    printf("Done Calibrating\n");

    // Store the encoder values from the PREVIOUS loop
    float last_l_deg = left_motor.get_position();
    float last_r_deg = right_motor.get_position();
    float last = pros::millis();

    bool enableOp = true;
    while (enableOp) {

        do_mineral_detection(optical_sensor);
        printf("%.2f | %.2f | %.2f \n", optical_sensor.get_hue(),
               optical_sensor.get_saturation(),
               optical_sensor.get_brightness());
        pros::screen::print(
            pros::E_TEXT_MEDIUM, 1, " h %.2f | s %.2f | v %.2f \n",
            optical_sensor.get_hue(), optical_sensor.get_saturation(),
            optical_sensor.get_brightness());

        float now = pros::millis();
        float dt = (now - last) * 0.001f;
        // Avoid dividing by zero on the very first loop
        if (dt <= 0.0f) {
            pros::delay(10);
            continue;
        }
        last = now;

        update_position(imu, left_motor, right_motor, tuning_vals, position,
                        last_l_deg, last_r_deg, dt);

        // Toggle Tuning Mode with 'A'
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            autonmode = !autonmode;
            distancePID.reset();
            left_motor.move(0);
            right_motor.move(0);
        }

        if (autonmode) {
            // --- UI LOGIC ---
            if (controller.get_digital_new_press(
                    pros::E_CONTROLLER_DIGITAL_RIGHT))
                selected_idx = (selected_idx + 1) % NUM_PARAMS;
            if (controller.get_digital_new_press(
                    pros::E_CONTROLLER_DIGITAL_LEFT))
                selected_idx = (selected_idx - 1 + NUM_PARAMS) % NUM_PARAMS;
            if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
                *params[selected_idx] += steps[selected_idx];
            if (controller.get_digital_new_press(
                    pros::E_CONTROLLER_DIGITAL_DOWN)) {
                *params[selected_idx] -= steps[selected_idx];
                if (*params[selected_idx] < 0)
                    *params[selected_idx] = 0; // Prevent negatives
            }

            // --- PATH EXECUTION ---
            // auto current_path = going_forward ? forward_path : reverse_path;
            auto current_path = final_path;
            // follower.loop = true;
            auto out = follower.update(position, current_path, dt);

            if (out.done) {
                enableOp = false;
                // We arrived! Swap directions and reset PID for the next trip
                going_forward = !going_forward;
                distancePID.reset();
                left_motor.move(0);
                right_motor.move(0);
                pros::delay(500); // Wait half a second before shooting back
            } else {
                // Apply clamped output to motors
                left_motor.move(std::clamp((int)out.left, -127, 127));
                right_motor.move(std::clamp((int)out.right, -127, 127));
            }

            // --- TELEMETRY ---
            // pros::screen::print(pros::E_TEXT_LARGE, 1, "> %s <",
            //                     names[selected_idx]);
            // pros::screen::print(pros::E_TEXT_LARGE, 3, "Val: %.3f",
            //                     *params[selected_idx]);

            // Print to terminal so you don't have to stare at the brain
            printf("Tuning [%s]: %.3f | X: %.2f | Y: %.2f | H: %.2f\n",
                   names[selected_idx], *params[selected_idx], position.x,
                   position.y, position.heading * (180.f / M_PI));

        } else {
            // pros::screen::print(pros::E_TEXT_LARGE, 2, "MANUAL DRIVE ");
            // pros::screen::print(pros::E_TEXT_MEDIUM, 3, "Press A to Tune ");
            // pros::screen::print(pros::E_TEXT_MEDIUM, 4,
            //                     "x: %.2f | y: %.2f | head: %.2f", position.x,
            //                     position.y, position.heading * (180.f /
            //                     M_PI));
            do_tank_drive(left_motor, right_motor, controller);
        }

        pros::delay(10);
    }
}
