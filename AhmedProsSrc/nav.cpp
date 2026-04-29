#include "main.h"

// Define ports here so this file knows where the motors are
const std::uint8_t LEFT_PORT = 20; 
const std::uint8_t RIGHT_PORT = 10;

void autonomous() {
    // Hardware Setup
    pros::Motor left_motor(LEFT_PORT, pros::MotorGears::green);
    pros::Motor right_motor(RIGHT_PORT, pros::MotorGears::green);

    // Move forward at half speed (60 out of 127) [cite: 2]
    left_motor.move(-60); // Negative if motor is inverted
    right_motor.move(60);

    // Wait for 2 seconds
    pros::delay(2000);

	// 4. Stop briefly before turning (Prevents tipping)
    left_motor.move(0);
    right_motor.move(0);
    pros::delay(200);

    // 5. Turn Right 
    // To turn, motors must move in opposite directions
    left_motor.move(-60);  // Left moves "forward"
    right_motor.move(-60); // Right moves "backward"

    // 6. Wait for the turn to complete
    // Adjust this time (750ms) to get a perfect 90-degree turn
    pros::delay(750);

    // Stop the motors
    left_motor.move(0);
    right_motor.move(0);
}
