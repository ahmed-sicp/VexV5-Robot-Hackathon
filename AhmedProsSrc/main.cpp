#include "main.h"

// Hardware Ports [cite: 5, 6]
const std::uint8_t LEFT_PORT = 20;
const std::uint8_t RIGHT_PORT = 10;
const std::uint8_t IMU_PORT = 5;
const std::uint8_t OPTICAL_PORT = 6;

// FORWARD DECLARATION 
// This tells main.cpp that this function exists in the other .cpp files
void autonomous();
void update_screen_with_color(pros::Optical &sensor);

void initialize() {
    // Calibrate the Optical Sensor LED [cite: 9]
    pros::Optical optical_sensor(OPTICAL_PORT);
    optical_sensor.set_led_pwm(100);
}

void opcontrol() {
    // 1. Run the move forward logic once
    autonomous(); 

    // 2. Then enter your normal loop for the screen/sensors
    pros::Optical optical_sensor(OPTICAL_PORT);

    while (true) {
        // Call the function from color.cpp [cite: 10]
        update_screen_with_color(optical_sensor);
        pros::delay(20);
    }
}

// NOTE: autonomous() is intentionally removed from here 
// because it is now defined in nav.cpp. 
