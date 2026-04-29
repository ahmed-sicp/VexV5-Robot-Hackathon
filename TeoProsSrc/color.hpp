#include "main.h"
#include "pros/colors.hpp"
#include "pros/optical.hpp"
#include "pros/screen.hpp"

void do_mineral_detection(pros::Optical &sensor) {
    // Check yellow and green
    // Yellow
    if (sensor.get_hue() > 40 && sensor.get_hue() < 70 &&
        sensor.get_saturation() > 0.5 && sensor.get_brightness() > 0.1) {
        printf("Detecting YELLOW!\t");
        pros::screen::set_eraser(pros::Color::yellow);
        pros::screen::erase();
    }
    // Green
    else if (sensor.get_hue() > 71 && sensor.get_hue() < 150 &&
             sensor.get_saturation() > 0.4 && sensor.get_brightness() > 0.1) {
        printf("Detecting GREEN!\t");
        pros::screen::set_eraser(pros::Color::green);
        pros::screen::erase();
    } else {
        printf("Detecting nu'in!\t");
        pros::screen::set_eraser(pros::Color::black);
        pros::screen::erase();
    }
}
