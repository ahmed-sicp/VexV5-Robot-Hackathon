// pos.hpp

#pragma once
#include "pros/screen.h"
#include <cmath>
#include <vector>

/**
 * Constants for the specific robot physical build.
 */
class BotValues {
  public:
    float rightRatio; // Distance per encoder degree
    float leftRatio;
    float trackWidth; // Distance between wheels
    BotValues(float rightRatio, float leftRatio, float trackWidth)
        : rightRatio(rightRatio), leftRatio(leftRatio), trackWidth(trackWidth) {
    }
};

/**
 * Represents the velocity (change in state).
 * API-agnostic: Units are distance/sec and radians/sec.
 */
class Vel {
  public:
    float linear, angular;

    Vel(float linear, float angular) : linear(linear), angular(angular) {}

    // Factory method to calculate velocity from raw hardware readings
    static Vel from_encoders(const BotValues &values, float leftDegPerSec,
                             float rightDegPerSec) {
        float left = leftDegPerSec * values.leftRatio;
        float right = rightDegPerSec * values.rightRatio;
        float lin = (left + right) * 0.5f;
        float ang = (right - left) / values.trackWidth;
        return Vel(lin, ang);
    }
};

class Pos {
  public:
    float x, y, heading; // Heading in radians

    Pos(float x, float y, float heading) : x(x), y(y), heading(heading) {}

    /**
     * Updates the position using an arc-based approximation.
     * newHeading: the raw heading from the IMU in radians
     */
    void apply_with_imu(Vel v, float newHeading, float dt) {
        float dLin = v.linear * dt;

        // We use the IMU's actual change rather than calculating it from motors
        float avgHeading = (heading + newHeading) * 0.5f;

        x += dLin * std::cos(avgHeading);
        y += dLin * std::sin(avgHeading);
        heading = newHeading;

        // DO NOT DO THIS, because it leads to the ever present "average of
        // 179deg and -179deg is 0deg, when it's actually +-180deg" Standard
        // normalization heading = std::atan2(std::sin(heading),
        // std::cos(heading));
    }
};

class PID {
  public:
    float kp, ki, kd;
    float error, lastError, integral, derivative;

    PID(float kp, float ki, float kd)
        : kp(kp), ki(ki), kd(kd), error(0), lastError(0), integral(0),
          derivative(0) {}

    float calculate(float target, float current, float dt) {
        if (dt <= 0)
            return 0;
        error = target - current;
        integral += error * dt;
        derivative = (error - lastError) / dt;
        if (derivative > 0.f)
            derivative = 0.f;
        lastError = error;

        // Simple integral anti-windup: cap the integral contribution
        if (std::abs(error) < 0.1)
            integral = 0;

        return (error * kp) + (integral * ki) + (derivative * kd);
    }

    void reset() { error = lastError = integral = derivative = 0; }
};

#include <algorithm>
#include <cmath>
#include <vector>

struct PathPoint {
    float x, y;
    bool reverse; // Drive backward to this point?
};
#include "main.h"
class PathFollower {
  public:
    float lookaheadDist = 0.5f; // TUNABLE: How far ahead the robot looks
    float trackWidth;
    size_t current_point;
    bool loop;
    PID &distPID; // TUNABLE: Controls speed based on distance to final target

    PathFollower(float tw, PID &dPid)
        : trackWidth(tw), distPID(dPid), current_point(0), loop(false) {}

    struct Output {
        float left, right;
        bool done;
    };

    Output update(const Pos &current, const std::vector<PathPoint> &path,
                  float dt) {

        // 1. SAFE POINT ADVANCEMENT
        while (true) {
            if (current_point >= path.size()) {
                if (loop && !path.empty()) {
                    printf("Looping...\n");
                    current_point = 0;
                } else {
                    printf("Done with path\n");
                    return {0, 0, true};
                }
            }

            PathPoint target = path[current_point];
            float dx = target.x - current.x;
            float dy = target.y - current.y;
            float dist_to_end = std::sqrt(dx * dx + dy * dy);

            float effective_heading = current.heading;
            if (target.reverse) {
                effective_heading += M_PI;
            }

            float localY = dx * std::cos(effective_heading) +
                           dy * std::sin(effective_heading);
            if (target.reverse) {
                localY *= -1.f;
            }

            // Trigger if we are close, OR if we grazed past it
            if (dist_to_end < 0.2f ||
                (localY < 0.0f && dist_to_end < lookaheadDist)) {
                printf("Reached point %zu...\n", current_point);
                current_point++;

                distPID.reset(); // CRITICAL: Prevents the derivative kick!
            } else {
                break;
            }
        }

        // Safety return in case the loop above just finished the path
        if (current_point >= path.size()) {
            return {0, 0, true};
        }

        // 2. PURE PURSUIT MATH
        PathPoint target = path[current_point];
        pros::screen::print(pros::E_TEXT_MEDIUM, 2,
                            "Pathing to [%zu] %.2f, %.2f", current_point,
                            target.x, target.y);
        pros::screen::print(pros::E_TEXT_MEDIUM, 3,
                            "x: %.2f | y: %.2f | head: %.2f", current.x,
                            current.y, current.heading * (180.f / M_PI));

        float dx = target.x - current.x;
        float dy = target.y - current.y;
        float dist_to_end = std::sqrt(dx * dx + dy * dy);

        float effective_heading = current.heading;
        if (target.reverse) {
            effective_heading += M_PI;
        }

        float localX =
            dx * std::sin(effective_heading) - dy * std::cos(effective_heading);

        float L2 = lookaheadDist * lookaheadDist;
        if (L2 < 0.01f)
            L2 = 0.01f;
        float curvature = (2.0f * localX) / L2;

        // 3. CRUISE CONTROL PID
        float base_speed = 0.0f;

        if (!loop && current_point == path.size() - 1) {
            // We are on the final point. Calculate speed normally to decelerate
            // to a stop.
            base_speed = distPID.calculate(dist_to_end, 0.0f, dt) * 1.2;
        } else {
            // Intermediate point! Feed the PID a constant 1-tile error to
            // maintain a steady cruising speed.
            base_speed = distPID.calculate(1.0f, 0.0f, dt);

            // Automatically slow down if the corner is extremely sharp
            if (std::abs(curvature) > 1.5f) {
                base_speed *= 0.6f;
            }
        }

        // 4. REVERSE AND DIFFERENTIAL KINEMATICS
        if (target.reverse) {
            base_speed = -base_speed;
            curvature = -curvature;
        }

        // base_speed *= 0.1;
        curvature *= 0.8;
        // You can remove your manual `base_speed *= 0.1;` test line once this
        // is implemented!
        float left_out = base_speed * (2.0f + curvature * trackWidth) / 2.0f;
        float right_out = base_speed * (2.0f - curvature * trackWidth) / 2.0f;

        // Invert the left motor for your specific hardware setup
        return {-left_out, right_out, false};
    }
};
