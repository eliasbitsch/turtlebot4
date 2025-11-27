#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <iomanip>
#include <algorithm> // For std::min/max


// =========================================================================
// CLASS 1: MockLaserScanner (For development without physical hardware)
// =========================================================================

/**
 * @brief Simulates essential laser scanner readings needed for wall following.
 * * In a real robot, these values (front_dist, side_dist) would be derived
 * from processing the full laser scan array (e.g., using RANSAC/PCA).
 */
class MockLaserScanner {
public:
    // Define scenarios for testing
    enum Scenario {
        PERFECT_PARALLEL,
        CONVERGING_TO_WALL,
        DIVERGING_FROM_WALL,
        FRONT_OBSTACLE
    };

    /**
     * @brief Generates simulated sensor data based on a scenario.
     * @param scenario The current testing scenario.
     * @param current_side_dist The current distance to the side wall.
     * @param front_dist Output parameter for the simulated front distance.
     * @param side_dist Output parameter for the simulated side distance.
     */
    void get_scan_data(Scenario scenario, double current_side_dist, double& front_dist, double& side_dist) {

        // --- 1. Base Readings ---
        front_dist = 2.0; // Default: clear in front
        side_dist = current_side_dist;

        // --- 2. Apply Scenario Logic ---
        switch (scenario) {
            case PERFECT_PARALLEL:
                // Robot is perfect parallel to the wall (side distances are equal)
                break;

            case CONVERGING_TO_WALL:
                // Robot is angling toward the wall. Angle difference will make
                // the distance slightly closer at the front-side (used for angle error)
                // We'll simulate that the side reading is slightly less than current_side_dist
                side_dist = current_side_dist * 0.98;
                break;

            case DIVERGING_FROM_WALL:
                // Robot is angling away from the wall.
                side_dist = current_side_dist * 1.02;
                break;

            case FRONT_OBSTACLE:
                front_dist = 0.2; // Simulates an object blocking the path
                side_dist = current_side_dist;
                break;
        }

        // Ensure distances are not negative
        front_dist = std::max(0.0, front_dist);
        side_dist = std::max(0.0, side_dist);
    }
};


// =========================================================================
// CLASS 2: WallFollowerController (The core control logic)
// =========================================================================

/**
 * @brief Implements Proportional (P) control for wall following.
 * * This class calculates the required angular velocity (w) based on the distance
 * error from the side wall.
 */
class WallFollowerController {
public:
    WallFollowerController(double kp_dist = 5.0, double kp_angle = 10.0, double desired_dist = 0.5)
        : KP_DISTANCE(kp_dist), KP_ANGLE(kp_angle), DESIRED_DISTANCE_(desired_dist) {}

    /**
     * @brief Computes the linear and angular velocities required for wall following.
     * @param front_dist The distance reading from the front of the robot.
     * @param side_dist The perpendicular distance reading from the side of the wall.
     * @param angle_to_wall_error The calculated angular error relative to the wall (from RANSAC/PCA).
     * @param out_linear Output parameter for the resulting linear speed (v).
     * @param out_angular Output parameter for the resulting angular speed (w).
     */
    void computeControl(double front_dist, double side_dist, double &out_linear, double &out_angular) {

        // --- 1. Constants and Initialization ---
        const double SAFE_LINEAR_VEL = 0.15; // Set constant forward speed

        // Safety Thresholds
        const double EMERGENCY_STOP_DIST = 0.3; // Stop if closer than 30 cm
        const double MAX_ANGULAR = 2.84;        // Max rotational speed
        const double MAX_LINEAR = 0.22;         // Max forward speed

        out_linear = SAFE_LINEAR_VEL;
        out_angular = 0.0;

        // --- 2. Calculate Primary Distance Error (e_d) ---
        // Positive error means the robot is too far from the wall -> turn INTO the wall (positive angular)
        // Negative error means the robot is too close to the wall -> turn AWAY from the wall (negative angular)
        double distance_error = DESIRED_DISTANCE_ - side_dist;

        // --- 3. Calculate Angular Velocity (w) ---
        // We use a P-controller based only on the distance error for simplicity.
        // In a real system, a second term (KP_ANGLE * angle_to_wall_error) is added.

        double w_correction = KP_DISTANCE * distance_error;

        out_angular = w_correction;

        // --- 4. Emergency Stop / Avoidance Logic ---
        if (front_dist < EMERGENCY_STOP_DIST) {
            // Priority 1: Stop and turn away from the obstacle
            out_linear = 0.0;
            // Turn sharply in the direction *away* from the wall (assuming wall is on the right/positive turn)
            out_angular = MAX_ANGULAR;
        }

        // --- 5. Apply Final Limits ---
        out_linear = std::min(MAX_LINEAR, std::max(-MAX_LINEAR, out_linear));
        out_angular = std::min(MAX_ANGULAR, std::max(-MAX_ANGULAR, out_angular));
    }

private:
    const double KP_DISTANCE;
    const double KP_ANGLE;
    const double DESIRED_DISTANCE_;
};


// =========================================================================
// MAIN SIMULATION LOOP
// =========================================================================

int main() {
    // --- 1. Setup ---
    WallFollowerController controller;
    MockLaserScanner scanner;

    // Initial State (Simulated environment)
    double robot_side_dist = 0.8; // Robot starts 0.8m from the wall
    double robot_front_dist = 2.0;

    const double dt = 0.1; // simulation timestep (s)
    const int max_steps = 200;

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Starting Wall Follower Simulation (Target: 0.5m)\n";
    std::cout << "------------------------------------------------------\n";

    // --- 2. Simulation Loop ---
    for (int step = 0; step < max_steps; ++step) {
        double v = 0.0, w = 0.0;

        // --- SCENARIO MANAGEMENT ---
        MockLaserScanner::Scenario scenario;
        if (step < 50) {
            // Initial phase: Robot is far away (0.8m) and must converge.
            scenario = MockLaserScanner::PERFECT_PARALLEL;
        } else if (step > 150) {
            // Later phase: Simulate a sudden object in front
            scenario = MockLaserScanner::FRONT_OBSTACLE;
        } else {
            scenario = MockLaserScanner::PERFECT_PARALLEL;
        }

        // Get mocked sensor data
        scanner.get_scan_data(scenario, robot_side_dist, robot_front_dist, robot_side_dist);

        // Compute control commands based on mocked sensor data
        controller.computeControl(robot_front_dist, robot_side_dist, v, w);

        // --- Simulate Robot Movement ---
        // We simulate the change in distance over time based on angular velocity (w)

        // Simple integration of angular velocity (w)
        double side_change = w * 0.01 * dt; // Simple proxy for how fast the distance changes
        robot_side_dist += side_change;

        // Clamp distance to reasonable limits
        robot_side_dist = std::min(2.0, std::max(0.01, robot_side_dist));


        // --- Output ---
        if (step % 10 == 0 || step == max_steps - 1 || robot_front_dist < 0.3) {
            std::string status = "FOLLOWING";
            if (robot_front_dist < 0.3) status = "*** EMERGENCY STOP ***";

            std::cout << "Step " << std::setw(3) << step << " | ";
            std::cout << "Side Dist: " << robot_side_dist << "m | ";
            std::cout << "V: " << v << " | W: " << w << " | ";
            std::cout << status << "\n";
        }

       // std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "------------------------------------------------------\n";
    std::cout << "Simulation complete. Final side distance: " << robot_side_dist << "m\n";

    return 0;
}
