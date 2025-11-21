#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include <iomanip>

// Plain C++ version of the ROS-based LinearController
// - Single file, no ROS dependencies
// - Simulates a simple differential-drive robot using control law from the original

class LinearController {
public:
    LinearController(double kp_linear = 3.0,
                     double k_alpha = 8.0,
                     double k_beta = -1.5,
                     double pos_threshold = 0.1,
                     double ang_threshold = 0.1)
        : kp_linear_(kp_linear), k_alpha_(k_alpha), k_beta_(k_beta),
          POSITION_THRESHOLD(pos_threshold), ANGLE_THRESHOLD(ang_threshold) {}

    // Compute control from current pose to goal
    void computeControl(double current_x, double current_y, double current_theta,
                        double goal_x, double goal_y, double goal_theta,
                        double &out_linear, double &out_angular) {
        double dx = goal_x - current_x;
        double dy = goal_y - current_y;
        double distance = std::sqrt(dx*dx + dy*dy);

        double angle_to_goal = std::atan2(dy, dx);
        double alpha = angle_to_goal - current_theta;
        alpha = normalizeAngle(alpha);

        double beta = goal_theta - current_theta - alpha;
        beta = normalizeAngle(beta);

        out_linear = 0.0;
        out_angular = 0.0;

        // Stop when both position and orientation are within thresholds
        if (distance < POSITION_THRESHOLD && std::fabs(beta) < ANGLE_THRESHOLD) {
            out_linear = 0.0;
            out_angular = 0.0;
            return;
        }

        // Move toward the goal position (only linear speed) if not close enough
        if (distance >= POSITION_THRESHOLD) {
            out_linear = kp_linear_ * distance * std::cos(alpha);
            out_angular = (k_alpha_ * alpha) + (k_beta_ * beta);
        } else { // At position, rotate to final orientation
            out_linear = 0.0;
            out_angular = k_beta_ * beta;
        }

        // Limit speeds (use same limits as original code)
        const double MAX_LINEAR = 0.22;
        const double MAX_ANGULAR = 2.84;
        if (out_linear > MAX_LINEAR) out_linear = MAX_LINEAR;
        if (out_linear < -MAX_LINEAR) out_linear = -MAX_LINEAR;
        if (out_angular > MAX_ANGULAR) out_angular = MAX_ANGULAR;
        if (out_angular < -MAX_ANGULAR) out_angular = -MAX_ANGULAR;
    }

    static double normalizeAngle(double a) {
        while (a > M_PI) a -= 2.0*M_PI;
        while (a < -M_PI) a += 2.0*M_PI;
        return a;
    }

private:
    double kp_linear_;
    double k_alpha_;
    double k_beta_;
    const double POSITION_THRESHOLD;
    const double ANGLE_THRESHOLD;
};

int main(int argc, char** argv) {
    // Defaults copied from original ROS node
    double goal_x = 1.0;
    double goal_y = 1.0;
    double goal_theta = 0.0;

    double start_x = 0.0;
    double start_y = 0.0;
    double start_theta = 0.0;

    double kp_linear = 3.0;
    double k_alpha = 8.0;
    double k_beta = -1.5;

    // Optional CLI: goal_x goal_y goal_theta [start_x start_y start_theta]
    if (argc >= 4) {
        goal_x = std::stod(argv[1]);
        goal_y = std::stod(argv[2]);
        goal_theta = std::stod(argv[3]);
    }
    if (argc >= 7) {
        start_x = std::stod(argv[4]);
        start_y = std::stod(argv[5]);
        start_theta = std::stod(argv[6]);
    }

    LinearController controller(kp_linear, k_alpha, k_beta);

    double x = start_x;
    double y = start_y;
    double theta = start_theta;

    const double dt = 0.1; // simulation timestep (s)
    const int max_steps = 1000;

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Starting simulation: goal=(" << goal_x << ", " << goal_y << ", " << goal_theta << ")\n";
    std::cout << "start pose=(" << x << ", " << y << ", " << theta << ")\n";

    bool reached = false;
    for (int step = 0; step < max_steps; ++step) {
        double v=0.0, w=0.0;
        controller.computeControl(x,y,theta,goal_x,goal_y,goal_theta,v,w);

        // Integrate simple unicycle model
        x += v * std::cos(theta) * dt;
        y += v * std::sin(theta) * dt;
        theta += w * dt;
        theta = LinearController::normalizeAngle(theta);

        double dx = goal_x - x;
        double dy = goal_y - y;
        double dist = std::sqrt(dx*dx + dy*dy);
        double alpha = std::atan2(dy, dx) - theta;
        alpha = LinearController::normalizeAngle(alpha);
        double beta = goal_theta - theta - alpha;
        beta = LinearController::normalizeAngle(beta);

        if (dist < 0.1 && std::fabs(beta) < 0.1) {
            std::cout << "Step " << step << ": pose=(" << x << ", " << y << ", " << theta << ") reached goal\n";
            reached = true;
            break;
        }

        if (step % 10 == 0) {
            std::cout << "Step " << step << ": pose=(" << x << ", " << y << ", " << theta << ") ";
            std::cout << "v=" << v << " w=" << w << " dist=" << dist << " beta=" << beta << "\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!reached) {
        std::cout << "Did not reach goal within max steps. Final pose=(" << x << ", " << y << ", " << theta << ")\n";
    }

    return 0;
}
