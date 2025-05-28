#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>

using json = nlohmann::json;
using Waypoint = std::vector<double>;

constexpr double XY_TOLERANCE = 0.0005;  // 0.5 mm

bool is_xy_within_tolerance(const Waypoint& expected, const Waypoint& actual) {
    return std::abs(expected[0] - actual[0]) <= XY_TOLERANCE &&
           std::abs(expected[1] - actual[1]) <= XY_TOLERANCE;
}

std::vector<Waypoint> load_waypoints(const std::string& filepath) {
    std::ifstream file(filepath);
    json j;
    file >> j;
    return j["splines"][0]["waypoints"].get<std::vector<Waypoint>>();
}

TEST(ToolPathAccuracyTest, XYMatchedWithinTolerance) {
    // Use for simulation testing
    const std::string expected_path = "/home/jarred/git/DalESelfEBot/ur3_control/config/sim_circle_waypoints.json";
    const std::string actual_path = "/home/jarred/git/DalESelfEBot/ur3_control/scripts/sim_ground_truth.json";
    // Use for hardware testing
    // const std::string expected_path = "/home/jarred/git/DalESelfEBot/ur3_control/config/hardware_circle_waypoints.json";
    // const std::string actual_path = "/home/jarred/git/DalESelfEBot/ur3_control/scripts/hardware_ground_truth.json";

    auto expected_waypoints = load_waypoints(expected_path);
    auto actual_waypoints = load_waypoints(actual_path);

    ASSERT_GT(actual_waypoints.size(), 0) << "Actual path has no waypoints.";
    ASSERT_GT(expected_waypoints.size(), 0) << "Expected tool path has no waypoints.";

    size_t actual_index = 0;

    for (size_t i = 0; i < expected_waypoints.size(); ++i) {
        bool match_found = false;

        // Scan forward in actual waypoints
        for (; actual_index < actual_waypoints.size(); ++actual_index) {
            if (is_xy_within_tolerance(expected_waypoints[i], actual_waypoints[actual_index])) {
                match_found = true;
                ++actual_index;  // move forward for the next check
                break;
            }
        }

        EXPECT_TRUE(match_found) << "Expected waypoint " << i << " (x=" << expected_waypoints[i][0]
                                 << ", y=" << expected_waypoints[i][1]
                                 << ") was not reached within ±0.5 mm tolerance.";
    }
}
