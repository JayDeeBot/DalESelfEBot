#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <cmath>
#include <string>

using json = nlohmann::json;

constexpr double Z_TOLERANCE = 0.0005; // 0.5 mm

double compute_average_z_from_yaml(const std::string& yaml_path) {
    YAML::Node root = YAML::LoadFile(yaml_path);
    const auto& corners = root["corner_positions"];

    double sum_z = 0.0;
    for (const auto& point : corners) {
        sum_z += point["z"].as<double>();
    }

    return sum_z / corners.size();
}

std::vector<double> load_z_values_from_ground_truth(const std::string& json_path) {
    std::ifstream f(json_path);
    json j;
    f >> j;

    std::vector<double> z_values;
    for (const auto& waypoint : j["splines"][0]["waypoints"]) {
        if (waypoint.size() >= 3) {
            z_values.push_back(waypoint[2]);
        }
    }

    return z_values;
}

TEST(ToolPathZTest, ZWithinTolerance) {
    const std::string reference_yaml = "/home/jarred/git/DalESelfEBot/ur3_localisation/config/sim_params.yaml";
    // const std::string reference_yaml = "/home/jarred/git/DalESelfEBot/ur3_localisation/config/hardware_params_2.yaml";
    // Use for simulation testing
    const std::string path_json = "/home/jarred/git/DalESelfEBot/ur3_control/scripts/sim_ground_truth.json";
    // Use for hardware testing
    // const std::string path_json = "/home/jarred/git/DalESelfEBot/ur3_control/scripts/hardware_ground_truth.json";
    
    double expected_z = compute_average_z_from_yaml(reference_yaml);
    auto actual_z_values = load_z_values_from_ground_truth(path_json);

    ASSERT_GT(actual_z_values.size(), 0) << "Ground truth path contains no z-values.";

    for (size_t i = 0; i < actual_z_values.size(); ++i) {
        double z = actual_z_values[i];
        double dz = std::abs(z - expected_z);

        EXPECT_LE(dz, Z_TOLERANCE)
            << "Z-value at waypoint " << i << " is out of tolerance.\n"
            << "  Actual z = " << z << ", Expected avg z = " << expected_z
            << ", Difference = " << dz << " m";
    }
}
