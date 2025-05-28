#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <string>
#include <vector>

struct Pose {
    double x, y, z;
};

std::vector<Pose> loadCornerPositions(const std::string& filename) {
    std::vector<Pose> poses;
    YAML::Node config = YAML::LoadFile(filename);
    for (const auto& node : config["corner_positions"]) {
        Pose p;
        p.x = node["x"].as<double>();
        p.y = node["y"].as<double>();
        p.z = node["z"].as<double>();
        poses.push_back(p);
    }
    return poses;
}

TEST(CornerPositionsStrictTest, PerFieldComparison) {
    // Use for simulation testing
    const std::string recorded_file = "/home/jarred/git/DalESelfEBot/ur3_localisation/config/sim_params.yaml";
    const std::string ground_truth_file = "/home/jarred/git/DalESelfEBot/ur3_localisation/scripts/sim_ground_truth.yaml";
    // Use for simulation hardware
    // const std::string recorded_file = "/home/jarred/git/DalESelfEBot/ur3_localisation/config/hardware_params.yaml";
    // const std::string ground_truth_file = "/home/jarred/git/DalESelfEBot/ur3_localisation/scripts/hardware_ground_truth.yaml";

    auto recorded = loadCornerPositions(recorded_file);
    auto expected = loadCornerPositions(ground_truth_file);

    ASSERT_EQ(recorded.size(), expected.size()) << "Mismatch in number of corner positions";

    const double pos_tol = 0.0005;   // 0.5 mm position tolerance

    for (size_t i = 0; i < expected.size(); ++i) {
        const auto& r = recorded[i];
        const auto& e = expected[i];

        EXPECT_NEAR(r.x, e.x, pos_tol) << "Point " << i << ": x out of tolerance";
        EXPECT_NEAR(r.y, e.y, pos_tol) << "Point " << i << ": y out of tolerance";
        EXPECT_NEAR(r.z, e.z, pos_tol) << "Point " << i << ": z out of tolerance";
    }
}
