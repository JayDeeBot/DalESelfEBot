#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
#include <fstream>

TEST(FourLocationsTest, ValidFormat) {
    // Load the YAML file
    YAML::Node config = YAML::LoadFile("/home/jarred/git/DalESelfEBot/ur3_localisation/config/sim_params.yaml"); // Use for simulation testing
    // YAML::Node config = YAML::LoadFile("/home/jarred/git/DalESelfEBot/ur3_localisation/config/hardware_params.yaml"); // Use for hardware testing

    // Check if 'corner_positions' exists and is a sequence
    ASSERT_TRUE(config["corner_positions"]) << "'corner_positions' key is missing.";
    ASSERT_TRUE(config["corner_positions"].IsSequence()) << "'corner_positions' must be a sequence.";

    auto positions = config["corner_positions"];
    ASSERT_EQ(positions.size(), 4) << "Expected 4 corner positions.";

    for (size_t i = 0; i < positions.size(); ++i) {
        const auto& pos = positions[i];
        EXPECT_TRUE(pos["x"]) << "Missing 'x' in position " << i;
        EXPECT_TRUE(pos["y"]) << "Missing 'y' in position " << i;
        EXPECT_TRUE(pos["z"]) << "Missing 'z' in position " << i;
        EXPECT_TRUE(pos["qx"]) << "Missing 'qx' in position " << i;
        EXPECT_TRUE(pos["qy"]) << "Missing 'qy' in position " << i;
        EXPECT_TRUE(pos["qz"]) << "Missing 'qz' in position " << i;
        EXPECT_TRUE(pos["qw"]) << "Missing 'qw' in position " << i;

        // Optionally, test that these are of type float/double
        EXPECT_TRUE(pos["x"].IsScalar());
        EXPECT_TRUE(pos["y"].IsScalar());
        EXPECT_TRUE(pos["z"].IsScalar());
        EXPECT_TRUE(pos["qx"].IsScalar());
        EXPECT_TRUE(pos["qy"].IsScalar());
        EXPECT_TRUE(pos["qz"].IsScalar());
        EXPECT_TRUE(pos["qw"].IsScalar());
    }
}
