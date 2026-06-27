#include <gtest/gtest.h>
#include "config/ConfigReader.h"
#include <fstream>
#include <filesystem>

TEST(ConfigTest, LoadValidConfig) {
    // Create a temporary JSON file
    std::string test_file = "test_config.json";
    std::ofstream out(test_file);
    out << "{\n"
        << "  \"GT_POSE_PATH\": \"/path/to/gt\",\n"
        << "  \"TOTAL_FRAMES_TO_PROCESS\": 100\n"
        << "}";
    out.close();

    ConfigReader reader(test_file);

    EXPECT_EQ(reader.get<std::string>("GT_POSE_PATH"), "/path/to/gt");
    EXPECT_EQ(reader.get<int>("TOTAL_FRAMES_TO_PROCESS"), 100);

    // Default value fallback logic (as used in main)
    int default_val = reader.hasKey("NON_EXISTENT_KEY") ? reader.get<int>("NON_EXISTENT_KEY") : 42;
    EXPECT_EQ(default_val, 42);

    std::filesystem::remove(test_file);
}

TEST(ConfigTest, MissingKeyThrows) {
    std::string test_file = "test_config2.json";
    std::ofstream out(test_file);
    out << "{\n"
        << "  \"VALID_KEY\": 1\n"
        << "}";
    out.close();

    ConfigReader reader(test_file);

    EXPECT_THROW(reader.get<int>("MISSING_KEY"), std::runtime_error);

    std::filesystem::remove(test_file);
}
