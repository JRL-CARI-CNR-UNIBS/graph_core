#include <gtest/gtest.h>
#include <cstdlib>
#include <iostream>
#include <string>

void runBinary(const std::string& binary_path, const std::string& logger_param)
{
  std::cout << "Running test binary: " << binary_path << " with logger file: " << logger_param << "..." << std::endl;

  std::string command = binary_path + " " + logger_param;

  int result = std::system(command.c_str());
  ASSERT_EQ(result, 0) << "Test binary " << binary_path << " failed with exit code " << result;
}

TEST(TestSuite, KdTreeTest)
{
  runBinary("./kdtree_test", "logger_param_gtest.yaml");
}

TEST(TestSuite, NodeConnectionTest)
{
  runBinary("./node_connection_test", "logger_param_gtest.yaml");
}

TEST(TestSuite, ComputePathTest)
{
  runBinary("./compute_path_test", "logger_param_gtest.yaml");
}

TEST(TestSuite, PathPostProcessingTest)
{
  runBinary("./path_post_processing_test", "logger_param_gtest.yaml");
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
