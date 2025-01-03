#include <gtest/gtest.h>
#include <cstdlib>
#include <iostream>
#include <string>

std::string logger_file_ = "/logger_param_gtest.yaml";

void runBinary(const std::string& binary_path)
{
  std::cout << "Running test binary: " << binary_path << " with logger file: " << logger_file_ << "..." << std::endl;

  std::string command = binary_path + " " + logger_file_;

  int result = std::system(command.c_str());
  ASSERT_EQ(result, 0) << "Test binary " << binary_path << " failed with exit code " << result;
}

TEST(TestSuite, KdTreeTest)
{
  runBinary("./kdtree_test");
}

TEST(TestSuite, NodeConnectionTest)
{
  runBinary("./node_connection_test");
}

TEST(TestSuite, ComputePathTest)
{
  runBinary("./compute_path_test");
}

TEST(TestSuite, PathPostProcessingTest)
{
  runBinary("./path_post_processing_test");
}

int main(int argc, char** argv)
{
  if (argc > 1)
    logger_file_ = std::string(argv[1]);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
