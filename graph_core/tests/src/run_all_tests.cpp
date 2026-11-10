#include <gtest/gtest.h>
#include <graph_core/params.h>
#include <cstdlib>
#include <iostream>
#include <string>

// Log level passed to the test binaries (trace, debug, info, warn, error, fatal)
std::string log_level_ = "warn";

void runBinary(const std::string& binary_path)
{
  std::cout << "Running test binary: " << binary_path << " with log level: " << log_level_ << "..." << std::endl;

  std::string command = binary_path + " " + log_level_;

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

// Parameters: conversions done by get_param
TEST(Params, Conversions)
{
  graph::core::LoggerPtr logger;  // null logger: nothing is printed
  graph::core::ParamMap params = { { "int", int64_t(3) },
                                   { "negative", int64_t(-1) },
                                   { "double", 0.5 },
                                   { "flag", true },
                                   { "name", std::string("rrt") },
                                   { "vec", std::vector<double>{ 1.0, 2.0, 3.0 } },
                                   { "int_vec", std::vector<int64_t>{ 1, 2 } } };

  double d;
  EXPECT_TRUE(graph::core::get_param(logger, params, "int", d));  // int -> double
  EXPECT_DOUBLE_EQ(d, 3.0);
  EXPECT_TRUE(graph::core::get_param(logger, params, "double", d));
  EXPECT_DOUBLE_EQ(d, 0.5);

  unsigned int u;
  EXPECT_TRUE(graph::core::get_param(logger, params, "int", u));
  EXPECT_EQ(u, 3u);
  EXPECT_THROW(graph::core::get_param(logger, params, "negative", u), std::invalid_argument);  // sign check
  size_t sz;
  EXPECT_TRUE(graph::core::get_param(logger, params, "int", sz));
  EXPECT_EQ(sz, 3u);

  int i;
  EXPECT_THROW(graph::core::get_param(logger, params, "double", i), std::invalid_argument);  // no double -> int

  bool b = false;
  EXPECT_TRUE(graph::core::get_param(logger, params, "flag", b));
  EXPECT_TRUE(b);
  std::string str;
  EXPECT_TRUE(graph::core::get_param(logger, params, "name", str));
  EXPECT_EQ(str, "rrt");
  EXPECT_THROW(graph::core::get_param(logger, params, "name", d), std::invalid_argument);

  Eigen::VectorXd v;
  EXPECT_TRUE(graph::core::get_param(logger, params, "vec", v));
  EXPECT_EQ(v.size(), 3);
  EXPECT_DOUBLE_EQ(v(2), 3.0);
  EXPECT_TRUE(graph::core::get_param(logger, params, "int_vec", v));  // integer array -> Eigen
  EXPECT_DOUBLE_EQ(v(1), 2.0);
  Eigen::Vector3d v3;
  EXPECT_TRUE(graph::core::get_param(logger, params, "vec", v3));
  EXPECT_THROW(graph::core::get_param(logger, params, "int_vec", v3), std::invalid_argument);  // wrong size

  std::vector<double> sv;
  EXPECT_TRUE(graph::core::get_param(logger, params, "int_vec", sv));
  EXPECT_EQ(sv.size(), 2u);
  EXPECT_THROW(graph::core::get_param(logger, params, "double", sv), std::invalid_argument);  // scalar -> array

  // Missing parameters
  d = 7.0;
  EXPECT_FALSE(graph::core::get_param(logger, params, "missing", d));
  EXPECT_DOUBLE_EQ(d, 7.0);
  EXPECT_FALSE(graph::core::get_param(logger, params, "missing", d, 1.5));
  EXPECT_DOUBLE_EQ(d, 1.5);
  EXPECT_TRUE(graph::core::get_param(logger, params, "double", d, 1.5));
  EXPECT_DOUBLE_EQ(d, 0.5);
}

// Parameters: loading from YAML
TEST(Params, FromYAML)
{
  YAML::Node yaml = YAML::Load(R"(
solver:
  max_distance: 1
  rewire_factor: 1.1
  use_kdtree: true
  name: rrt_star
  nested:
    value: 2
lb: [-1, -2.5, 3]
joints: [a, b]
ids: [1, 2]
path: [[0, 0], [1, 1]]
)");
  graph::core::ParamMap params = graph::core::paramsFromYAML(yaml);

  EXPECT_TRUE(std::holds_alternative<int64_t>(params.at("solver/max_distance")));
  EXPECT_TRUE(std::holds_alternative<double>(params.at("solver/rewire_factor")));
  EXPECT_TRUE(std::holds_alternative<bool>(params.at("solver/use_kdtree")));
  EXPECT_TRUE(std::holds_alternative<std::string>(params.at("solver/name")));
  EXPECT_TRUE(std::holds_alternative<int64_t>(params.at("solver/nested/value")));
  EXPECT_TRUE(std::holds_alternative<std::vector<double>>(params.at("lb")));
  EXPECT_TRUE(std::holds_alternative<std::vector<std::string>>(params.at("joints")));
  EXPECT_TRUE(std::holds_alternative<std::vector<int64_t>>(params.at("ids")));
  EXPECT_EQ(params.count("path"), 0u);  // sequences of sequences are data, not parameters

  graph::core::ParamMap solver = graph::core::subParams(params, "solver");
  EXPECT_EQ(solver.size(), 5u);
  double max_distance;
  EXPECT_TRUE(graph::core::get_param(nullptr, solver, "max_distance", max_distance));
  EXPECT_DOUBLE_EQ(max_distance, 1.0);
}

int main(int argc, char** argv)
{
  if (argc > 1)
    log_level_ = std::string(argv[1]);

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
