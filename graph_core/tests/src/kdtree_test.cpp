#include <cnr_logger/cnr_logger.h>
#include <graph_core/datastructure/kdtree.h>
#include <random>

int main(int argc, char** argv)
{
  // Load the logger's configuration
  std::string path_to_config_folder = std::string(TEST_DIR);
  std::string logger_file = path_to_config_folder + "/logger_param.yaml";

  if (argc > 1)
    logger_file = path_to_config_folder + "/" + std::string(argv[1]);  // or take it from argument

  cnr_logger::TraceLoggerPtr logger = std::make_shared<cnr_logger::TraceLogger>("graph_core_tests_logger", logger_file);

  int n_kdnodes = 10;
  int threshold = std::numeric_limits<int>::max();

  if (argc == 3)
    n_kdnodes = std::atoi(argv[2]);

  if (argc == 4)
  {
    n_kdnodes = std::atoi(argv[2]);
    threshold = std::atoi(argv[3]);
  }

  if (argc > 4)
    CNR_WARN(logger,
             cnr_logger::RESET() << cnr_logger::BOLDYELLOW() << "Number of inputs to the program should be <=3");

  graph::core::KdTreePtr kdtree = std::make_shared<graph::core::KdTree>(logger);
  kdtree->deletedNodesThreshold(threshold);

  std::vector<graph::core::NodePtr> nodes;
  for (int i = 0; i < n_kdnodes; i++)
  {
    Eigen::VectorXd q(3);
    q.setRandom();
    graph::core::NodePtr node = std::make_shared<graph::core::Node>(q, logger);
    nodes.push_back(node);

    CNR_INFO(logger,
             cnr_logger::RESET() << cnr_logger::WHITE() << " Random node -> " << q.transpose() << " (" << node << ")");

    kdtree->insert(node);
  }

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::YELLOW() << " KdTree: \n" << *kdtree);

  std::mt19937 rng(std::random_device{}());
  std::uniform_int_distribution<int> dist(0, n_kdnodes - 1);
  int n1, n2, n3;
  n1 = dist(rng);
  n2 = n3 = n1;
  while (n2 == n1)
    n2 = dist(rng);
  while (n3 == n1 || n3 == n2)
    n3 = dist(rng);

  graph::core::KdNodePtr kdn1, kdn2, kdn3;
  kdtree->findNode(nodes.at(n1), kdn1);
  kdtree->findNode(nodes.at(n2), kdn2);
  kdtree->findNode(nodes.at(n3), kdn3);

  CNR_INFO(logger, cnr_logger::RESET() << "Removing 3 nodes:\n\t-" << kdn1 << "\n\t-" << kdn2 << "\n\t-" << kdn3);
  kdtree->deleteNode(nodes.at(n1));
  kdtree->deleteNode(nodes.at(n2));
  kdtree->deleteNode(nodes.at(n3));

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::YELLOW() << " KdTree: \n" << *kdtree);
  kdtree->print_deleted_nodes_ = true;
  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::BLUE() << " KdTree with also deleted nodes: \n" << *kdtree);

  return 0;
}
