#include <cnr_logger/cnr_logger.h>
#include <graph_core/graph/connection.h>
#include <graph_core/graph/node.h>

int main(int argc, char** argv)
{
  // Load the logger's configuration
  std::string path_to_config_folder = std::string(TEST_DIR);
  std::string logger_file = path_to_config_folder + "/logger_param.yaml";

  if (argc > 1)
    logger_file = path_to_config_folder + "/" + std::string(argv[1]);  // or take it from argument

  cnr_logger::TraceLoggerPtr logger = std::make_shared<cnr_logger::TraceLogger>("node_connection_test", logger_file);

  // Tests
  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::WHITE() << " --- Create random configurations ---");

  Eigen::VectorXd q1(6);
  q1.setRandom();
  Eigen::VectorXd q2(6);
  q2.setRandom();

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::GREEN() << "q1 " << q1.transpose());
  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::GREEN() << "q2 " << q2.transpose());

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::BOLDGREEN() << "Done!");

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::WHITE() << "--- Create nodes on these configurations ---");

  graph::core::NodePtr parent = std::make_shared<graph::core::Node>(q1, logger);
  graph::core::NodePtr child = std::make_shared<graph::core::Node>(q2, logger);

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::BOLDGREEN() << "Done!");

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::WHITE() << "--- Display the nodes ---");

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::BLUE() << "parent: \n" << *parent);
  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::CYAN() << "child:  \n" << *child);

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::BOLDGREEN() << "Done!");

  CNR_INFO(logger,
           cnr_logger::RESET() << cnr_logger::WHITE() << "--- Create and add a connection between the nodes ---");
  graph::core::ConnectionPtr connection = std::make_shared<graph::core::Connection>(parent, child, logger);
  double cost = (parent->getConfiguration() - child->getConfiguration()).norm();
  connection->setCost(cost);
  connection->add();

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::BOLDGREEN() << "Done!");

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::WHITE() << "--- Display the connection ---");
  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::YELLOW() << "connection: \n" << *connection);

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::WHITE() << "--- Check nodes connections ---");
  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::BLUE() << "parent: \n" << *parent);
  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::CYAN() << "child:  \n" << *child);

  bool valid = ((parent->getParentConnectionsSize() == 0) && (parent->getChildConnectionsSize() == 1) &&
                (parent->getNetParentConnectionsSize() == 0) && (parent->getNetChildConnectionsSize() == 0) &&
                (child->getParentConnectionsSize() == 1) && (child->getChildConnectionsSize() == 0) &&
                (child->getNetParentConnectionsSize() == 0) && (child->getNetChildConnectionsSize() == 0));
  if (not valid)
  {
    CNR_FATAL(logger, "something went wrong with nodes connections");
    throw std::runtime_error("something went wrong with nodes connections");
  }

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::BOLDGREEN() << "Done!");

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::WHITE()
                                       << "--- Create and add a net connection "
                                          "between the child a new node ---");
  Eigen::VectorXd q3(6);
  q3.setRandom();
  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::GREEN() << "q3 " << q3.transpose());

  graph::core::NodePtr net_parent = std::make_shared<graph::core::Node>(q3, logger);
  graph::core::ConnectionPtr net_connection =
      std::make_shared<graph::core::Connection>(net_parent, child, logger, true);
  cost = (net_parent->getConfiguration() - child->getConfiguration()).norm();
  net_connection->setCost(cost);
  net_connection->add();

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::YELLOW() << "net connection: \n" << *net_connection);
  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::BLUE() << "net parent: \n" << *net_parent);
  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::CYAN() << "child:  \n" << *child);

  valid = ((parent->getParentConnectionsSize() == 0) && (parent->getChildConnectionsSize() == 1) &&
           (parent->getNetParentConnectionsSize() == 0) && (parent->getNetChildConnectionsSize() == 0) &&
           (net_parent->getParentConnectionsSize() == 0) && (net_parent->getChildConnectionsSize() == 0) &&
           (net_parent->getNetParentConnectionsSize() == 0) && (net_parent->getNetChildConnectionsSize() == 1) &&
           (child->getParentConnectionsSize() == 1) && (child->getChildConnectionsSize() == 0) &&
           (child->getNetParentConnectionsSize() == 1) && (child->getNetChildConnectionsSize() == 0));

  if (not valid)
  {
    CNR_FATAL(logger, "something went wrong with nodes connections");
    throw std::runtime_error("something went wrong with nodes connections");
  }

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::BOLDGREEN() << "Done!");

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::WHITE() << "--- Switch net and standard connections ---");
  child->switchParentConnection(net_connection);

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::YELLOW() << "connection (now net): \n" << *connection);
  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::YELLOW() << "net connection (now not net): \n"
                                       << *net_connection);

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::BLUE() << "parent (now net): \n" << *parent);
  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::BLUE() << "net parent (now not net): \n" << *net_parent);
  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::CYAN() << "child:  \n" << *child);

  valid = ((parent->getParentConnectionsSize() == 0) && (parent->getChildConnectionsSize() == 0) &&
           (parent->getNetParentConnectionsSize() == 0) && (parent->getNetChildConnectionsSize() == 1) &&
           (net_parent->getParentConnectionsSize() == 0) && (net_parent->getChildConnectionsSize() == 1) &&
           (net_parent->getNetParentConnectionsSize() == 0) && (net_parent->getNetChildConnectionsSize() == 0) &&
           (child->getParentConnectionsSize() == 1) && (child->getChildConnectionsSize() == 0) &&
           (child->getNetParentConnectionsSize() == 1) && (child->getNetChildConnectionsSize() == 0));

  if (not valid)
  {
    CNR_FATAL(logger, "something went wrong with connections switch");
    throw std::runtime_error("something went wrong with connections switch");
  }

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::BOLDGREEN() << "Done!");

  CNR_INFO(logger,
           cnr_logger::RESET() << cnr_logger::WHITE() << "--- Disconnect child to remove all the connections ---");
  child->disconnect();

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::BLUE() << "child: \n" << *child);
  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::YELLOW() << "connection: \n" << *connection);
  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::YELLOW() << "net connection: \n" << *net_connection);

  valid = ((parent->getParentConnectionsSize() == 0) && (parent->getChildConnectionsSize() == 0) &&
           (parent->getNetParentConnectionsSize() == 0) && (parent->getNetChildConnectionsSize() == 0) &&
           (net_parent->getParentConnectionsSize() == 0) && (net_parent->getChildConnectionsSize() == 0) &&
           (net_parent->getNetParentConnectionsSize() == 0) && (net_parent->getNetChildConnectionsSize() == 0) &&
           (child->getParentConnectionsSize() == 0) && (child->getChildConnectionsSize() == 0) &&
           (child->getNetParentConnectionsSize() == 0) && (child->getNetChildConnectionsSize() == 0));

  if (not valid)
  {
    CNR_FATAL(logger, "something went wrong with node disconnect");
    throw std::runtime_error("something went wrong with node disconnect");
  }

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::BOLDGREEN() << "Done!");

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::WHITE() << "--- Add connection again (not net) ---");
  connection->add(false);

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::YELLOW() << "connection: \n" << *connection);
  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::BLUE() << "parent: \n" << *parent);
  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::CYAN() << "child:  \n" << *child);

  valid = ((parent->getParentConnectionsSize() == 0) && (parent->getChildConnectionsSize() == 1) &&
           (parent->getNetParentConnectionsSize() == 0) && (parent->getNetChildConnectionsSize() == 0) &&
           (child->getParentConnectionsSize() == 1) && (child->getChildConnectionsSize() == 0) &&
           (child->getNetParentConnectionsSize() == 0) && (child->getNetChildConnectionsSize() == 0));

  if (not valid)
  {
    CNR_FATAL(logger, "something went wrong with connection add");
    throw std::runtime_error("something went wrong with connection add");
  }

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::BOLDGREEN() << "Done!");

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::WHITE() << "--- Remove connection ---");
  connection->remove();

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::YELLOW() << "connection: \n" << *connection);
  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::BLUE() << "parent: \n" << *parent);
  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::CYAN() << "child:  \n" << *child);

  valid = ((parent->getParentConnectionsSize() == 0) && (parent->getChildConnectionsSize() == 0) &&
           (parent->getNetParentConnectionsSize() == 0) && (parent->getNetChildConnectionsSize() == 0) &&
           (child->getParentConnectionsSize() == 0) && (child->getChildConnectionsSize() == 0) &&
           (child->getNetParentConnectionsSize() == 0) && (child->getNetChildConnectionsSize() == 0));

  if (not valid)
  {
    CNR_FATAL(logger, "something went wrong with connection remove");
    throw std::runtime_error("something went wrong with connection remove");
  }

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::BOLDGREEN() << "Done!");

  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::WHITE() << "--- Check parallelism of connections ---");
  if (connection->isParallel(net_connection))
  {
    CNR_FATAL(logger, "something went wrong with connections parallelism");
    throw std::runtime_error("something went wrong with connections parallelism");
  }

  Eigen::VectorXd q_mid = (q1 - q2) / 2;
  graph::core::NodePtr node_mid = std::make_shared<graph::core::Node>(q_mid, logger);
  graph::core::ConnectionPtr connection_parallel = std::make_shared<graph::core::Connection>(parent, node_mid, logger);

  if (connection->isParallel(connection_parallel))
  {
    CNR_FATAL(logger, "something went wrong with connections parallelism");
    throw std::runtime_error("something went wrong with connections parallelism");
  }
  else

    CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::BOLDGREEN() << "Done!");

  return 0;
}
