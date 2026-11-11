#include "test_utils.h"
#include <graph_core/graph/connection.h>
#include <graph_core/graph/node.h>

int main(int argc, char** argv)
{
  // Logger: the log level can be passed as first argument
  graph::core::LoggerPtr logger = graph_core_tests::makeLogger("node_connection_test", argc, argv);
  if (not logger)
    return 1;

  // Tests
  GC_INFO(logger, graph::core::color::RESET()
                      << graph::core::color::WHITE() << " --- Create random configurations ---");

  Eigen::VectorXd q1(6);
  q1.setRandom();
  Eigen::VectorXd q2(6);
  q2.setRandom();

  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::GREEN() << "q1 " << q1.transpose());
  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::GREEN() << "q2 " << q2.transpose());

  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::BOLDGREEN() << "Done!");

  GC_INFO(logger, graph::core::color::RESET()
                      << graph::core::color::WHITE() << "--- Create nodes on these configurations ---");

  graph::core::NodePtr parent = std::make_shared<graph::core::Node>(q1, logger);
  graph::core::NodePtr child = std::make_shared<graph::core::Node>(q2, logger);

  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::BOLDGREEN() << "Done!");

  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::WHITE() << "--- Display the nodes ---");

  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::BLUE() << "parent: \n" << *parent);
  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::CYAN() << "child:  \n" << *child);

  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::BOLDGREEN() << "Done!");

  GC_INFO(logger, graph::core::color::RESET()
                      << graph::core::color::WHITE() << "--- Create and add a connection between the nodes ---");
  graph::core::ConnectionPtr connection = std::make_shared<graph::core::Connection>(parent, child, logger);
  double cost = (parent->getConfiguration() - child->getConfiguration()).norm();
  connection->setCost(cost);
  connection->add();

  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::BOLDGREEN() << "Done!");

  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::WHITE() << "--- Display the connection ---");
  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::YELLOW() << "connection: \n" << *connection);

  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::WHITE() << "--- Check nodes connections ---");
  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::BLUE() << "parent: \n" << *parent);
  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::CYAN() << "child:  \n" << *child);

  bool valid = ((parent->getParentConnectionsSize() == 0) && (parent->getChildConnectionsSize() == 1) &&
                (parent->getNetParentConnectionsSize() == 0) && (parent->getNetChildConnectionsSize() == 0) &&
                (child->getParentConnectionsSize() == 1) && (child->getChildConnectionsSize() == 0) &&
                (child->getNetParentConnectionsSize() == 0) && (child->getNetChildConnectionsSize() == 0));
  if (not valid)
  {
    GC_FATAL(logger, "something went wrong with nodes connections");
    throw std::runtime_error("something went wrong with nodes connections");
  }

  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::BOLDGREEN() << "Done!");

  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::WHITE()
                                              << "--- Create and add a net connection "
                                                 "between the child a new node ---");
  Eigen::VectorXd q3(6);
  q3.setRandom();
  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::GREEN() << "q3 " << q3.transpose());

  graph::core::NodePtr net_parent = std::make_shared<graph::core::Node>(q3, logger);
  graph::core::ConnectionPtr net_connection =
      std::make_shared<graph::core::Connection>(net_parent, child, logger, true);
  cost = (net_parent->getConfiguration() - child->getConfiguration()).norm();
  net_connection->setCost(cost);
  net_connection->add();

  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::YELLOW() << "net connection: \n"
                                              << *net_connection);
  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::BLUE() << "net parent: \n" << *net_parent);
  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::CYAN() << "child:  \n" << *child);

  valid = ((parent->getParentConnectionsSize() == 0) && (parent->getChildConnectionsSize() == 1) &&
           (parent->getNetParentConnectionsSize() == 0) && (parent->getNetChildConnectionsSize() == 0) &&
           (net_parent->getParentConnectionsSize() == 0) && (net_parent->getChildConnectionsSize() == 0) &&
           (net_parent->getNetParentConnectionsSize() == 0) && (net_parent->getNetChildConnectionsSize() == 1) &&
           (child->getParentConnectionsSize() == 1) && (child->getChildConnectionsSize() == 0) &&
           (child->getNetParentConnectionsSize() == 1) && (child->getNetChildConnectionsSize() == 0));

  if (not valid)
  {
    GC_FATAL(logger, "something went wrong with nodes connections");
    throw std::runtime_error("something went wrong with nodes connections");
  }

  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::BOLDGREEN() << "Done!");

  GC_INFO(logger, graph::core::color::RESET()
                      << graph::core::color::WHITE() << "--- Switch net and standard connections ---");
  child->switchParentConnection(net_connection);

  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::YELLOW() << "connection (now net): \n"
                                              << *connection);
  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::YELLOW() << "net connection (now not net): \n"
                                              << *net_connection);

  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::BLUE() << "parent (now net): \n" << *parent);
  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::BLUE() << "net parent (now not net): \n"
                                              << *net_parent);
  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::CYAN() << "child:  \n" << *child);

  valid = ((parent->getParentConnectionsSize() == 0) && (parent->getChildConnectionsSize() == 0) &&
           (parent->getNetParentConnectionsSize() == 0) && (parent->getNetChildConnectionsSize() == 1) &&
           (net_parent->getParentConnectionsSize() == 0) && (net_parent->getChildConnectionsSize() == 1) &&
           (net_parent->getNetParentConnectionsSize() == 0) && (net_parent->getNetChildConnectionsSize() == 0) &&
           (child->getParentConnectionsSize() == 1) && (child->getChildConnectionsSize() == 0) &&
           (child->getNetParentConnectionsSize() == 1) && (child->getNetChildConnectionsSize() == 0));

  if (not valid)
  {
    GC_FATAL(logger, "something went wrong with connections switch");
    throw std::runtime_error("something went wrong with connections switch");
  }

  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::BOLDGREEN() << "Done!");

  GC_INFO(logger, graph::core::color::RESET()
                      << graph::core::color::WHITE() << "--- Disconnect child to remove all the connections ---");
  child->disconnect();

  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::BLUE() << "child: \n" << *child);
  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::YELLOW() << "connection: \n" << *connection);
  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::YELLOW() << "net connection: \n"
                                              << *net_connection);

  valid = ((parent->getParentConnectionsSize() == 0) && (parent->getChildConnectionsSize() == 0) &&
           (parent->getNetParentConnectionsSize() == 0) && (parent->getNetChildConnectionsSize() == 0) &&
           (net_parent->getParentConnectionsSize() == 0) && (net_parent->getChildConnectionsSize() == 0) &&
           (net_parent->getNetParentConnectionsSize() == 0) && (net_parent->getNetChildConnectionsSize() == 0) &&
           (child->getParentConnectionsSize() == 0) && (child->getChildConnectionsSize() == 0) &&
           (child->getNetParentConnectionsSize() == 0) && (child->getNetChildConnectionsSize() == 0));

  if (not valid)
  {
    GC_FATAL(logger, "something went wrong with node disconnect");
    throw std::runtime_error("something went wrong with node disconnect");
  }

  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::BOLDGREEN() << "Done!");

  GC_INFO(logger, graph::core::color::RESET()
                      << graph::core::color::WHITE() << "--- Add connection again (not net) ---");
  connection->add(false);

  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::YELLOW() << "connection: \n" << *connection);
  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::BLUE() << "parent: \n" << *parent);
  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::CYAN() << "child:  \n" << *child);

  valid = ((parent->getParentConnectionsSize() == 0) && (parent->getChildConnectionsSize() == 1) &&
           (parent->getNetParentConnectionsSize() == 0) && (parent->getNetChildConnectionsSize() == 0) &&
           (child->getParentConnectionsSize() == 1) && (child->getChildConnectionsSize() == 0) &&
           (child->getNetParentConnectionsSize() == 0) && (child->getNetChildConnectionsSize() == 0));

  if (not valid)
  {
    GC_FATAL(logger, "something went wrong with connection add");
    throw std::runtime_error("something went wrong with connection add");
  }

  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::BOLDGREEN() << "Done!");

  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::WHITE() << "--- Remove connection ---");
  connection->remove();

  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::YELLOW() << "connection: \n" << *connection);
  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::BLUE() << "parent: \n" << *parent);
  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::CYAN() << "child:  \n" << *child);

  valid = ((parent->getParentConnectionsSize() == 0) && (parent->getChildConnectionsSize() == 0) &&
           (parent->getNetParentConnectionsSize() == 0) && (parent->getNetChildConnectionsSize() == 0) &&
           (child->getParentConnectionsSize() == 0) && (child->getChildConnectionsSize() == 0) &&
           (child->getNetParentConnectionsSize() == 0) && (child->getNetChildConnectionsSize() == 0));

  if (not valid)
  {
    GC_FATAL(logger, "something went wrong with connection remove");
    throw std::runtime_error("something went wrong with connection remove");
  }

  GC_INFO(logger, graph::core::color::RESET() << graph::core::color::BOLDGREEN() << "Done!");

  GC_INFO(logger, graph::core::color::RESET()
                      << graph::core::color::WHITE() << "--- Check parallelism of connections ---");
  if (connection->isParallel(net_connection))
  {
    GC_FATAL(logger, "something went wrong with connections parallelism");
    throw std::runtime_error("something went wrong with connections parallelism");
  }

  Eigen::VectorXd q_mid = (q1 - q2) / 2;
  graph::core::NodePtr node_mid = std::make_shared<graph::core::Node>(q_mid, logger);
  graph::core::ConnectionPtr connection_parallel = std::make_shared<graph::core::Connection>(parent, node_mid, logger);

  if (connection->isParallel(connection_parallel))
  {
    GC_FATAL(logger, "something went wrong with connections parallelism");
    throw std::runtime_error("something went wrong with connections parallelism");
  }
  else

    GC_INFO(logger, graph::core::color::RESET() << graph::core::color::BOLDGREEN() << "Done!");

  return 0;
}
