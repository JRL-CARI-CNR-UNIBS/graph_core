#include <graph_core/graph/connection.h>
#include <graph_core/graph/node.h>
#include <cnr_logger/cnr_logger.h>

//To get the path to logger params file
#include <filesystem>

int main(int argc, char **argv)
{

//  // Get the path to the logger params file
//  std::filesystem::path file_path = std::filesystem::read_symlink("/proc/self/exe").parent_path().parent_path().parent_path()
//      / "cari_motion_planning/graph_core/tests/logger_param.yaml";

//  // Create the logger
//  cnr_logger::TraceLoggerPtr logger=std::make_shared<cnr_logger::TraceLogger>("graph_test", file_path.string());

//  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::WHITE() << " --- Create random configurations ---");

//  Eigen::VectorXd q1(6); q1.setRandom();
//  Eigen::VectorXd q2(6); q2.setRandom();

//  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::GREEN() <<"q1 "<<q1.transpose());
//  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::GREEN() <<"q2 "<<q2.transpose());

//  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::BOLDGREEN() << "Done!");

//  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::WHITE() << "--- Create nodes on these configurations ---");

//  pathplan::NodePtr parent = std::make_shared<pathplan::Node>(q1,logger);
//  pathplan::NodePtr child  = std::make_shared<pathplan::Node>(q2,logger);

//  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::BOLDGREEN() << "Done!");

//  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::WHITE() << "--- Display the nodes ---");

//  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::BLUE() << "parent: \n" << *parent);
//  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::CYAN() << "child:  \n" << *child);

//  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::BOLDGREEN() << "Done!");

//  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::WHITE() << "--- Create and add a connection between the nodes ---");
//  pathplan::ConnectionPtr connection = std::make_shared<pathplan::Connection>(parent,child,logger);
//  double cost = (parent->getConfiguration()-child->getConfiguration()).norm();
//  connection->setCost(cost);
//  connection->add();

//  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::BOLDGREEN() << "Done!");

//  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::WHITE() << "--- Display the connection ---");
//  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::YELLOW() << "connection: \n"<<*connection);

//  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::WHITE() << "--- Check nodes connections ---");
//  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::BLUE() << "parent: \n" << *parent);
//  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::CYAN() << "child:  \n" << *child);

//  bool valid = ((parent->getParentConnectionsSize()     == 0) && (parent->getChildConnectionsSize()    == 1) &&
//                (parent->getNetParentConnectionsSize()) == 0  && (parent->getNetChildConnectionsSize() == 0) &&
//                (child ->getParentConnectionsSize()     == 1) && (child->getChildConnectionsSize()     == 0) &&
//                (child->getNetParentConnectionsSize())  == 0  && (child->getNetChildConnectionsSize()  == 0));
//  if(not valid)
//  {
//    CNR_FATAL(logger,"something went wrong with nodes connections");
//    throw std::runtime_error("something went wrong with nodes connections");
//  }

//  CNR_INFO(logger, cnr_logger::RESET() << cnr_logger::BOLDGREEN() << "Done!");

  return 0;
}
