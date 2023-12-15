#include <graph_core/graph/connection.h>
#include <graph_core/graph/node.h>

int main(int argc, char **argv)
{
  Eigen::VectorXd q1(6); q1.setRandom();
  Eigen::VectorXd q2(6); q2.setRandom();

  std::cout <<"q1 "<<q1.transpose() << std::endl;
  std::cout <<"q2 "<<q2.transpose() << std::endl;

  cnr_logger::TraceLoggerPtr logger=std::make_shared<cnr_logger::TraceLogger>("graph_test",
                                                                              "/home/jacobi/projects/cari_motion_planning/cari_motion_planning/logger.yaml");

  CNR_INFO(logger, cnr_logger::RED() << "prova");
  graph_core::NodePtr parent = std::make_shared<graph_core::Node>(q1,logger);
  graph_core::NodePtr child  = std::make_shared<graph_core::Node>(q2,logger);

  std::cout << "parent "<<*parent << std::endl;
  std::cout << "child "<<*child << std::endl;

  graph_core::ConnectionPtr conn = std::make_shared<graph_core::Connection>(parent,child,logger);
  conn->add();
  std::cout << "conn "<<*conn << std::endl;

  return 0;
}
