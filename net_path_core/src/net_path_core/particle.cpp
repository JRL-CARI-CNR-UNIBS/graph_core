#include <net_path_core/particle.h>

namespace ha_planner
{

Particle::Particle(const Path &path,
                   const double& cost,
                   const ParticleParam &params,
                   const NodeParams& node_parameters,
                   const ConnectionParam& connection_parameters):
  m_node_parameters(node_parameters),
  m_connection_parameters(connection_parameters),
  m_params(params),
  m_path(path),
  m_next_path(path),
  m_local_best(cost)
{
  m_ud=std::uniform_real_distribution<double>(0,1);

  m_velocity.resize(path.size()-1);
  for (unsigned int idx=0;idx<m_velocity.size();idx++)
  {
    m_velocity.at(idx).resize(m_params.dof,0);
  }

}

Path Particle::computeNextPath(const Path &best_path)
{
//  double g_g=m_params.c_g*m_ud(m_gen);
//  double g_l=m_params.c_l*m_ud(m_gen);

//  NodePtr parent=m_path.at(0)->getParent();
//  for (unsigned int idx=0;idx<m_velocity.size();idx++)
//  {
//    std::vector<double> jnts(m_params.dof);
//    for (unsigned int idof=0;idof<m_params.dof;idof++)
//    {
//      m_velocity.at(idx).at(idof)=g_l*(m_best_local_path->getParent()->getJoints()->getParent()->getJoints().at(idof)-m_path.at(idx)->getParent()->getJoints().at(idof))+
//                                   g_g*(best_path->getParent()->getJoints().at(idof)-m_path.at(idx)->getParent()->getJoints().at(idof))+
//                                   m_params.w*m_velocity.at(idx).at(idof);
//      jnts.at(idof)=m_path->getParent()->getJoints().at(idx).at(idof)+m_velocity.at(idx).at(idof);
//    }
//    NodePtr child=std::make_shared<Node>(jnts,m_node_parameters,m_connection_parameters);
//    m_next_path.at(idx)=std::make_shared<Connection>(parent,child,m_connection_parameters);


//// crea nodo, crea connessione
//  }
  return m_next_path;
}
}
