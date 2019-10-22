#ifndef particle_1107
#define particle_1107

#include <net_path_core/net_path_core.h>

namespace ha_planner
{

class Particle: public std::enable_shared_from_this<ha_planner::Connection>
{
protected:
  const ParticleParam& m_params;
  const NodeParams& m_node_parameters;
  const ConnectionParam& m_connection_parameters;
  Path m_path;
  Path m_next_path;
  Path m_best_local_path;
  std::vector<std::vector<double>> m_velocity;
  double m_local_best;

  std::random_device m_rd;
  std::mt19937 m_gen;
  std::uniform_real_distribution<double> m_ud;

public:
  Particle(const Path& path, const double& cost,   const ParticleParam& params, const NodeParams& node_parameters, const ConnectionParam& connection_parameters);
  Path computeNextPath(const Path& best_path);
  void storePath(const Path& path, const double& cost);
};


}

#endif
