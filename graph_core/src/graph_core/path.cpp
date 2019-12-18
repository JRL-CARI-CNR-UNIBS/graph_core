#include <graph_core/path.h>


namespace ha_planner
{
  Path::Path(const std::vector<ConnectionPtr> connections)
  {
    m_connections=connections;
  }

}
