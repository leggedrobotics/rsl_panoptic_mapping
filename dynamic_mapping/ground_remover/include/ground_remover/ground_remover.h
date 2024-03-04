#ifndef GROUND_REMOVER_H
#define GROUND_REMOVER_H

#include "typedefs.hpp"

namespace ground_remover {
class GroundRemover {
 public:
  virtual void removeGround(PointCloud& in, PointCloud& out) = 0;
};
}  // namespace ground_remover
#endif