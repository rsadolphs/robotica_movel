#ifndef POTENTIAL_HPP
#define POTENTIAL_HPP

#include <vector>
#include "Mapping.hpp"

namespace Potential {

struct FieldState {
  bool valid = false;
  int minX = 0;
  int minY = 0;
  int maxX = 0;
  int maxY = 0;
  int width = 0;
  int height = 0;
  std::vector<float> values;
  std::vector<char> active;
};

struct TargetYaw {
  bool valid = false;
  float yaw = 0.0f;
};

void updateRobotPose(float x, float y, float theta);
TargetYaw getLatestTargetYaw();
FieldState getLatestFieldState();
void* potentialThreadFunction(void* arg);

} // namespace Potential

#endif // POTENTIAL_HPP
