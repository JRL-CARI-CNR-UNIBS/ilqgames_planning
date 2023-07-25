#include <ilqgames_planning/point_player_3d.h>
#include <ilqgames/utils/types.h>

namespace ilqgames_planning {

// Constexprs for state indices.
const ilqgames::Dimension PointPlayer3D::kNumXDims = 6;
const ilqgames::Dimension PointPlayer3D::kPxIdx = 0;
const ilqgames::Dimension PointPlayer3D::kVxIdx = 1;
const ilqgames::Dimension PointPlayer3D::kPyIdx = 2;
const ilqgames::Dimension PointPlayer3D::kVyIdx = 3;
const ilqgames::Dimension PointPlayer3D::kPzIdx = 4;
const ilqgames::Dimension PointPlayer3D::kVzIdx = 5;

// Constexprs for control indices.
const ilqgames::Dimension PointPlayer3D::kNumUDims = 3;
const ilqgames::Dimension PointPlayer3D::kAxIdx = 0;
const ilqgames::Dimension PointPlayer3D::kAyIdx = 1;
const ilqgames::Dimension PointPlayer3D::kAzIdx = 2;

}  // namespace ilqgames_planning
