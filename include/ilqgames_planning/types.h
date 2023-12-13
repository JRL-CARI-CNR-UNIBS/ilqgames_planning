#ifndef ILQGAMES_PLANNING_TYPES_H
#define ILQGAMES_PLANNING_TYPES_H


#include <Eigen/StdVector>

// This macro checks the verbosity level before logging the message at WARNING level
#define VLOG_WARNING(verbose_level) \
    if (VLOG_IS_ON(verbose_level)) \
        LOG(WARNING)

// This macro checks the verbosity level before logging the message at ERROR level
#define VLOG_ERROR(verbose_level) \
    if (VLOG_IS_ON(verbose_level)) \
        LOG(ERROR)


namespace ilqgames_planning {

// ------------------------ THIRD PARTY TYPEDEFS ---------------------------- //

using Eigen::MatrixXf;
using Eigen::VectorXf;

// --------------------------------- TYPES ---------------------------------- //

using Point3 = Eigen::Vector3f;

#ifdef __APPLE__
using PointList3 = std::vector<Point3, Eigen::aligned_allocator<Point3>>;
using Time = float;
#else
using PointList3 = std::vector<Point3>;
using Time = double;
#endif

}  // namespace ilqgames_planning

#endif
