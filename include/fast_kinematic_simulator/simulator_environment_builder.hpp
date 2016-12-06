#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <functional>
#include <chrono>
#include <random>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/voxel_grid.hpp>
#include <sdf_tools/tagged_object_collision_map.hpp>
#include <sdf_tools/sdf.hpp>
#include <uncertainty_planning_core/simple_simulator_interface.hpp>

#ifndef SIMULATOR_ENVIRONMENT_BUILDER_HPP
#define SIMULATOR_ENVIRONMENT_BUILDER_HPP

namespace simulator_environment_builder
{
    struct OBSTACLE_CONFIG
    {
        Eigen::Affine3d pose;
        Eigen::Vector3d extents;
        uint32_t object_id;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        OBSTACLE_CONFIG(const uint32_t in_object_id, const Eigen::Affine3d& in_pose, const Eigen::Vector3d& in_extents) : pose(in_pose), extents(in_extents)
        {
            assert(in_object_id > 0);
            object_id = in_object_id;
        }

        OBSTACLE_CONFIG(const uint32_t in_object_id, const Eigen::Vector3d& in_translation, const Eigen::Quaterniond& in_orientation, const Eigen::Vector3d& in_extents)
        {
            assert(in_object_id > 0);
            object_id = in_object_id;
            pose = (Eigen::Translation3d)in_translation * in_orientation;
            extents = in_extents;
        }

        OBSTACLE_CONFIG() : pose(Eigen::Affine3d::Identity()), extents(0.0, 0.0, 0.0), object_id(0u) {}
    };

    struct RawCellSurfaceNormal
    {
        Eigen::Vector3d normal;
        Eigen::Vector3d entry_direction;

        RawCellSurfaceNormal(const Eigen::Vector3d& in_normal, const Eigen::Vector3d& in_direction) : normal(in_normal), entry_direction(in_direction) {}

        RawCellSurfaceNormal() : normal(Eigen::Vector3d(0.0, 0.0, 0.0)), entry_direction(Eigen::Vector3d(0.0, 0.0, 0.0)) {}
    };

    /* Discretize a cuboid obstacle to resolution-sized cells */
    std::vector<std::pair<Eigen::Vector3d, sdf_tools::TAGGED_OBJECT_COLLISION_CELL>> DiscretizeObstacle(const OBSTACLE_CONFIG& obstacle, const double resolution);

    /* Build certain special case environments */
    sdf_tools::TaggedObjectCollisionMapGrid BuildEnvironment(const std::string& environment_id, const double resolution);

    /* Build a new environment from the provided obstacles */
    sdf_tools::TaggedObjectCollisionMapGrid BuildEnvironment(const std::vector<OBSTACLE_CONFIG>& obstacles, const double resolution);

    void UpdateSurfaceNormalGridCell(const std::vector<RawCellSurfaceNormal>& raw_surface_normals, const Eigen::Affine3d& transform, const Eigen::Vector3d& cell_location, const sdf_tools::SignedDistanceField& environment_sdf, simple_simulator_interface::SurfaceNormalGrid& surface_normals_grid);

    void AdjustSurfaceNormalGridForAllFlatSurfaces(const sdf_tools::SignedDistanceField& environment_sdf, simple_simulator_interface::SurfaceNormalGrid& surface_normals_grid);

    simple_simulator_interface::SurfaceNormalGrid BuildSurfaceNormalsGrid(const std::string& environment_id, const sdf_tools::SignedDistanceField& environment_sdf);

    simple_simulator_interface::SurfaceNormalGrid BuildSurfaceNormalsGrid(const std::vector<OBSTACLE_CONFIG>& obstacles, const sdf_tools::SignedDistanceField& environment_sdf);

    class EnvironmentComponents
    {
    private:

        sdf_tools::TaggedObjectCollisionMapGrid environment_;
        sdf_tools::SignedDistanceField environment_sdf_;
        simple_simulator_interface::SurfaceNormalGrid surface_normals_grid_;

    public:

        EnvironmentComponents(const sdf_tools::TaggedObjectCollisionMapGrid& environment, const sdf_tools::SignedDistanceField& environment_sdf, const simple_simulator_interface::SurfaceNormalGrid& surface_normals_grid) : environment_(environment), environment_sdf_(environment_sdf), surface_normals_grid_(surface_normals_grid) {}

        inline const sdf_tools::TaggedObjectCollisionMapGrid& GetEnvironment()
        {
            return environment_;
        }

        inline const sdf_tools::SignedDistanceField& GetEnvironmentSDF()
        {
            return environment_sdf_;
        }

        inline const simple_simulator_interface::SurfaceNormalGrid& GetSurfaceNormalsGrid()
        {
            return surface_normals_grid_;
        }
    };

    EnvironmentComponents BuildCompleteEnvironment(const std::vector<OBSTACLE_CONFIG>& obstacles, const double resolution);

    EnvironmentComponents BuildCompleteEnvironment(const std::string& environment_id, const double resolution);
}

#endif // SIMULATOR_ENVIRONMENT_BUILDER_HPP
