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
#include <fast_kinematic_simulator/simulator_environment_builder.hpp>

/* Discretize a cuboid obstacle to resolution-sized cells */
std::vector<std::pair<Eigen::Vector3d, sdf_tools::TAGGED_OBJECT_COLLISION_CELL>> simulator_environment_builder::DiscretizeObstacle(const OBSTACLE_CONFIG& obstacle, const double resolution)
{
    const double effective_resolution = resolution * 0.5;
    std::vector<std::pair<Eigen::Vector3d, sdf_tools::TAGGED_OBJECT_COLLISION_CELL>> cells;
    // Make the cell for the object
    sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0, obstacle.object_id);
    // Generate all cells for the object
    int32_t x_cells = (int32_t)(obstacle.extents.x() * 2.0 * (1.0 / effective_resolution));
    int32_t y_cells = (int32_t)(obstacle.extents.y() * 2.0 * (1.0 / effective_resolution));
    int32_t z_cells = (int32_t)(obstacle.extents.z() * 2.0 * (1.0 / effective_resolution));
    for (int32_t xidx = 0; xidx < x_cells; xidx++)
    {
        for (int32_t yidx = 0; yidx < y_cells; yidx++)
        {
            for (int32_t zidx = 0; zidx < z_cells; zidx++)
            {
                double x_location = -(obstacle.extents.x() - (resolution * 0.5)) + (effective_resolution * xidx);
                double y_location = -(obstacle.extents.y() - (resolution * 0.5)) + (effective_resolution * yidx);
                double z_location = -(obstacle.extents.z() - (resolution * 0.5)) + (effective_resolution * zidx);
                Eigen::Vector3d cell_location(x_location, y_location, z_location);
                cells.push_back(std::pair<Eigen::Vector3d, sdf_tools::TAGGED_OBJECT_COLLISION_CELL>(cell_location, object_cell));
            }
        }
    }
    return cells;
}

/* Build certain special case environments */
sdf_tools::TaggedObjectCollisionMapGrid simulator_environment_builder::BuildEnvironment(const std::string& environment_id, const double resolution)
{
    std::cout << "Generating the " << environment_id << " environment" << std::endl;
    if (environment_id == "se3_path_metric_test")
    {
        double grid_x_size = 10.0;
        double grid_y_size = 10.0;
        double grid_z_size = 10.0;
        // The grid origin is the minimum point, with identity rotation
        Eigen::Translation3d grid_origin_translation(0.0, 0.0, 0.0);
        Eigen::Quaterniond grid_origin_rotation = Eigen::Quaterniond::Identity();
        Eigen::Affine3d grid_origin_transform = grid_origin_translation * grid_origin_rotation;
        // Make the grid
        sdf_tools::TAGGED_OBJECT_COLLISION_CELL default_cell(1.0, 1u, 0u, 0u); // Everything is filled by default
        sdf_tools::TaggedObjectCollisionMapGrid grid(grid_origin_transform, "uncertainty_planning_simulator", resolution, grid_x_size, grid_y_size, grid_z_size, default_cell);
        for (int64_t x_idx = 0; x_idx < grid.GetNumXCells(); x_idx++)
        {
            for (int64_t y_idx = 0; y_idx < grid.GetNumYCells(); y_idx++)
            {
                for (int64_t z_idx = 0; z_idx < grid.GetNumZCells(); z_idx++)
                {
                    const Eigen::Vector3d location = EigenHelpers::StdVectorDoubleToEigenVector3d(grid.GridIndexToLocation(x_idx, y_idx, z_idx));
                    const double& x = location.x();
                    const double& y = location.y();
                    const double& z = location.z();
                    // Check if the cell belongs to any of the regions of freespace
                    if (x > 0.5 && y > 0.5 && x <= 9.5 && y <= 9.5 && z > 0.5 && z < 4.5)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(1u);
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                    }
                    if (x > 0.5 && y > 0.5 && x <= 9.5 && y <= 9.5 && z > 5.5 && z < 9.5)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(2u);
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                    }
                    if (x > 4.75 && y > 4.75 && x <= 5.25 && y <= 5.25 && z > 0.5 && z < 9.5)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(3u);
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                    }
                    if (x <= 0.5 || x > 9.5 || y <= 0.5 || y > 9.5 || z <= 0.5 || z > 9.5)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                    }
                }
            }
        }
        return grid;
    }
    else if (environment_id == "se2_path_metric_test")
    {
        double grid_x_size = 10.0;
        double grid_y_size = 10.0;
        double grid_z_size = 1.0;
        // The grid origin is the minimum point, with identity rotation
        Eigen::Translation3d grid_origin_translation(0.0, 0.0, -0.5);
        Eigen::Quaterniond grid_origin_rotation = Eigen::Quaterniond::Identity();
        Eigen::Affine3d grid_origin_transform = grid_origin_translation * grid_origin_rotation;
        // Make the grid
        sdf_tools::TAGGED_OBJECT_COLLISION_CELL default_cell(1.0, 1u, 0u, 0u); // Everything is filled by default
        sdf_tools::TaggedObjectCollisionMapGrid grid(grid_origin_transform, "uncertainty_planning_simulator", resolution, grid_x_size, grid_y_size, grid_z_size, default_cell);
        for (int64_t x_idx = 0; x_idx < grid.GetNumXCells(); x_idx++)
        {
            for (int64_t y_idx = 0; y_idx < grid.GetNumYCells(); y_idx++)
            {
                for (int64_t z_idx = 0; z_idx < grid.GetNumZCells(); z_idx++)
                {
                    const Eigen::Vector3d location = EigenHelpers::StdVectorDoubleToEigenVector3d(grid.GridIndexToLocation(x_idx, y_idx, z_idx));
                    const double& x = location.x();
                    const double& y = location.y();
                    //const double& z = location.z();
                    // Check if the cell belongs to any of the regions of freespace
                    if (x > 0.5 && y > 0.5 && x <= 4.5 && y <= 9.5)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(1u);
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                    }
                    if (x > 5.5 && y > 0.5 && x <= 9.5 && y <= 9.5)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(2u);
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                    }
                    if (x > 0.5 && y > 4.0 && x <= 9.5 && y <= 6.0)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(3u);
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                    }
                }
            }
        }
        return grid;
    }
    else if (environment_id == "nested_corners")
    {
        double grid_x_size = 10.0;
        double grid_y_size = 10.0;
        double grid_z_size = 10.0;
        // The grid origin is the minimum point, with identity rotation
        Eigen::Translation3d grid_origin_translation(0.0, 0.0, 0.0);
        Eigen::Quaterniond grid_origin_rotation = Eigen::Quaterniond::Identity();
        Eigen::Affine3d grid_origin_transform = grid_origin_translation * grid_origin_rotation;
        // Make the grid
        sdf_tools::TAGGED_OBJECT_COLLISION_CELL default_cell;
        sdf_tools::TaggedObjectCollisionMapGrid grid(grid_origin_transform, "uncertainty_planning_simulator", resolution, grid_x_size, grid_y_size, grid_z_size, default_cell);
        for (int64_t x_idx = 0; x_idx < grid.GetNumXCells(); x_idx++)
        {
            for (int64_t y_idx = 0; y_idx < grid.GetNumYCells(); y_idx++)
            {
                for (int64_t z_idx = 0; z_idx < grid.GetNumZCells(); z_idx++)
                {
                    const Eigen::Vector3d location = EigenHelpers::StdVectorDoubleToEigenVector3d(grid.GridIndexToLocation(x_idx, y_idx, z_idx));
                    const double& x = location.x();
                    const double& y = location.y();
                    const double& z = location.z();
                    // Set the object we belong to
                    // We assume that all objects are convex, so we can set the convex region as 1
                    // "Bottom bottom"
                    if (x > 1.0 && x <= 9.0 && y > 1.0 && y <= 9.0 && z > 1.0 && z<= 1.5)
                    {
                        const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0, 1u, 0u, 1u);
                        grid.Set(x_idx, y_idx, z_idx, object_cell);
                    }
                    // "Right right"
                    if (x > 1.0 && x <= 9.0 && y > 1.0 && y <= 1.5 && z > 1.0 && z<= 9.0)
                    {
                        const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0, 2u, 0u, 1u);
                        grid.Set(x_idx, y_idx, z_idx, object_cell);
                    }
                    // "Back back"
                    if (x > 1.0 && x <= 1.5 && y > 1.0 && y <= 9.0 && z > 1.0 && z<= 9.0)
                    {
                        const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0, 3u, 0u, 1u);
                        grid.Set(x_idx, y_idx, z_idx, object_cell);
                    }
                    // "Top bottom"
                    if (x > 2.0 && x <= 7.0 && y > 2.0 && y <= 7.0 && z > 2.0 && z<= 2.5)
                    {
                        const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0, 4u, 0u, 1u);
                        grid.Set(x_idx, y_idx, z_idx, object_cell);
                    }
                    // "Left right"
                    if (x > 2.0 && x <= 7.0 && y > 2.0 && y <= 2.5 && z > 2.0 && z<= 7.0)
                    {
                        const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0, 5u, 0u, 1u);
                        grid.Set(x_idx, y_idx, z_idx, object_cell);
                    }
                    // "Front back"
                    if (x > 2.0 && x <= 2.5 && y > 2.0 && y <= 7.0 && z > 2.0 && z<= 7.0)
                    {
                        const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0, 6u, 0u, 1u);
                        grid.Set(x_idx, y_idx, z_idx, object_cell);
                    }
                    // Set the free-space convex segment we belong to (if necessary)
                    if (grid.GetImmutable(x_idx, y_idx, z_idx).first.occupancy < 0.5)
                    {
                        // There are 13 convex regions, we can belong to multiple regions, so we check for each
                        // First three
                        if (x <= 1.0)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(1u);
                        }
                        if (y <= 1.0)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(2u);
                        }
                        if (z <= 1.0)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(3u);
                        }
                        // Second three
                        if ((x > 1.5) && (x <= 2.0) && (y > 1.5) && (z > 1.5))
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(4u);
                        }
                        if ((x > 1.5) && (y > 1.5) && (y <= 2.0) && (z > 1.5))
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(5u);
                        }
                        if ((x > 1.5) && (y > 1.5) && (z > 1.5) && (z <= 2.0))
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(6u);
                        }
                        // Third three
                        if (x > 9.0)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(7u);
                        }
                        if (y > 9.0)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(8u);
                        }
                        if (z > 9.0)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(9u);
                        }
                        // Fourth three
                        if ((x > 7.0) && (y > 1.5) && (z > 1.5))
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(10u);
                        }
                        if ((x > 1.5) && (y > 7.0) && (z > 1.5))
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(11u);
                        }
                        if ((x > 1.5) && (y > 1.5) && (z > 7.0))
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(12u);
                        }
                        // Last one
                        if ((x > 2.5) && (y > 2.5) && (z > 2.5))
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(13u);
                        }
                    }
                }
            }
        }
        return grid;
    }
    else if (environment_id == "peg_in_hole")
    {
        double grid_x_size = 14.0;
        double grid_y_size = 14.0;
        double grid_z_size = 14.0;
        // The grid origin is the minimum point, with identity rotation
        Eigen::Translation3d grid_origin_translation(-2.0, -2.0, -2.0);
        Eigen::Quaterniond grid_origin_rotation = Eigen::Quaterniond::Identity();
        Eigen::Affine3d grid_origin_transform = grid_origin_translation * grid_origin_rotation;
        // Make the grid
        sdf_tools::TAGGED_OBJECT_COLLISION_CELL default_cell(1.0f, 0u, 0u, 0u);
        sdf_tools::TaggedObjectCollisionMapGrid grid(grid_origin_transform, "uncertainty_planning_simulator", resolution, grid_x_size, grid_y_size, grid_z_size, default_cell);
        for (int64_t x_idx = 0; x_idx < grid.GetNumXCells(); x_idx++)
        {
            for (int64_t y_idx = 0; y_idx < grid.GetNumYCells(); y_idx++)
            {
                for (int64_t z_idx = 0; z_idx < grid.GetNumZCells(); z_idx++)
                {
                    const Eigen::Vector3d location = EigenHelpers::StdVectorDoubleToEigenVector3d(grid.GridIndexToLocation(x_idx, y_idx, z_idx));
                    const double& x = location.x();
                    const double& y = location.y();
                    const double& z = location.z();
                    // Set the object we belong to
                    // We assume that all objects are convex, so we can set the convex region as 1
                    // "Bottom bottom"
                    if (x_idx <= 16 || y_idx <= 16 || z_idx <= 16 || x_idx >= (grid.GetNumXCells() - 16)  || y_idx >= (grid.GetNumYCells() - 16) || z_idx >= (grid.GetNumZCells() - 16))
                    {
                        const sdf_tools::TAGGED_OBJECT_COLLISION_CELL buffer_cell(1.0f, 0u, 0u, 0u);
                        grid.Set(x_idx, y_idx, z_idx, buffer_cell);
                    }
                    else
                    {
                        if (z <= 3.0)
                        {
                            if (x <= 2.0 || y <= 2.0)
                            {
                                const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0, 1u, 0u, 1u);
                                grid.Set(x_idx, y_idx, z_idx, object_cell);
                            }
                            else if (x > 2.5 || y > 2.5)
                            {
                                const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0, 1u, 0u, 1u);
                                grid.Set(x_idx, y_idx, z_idx, object_cell);
                            }
                            else if (z <= resolution)
                            {
                                const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0, 1u, 0u, 1u);
                                grid.Set(x_idx, y_idx, z_idx, object_cell);
                            }
                            else
                            {
                                const sdf_tools::TAGGED_OBJECT_COLLISION_CELL free_cell(0.0f, 0u, 0u, 0u);
                                grid.Set(x_idx, y_idx, z_idx, free_cell);
                            }
                        }
                        else
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL free_cell(0.0f, 0u, 0u, 0u);
                            grid.Set(x_idx, y_idx, z_idx, free_cell);
                        }
                    }
                    // Set the free-space convex segment we belong to (if necessary)
                    if (grid.GetImmutable(x_idx, y_idx, z_idx).first.occupancy < 0.5)
                    {
                        // There are 2 convex regions, we can belong to multiple regions, so we check for each
                        if (z > 3.0)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(1u);
                        }
                        if (x > 2.0 && y > 2.0 && x <= 2.5 && y <= 2.5 && z > resolution)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(2u);
                        }
                    }
                }
            }
        }
        return grid;
    }
    else if (environment_id == "se3_cluttered")
    {
        double grid_x_size = 12.0;
        double grid_y_size = 12.0;
        double grid_z_size = 12.0;
        // The grid origin is the minimum point, with identity rotation
        Eigen::Translation3d grid_origin_translation(-1.0, -1.0, -1.0);
        Eigen::Quaterniond grid_origin_rotation = Eigen::Quaterniond::Identity();
        Eigen::Affine3d grid_origin_transform = grid_origin_translation * grid_origin_rotation;
        // Make the grid
        sdf_tools::TAGGED_OBJECT_COLLISION_CELL default_cell(0.0f, 0u, 0u, 0u);
        sdf_tools::TaggedObjectCollisionMapGrid grid(grid_origin_transform, "uncertainty_planning_simulator", resolution, grid_x_size, grid_y_size, grid_z_size, default_cell);
        for (int64_t x_idx = 0; x_idx < grid.GetNumXCells(); x_idx++)
        {
            for (int64_t y_idx = 0; y_idx < grid.GetNumYCells(); y_idx++)
            {
                for (int64_t z_idx = 0; z_idx < grid.GetNumZCells(); z_idx++)
                {
                    const Eigen::Vector3d location = EigenHelpers::StdVectorDoubleToEigenVector3d(grid.GridIndexToLocation(x_idx, y_idx, z_idx));
                    const double& x = location.x();
                    const double& y = location.y();
                    //const double& z = location.z();
                    // Make some of the exterior walls opaque
                    if (x_idx <= 8 || y_idx <= 8 || z_idx <= 8 || x_idx >= (grid.GetNumXCells() - 8) || y_idx >= (grid.GetNumYCells() - 8))
                    {
                        const sdf_tools::TAGGED_OBJECT_COLLISION_CELL buffer_cell(1.0f, 1u, 0u, 0u);
                        grid.Set(x_idx, y_idx, z_idx, buffer_cell);
                    }
                    else if (z_idx >= (grid.GetNumZCells() - 8))
                    {
                        const sdf_tools::TAGGED_OBJECT_COLLISION_CELL buffer_cell(1.0f, 0u, 0u, 0u);
                        grid.Set(x_idx, y_idx, z_idx, buffer_cell);
                    }
                    else
                    {
                        // Set the interior 10x10x10 meter area
                        if (x > 1.0 && x <= 3.0 && y > 1.0 && y <= 3.0)
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0f, 7u, 0u, 0u);
                            grid.Set(x_idx, y_idx, z_idx, object_cell);
                        }
                        else if (x > 5.0 && x <= 8.0 && y > 0.0 && y <= 2.0)
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0f, 7u, 0u, 0u);
                            grid.Set(x_idx, y_idx, z_idx, object_cell);
                        }
                        else if (x > 4.0 && x <= 9.0 && y > 3.0 && y <= 5.0)
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0f, 7u, 0u, 0u);
                            grid.Set(x_idx, y_idx, z_idx, object_cell);
                        }
                        else if (x > 0.0 && x <= 2.0 && y > 4.0 && y <= 7.0)
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0f, 7u, 0u, 0u);
                            grid.Set(x_idx, y_idx, z_idx, object_cell);
                        }
                        else if (x > 3.0 && x <= 6.0&& y > 6.0 && y <= 8.0)
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0f, 7u, 0u, 0u);
                            grid.Set(x_idx, y_idx, z_idx, object_cell);
                        }
                        else if (x > 0.0 && x <= 5.0 && y > 9.0 && y <= 10.0)
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0f, 7u, 0u, 0u);
                            grid.Set(x_idx, y_idx, z_idx, object_cell);
                        }
                        else if (x > 8.0 && x <= 10.0 && y > 6.0 && y <= 10.0)
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0f, 7u, 0u, 0u);
                            grid.Set(x_idx, y_idx, z_idx, object_cell);
                        }
                        else
                        {
                            // Set the convex regions
                            if (x > 0.0 && x <= 5.0 && y > 0.0 && y <= 1.0)
                            {
                                grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(1u);
                            }
                            if (x > 0.0 && x <= 1.0 && y > 0.0 && y <= 4.0)
                            {
                                grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(2u);
                            }
                            if (x > 0.0 && x <= 4.0 && y > 3.0 && y <= 4.0)
                            {
                                grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(3u);
                            }
                            if (x > 3.0 && x <= 4.0 && y > 0.0 && y <= 6.0)
                            {
                                grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(4u);
                            }
                            if (x > 3.0 && x <= 5.0 && y > 0.0 && y <= 3.0)
                            {
                                grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(5u);
                            }
                            if (x > 3.0 && x <= 10.0 && y > 2.0 && y <= 3.0)
                            {
                                grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(6u);
                            }
                            if (x > 8.0 && x <= 10.0 && y > 0.0 && y <= 3.0)
                            {
                                grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(7u);
                            }
                            if (x > 9.0 && x <= 10.0 && y > 0.0 && y <= 6.0)
                            {
                                grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(8u);
                            }
                            if (x > 2.0 && x <= 10.0 && y > 5.0 && y <= 6.0)
                            {
                                grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(9u);
                            }
                            if (x > 2.0 && x <= 3.0 && y > 3.0 && y <= 9.0)
                            {
                                grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(10u);
                            }
                            if (x > 6.0 && x <= 8.0 && y > 5.0 && y <= 10.0)
                            {
                                grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(11u);
                            }
                            if (x > 0.0 && x <= 3.0 && y > 7.0 && y <= 9.0)
                            {
                                grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(12u);
                            }
                            if (x > 5.0 && x <= 8.0 && y > 8.0 && y <= 10.0)
                            {
                                grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(13u);
                            }
                            if (x > 0.0 && x <= 8.0 && y > 8.0 && y <= 9.0)
                            {
                                grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(14u);
                            }
                            if (x > 2.0 && x <= 4.0 && y > 3.0 && y <= 6.0)
                            {
                                grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(15u);
                            }
                        }
                    }
                }
            }
        }
        return grid;
    }
    else if (environment_id == "se3_cluttered_new")
    {
        double grid_x_size = 12.0;
        double grid_y_size = 12.0;
        double grid_z_size = 6.0;
        // The grid origin is the minimum point, with identity rotation
        Eigen::Translation3d grid_origin_translation(-1.0, -1.0, -1.0);
        Eigen::Quaterniond grid_origin_rotation = Eigen::Quaterniond::Identity();
        Eigen::Affine3d grid_origin_transform = grid_origin_translation * grid_origin_rotation;
        // Make the grid
        sdf_tools::TAGGED_OBJECT_COLLISION_CELL default_cell(0.0f, 0u, 0u, 0u);
        sdf_tools::TaggedObjectCollisionMapGrid grid(grid_origin_transform, "uncertainty_planning_simulator", resolution, grid_x_size, grid_y_size, grid_z_size, default_cell);
        for (int64_t x_idx = 0; x_idx < grid.GetNumXCells(); x_idx++)
        {
            for (int64_t y_idx = 0; y_idx < grid.GetNumYCells(); y_idx++)
            {
                for (int64_t z_idx = 0; z_idx < grid.GetNumZCells(); z_idx++)
                {
                    const Eigen::Vector3d location = EigenHelpers::StdVectorDoubleToEigenVector3d(grid.GridIndexToLocation(x_idx, y_idx, z_idx));
                    const double& x = location.x();
                    const double& y = location.y();
                    const double& z = location.z();
                    // Make some of the exterior walls opaque
                    if (x_idx <= 8 || y_idx <= 8 || z_idx <= 8 || x_idx >= (grid.GetNumXCells() - 8) || y_idx >= (grid.GetNumYCells() - 8))
                    {
                        const sdf_tools::TAGGED_OBJECT_COLLISION_CELL buffer_cell(1.0f, 1u, 0u, 0u);
                        grid.Set(x_idx, y_idx, z_idx, buffer_cell);
                    }
                    // Transparent top
                    else if (z_idx >= (grid.GetNumZCells() - 8))
                    {
                        const sdf_tools::TAGGED_OBJECT_COLLISION_CELL buffer_cell(1.0f, 0u, 0u, 0u);
                        grid.Set(x_idx, y_idx, z_idx, buffer_cell);
                    }
                    else
                    {
                        // Set the interior 10x10x10 meter area
                        if (y > 2.0 && y <= 4.0)
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0f, 7u, 0u, 0u);
                            grid.Set(x_idx, y_idx, z_idx, object_cell);
                        }
                        else if (y > 6.0 && y <= 8.0)
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0f, 7u, 0u, 0u);
                            grid.Set(x_idx, y_idx, z_idx, object_cell);
                        }
                        // Main regions
                        if (y <= 2)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(1u);
                        }
                        if (y > 8.0)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(3u);
                        }
                        if (y > 4.0 && y <= 6.0)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(2u);
                        }
                        // Hole regions
                        if (z > 1.5 && z <= 3.5 && x > 1.5 && x <= 3.5 && y > 4.0)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(4u);
                        }
                        if (z > 1.5 && z <= 3.5 && x > 6.5 && x <= 8.5 && y <= 6.0)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(5u);
                        }
                    }
                }
            }
        }
        return grid;
    }
    else if (environment_id == "se3_box_puzzle")
    {
        double grid_x_size = 12.0;
        double grid_y_size = 12.0;
        double grid_z_size = 12.0;
        // The grid origin is the minimum point, with identity rotation
        Eigen::Translation3d grid_origin_translation(-1.0, -1.0, -1.0);
        Eigen::Quaterniond grid_origin_rotation = Eigen::Quaterniond::Identity();
        Eigen::Affine3d grid_origin_transform = grid_origin_translation * grid_origin_rotation;
        // Make the grid
        sdf_tools::TAGGED_OBJECT_COLLISION_CELL default_cell(0.0f, 0u, 0u, 0u);
        sdf_tools::TaggedObjectCollisionMapGrid grid(grid_origin_transform, "uncertainty_planning_simulator", resolution, grid_x_size, grid_y_size, grid_z_size, default_cell);
        for (int64_t x_idx = 0; x_idx < grid.GetNumXCells(); x_idx++)
        {
            for (int64_t y_idx = 0; y_idx < grid.GetNumYCells(); y_idx++)
            {
                for (int64_t z_idx = 0; z_idx < grid.GetNumZCells(); z_idx++)
                {
                    const Eigen::Vector3d location = EigenHelpers::StdVectorDoubleToEigenVector3d(grid.GridIndexToLocation(x_idx, y_idx, z_idx));
                    const double& x = location.x();
                    const double& y = location.y();
                    const double& z = location.z();
                    // Set the object we belong to
                    // Make some of the exterior walls opaque
                    if (x_idx <= 8 || y_idx <= 8 || z_idx <= 8 || x_idx >= (grid.GetNumXCells() - 8) || y_idx >= (grid.GetNumYCells() - 8) || z_idx >= (grid.GetNumZCells() - 8))
                    {
                        const sdf_tools::TAGGED_OBJECT_COLLISION_CELL buffer_cell(1.0f, 0u, 0u, 0u);
                        grid.Set(x_idx, y_idx, z_idx, buffer_cell);
                    }
                    else
                    {
                        // Make central planes(s)
                        if (x > 4.5 && x <= 5.5)
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0f, 1u, 0u, 0u);
                            grid.Set(x_idx, y_idx, z_idx, object_cell);
                        }
                        else if (y > 4.5 && y <= 5.5)
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0f, 2u, 0u, 0u);
                            grid.Set(x_idx, y_idx, z_idx, object_cell);
                        }
                        else if (z > 4.5 && z <= 5.5)
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0f, 3u, 0u, 0u);
                            grid.Set(x_idx, y_idx, z_idx, object_cell);
                        }
                        // Make the 8 interior voids
                        else if (x <= 4.5 && y <= 4.5 && z <= 4.5)
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(0.0f, 0u, 0u, 1u);
                            grid.Set(x_idx, y_idx, z_idx, object_cell);
                        }
                        else if (x <= 4.5 && y <= 4.5 && z > 5.5)
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(0.0f, 0u, 0u, 2u);
                            grid.Set(x_idx, y_idx, z_idx, object_cell);
                        }
                        else if (x <= 4.5 && y > 5.5 && z <= 4.5)
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(0.0f, 0u, 0u, 4u);
                            grid.Set(x_idx, y_idx, z_idx, object_cell);
                        }
                        else if (x <= 4.5 && y > 5.5 && z > 5.5)
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(0.0f, 0u, 0u, 8u);
                            grid.Set(x_idx, y_idx, z_idx, object_cell);
                        }
                        else if (x > 5.5 && y <= 4.5 && z <= 4.5)
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(0.0f, 0u, 0u, 16u);
                            grid.Set(x_idx, y_idx, z_idx, object_cell);
                        }
                        else if (x > 5.5 && y <= 4.5 && z > 5.5)
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(0.0f, 0u, 0u, 32u);
                            grid.Set(x_idx, y_idx, z_idx, object_cell);
                        }
                        else if (x > 5.5 && y > 5.5 && z <= 4.5)
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(0.0f, 0u, 0u, 64u);
                            grid.Set(x_idx, y_idx, z_idx, object_cell);
                        }
                        else if (x > 5.5 && y > 5.5 && z > 5.5)
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(0.0f, 0u, 0u, 128u);
                            grid.Set(x_idx, y_idx, z_idx, object_cell);
                        }
                        // Make the four x-axis oriented interior channels
                        if (y > 5.5 && y <= 7.0 && z > 5.5 && z <= 7.0)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(9u);
                        }
                        else if (y > 3.0 && y <= 4.5 && z > 5.5 && z <= 7.0)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(10u);
                        }
                        else if (y > 5.5 && y <= 7.0 && z > 3.0 && z <= 4.5)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(11u);
                        }
                        else if (y > 3.0 && y <= 4.5 && z > 3.0 && z <= 4.5)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(12u);
                        }
                        // Make the two z-axis oriented interior channels
                        if (y > 5.5 && y <= 7.0 && x > 5.5 && x <= 7.0)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(13u);
                        }
                        else if (y > 3.0 && y <= 4.5 && x > 5.5 && x <= 7.0)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(14u);
                        }
                        // Add the 1 y-axis oriented interior channel
                        if (x > 3.0 && x <= 4.5 && z > 3.0 && z <= 4.5)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(15u);
                        }
                    }
                }
            }
        }
        return grid;
    }
    else if (environment_id == "empty_baxter_env")
    {
        double grid_x_size = 3.0;
        double grid_y_size = 3.0;
        double grid_z_size = 3.0;
        // The grid origin is the minimum point, with identity rotation
        Eigen::Translation3d grid_origin_translation(-1.5, -2.0, -1.0);
        Eigen::Quaterniond grid_origin_rotation = Eigen::Quaterniond::Identity();
        Eigen::Affine3d grid_origin_transform = grid_origin_translation * grid_origin_rotation;
        // Make the grid
        sdf_tools::TAGGED_OBJECT_COLLISION_CELL default_cell(0.0f, 0u, 0u, 1u);
        sdf_tools::TaggedObjectCollisionMapGrid grid(grid_origin_transform, "uncertainty_planning_simulator", resolution, grid_x_size, grid_y_size, grid_z_size, default_cell);
        std::cout << "New environment with dimensions (" << grid.GetNumXCells() << ", " << grid.GetNumYCells() << ", " << grid.GetNumZCells() << ")" << std::endl;
        return grid;
    }
    else if (environment_id == "baxter_env")
    {
        double grid_x_size = 3.0;
        double grid_y_size = 3.0;
        double grid_z_size = 3.0;
        // The grid origin is the minimum point, with identity rotation
        Eigen::Translation3d grid_origin_translation(-1.5, -2.0, -1.0);
        Eigen::Quaterniond grid_origin_rotation = Eigen::Quaterniond::Identity();
        Eigen::Affine3d grid_origin_transform = grid_origin_translation * grid_origin_rotation;
        // Make the grid
        sdf_tools::TAGGED_OBJECT_COLLISION_CELL default_cell(1.0f, 1u, 0u, 0u);
        sdf_tools::TaggedObjectCollisionMapGrid grid(grid_origin_transform, "uncertainty_planning_simulator", resolution, grid_x_size, grid_y_size, grid_z_size, default_cell);
        std::cout << "New environment with dimensions (" << grid.GetNumXCells() << ", " << grid.GetNumYCells() << ", " << grid.GetNumZCells() << ")" << std::endl;
        for (int64_t x_idx = 0; x_idx < grid.GetNumXCells(); x_idx++)
        {
            for (int64_t y_idx = 0; y_idx < grid.GetNumYCells(); y_idx++)
            {
                for (int64_t z_idx = 0; z_idx < grid.GetNumZCells(); z_idx++)
                {
                    const Eigen::Vector3d location = EigenHelpers::StdVectorDoubleToEigenVector3d(grid.GridIndexToLocation(x_idx, y_idx, z_idx));
                    const double& x = location.x();
                    const double& y = location.y();
                    const double& z = location.z();
                    const double hole_center_x = 1.03;
                    const double hole_center_y = -0.40;
                    const double hole_center_z = 0.13;
                    const double hole_center_to_side_edge = 0.27305;
                    const double hole_center_to_top_edge = 0.3048;
                    const double min_y = hole_center_y - hole_center_to_side_edge;
                    const double max_y = hole_center_y + hole_center_to_side_edge;
                    const double min_z = hole_center_z - hole_center_to_top_edge;
                    const double max_z = hole_center_z + hole_center_to_top_edge;
                    if (z <= min_z)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(1u);
                    }
                    else if (z > max_z)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(2u);
                    }
                    if (y <= min_y)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(3u);
                    }
                    else if (y > max_y)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(4u);
                    }
                    if (x <= hole_center_x)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(5u);
                    }
                    // Make the hole
                    const double hole_width = 0.04;
                    const double hole_height = 0.04;
                    const double hole_max_y = hole_center_y + (hole_width * 0.5);
                    const double hole_min_y = hole_center_y - (hole_width * 0.5);
                    const double hole_max_z = hole_center_z + (hole_height * 0.5);
                    const double hole_min_z = hole_center_z - (hole_height * 0.5);
                    if (y <= hole_max_y && y > hole_min_y && z <= hole_max_z && z > hole_min_z)
                    {
                        if (x > hole_center_x)
                        {
                            std::cout << "Real hole X: " << x << " Y: " << y << " Z: " << z << std::endl;
                        }
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(6u);
                    }
                }
            }
        }
        //grid.Set(0.0, 0.0, -0.25, sdf_tools::TAGGED_OBJECT_COLLISION_CELL(1.0f, 1u));
        return grid;
    }
    else if (environment_id == "baxter_blocked_env")
    {
        double grid_x_size = 3.0;
        double grid_y_size = 3.0;
        double grid_z_size = 3.0;
        // The grid origin is the minimum point, with identity rotation
        Eigen::Translation3d grid_origin_translation(-1.5, -2.0, -1.0);
        Eigen::Quaterniond grid_origin_rotation = Eigen::Quaterniond::Identity();
        Eigen::Affine3d grid_origin_transform = grid_origin_translation * grid_origin_rotation;
        // Make the grid
        sdf_tools::TAGGED_OBJECT_COLLISION_CELL default_cell(1.0f, 0u, 0u, 0u);
        sdf_tools::TaggedObjectCollisionMapGrid grid(grid_origin_transform, "uncertainty_planning_simulator", resolution, grid_x_size, grid_y_size, grid_z_size, default_cell);
        std::cout << "New environment with dimensions (" << grid.GetNumXCells() << ", " << grid.GetNumYCells() << ", " << grid.GetNumZCells() << ")" << std::endl;
        for (int64_t x_idx = 0; x_idx < grid.GetNumXCells(); x_idx++)
        {
            for (int64_t y_idx = 0; y_idx < grid.GetNumYCells(); y_idx++)
            {
                for (int64_t z_idx = 0; z_idx < grid.GetNumZCells(); z_idx++)
                {
                    const Eigen::Vector3d location = EigenHelpers::StdVectorDoubleToEigenVector3d(grid.GridIndexToLocation(x_idx, y_idx, z_idx));
                    const double& x = location.x();
                    const double& y = location.y();
                    const double& z = location.z();
                    const double hole_center_x = 1.03;
                    const double hole_center_y = -0.40;
                    const double hole_center_z = 0.13;
                    // Make two obstructing blocks
                    const double left_block_depth = 4.0 * resolution;
                    const double right_block_depth = 2.0 * resolution;
                    const double left_block_width = 10.0 * resolution;
                    const double left_block_height = 10.0 * resolution;
                    const double right_block_width = 6.0 * resolution;
                    const double right_block_height = 4.0 * resolution;
                    const double left_block_min_x = hole_center_x - left_block_depth;
                    const double left_block_max_x = hole_center_x;
                    const double left_block_min_y = hole_center_y + (resolution * 0.0);
                    const double left_block_max_y = left_block_min_y + left_block_width;
                    const double left_block_min_z = hole_center_z - (resolution * 12.0);
                    const double left_block_max_z = left_block_min_z + left_block_height;
                    const double right_block_min_x = hole_center_x - right_block_depth;
                    const double right_block_max_x = hole_center_x;
                    const double right_block_min_y = hole_center_y - (resolution * 6.0);
                    const double right_block_max_y = right_block_min_y + right_block_width;
                    const double right_block_min_z = hole_center_z - (resolution * 7.0);
                    const double right_block_max_z = right_block_min_z + right_block_height;
                    if (x > left_block_min_x && x <= left_block_max_x && y > left_block_min_y && y <= left_block_max_y && z > left_block_min_z && z <= left_block_max_z)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 1.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 1u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.convex_segment = 0u;
                    }
                    else if (x > right_block_min_x && x <= right_block_max_x && y > right_block_min_y && y <= right_block_max_y && z > right_block_min_z && z <= right_block_max_z)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 1.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 1u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.convex_segment = 0u;
                    }
                    else if (x > hole_center_x)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 1.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 1u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.convex_segment = 0u;
                    }
                    else if (x <= hole_center_x)
                    {
                        if (y > left_block_max_y)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(1u);
                        }
                        else if (y <= right_block_min_y)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(2u);
                        }
                        if (z > left_block_max_z)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(3u);
                        }
                        else if (z <= left_block_min_z)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(4u);
                        }
                        if (y <= left_block_min_y)
                        {
                            if (z > right_block_max_z)
                            {
                                grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                                grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                                grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                                grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(5u);
                            }
                            else if (z <= right_block_min_z)
                            {
                                grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                                grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                                grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                                grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(6u);
                            }
                            if (x <= right_block_min_x)
                            {
                                grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                                grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                                grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                                grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(7u);
                            }
                        }
                        if (x <= left_block_min_x)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(8u);
                        }
                    }
                    // Make the hole
                    const double hole_width = 0.04;
                    const double hole_height = 0.04;
                    const double hole_max_y = hole_center_y + (hole_width * 0.5);
                    const double hole_min_y = hole_center_y - (hole_width * 0.5);
                    const double hole_max_z = hole_center_z + (hole_height * 0.5);
                    const double hole_min_z = hole_center_z - (hole_height * 0.5);
                    if (y <= hole_max_y && y > hole_min_y && z <= hole_max_z && z > hole_min_z)
                    {
                        if (x > hole_center_x)
                        {
                            std::cout << "Real hole X: " << x << " Y: " << y << " Z: " << z << std::endl;
                        }
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(9u);
                    }
                    const double outer_hole_max_y = hole_center_y + (hole_width * 0.5);
                    const double outer_hole_min_y = hole_center_y - (hole_width * 3.5);
                    const double outer_hole_max_z = hole_center_z + (hole_height * 1.5);
                    const double outer_hole_min_z = hole_center_z - (hole_height * 1.5);
                    if (y <= outer_hole_max_y && y > outer_hole_min_y && z <= outer_hole_max_z && z > outer_hole_min_z)
                    {
                        if (x <= hole_center_x + (resolution * 2.0))
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(10u);
                        }
                    }
                }
            }
        }
        //grid.Set(0.0, 0.0, -0.25, sdf_tools::TAGGED_OBJECT_COLLISION_CELL(1.0f, 1u));
        return grid;
    }
    else if (environment_id == "baxter_blocked_test_env")
    {
        double grid_x_size = 3.0;
        double grid_y_size = 3.0;
        double grid_z_size = 3.0;
        // The grid origin is the minimum point, with identity rotation
        Eigen::Translation3d grid_origin_translation(-1.5, -2.0, -1.0);
        Eigen::Quaterniond grid_origin_rotation = Eigen::Quaterniond::Identity();
        Eigen::Affine3d grid_origin_transform = grid_origin_translation * grid_origin_rotation;
        // Make the grid
        sdf_tools::TAGGED_OBJECT_COLLISION_CELL default_cell(1.0f, 0u, 0u, 0u);
        sdf_tools::TaggedObjectCollisionMapGrid grid(grid_origin_transform, "uncertainty_planning_simulator", resolution, grid_x_size, grid_y_size, grid_z_size, default_cell);
        std::cout << "New environment with dimensions (" << grid.GetNumXCells() << ", " << grid.GetNumYCells() << ", " << grid.GetNumZCells() << ")" << std::endl;
        for (int64_t x_idx = 0; x_idx < grid.GetNumXCells(); x_idx++)
        {
            for (int64_t y_idx = 0; y_idx < grid.GetNumYCells(); y_idx++)
            {
                for (int64_t z_idx = 0; z_idx < grid.GetNumZCells(); z_idx++)
                {
                    const Eigen::Vector3d location = EigenHelpers::StdVectorDoubleToEigenVector3d(grid.GridIndexToLocation(x_idx, y_idx, z_idx));
                    const double& x = location.x();
                    const double& y = location.y();
                    const double& z = location.z();
                    const double hole_center_x = 1.03;
                    const double hole_center_y = -0.40;
                    const double hole_center_z = 0.13;
                    // Make two obstructing blocks
                    const double left_block_depth = 3.0 * resolution;
                    const double right_block_depth = 2.0 * resolution;
                    const double left_block_width = 10.0 * resolution;
                    const double left_block_height = 10.0 * resolution;
                    const double right_block_width = 6.0 * resolution;
                    const double right_block_height = 4.0 * resolution;
                    const double left_block_min_x = hole_center_x - left_block_depth;
                    const double left_block_max_x = hole_center_x;
                    const double left_block_min_y = hole_center_y + (resolution * 0.0);
                    const double left_block_max_y = left_block_min_y + left_block_width;
                    const double left_block_min_z = hole_center_z - (resolution * 13.0);
                    const double left_block_max_z = left_block_min_z + left_block_height;
                    const double right_block_min_x = hole_center_x - right_block_depth;
                    const double right_block_max_x = hole_center_x;
                    const double right_block_min_y = hole_center_y - (resolution * 6.0);
                    const double right_block_max_y = right_block_min_y + right_block_width;
                    const double right_block_min_z = hole_center_z - (resolution * 7.0);
                    const double right_block_max_z = right_block_min_z + right_block_height;
                    const double upper_block_depth = 2.0 * resolution;
                    const double upper_block_width = 10.0 * resolution;
                    const double upper_block_height = 6.0 * resolution;
                    const double upper_block_min_x = hole_center_x - upper_block_depth;
                    const double upper_block_max_x = hole_center_x;
                    const double upper_block_min_y = hole_center_y - (resolution * 6.0);
                    const double upper_block_max_y = upper_block_min_y + upper_block_width;
                    const double upper_block_min_z = hole_center_z + (resolution * 3.0);
                    const double upper_block_max_z = upper_block_min_z + upper_block_height;
                    if (x > upper_block_min_x && x <= upper_block_max_x && y > upper_block_min_y && y <= upper_block_max_y && z > upper_block_min_z && z <= upper_block_max_z)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 1.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 1u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.convex_segment = 0u;
                    }
                    else if (x > left_block_min_x && x <= left_block_max_x && y > left_block_min_y && y <= left_block_max_y && z > left_block_min_z && z <= left_block_max_z)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 1.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 1u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.convex_segment = 0u;
                    }
                    else if (x > right_block_min_x && x <= right_block_max_x && y > right_block_min_y && y <= right_block_max_y && z > right_block_min_z && z <= right_block_max_z)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 1.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 1u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.convex_segment = 0u;
                    }
                    else if (x > hole_center_x)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 1.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 1u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.convex_segment = 0u;
                    }
                    else if (x <= hole_center_x)
                    {
                        // YES
                        if (y > left_block_max_y)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(1u);
                        }
                        // YES
                        else if (y <= right_block_min_y)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(2u);
                        }
                        // YES
                        if (z > upper_block_max_z)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(3u);
                        }
                        // YES
                        else if (z <= upper_block_min_z && z > left_block_max_z)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(4u);
                        }
                        // YES
                        else if (z <= left_block_min_z)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(5u);
                        }
                        // YES
                        if (z > left_block_max_z && y > upper_block_max_y)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(6u);
                        }
                        // YES
                        else if (z <= right_block_min_z && y <= left_block_min_y)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(7u);
                        }
                        // YES
                        if (z > left_block_max_z && x <= upper_block_min_x)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(8u);
                        }
                        // YES
                        if (y <= left_block_min_y && x <= right_block_min_x)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(9u);
                        }
                        // YES
                        if (x <= left_block_min_x)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(10u);
                        }
                    }
                    // Make the hole
                    const double hole_width = 0.04;
                    const double hole_height = 0.04;
                    const double hole_max_y = hole_center_y + (hole_width * 0.5);
                    const double hole_min_y = hole_center_y - (hole_width * 0.5);
                    const double hole_max_z = hole_center_z + (hole_height * 0.5);
                    const double hole_min_z = hole_center_z - (hole_height * 0.5);
                    if (y <= hole_max_y && y > hole_min_y && z <= hole_max_z && z > hole_min_z)
                    {
                        if (x > hole_center_x)
                        {
                            std::cout << "Real hole X: " << x << " Y: " << y << " Z: " << z << std::endl;
                        }
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(11u);
                    }
                    const double outer_hole_max_y = hole_center_y + (hole_width * 0.5);
                    const double outer_hole_min_y = hole_center_y - (hole_width * 3.5);
                    const double outer_hole_max_z = hole_center_z + (hole_height * 1.5);
                    const double outer_hole_min_z = hole_center_z - (hole_height * 1.5);
                    if (y <= outer_hole_max_y && y > outer_hole_min_y && z <= outer_hole_max_z && z > outer_hole_min_z)
                    {
                        if (x <= hole_center_x + (resolution * 1.0))
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(12u);
                        }
                    }
                }
            }
        }
        //grid.Set(0.0, 0.0, -0.25, sdf_tools::TAGGED_OBJECT_COLLISION_CELL(1.0f, 1u));
        return grid;
    }
    else if (environment_id == "baxter_blocked_test_mod_env")
    {
        double grid_x_size = 3.0;
        double grid_y_size = 3.0;
        double grid_z_size = 3.0;
        // The grid origin is the minimum point, with identity rotation
        Eigen::Translation3d grid_origin_translation(-1.5, -2.0, -1.0);
        Eigen::Quaterniond grid_origin_rotation = Eigen::Quaterniond::Identity();
        Eigen::Affine3d grid_origin_transform = grid_origin_translation * grid_origin_rotation;
        // Make the grid
        sdf_tools::TAGGED_OBJECT_COLLISION_CELL default_cell(1.0f, 0u, 0u, 0u);
        sdf_tools::TaggedObjectCollisionMapGrid grid(grid_origin_transform, "uncertainty_planning_simulator", resolution, grid_x_size, grid_y_size, grid_z_size, default_cell);
        std::cout << "New environment with dimensions (" << grid.GetNumXCells() << ", " << grid.GetNumYCells() << ", " << grid.GetNumZCells() << ")" << std::endl;
        for (int64_t x_idx = 0; x_idx < grid.GetNumXCells(); x_idx++)
        {
            for (int64_t y_idx = 0; y_idx < grid.GetNumYCells(); y_idx++)
            {
                for (int64_t z_idx = 0; z_idx < grid.GetNumZCells(); z_idx++)
                {
                    const Eigen::Vector3d location = EigenHelpers::StdVectorDoubleToEigenVector3d(grid.GridIndexToLocation(x_idx, y_idx, z_idx));
                    const double& x = location.x();
                    const double& y = location.y();
                    const double& z = location.z();
                    const double hole_center_x = 1.03;
                    const double hole_center_y = -0.40;
                    const double hole_center_z = 0.13;
                    // Make two obstructing blocks
                    const double left_block_depth = 3.0 * resolution;
                    const double right_block_depth = 2.0 * resolution;
                    const double left_block_width = 10.0 * resolution;
                    const double left_block_height = 10.0 * resolution;
                    const double right_block_width = 6.0 * resolution;
                    const double right_block_height = 4.0 * resolution;
                    const double left_block_min_x = hole_center_x - left_block_depth;
                    const double left_block_max_x = hole_center_x;
                    const double left_block_min_y = hole_center_y + (resolution * 0.0);
                    const double left_block_max_y = left_block_min_y + left_block_width;
                    const double left_block_min_z = hole_center_z - (resolution * 13.0);
                    const double left_block_max_z = left_block_min_z + left_block_height;
                    const double right_block_min_x = hole_center_x - right_block_depth;
                    const double right_block_max_x = hole_center_x;
                    const double right_block_min_y = hole_center_y - (resolution * 6.0);
                    const double right_block_max_y = right_block_min_y + right_block_width;
                    const double right_block_min_z = hole_center_z - (resolution * 7.0);
                    const double right_block_max_z = right_block_min_z + right_block_height;
                    const double upper_block_depth = 2.0 * resolution;
                    const double upper_block_width = 10.0 * resolution;
                    const double upper_block_height = 6.0 * resolution;
                    const double upper_block_min_x = hole_center_x - upper_block_depth;
                    const double upper_block_max_x = hole_center_x;
                    const double upper_block_min_y = hole_center_y - (resolution * 6.0);
                    const double upper_block_max_y = upper_block_min_y + upper_block_width;
                    const double upper_block_min_z = hole_center_z + (resolution * 3.0);
                    const double upper_block_max_z = upper_block_min_z + upper_block_height;
                    if (x > upper_block_min_x && x <= upper_block_max_x && y > upper_block_min_y && y <= upper_block_max_y && z > upper_block_min_z && z <= upper_block_max_z)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 1.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 1u;//2u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.convex_segment = 0u;
                    }
                    else if (x > left_block_min_x && x <= left_block_max_x && y > left_block_min_y && y <= left_block_max_y && z > left_block_min_z && z <= left_block_max_z)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 1.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 1u;//3u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.convex_segment = 0u;
                    }
                    else if (x > right_block_min_x && x <= right_block_max_x && y > right_block_min_y && y <= right_block_max_y && z > right_block_min_z && z <= right_block_max_z)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 1.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 1u;//4u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.convex_segment = 0u;
                    }
                    else if (x > hole_center_x)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 1.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 1u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.convex_segment = 0u;
                    }
                    else if (x <= hole_center_x)
                    {
                        // YES
                        if (y > left_block_max_y)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(1u);
                        }
                        // YES
                        else if (y <= right_block_min_y)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(2u);
                        }
                        // YES
                        if (z > upper_block_max_z)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(3u);
                        }
                        // YES
                        else if (z <= upper_block_min_z && z > left_block_max_z)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(4u);
                        }
                        // YES
                        else if (z <= left_block_min_z)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(5u);
                        }
                        // YES
                        if (z > left_block_max_z && y > upper_block_max_y)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(6u);
                        }
                        // YES
                        else if (z <= right_block_min_z && y <= left_block_min_y)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(7u);
                        }
                        // YES
                        if (z > left_block_max_z && x <= upper_block_min_x)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(8u);
                        }
                        // YES
                        if (y <= left_block_min_y && x <= right_block_min_x)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(9u);
                        }
                        // YES
                        if (x <= left_block_min_x)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(10u);
                        }
                    }
                    // Make the hole
                    const double hole_width = 0.04;
                    const double hole_height = 0.04;
                    const double hole_max_y = hole_center_y + (hole_width * 0.5);
                    const double hole_min_y = hole_center_y - (hole_width * 1.0);//0.5);
                    const double hole_max_z = hole_center_z + (hole_height * 0.5);
                    const double hole_min_z = hole_center_z - (hole_height * 0.5);
                    if (y <= hole_max_y && y > hole_min_y && z <= hole_max_z && z > hole_min_z)
                    {
                        if (x > hole_center_x)
                        {
                            std::cout << "Real hole X: " << x << " Y: " << y << " Z: " << z << std::endl;
                        }
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(11u);
                    }
//                            const double step_hole_max_y = hole_center_y + (hole_width * 0.5);
//                            const double step_hole_min_y = hole_center_y - (hole_width * 1.5);
//                            const double step_hole_max_z = hole_center_z + (hole_height * 0.5);
//                            const double step_hole_min_z = hole_center_z - (hole_height * 0.5);
//                            if (y <= step_hole_max_y && y > step_hole_min_y && z <= step_hole_max_z && z > step_hole_min_z)
//                            {
//                                if (x <= hole_center_x + (resolution * 2.0))
//                                {
//                                    grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
//                                    grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
//                                    grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
//                                    grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(12u);
//                                }
//                            }
                    const double outer_hole_max_y = hole_center_y + (hole_width * 0.5);
                    const double outer_hole_min_y = hole_center_y - (hole_width * 3.5);
                    const double outer_hole_max_z = hole_center_z + (hole_height * 1.5);
                    const double outer_hole_min_z = hole_center_z - (hole_height * 1.5);
                    if (y <= outer_hole_max_y && y > outer_hole_min_y && z <= outer_hole_max_z && z > outer_hole_min_z)
                    {
                        if (x <= hole_center_x + (resolution * 1.0))
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(12u);
                        }
//                                else if (grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy == 1.0f)
//                                {
//                                    grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 5u;
//                                }
                    }
                }
            }
        }
        //grid.Set(0.0, 0.0, -0.25, sdf_tools::TAGGED_OBJECT_COLLISION_CELL(1.0f, 1u));
        return grid;
    }
    else if (environment_id == "baxter_wrist_key_env")
    {
        double grid_x_size = 3.0;
        double grid_y_size = 3.0;
        double grid_z_size = 3.0;
        // The grid origin is the minimum point, with identity rotation
        Eigen::Translation3d grid_origin_translation(-1.5, -2.0, -1.0);
        Eigen::Quaterniond grid_origin_rotation = Eigen::Quaterniond::Identity();
        Eigen::Affine3d grid_origin_transform = grid_origin_translation * grid_origin_rotation;
        // Make the grid
        sdf_tools::TAGGED_OBJECT_COLLISION_CELL default_cell(1.0f, 0u, 0u, 0u);
        sdf_tools::TaggedObjectCollisionMapGrid grid(grid_origin_transform, "uncertainty_planning_simulator", resolution, grid_x_size, grid_y_size, grid_z_size, default_cell);
        std::cout << "New environment with dimensions (" << grid.GetNumXCells() << ", " << grid.GetNumYCells() << ", " << grid.GetNumZCells() << ")" << std::endl;
        for (int64_t x_idx = 0; x_idx < grid.GetNumXCells(); x_idx++)
        {
            for (int64_t y_idx = 0; y_idx < grid.GetNumYCells(); y_idx++)
            {
                for (int64_t z_idx = 0; z_idx < grid.GetNumZCells(); z_idx++)
                {
                    /*
                        Real hole X: 1.04688 Y: -0.390625 Z: 0.140625
                        Real hole X: 1.07812 Y: -0.390625 Z: 0.140625
                        Real hole X: 1.10938 Y: -0.390625 Z: 0.140625
                        Real hole X: 1.14062 Y: -0.390625 Z: 0.140625
                        Real hole X: 1.17188 Y: -0.390625 Z: 0.140625
                        Real hole X: 1.20312 Y: -0.390625 Z: 0.140625
                        Real hole X: 1.23438 Y: -0.390625 Z: 0.140625
                        Real hole X: 1.26562 Y: -0.390625 Z: 0.140625
                        Real hole X: 1.29688 Y: -0.390625 Z: 0.140625
                        Real hole X: 1.32812 Y: -0.390625 Z: 0.140625
                        Real hole X: 1.35938 Y: -0.390625 Z: 0.140625
                        Real hole X: 1.39062 Y: -0.390625 Z: 0.140625
                        Real hole X: 1.42188 Y: -0.390625 Z: 0.140625
                        Real hole X: 1.45312 Y: -0.390625 Z: 0.140625
                        Real hole X: 1.48438 Y: -0.390625 Z: 0.140625
                     */
                    const Eigen::Vector3d location = EigenHelpers::StdVectorDoubleToEigenVector3d(grid.GridIndexToLocation(x_idx, y_idx, z_idx));
                    const double& x = location.x();
                    const double& y = location.y();
                    const double& z = location.z();
                    const double key_tip_x = 1.225;
                    const double key_base_x = 1.0;
                    const double key_center_y = -0.390625; //-0.40;
                    const double key_center_z = 0.140625; //0.13;


                    // Solid backing wall
                    if (x > key_tip_x)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 1.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 1u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.convex_segment = 0u;
                    }
                    // Intervening blocks
                    else if (x <= key_tip_x && x > key_base_x)
                    {
                        if (y > (key_center_y + (resolution * 0.5)) && z <= (key_center_z - (resolution * 0.5)))
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 1.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 1u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.convex_segment = 0u;
                        }
                        if (y <= (key_center_y - (resolution * 3.5)) && z <= (key_center_z - (resolution * 0.5)))
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 1.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 1u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.convex_segment = 0u;
                        }
                        if (y > (key_center_y + (resolution * 0.5)) && z > (key_center_z + (resolution * 3.5)))
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 1.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 1u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.convex_segment = 0u;
                        }
//                                if (y <= (key_center_y - (resolution * 3.5)) && z > (key_center_z + (resolution * 3.5)))
//                                {
//                                    grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 1.0f;
//                                    grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 1u;
//                                    grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
//                                    grid.GetMutable(x_idx, y_idx, z_idx).first.convex_segment = 0u;
//                                }
                    }
                    else
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(1u);
                    }
                    if (x <= key_tip_x)
                    {
                        if (y <= (key_center_y + (resolution * 0.5)) && z > (key_center_z - (resolution * 0.5)))
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(2u);
                        }
                        if (y > (key_center_y - (resolution * 3.5)) && y <= (key_center_y + (resolution * 0.5)))
                        {
                            if (z > (key_center_z - (resolution * 4.5)) && z <= (key_center_z - (resolution * 0.5)))
                            {
                                if (x <= (key_base_x + (resolution * 3.5)) && x > key_base_x)
                                {
                                    std::cout << "Lip @ " << x << ", " << y << ", " << z << std::endl;
                                    grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 1.0f;
                                    grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 1u;
                                    grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                                    grid.GetMutable(x_idx, y_idx, z_idx).first.convex_segment = 0u;
                                }
                            }
                            else if (z <= (key_center_z - (resolution * 4.5)))
                            {
                                if (x > key_base_x)
                                {
                                    grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 1.0f;
                                    grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 1u;
                                    grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                                    grid.GetMutable(x_idx, y_idx, z_idx).first.convex_segment = 0u;
                                }
                            }
                            if (z > (key_center_z - (resolution * 4.5)))
                            {
                                if (x > (key_base_x + (resolution * 2.5)))
                                {
                                    grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                                    grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                                    grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                                    grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(3u);
                                }
                            }
                        }
                        if (z > (key_center_z - (resolution * 0.5)) && z <= (key_center_z + (resolution * 3.5)))
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(4u);
                        }
                    }
//                            // Bound the environment?
//                            else
//                            {
//                                if (x_idx < 4 || y_idx < 4 || z_idx < 4 || x_idx >= (grid.GetNumXCells() - 4) || y_idx >= (grid.GetNumYCells() - 4) || z_idx >= (grid.GetNumZCells() - 4))
//                                {
//                                    grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 1.0f;
//                                    grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
//                                    grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
//                                    grid.GetMutable(x_idx, y_idx, z_idx).first.convex_segment = 0u;
//                                }
//                                else
//                                {
//                                    grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
//                                    grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
//                                    grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
//                                    grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(1u);
//                                }
//                            }
                }
            }
        }
        //grid.Set(0.0, 0.0, -0.25, sdf_tools::TAGGED_OBJECT_COLLISION_CELL(1.0f, 1u));
        return grid;
    }
    else if (environment_id == "baxter_lip_env")
    {
        double grid_x_size = 3.0;
        double grid_y_size = 3.0;
        double grid_z_size = 3.0;
        // The grid origin is the minimum point, with identity rotation
        Eigen::Translation3d grid_origin_translation(-1.5, -2.0, -1.0);
        Eigen::Quaterniond grid_origin_rotation = Eigen::Quaterniond::Identity();
        Eigen::Affine3d grid_origin_transform = grid_origin_translation * grid_origin_rotation;
        // Make the grid
        sdf_tools::TAGGED_OBJECT_COLLISION_CELL default_cell(1.0f, 1u, 0u, 0u);
        sdf_tools::TaggedObjectCollisionMapGrid grid(grid_origin_transform, "uncertainty_planning_simulator", resolution, grid_x_size, grid_y_size, grid_z_size, default_cell);
        std::cout << "New environment with dimensions (" << grid.GetNumXCells() << ", " << grid.GetNumYCells() << ", " << grid.GetNumZCells() << ")" << std::endl;
        for (int64_t x_idx = 0; x_idx < grid.GetNumXCells(); x_idx++)
        {
            for (int64_t y_idx = 0; y_idx < grid.GetNumYCells(); y_idx++)
            {
                for (int64_t z_idx = 0; z_idx < grid.GetNumZCells(); z_idx++)
                {
                    const Eigen::Vector3d location = EigenHelpers::StdVectorDoubleToEigenVector3d(grid.GridIndexToLocation(x_idx, y_idx, z_idx));
                    const double& x = location.x();
                    const double& y = location.y();
                    const double& z = location.z();
                    const double hole_center_x = 1.03;
                    const double hole_center_y = -0.40;
                    const double hole_center_z = 0.13;
                    const double hole_center_to_side_edge = 0.27305;
                    const double hole_center_to_top_edge = 0.3048;
                    const double hole_width = 0.04;
                    const double hole_height = 0.04;
                    const double lip_width = 0.02;
                    const double above_hole_x_min = hole_center_x;
                    const double below_hole_x_min = hole_center_x - lip_width;
                    const double hole_max_y = hole_center_y + (hole_width * 0.5);
                    const double hole_min_y = hole_center_y - (hole_width * 0.5);
                    const double hole_max_z = hole_center_z + (hole_height * 0.5);
                    const double hole_min_z = hole_center_z - (hole_height * 0.5);
                    const double min_y = hole_center_y - hole_center_to_side_edge;
                    const double max_y = hole_center_y + hole_center_to_side_edge;
                    const double min_z = hole_center_z - hole_center_to_top_edge;
                    const double max_z = hole_center_z + hole_center_to_top_edge;
                    if (z <= min_z)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(1u);
                    }
                    else if (z > max_z)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(2u);
                    }
                    if (y <= min_y)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(3u);
                    }
                    else if (y > max_y)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(4u);
                    }
                    if (x <= above_hole_x_min)
                    {
                        if (z > hole_min_z)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(5u);
                        }
                    }
                    if (x <= below_hole_x_min)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(6u);
                    }
                    // Make the hole
                    if (y <= hole_max_y && y > hole_min_y && z <= hole_max_z && z > hole_min_z)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.component = 0u;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(6u);
                    }
                }
            }
        }
        //grid.Set(0.0, 0.0, -0.25, sdf_tools::TAGGED_OBJECT_COLLISION_CELL(1.0f, 1u));
        return grid;
    }
    else if (environment_id == "blocked_peg_in_hole")
    {
        double grid_x_size = 10.0;
        double grid_y_size = 10.0;
        double grid_z_size = 10.0;
        // The grid origin is the minimum point, with identity rotation
        Eigen::Translation3d grid_origin_translation(0.0, 0.0, 0.0);
        Eigen::Quaterniond grid_origin_rotation = Eigen::Quaterniond::Identity();
        Eigen::Affine3d grid_origin_transform = grid_origin_translation * grid_origin_rotation;
        // Make the grid
        sdf_tools::TAGGED_OBJECT_COLLISION_CELL default_cell;
        sdf_tools::TaggedObjectCollisionMapGrid grid(grid_origin_transform, "uncertainty_planning_simulator", resolution, grid_x_size, grid_y_size, grid_z_size, default_cell);
        for (int64_t x_idx = 0; x_idx < grid.GetNumXCells(); x_idx++)
        {
            for (int64_t y_idx = 0; y_idx < grid.GetNumYCells(); y_idx++)
            {
                for (int64_t z_idx = 0; z_idx < grid.GetNumZCells(); z_idx++)
                {
                    const Eigen::Vector3d location = EigenHelpers::StdVectorDoubleToEigenVector3d(grid.GridIndexToLocation(x_idx, y_idx, z_idx));
                    const double& x = location.x();
                    const double& y = location.y();
                    const double& z = location.z();
                    // Set the object we belong to
                    // We assume that all objects are convex, so we can set the convex region as 1
                    // "Bottom bottom"
                    if (z <= 3.0)
                    {
                        if (x <= 2.0 || y <= 2.0)
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0, 1u, 0u, 1u);
                            grid.Set(x_idx, y_idx, z_idx, object_cell);
                        }
                        else if (x > 2.5 || y > 2.5)
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0, 1u, 0u, 1u);
                            grid.Set(x_idx, y_idx, z_idx, object_cell);
                        }
                        else if (z <= resolution)
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0, 1u, 0u, 1u);
                            grid.Set(x_idx, y_idx, z_idx, object_cell);
                        }
                    }
                    else if (z <= 4.5)
                    {
                        if (y > 1.5 && y <= 5.0 && x > 3.0 && x <= 3.5)
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0, 1u, 0u, 1u);
                            grid.Set(x_idx, y_idx, z_idx, object_cell);
                        }
                    }
                    // Set the free-space convex segment we belong to (if necessary)
                    if (grid.GetImmutable(x_idx, y_idx, z_idx).first.occupancy < 0.5)
                    {
                        // There are 2 convex regions, we can belong to multiple regions, so we check for each
                        if (z > 3.0)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(1u);
                        }
                        if (x > 2.0 && y > 2.0 && x <= 2.5 && y <= 2.5 && z > resolution)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(2u);
                        }

                    }
                }
            }
        }
        return grid;
    }
    else if (environment_id == "inset_peg_in_hole")
    {
        double grid_x_size = 10.0;
        double grid_y_size = 10.0;
        double grid_z_size = 10.0;
        // The grid origin is the minimum point, with identity rotation
        Eigen::Translation3d grid_origin_translation(0.0, 0.0, 0.0);
        Eigen::Quaterniond grid_origin_rotation = Eigen::Quaterniond::Identity();
        Eigen::Affine3d grid_origin_transform = grid_origin_translation * grid_origin_rotation;
        // Make the grid
        sdf_tools::TAGGED_OBJECT_COLLISION_CELL default_cell;
        sdf_tools::TaggedObjectCollisionMapGrid grid(grid_origin_transform, "uncertainty_planning_simulator", resolution, grid_x_size, grid_y_size, grid_z_size, default_cell);
        for (int64_t x_idx = 0; x_idx < grid.GetNumXCells(); x_idx++)
        {
            for (int64_t y_idx = 0; y_idx < grid.GetNumYCells(); y_idx++)
            {
                for (int64_t z_idx = 0; z_idx < grid.GetNumZCells(); z_idx++)
                {
                    const Eigen::Vector3d location = EigenHelpers::StdVectorDoubleToEigenVector3d(grid.GridIndexToLocation(x_idx, y_idx, z_idx));
                    const double& x = location.x();
                    const double& y = location.y();
                    const double& z = location.z();
                    // Set the object we belong to
                    // We assume that all objects are convex, so we can set the convex region as 1
                    // "Bottom bottom"
                    if (z <= 2.0)
                    {
                        if (x <= 2.0 || y <= 2.0)
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0, 1u, 0u, 1u);
                            grid.Set(x_idx, y_idx, z_idx, object_cell);
                        }
                        else if ((x > 2.5 || y > 2.5) && (z <= 1.0))
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0, 1u, 0u, 1u);
                            grid.Set(x_idx, y_idx, z_idx, object_cell);
                        }
                        else if (z <= resolution)
                        {
                            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0, 1u, 0u, 1u);
                            grid.Set(x_idx, y_idx, z_idx, object_cell);
                        }
                    }
                    // Set the free-space convex segment we belong to (if necessary)
                    if (grid.GetImmutable(x_idx, y_idx, z_idx).first.occupancy < 0.5)
                    {
                        // There are 2 convex regions, we can belong to multiple regions, so we check for each
                        if (z > 2.0)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(1u);
                        }
                        if (x > 2.0 && y > 2.0 && x <= 2.5 && y <= 2.5 && z > resolution)
                        {
                            grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(2u);
                        }

                    }
                }
            }
        }
        return grid;
    }
    else if (environment_id == "se2_maze")
    {
        double grid_x_size = 10.0;
        double grid_y_size = 10.0;
        double grid_z_size = 1.0;
        // The grid origin is the minimum point, with identity rotation
        Eigen::Translation3d grid_origin_translation(0.0, 0.0, -0.5);
        Eigen::Quaterniond grid_origin_rotation = Eigen::Quaterniond::Identity();
        Eigen::Affine3d grid_origin_transform = grid_origin_translation * grid_origin_rotation;
        // Make the grid
        sdf_tools::TAGGED_OBJECT_COLLISION_CELL default_cell(1.0, 1u, 0u, 1u); // Everything is filled by default
        sdf_tools::TaggedObjectCollisionMapGrid grid(grid_origin_transform, "uncertainty_planning_simulator", resolution, grid_x_size, grid_y_size, grid_z_size, default_cell);
        for (int64_t x_idx = 0; x_idx < grid.GetNumXCells(); x_idx++)
        {
            for (int64_t y_idx = 0; y_idx < grid.GetNumYCells(); y_idx++)
            {
                for (int64_t z_idx = 0; z_idx < grid.GetNumZCells(); z_idx++)
                {
                    const Eigen::Vector3d location = EigenHelpers::StdVectorDoubleToEigenVector3d(grid.GridIndexToLocation(x_idx, y_idx, z_idx));
                    const double& x = location.x();
                    const double& y = location.y();
                    //const double& z = location.z();
                    // Check if the cell belongs to any of the regions of freespace
                    if (x > 0.5 && y > 0.5 && x <= 3.0 && y <= 9.5)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(1u);
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                    }
                    if (x > 3.5 && y > 0.5 && x <= 6.5 && y <= 9.5)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(2u);
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                    }
                    if (x > 7.0 && y > 0.5 && x <= 9.5 && y <= 9.5)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(3u);
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                    }
                    if (x > 0.5 && y > 7.0 && x <= 9.5 && y <= 9.0)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(4u);
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                    }
                    if (x > 0.5 && y > 1.0 && x <= 9.5 && y <= 3.0)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(5u);
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                    }
                }
            }
        }
        return grid;
    }
    else if (environment_id == "se2_maze_blocked")
    {
        double grid_x_size = 10.0;
        double grid_y_size = 10.0;
        double grid_z_size = 1.0;
        // The grid origin is the minimum point, with identity rotation
        Eigen::Translation3d grid_origin_translation(0.0, 0.0, -0.5);
        Eigen::Quaterniond grid_origin_rotation = Eigen::Quaterniond::Identity();
        Eigen::Affine3d grid_origin_transform = grid_origin_translation * grid_origin_rotation;
        // Make the grid
        sdf_tools::TAGGED_OBJECT_COLLISION_CELL default_cell(1.0, 1u, 0u, 1u); // Everything is filled by default
        sdf_tools::TaggedObjectCollisionMapGrid grid(grid_origin_transform, "uncertainty_planning_simulator", resolution, grid_x_size, grid_y_size, grid_z_size, default_cell);
        for (int64_t x_idx = 0; x_idx < grid.GetNumXCells(); x_idx++)
        {
            for (int64_t y_idx = 0; y_idx < grid.GetNumYCells(); y_idx++)
            {
                for (int64_t z_idx = 0; z_idx < grid.GetNumZCells(); z_idx++)
                {
                    const Eigen::Vector3d location = EigenHelpers::StdVectorDoubleToEigenVector3d(grid.GridIndexToLocation(x_idx, y_idx, z_idx));
                    const double& x = location.x();
                    const double& y = location.y();
                    //const double& z = location.z();
                    // Check if the cell belongs to any of the regions of freespace
                    if (x > 0.5 && y > 0.5 && x <= 3.0 && y <= 9.5)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(1u);
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                    }
                    if (x > 3.5 && y > 0.5 && x <= 6.5 && y <= 9.5)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(2u);
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                    }
                    if (x > 7.0 && y > 0.5 && x <= 9.5 && y <= 9.5)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(3u);
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                    }
                    if (x > 0.5 && y > 7.0 && x <= 9.5 && y <= 9.0)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(4u);
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                    }
                    if (x > 0.5 && y > 1.0 && x <= 9.5 && y <= 3.0)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.AddToConvexSegment(5u);
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 0.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 0u;
                    }
                    // Re-add the obstacle in the middle
                    if (x > 7.0 && y > 4.75 && x <= 9.5 && y <= 5.25)
                    {
                        grid.GetMutable(x_idx, y_idx, z_idx).first.occupancy = 1.0f;
                        grid.GetMutable(x_idx, y_idx, z_idx).first.object_id = 7u;
                    }
                }
            }
        }
        return grid;
    }
    else if (environment_id == "se2_cluttered")
    {
        throw std::invalid_argument("Not implemented yet");
    }
    else if (environment_id == "noisy_arm_cluttered")
    {
        throw std::invalid_argument("Not implemented yet");
    }
    else
    {
        throw std::invalid_argument("Unrecognized environment ID");
    }
}

/* Build a new environment from the provided obstacles */
sdf_tools::TaggedObjectCollisionMapGrid simulator_environment_builder::BuildEnvironment(const std::vector<OBSTACLE_CONFIG>& obstacles, const double resolution)
{
    if (obstacles.empty())
    {
        std::cerr << "No obstacles provided, generating the default environment" << std::endl;
        double grid_x_size = 10.0;
        double grid_y_size = 10.0;
        double grid_z_size = 10.0;
        // The grid origin is the minimum point, with identity rotation
        Eigen::Translation3d grid_origin_translation(0.0, 0.0, 0.0);
        Eigen::Quaterniond grid_origin_rotation = Eigen::Quaterniond::Identity();
        Eigen::Affine3d grid_origin_transform = grid_origin_translation * grid_origin_rotation;
        // Make the grid
        sdf_tools::TAGGED_OBJECT_COLLISION_CELL default_cell;
        sdf_tools::TaggedObjectCollisionMapGrid grid(grid_origin_transform, "uncertainty_planning_simulator", resolution, grid_x_size, grid_y_size, grid_z_size, default_cell);
        return grid;
    }
    else
    {
        std::cout << "Rebuilding the environment with " << obstacles.size() << " obstacles" << std::endl;
        // We need to loop through the obstacles, discretize each obstacle, and then find the size of the grid we need to store them
        bool xyz_bounds_initialized = false;
        double x_min = 0.0;
        double y_min = 0.0;
        double z_min = 0.0;
        double x_max = 0.0;
        double y_max = 0.0;
        double z_max = 0.0;
        std::vector<std::pair<Eigen::Vector3d, sdf_tools::TAGGED_OBJECT_COLLISION_CELL>> all_obstacle_cells;
        for (size_t idx = 0; idx < obstacles.size(); idx++)
        {
            const OBSTACLE_CONFIG& obstacle = obstacles[idx];
            std::vector<std::pair<Eigen::Vector3d, sdf_tools::TAGGED_OBJECT_COLLISION_CELL>> obstacle_cells = DiscretizeObstacle(obstacle, resolution);
            for (size_t cidx = 0; cidx < obstacle_cells.size(); cidx++)
            {
                const Eigen::Vector3d& relative_location = obstacle_cells[cidx].first;
                Eigen::Vector3d real_location = obstacle.pose * relative_location;
                all_obstacle_cells.push_back(std::pair<Eigen::Vector3d, sdf_tools::TAGGED_OBJECT_COLLISION_CELL>(real_location, obstacle_cells[cidx].second));
                // Check against the min/max extents
                if (xyz_bounds_initialized)
                {
                    if (real_location.x() < x_min)
                    {
                        x_min = real_location.x();
                    }
                    else if (real_location.x() > x_max)
                    {
                        x_max = real_location.x();
                    }
                    if (real_location.y() < y_min)
                    {
                        y_min = real_location.y();
                    }
                    else if (real_location.y() > y_max)
                    {
                        y_max = real_location.y();
                    }
                    if (real_location.z() < z_min)
                    {
                        z_min = real_location.z();
                    }
                    else if (real_location.z() > z_max)
                    {
                        z_max = real_location.z();
                    }
                }
                // If we haven't initialized the bounds yet, set them to the current position
                else
                {
                    x_min = real_location.x();
                    x_max = real_location.x();
                    y_min = real_location.y();
                    y_max = real_location.y();
                    z_min = real_location.z();
                    z_max = real_location.z();
                    xyz_bounds_initialized = true;
                }
            }
        }
        // Now that we've done that, we fill in the grid to store them
        // Key the center of the first cell off the the minimum value
        x_min -= (resolution * 0.5);
        y_min -= (resolution * 0.5);
        z_min -= (resolution * 0.5);
        // Add a 1-cell buffer to all sides
        x_min -= (resolution * 3.0);
        y_min -= (resolution * 3.0);
        z_min -= (resolution * 3.0);
        x_max += (resolution * 3.0);
        y_max += (resolution * 3.0);
        z_max += (resolution * 3.0);
        double grid_x_size = x_max - x_min;
        double grid_y_size = y_max - y_min;
        double grid_z_size = z_max - z_min;
        // The grid origin is the minimum point, with identity rotation
        Eigen::Translation3d grid_origin_translation(x_min, y_min, z_min);
        Eigen::Quaterniond grid_origin_rotation = Eigen::Quaterniond::Identity();
        Eigen::Affine3d grid_origin_transform = grid_origin_translation * grid_origin_rotation;
        // Make the grid
        sdf_tools::TAGGED_OBJECT_COLLISION_CELL default_cell;
        sdf_tools::TaggedObjectCollisionMapGrid grid(grid_origin_transform, "uncertainty_planning_simulator", resolution, grid_x_size, grid_y_size, grid_z_size, default_cell);
        // Fill it in
        for (size_t idx = 0; idx < all_obstacle_cells.size(); idx++)
        {
            const Eigen::Vector3d& location = all_obstacle_cells[idx].first;
            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL& cell = all_obstacle_cells[idx].second;
            grid.Set(location.x(), location.y(), location.z(), cell);
        }
        // Set the environment
        return grid;
    }
}

void simulator_environment_builder::UpdateSurfaceNormalGridCell(const std::vector<RawCellSurfaceNormal>& raw_surface_normals, const Eigen::Affine3d& transform, const Eigen::Vector3d& cell_location, const sdf_tools::SignedDistanceField& environment_sdf, simple_simulator_interface::SurfaceNormalGrid& surface_normals_grid)
{
    const Eigen::Vector3d world_location = transform * cell_location;
    // Let's check the penetration distance. We only want to update cells that are *actually* on the surface
    const float distance = environment_sdf.Get3d(world_location);
    // If we're within one cell of the surface, we update
    if (distance > -(environment_sdf.GetResolution() * 1.5))
    {
        // First, we clear any stored surface normals
        surface_normals_grid.ClearStoredSurfaceNormals(world_location);
        for (size_t idx = 0; idx < raw_surface_normals.size(); idx++)
        {
            const RawCellSurfaceNormal& current_surface_normal = raw_surface_normals[idx];
            const Eigen::Vector3d& raw_surface_normal = current_surface_normal.normal;
            const Eigen::Vector3d& raw_entry_direction = current_surface_normal.entry_direction;
            const Eigen::Vector3d real_surface_normal = (Eigen::Vector3d)(transform.rotation() * raw_surface_normal);
            const Eigen::Vector3d real_entry_direction = (Eigen::Vector3d)(transform.rotation() * raw_entry_direction);
            surface_normals_grid.InsertSurfaceNormal(world_location, real_surface_normal, real_entry_direction);
        }
    }
    else
    {
        // Do nothing otherwise
        ;
    }
}

void simulator_environment_builder::AdjustSurfaceNormalGridForAllFlatSurfaces(const sdf_tools::SignedDistanceField& environment_sdf, simple_simulator_interface::SurfaceNormalGrid& surface_normals_grid)
{
    for (int64_t x_idx = 0; x_idx < environment_sdf.GetNumXCells(); x_idx++)
    {
        for (int64_t y_idx = 0; y_idx < environment_sdf.GetNumYCells(); y_idx++)
        {
            for (int64_t z_idx = 0; z_idx < environment_sdf.GetNumZCells(); z_idx++)
            {
                const float distance = environment_sdf.Get(x_idx, y_idx, z_idx);
                if (distance < 0.0)
                {
                    const float xm1_distance = environment_sdf.Get(x_idx - 1, y_idx, z_idx);
                    const float xp1_distance = environment_sdf.Get(x_idx + 1, y_idx, z_idx);
                    const float ym1_distance = environment_sdf.Get(x_idx, y_idx - 1, z_idx);
                    const float yp1_distance = environment_sdf.Get(x_idx, y_idx + 1, z_idx);
                    const float zm1_distance = environment_sdf.Get(x_idx, y_idx, z_idx - 1);
                    const float zp1_distance = environment_sdf.Get(x_idx, y_idx, z_idx + 1);
                    const bool xm1_edge = (xm1_distance > 0.0);
                    const bool xp1_edge = (xp1_distance > 0.0);
                    const bool ym1_edge = (ym1_distance > 0.0);
                    const bool yp1_edge = (yp1_distance > 0.0);
                    const bool zm1_edge = (zm1_distance > 0.0);
                    const bool zp1_edge = (zp1_distance > 0.0);
                    if (xm1_edge || xp1_edge || ym1_edge || yp1_edge || zm1_edge || zp1_edge)
                    {
                        surface_normals_grid.ClearStoredSurfaceNormals(x_idx, y_idx, z_idx);
                        if (xm1_edge)
                        {
                            const Eigen::Vector3d normal(-1.0, 0.0, 0.0);
                            const Eigen::Vector3d entry(1.0, 0.0, 0.0);
                            surface_normals_grid.InsertSurfaceNormal(x_idx, y_idx, z_idx, normal, entry);
                        }
                        if (xp1_edge)
                        {
                            const Eigen::Vector3d normal(1.0, 0.0, 0.0);
                            const Eigen::Vector3d entry(-1.0, 0.0, 0.0);
                            surface_normals_grid.InsertSurfaceNormal(x_idx, y_idx, z_idx, normal, entry);
                        }
                        if (ym1_edge)
                        {
                            const Eigen::Vector3d normal(0.0, -1.0, 0.0);
                            const Eigen::Vector3d entry(0.0, 1.0, 0.0);
                            surface_normals_grid.InsertSurfaceNormal(x_idx, y_idx, z_idx, normal, entry);
                        }
                        if (yp1_edge)
                        {
                            const Eigen::Vector3d normal(0.0, 1.0, 0.0);
                            const Eigen::Vector3d entry(0.0, -1.0, 0.0);
                            surface_normals_grid.InsertSurfaceNormal(x_idx, y_idx, z_idx, normal, entry);
                        }
                        if (zm1_edge)
                        {
                            const Eigen::Vector3d normal(0.0, 0.0, -1.0);
                            const Eigen::Vector3d entry(0.0, 0.0, 1.0);
                            surface_normals_grid.InsertSurfaceNormal(x_idx, y_idx, z_idx, normal, entry);
                        }
                        if (zp1_edge)
                        {
                            const Eigen::Vector3d normal(0.0, 0.0, 1.0);
                            const Eigen::Vector3d entry(0.0, 0.0, -1.0);
                            surface_normals_grid.InsertSurfaceNormal(x_idx, y_idx, z_idx, normal, entry);
                        }
                    }
                }
            }
        }
    }
}

simple_simulator_interface::SurfaceNormalGrid simulator_environment_builder::BuildSurfaceNormalsGrid(const std::string& environment_id, const sdf_tools::SignedDistanceField& environment_sdf)
{
    // Make the grid
    simple_simulator_interface::SurfaceNormalGrid surface_normals_grid(environment_sdf.GetOriginTransform(), environment_sdf.GetResolution(), environment_sdf.GetXSize(), environment_sdf.GetYSize(), environment_sdf.GetZSize());
    // The naive start is to fill the surface normals grid with the gradient values from the SDF
    for (int64_t x_idx = 0; x_idx < environment_sdf.GetNumXCells(); x_idx++)
    {
        for (int64_t y_idx = 0; y_idx < environment_sdf.GetNumYCells(); y_idx++)
        {
            for (int64_t z_idx = 0; z_idx < environment_sdf.GetNumZCells(); z_idx++)
            {
                const float distance = environment_sdf.Get(x_idx, y_idx, z_idx);
                if (distance < 0.0)
                {
                    const Eigen::Vector3d gradient = EigenHelpers::StdVectorDoubleToEigenVector3d(environment_sdf.GetGradient(x_idx, y_idx, z_idx, true));
                    surface_normals_grid.InsertSurfaceNormal(x_idx, y_idx, z_idx, gradient, Eigen::Vector3d(0.0, 0.0, 0.0));
                }
            }
        }
    }
    // Now, as a second pass, we add environment-specific true surface normal(s)
    if (environment_id == "nested_corners")
    {
        ;
    }
    else if (environment_id == "peg_in_hole")
    {
        ;
    }
    else if (environment_id == "se3_cluttered")
    {
        ;
    }
    else if (environment_id == "se3_box_puzzle")
    {
        ;
    }
    else if (environment_id == "empty_baxter_env")
    {
        ;
    }
    else if (environment_id == "baxter_env")
    {
        ;
    }
    else if (environment_id == "baxter_blocked_env")
    {
        //Adjustsimple_simulator_interface::SurfaceNormalGridForAllFlatSurfaces(environment_sdf, surface_normals_grid);
    }
    else if (environment_id == "baxter_blocked_test_env")
    {
        //Adjustsimple_simulator_interface::SurfaceNormalGridForAllFlatSurfaces(environment_sdf, surface_normals_grid);
    }
    else if (environment_id == "baxter_blocked_test_mod_env")
    {
        //Adjustsimple_simulator_interface::SurfaceNormalGridForAllFlatSurfaces(environment_sdf, surface_normals_grid);
    }
    else if (environment_id == "baxter_wrist_key_env")
    {
        ;
    }
    else if (environment_id == "baxter_lip_env")
    {
        ;
    }
    else if (environment_id == "blocked_peg_in_hole")
    {
        ;
    }
    else if (environment_id == "inset_peg_in_hole")
    {
        ;
    }
    else if (environment_id == "se3_cluttered")
    {
        ;
    }
    else if (environment_id == "se2_maze")
    {
        ;
    }
    else if (environment_id == "se2_maze_blocked")
    {
        ;
    }
    else if (environment_id == "noisy_arm_cluttered")
    {
        ;
    }
    else if (environment_id == "se3_cluttered_new")
    {
        ;
    }
    else if (environment_id == "se3_path_metric_test")
    {
        ;
    }
    else if (environment_id == "se2_path_metric_test")
    {
        ;
    }
    else
    {
        throw std::invalid_argument("Unrecognized environment ID");
    }
    return surface_normals_grid;
}

simple_simulator_interface::SurfaceNormalGrid simulator_environment_builder::BuildSurfaceNormalsGrid(const std::vector<OBSTACLE_CONFIG>& obstacles, const sdf_tools::SignedDistanceField& environment_sdf)
{
    // Make the grid
    simple_simulator_interface::SurfaceNormalGrid surface_normals_grid(environment_sdf.GetOriginTransform(), environment_sdf.GetResolution(), environment_sdf.GetXSize(), environment_sdf.GetYSize(), environment_sdf.GetZSize());
    // The naive start is to fill the surface normals grid with the gradient values from the SDF
    for (int64_t x_idx = 0; x_idx < environment_sdf.GetNumXCells(); x_idx++)
    {
        for (int64_t y_idx = 0; y_idx < environment_sdf.GetNumYCells(); y_idx++)
        {
            for (int64_t z_idx = 0; z_idx < environment_sdf.GetNumZCells(); z_idx++)
            {
                const float distance = environment_sdf.Get(x_idx, y_idx, z_idx);
                if (distance < 0.0)
                {
                    const Eigen::Vector3d gradient = EigenHelpers::StdVectorDoubleToEigenVector3d(environment_sdf.GetGradient(x_idx, y_idx, z_idx, true));
                    surface_normals_grid.InsertSurfaceNormal(x_idx, y_idx, z_idx, gradient, Eigen::Vector3d(0.0, 0.0, 0.0));
                }
            }
        }
    }
    // Now, as a second pass, we go through the objects and compute the true surface normal(s) for every object
    for (size_t idx = 0; idx < obstacles.size(); idx++)
    {
        const OBSTACLE_CONFIG& current_obstacle = obstacles[idx];
        const double effective_resolution = environment_sdf.GetResolution() * 0.5;
        // Generate all cells for the object
        int32_t x_cells = (int32_t)(current_obstacle.extents.x() * 2.0 * (1.0 / effective_resolution));
        int32_t y_cells = (int32_t)(current_obstacle.extents.y() * 2.0 * (1.0 / effective_resolution));
        int32_t z_cells = (int32_t)(current_obstacle.extents.z() * 2.0 * (1.0 / effective_resolution));
        for (int32_t xidx = 0; xidx < x_cells; xidx++)
        {
            for (int32_t yidx = 0; yidx < y_cells; yidx++)
            {
                for (int32_t zidx = 0; zidx < z_cells; zidx++)
                {
                    // If we're on the edge of the obstacle
                    if ((xidx == 0) || (yidx == 0) || (zidx == 0) || (xidx == (x_cells - 1)) || (yidx == (y_cells - 1)) || (zidx == (z_cells - 1)))
                    {
                        double x_location = -(current_obstacle.extents.x() - effective_resolution) + (effective_resolution * xidx);
                        double y_location = -(current_obstacle.extents.y() - effective_resolution) + (effective_resolution * yidx);
                        double z_location = -(current_obstacle.extents.z() - effective_resolution) + (effective_resolution * zidx);
                        const Eigen::Vector3d local_cell_location(x_location, y_location, z_location);
                        // Go through all 26 cases
                        // Start with the 8 corners
                        if ((xidx == 0) && (yidx == 0) && (zidx == 0))
                        {
                            const RawCellSurfaceNormal normal1(-Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(-Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY());
                            const RawCellSurfaceNormal normal3(-Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2, normal3}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == 0) && (yidx == 0) && (zidx == (z_cells - 1)))
                        {
                            const RawCellSurfaceNormal normal1(-Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(-Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY());
                            const RawCellSurfaceNormal normal3(Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2, normal3}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == 0) && (yidx == (y_cells - 1)) && (zidx == 0))
                        {
                            const RawCellSurfaceNormal normal1(-Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitY());
                            const RawCellSurfaceNormal normal3(-Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2, normal3}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == 0) && (yidx == (y_cells - 1)) && (zidx == (z_cells - 1)))
                        {
                            const RawCellSurfaceNormal normal1(-Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitY());
                            const RawCellSurfaceNormal normal3(Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2, normal3}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == (x_cells - 1)) && (yidx == 0) && (zidx == 0))
                        {
                            const RawCellSurfaceNormal normal1(Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(-Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY());
                            const RawCellSurfaceNormal normal3(-Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2, normal3}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == (x_cells - 1)) && (yidx == 0) && (zidx == (z_cells - 1)))
                        {
                            const RawCellSurfaceNormal normal1(Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(-Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY());
                            const RawCellSurfaceNormal normal3(Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2, normal3}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == (x_cells - 1)) && (yidx == (y_cells - 1)) && (zidx == 0))
                        {
                            const RawCellSurfaceNormal normal1(Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitY());
                            const RawCellSurfaceNormal normal3(-Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2, normal3}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == (x_cells - 1)) && (yidx == (y_cells - 1)) && (zidx == (z_cells - 1)))
                        {
                            const RawCellSurfaceNormal normal1(Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitY());
                            const RawCellSurfaceNormal normal3(Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2, normal3}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        // Next, let's cover the 12 edges
                        else if ((xidx == 0) && (yidx == 0))
                        {
                            const RawCellSurfaceNormal normal1(-Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(-Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == 0) && (yidx == (y_cells - 1)))
                        {
                            const RawCellSurfaceNormal normal1(-Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitY());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == (x_cells - 1)) && (yidx == 0))
                        {
                            const RawCellSurfaceNormal normal1(Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(-Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == (x_cells - 1)) && (yidx == (y_cells - 1)))
                        {
                            const RawCellSurfaceNormal normal1(Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitY());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == 0) && (zidx == 0))
                        {
                            const RawCellSurfaceNormal normal1(-Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(-Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == 0) && (zidx == (z_cells - 1)))
                        {
                            const RawCellSurfaceNormal normal1(-Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == (x_cells - 1)) && (zidx == 0))
                        {
                            const RawCellSurfaceNormal normal1(Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(-Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == (x_cells - 1)) && (zidx == (z_cells - 1)))
                        {
                            const RawCellSurfaceNormal normal1(Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((yidx == 0) && (zidx == 0))
                        {
                            const RawCellSurfaceNormal normal1(-Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY());
                            const RawCellSurfaceNormal normal2(-Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((yidx == 0) && (zidx == (z_cells - 1)))
                        {
                            const RawCellSurfaceNormal normal1(-Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY());
                            const RawCellSurfaceNormal normal2(Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((yidx == (y_cells - 1)) && (zidx == 0))
                        {
                            const RawCellSurfaceNormal normal1(Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitY());
                            const RawCellSurfaceNormal normal2(-Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((yidx == (y_cells - 1)) && (zidx == (z_cells - 1)))
                        {
                            const RawCellSurfaceNormal normal1(Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitY());
                            const RawCellSurfaceNormal normal2(Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        // Finally, let's cover the 6 faces
                        else if (xidx == 0)
                        {
                            const RawCellSurfaceNormal normal(-Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if (xidx == (x_cells - 1))
                        {
                            const RawCellSurfaceNormal normal(Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if (yidx == 0)
                        {
                            const RawCellSurfaceNormal normal(-Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if (yidx == (y_cells - 1))
                        {
                            const RawCellSurfaceNormal normal(Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitY());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if (zidx == 0)
                        {
                            const RawCellSurfaceNormal normal(-Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if (zidx == (z_cells - 1))
                        {
                            const RawCellSurfaceNormal normal(Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                    }
                }
            }
        }
    }
    return surface_normals_grid;
}

simulator_environment_builder::EnvironmentComponents simulator_environment_builder::BuildCompleteEnvironment(const std::vector<OBSTACLE_CONFIG>& obstacles, const double resolution)
{
    const sdf_tools::TaggedObjectCollisionMapGrid environment = BuildEnvironment(obstacles, resolution);
    const sdf_tools::SignedDistanceField environment_sdf = environment.ExtractSignedDistanceField(std::numeric_limits<float>::infinity(), std::vector<uint32_t>()).first;
    const simple_simulator_interface::SurfaceNormalGrid surface_normal_grid = BuildSurfaceNormalsGrid(obstacles, environment_sdf);
    return simulator_environment_builder::EnvironmentComponents(environment, environment_sdf, surface_normal_grid);
}

simulator_environment_builder::EnvironmentComponents simulator_environment_builder::BuildCompleteEnvironment(const std::string& environment_id, const double resolution)
{
    const sdf_tools::TaggedObjectCollisionMapGrid environment = BuildEnvironment(environment_id, resolution);
    const sdf_tools::SignedDistanceField environment_sdf = environment.ExtractSignedDistanceField(std::numeric_limits<float>::infinity(), std::vector<uint32_t>()).first;
    const simple_simulator_interface::SurfaceNormalGrid surface_normal_grid = BuildSurfaceNormalsGrid(environment_id, environment_sdf);
    return simulator_environment_builder::EnvironmentComponents(environment, environment_sdf, surface_normal_grid);
}

