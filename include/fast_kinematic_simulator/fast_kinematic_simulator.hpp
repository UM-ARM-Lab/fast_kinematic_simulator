#include <uncertainty_planning_core/simple_simulator_interface.hpp>
#include <fast_kinematic_simulator/simple_particle_contact_simulator.hpp>
#include <uncertainty_planning_core/uncertainty_planning_core.hpp>

#ifndef FAST_KINEMATIC_SIMULATOR_HPP
#define FAST_KINEMATIC_SIMULATOR_HPP

namespace fast_kinematic_simulator
{
    uncertainty_planning_core::SE2SimulatorPtr MakeSE2Simulator(const sdf_tools::TaggedObjectCollisionMapGrid& environment, const sdf_tools::SignedDistanceField& environment_sdf, const simple_simulator_interface::SurfaceNormalGrid& surface_normals_grid, const simple_particle_contact_simulator::SimulatorSolverParameters& solver_config, const double simulation_controller_frequency, const int32_t debug_level);

    uncertainty_planning_core::SE3SimulatorPtr MakeSE3Simulator(const sdf_tools::TaggedObjectCollisionMapGrid& environment, const sdf_tools::SignedDistanceField& environment_sdf, const simple_simulator_interface::SurfaceNormalGrid& surface_normals_grid, const simple_particle_contact_simulator::SimulatorSolverParameters& solver_config, const double simulation_controller_frequency, const int32_t debug_level);

    uncertainty_planning_core::LinkedSimulatorPtr MakeLinkedSimulator(const sdf_tools::TaggedObjectCollisionMapGrid& environment, const sdf_tools::SignedDistanceField& environment_sdf, const simple_simulator_interface::SurfaceNormalGrid& surface_normals_grid, const simple_particle_contact_simulator::SimulatorSolverParameters& solver_config, const double simulation_controller_frequency, const int32_t debug_level);
}

#endif // FAST_KINEMATIC_SIMULATOR_HPP
