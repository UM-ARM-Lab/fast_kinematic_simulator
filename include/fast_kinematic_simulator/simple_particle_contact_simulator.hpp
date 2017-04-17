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
#include <atomic>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/voxel_grid.hpp>
#include <sdf_tools/tagged_object_collision_map.hpp>
#include <sdf_tools/sdf.hpp>
#include <uncertainty_planning_core/simple_simulator_interface.hpp>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <omp.h>

#ifndef SIMPLE_PARTICLE_CONTACT_SIMULATOR_HPP
#define SIMPLE_PARTICLE_CONTACT_SIMULATOR_HPP

namespace simple_particle_contact_simulator
{
    struct SimulatorSolverParameters
    {
        double environment_collision_check_tolerance;
        double resolve_correction_step_scaling_decay_rate;
        double resolve_correction_initial_step_size;
        double resolve_correction_min_step_scaling;
        uint32_t max_resolver_iterations;
        uint32_t resolve_correction_step_scaling_decay_iterations;
        bool failed_resolves_end_motion;

        SimulatorSolverParameters()
        {
            environment_collision_check_tolerance = 0.001;
            resolve_correction_step_scaling_decay_rate = 0.5;
            resolve_correction_initial_step_size = 1.0;
            resolve_correction_min_step_scaling = 0.03125;
            max_resolver_iterations = 25;
            resolve_correction_step_scaling_decay_iterations = 5;
            failed_resolves_end_motion = true;
        }
    };

    template<typename Robot, typename Configuration, typename RNG, typename ConfigAlloc=std::allocator<Configuration>>
    class SimpleParticleContactSimulator : public simple_simulator_interface::SimulatorInterface<Robot, Configuration, RNG, ConfigAlloc>
    {
    protected:

        bool initialized_;
        double simulation_controller_frequency_;
        double simulation_controller_interval_;
        double contact_distance_threshold_;
        double resolution_distance_threshold_;
        SimulatorSolverParameters solver_config_;
        mutable std::atomic<uint64_t> simulation_call_;
        mutable std::atomic<uint64_t> successful_resolves_;
        mutable std::atomic<uint64_t> unsuccessful_resolves_;
        mutable std::atomic<uint64_t> free_resolves_;
        mutable std::atomic<uint64_t> collision_resolves_;
        mutable std::atomic<uint64_t> fallback_resolves_;
        mutable std::atomic<uint64_t> unsuccessful_env_collision_resolves_;
        mutable std::atomic<uint64_t> unsuccessful_self_collision_resolves_;
        mutable std::atomic<uint64_t> recovered_unsuccessful_resolves_;

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        SimpleParticleContactSimulator(const sdf_tools::TaggedObjectCollisionMapGrid& environment, const sdf_tools::SignedDistanceField& environment_sdf, const simple_simulator_interface::SurfaceNormalGrid& surface_normals_grid, const SimulatorSolverParameters& solver_config, const double simulation_controller_frequency, const int32_t debug_level) : simple_simulator_interface::SimulatorInterface<Robot, Configuration, RNG, ConfigAlloc>(environment, environment_sdf, surface_normals_grid, debug_level)
        {
            contact_distance_threshold_ = 0.0;
            resolution_distance_threshold_ = 0.0;
            simulation_controller_frequency_ = std::abs(simulation_controller_frequency);
            simulation_controller_interval_ = 1.0 / simulation_controller_frequency;
            solver_config_ = solver_config;
            simulation_call_.store(0);
            ResetStatistics();
            initialized_ = true;
        }

        inline bool IsInitialized() const
        {
            return initialized_;
        }

        virtual std::map<std::string, double> GetStatistics() const
        {
            std::map<std::string, double> statistics;
            statistics["successful_resolves"] = (double)(successful_resolves_.load());
            statistics["unsuccessful_resolves"] = (double)(unsuccessful_resolves_.load());
            statistics["free_resolves"] = (double)(free_resolves_.load());
            statistics["collision_resolves"] = (double)(collision_resolves_.load());
            statistics["fallback_resolves"] = (double)(fallback_resolves_.load());
            statistics["unsuccessful_self_collision_resolves"] = (double)(unsuccessful_self_collision_resolves_.load());
            statistics["unsuccessful_env_collision_resolves"] = (double)(unsuccessful_env_collision_resolves_.load());
            statistics["recovered_unsuccessful_resolves"] = (double)(recovered_unsuccessful_resolves_.load());
            return statistics;
        }

        virtual void ResetStatistics()
        {
            successful_resolves_.store(0);
            unsuccessful_resolves_.store(0);
            free_resolves_.store(0);
            collision_resolves_.store(0);
            fallback_resolves_.store(0);
            unsuccessful_self_collision_resolves_.store(0);
            unsuccessful_env_collision_resolves_.store(0);
            recovered_unsuccessful_resolves_.store(0);
        }

        virtual std::vector<std::pair<Configuration, bool>> ForwardSimulateRobots(const Robot& immutable_robot, const std::vector<Configuration, ConfigAlloc>& start_positions, const std::vector<Configuration, ConfigAlloc>& target_positions, std::vector<RNG>& rngs, const double forward_simulation_time, const double simulation_shortcut_distance, const bool use_individual_jacobians, const bool allow_contacts, ros::Publisher& display_debug_publisher) const
        {
            if (start_positions.size() > 0)
            {
                assert((target_positions.size() == 1) || (target_positions.size() == start_positions.size()));
            }
            std::vector<std::pair<Configuration, bool>> propagated_points(start_positions.size());
            #pragma omp parallel for
            for (size_t idx = 0; idx < start_positions.size(); idx++)
            {
                const Configuration& initial_particle = start_positions[idx];
                const Configuration& target_position = (target_positions.size() == start_positions.size()) ? target_positions[idx] : target_positions.front();
                simple_simulator_interface::ForwardSimulationStepTrace<Configuration, ConfigAlloc> trace;
#if defined(_OPENMP)
                const size_t th_id = (size_t)omp_get_thread_num();
#else
                const size_t th_id = 0;
#endif
                propagated_points[idx] = ForwardSimulateRobot(immutable_robot, initial_particle, target_position, rngs[th_id], forward_simulation_time, simulation_shortcut_distance, use_individual_jacobians, allow_contacts, trace, false, display_debug_publisher);
            }
            return propagated_points;
        }

        virtual std::pair<Configuration, bool> ForwardSimulateRobot(const Robot& immutable_robot, const Configuration& start_position, const Configuration& target_position, RNG& rng, const double forward_simulation_time, const double simulation_shortcut_distance, const bool use_individual_jacobians, const bool allow_contacts, simple_simulator_interface::ForwardSimulationStepTrace<Configuration, ConfigAlloc>& trace, const bool enable_tracing, ros::Publisher& display_debug_publisher) const
        {
            Robot robot = immutable_robot;
            robot.ResetPosition(start_position);
            return ForwardSimulateMutableRobot(robot, target_position, rng, forward_simulation_time, simulation_shortcut_distance, use_individual_jacobians, allow_contacts, trace, enable_tracing, display_debug_publisher);
        }

        virtual std::pair<Configuration, bool> ForwardSimulateMutableRobot(Robot& robot, const Configuration& target_position, RNG& rng, const double forward_simulation_time, const double simulation_shortcut_distance, const bool use_individual_jacobians, const bool allow_contacts, simple_simulator_interface::ForwardSimulationStepTrace<Configuration, ConfigAlloc>& trace, const bool enable_tracing, ros::Publisher& display_debug_publisher) const
        {
            simulation_call_.fetch_add(1u);
            const uint64_t call_number = simulation_call_.load();
            const Configuration start_position = robot.GetPosition();
            // Forward simulate for the provided number of steps
            bool collided = false;
            const uint32_t forward_simulation_steps = std::max((uint32_t)(forward_simulation_time * simulation_controller_frequency_), 1u);
            bool any_resolve_failed = false;
            if (this->debug_level_ >= 1)
            {
                const std::string msg = "[" + std::to_string(call_number) + "] Starting simulation:\nStart: " + PrettyPrint::PrettyPrint(start_position) + "\nTarget: " + PrettyPrint::PrettyPrint(target_position);
                std::cout << msg << std::endl;
            }
            for (uint32_t step = 0; step < forward_simulation_steps; step++)
            {
                // Step forward via the simulator
                // Have robot compute next control input first
                // Then, in a second function *not* in a callback, apply that control input
                const Eigen::VectorXd control_action = robot.GenerateControlAction(target_position, simulation_controller_interval_, rng);
                const std::pair<Configuration, std::pair<bool, bool>> result = ResolveForwardSimulation(robot, control_action, simulation_controller_interval_, rng, use_individual_jacobians, allow_contacts, trace, enable_tracing, call_number, display_debug_publisher);
                const Configuration& resolved_configuration = result.first;
                const bool resolve_collided = result.second.first;
                const bool current_resolve_failed = result.second.second;
                if ((allow_contacts == true) || (resolve_collided == false))
                {
                    robot.UpdatePosition(resolved_configuration);
                    // Check if we've collided with the environment
                    if (resolve_collided)
                    {
                        collided = true;
                    }
                    // Check for stats
                    if (current_resolve_failed)
                    {
                        if (solver_config_.failed_resolves_end_motion)
                        {
                            break;
                        }
                        any_resolve_failed = true;
                    }
                    else
                    {
                        if (any_resolve_failed)
                        {
                            recovered_unsuccessful_resolves_.fetch_add(1);
                        }
                    }
                    // Last, but not least, check if we've gotten close enough the target state to short-circut the simulation
                    const double target_distance = robot.ComputeDistanceTo(target_position);
                    if (target_distance < simulation_shortcut_distance)
                    {
                        break;
                    }
                }
                else
                {
                    assert(resolve_collided == true);
                    // If we don't allow contacts, we don't update the position and we stop simulating
                    break;
                }
            }
            // Return the ending position of the robot and if it has collided during simulation
            const Configuration reached_position = robot.GetPosition();
            if (this->debug_level_ >= 1)
            {
                const std::string msg = "[" + std::to_string(call_number) + "] Forward simulated in " + std::to_string(forward_simulation_steps) + " steps from\nStart: " + PrettyPrint::PrettyPrint(start_position) + "\nTarget: " + PrettyPrint::PrettyPrint(target_position) + "\nReached: " + PrettyPrint::PrettyPrint(reached_position);
                std::cout << msg << std::endl;
            }
            return std::pair<Configuration, bool>(reached_position, collided);
        }

        inline bool CheckEnvironmentCollision(const Robot& robot, const std::vector<std::pair<std::string, std::shared_ptr<EigenHelpers::VectorVector4d>>>& robot_links_points, const double collision_threshold) const
        {
            const double real_collision_threshold = collision_threshold - (solver_config_.environment_collision_check_tolerance * this->environment_sdf_.GetResolution());
            // Now, go through the links and points of the robot for collision checking
            for (size_t link_idx = 0; link_idx < robot_links_points.size(); link_idx++)
            {
                // Grab the link name and points
                const std::string& link_name = robot_links_points[link_idx].first;
                const EigenHelpers::VectorVector4d& link_points = *(robot_links_points[link_idx].second);
                // Get the transform of the current link
                const Eigen::Affine3d link_transform = robot.GetLinkTransform(link_name);
                // Now, go through the points of the link
                for (size_t point_idx = 0; point_idx < link_points.size(); point_idx++)
                {
                    // Transform the link point into the environment frame
                    const Eigen::Vector4d& link_relative_point = link_points[point_idx];
                    const Eigen::Vector4d environment_relative_point = link_transform * link_relative_point;
                    const std::pair<float, bool> sdf_check = this->environment_sdf_.GetSafe4d(environment_relative_point);
                    //const std::pair<double, bool> sdf_check = this->environment_sdf_.EstimateDistance(environment_relative_point);
                    if (sdf_check.second == false)
                    {
                        if (this->debug_level_ >= 1)
                        {
                            const std::string msg = "Point at " + PrettyPrint::PrettyPrint(environment_relative_point) + " out of bounds";
                            std::cerr << msg << std::endl;
                        }
#ifdef ASSERT_ON_OUT_OF_BOUNDS
                        assert(false);
#endif
                    }
                    // We only work with points in collision
                    if (sdf_check.first < real_collision_threshold)
                    {
                        if (sdf_check.first < (real_collision_threshold - this->environment_sdf_.GetResolution()))
                        {
                            if (this->debug_level_ >= 25)
                            {
                                std::cout << "Point at " << PrettyPrint::PrettyPrint(environment_relative_point) << " in collision with SDF distance " << sdf_check.first << " and threshold " << real_collision_threshold;
                            }
                            return true;
                        }
                        else
                        {
                            const double estimated_distance = this->environment_sdf_.EstimateDistance4d(environment_relative_point).first;
                            if (estimated_distance < real_collision_threshold)
                            {
                                if (this->debug_level_ >= 25)
                                {
                                    std::cout << "Point at " << PrettyPrint::PrettyPrint(environment_relative_point) << " in collision with SDF distance " << sdf_check.first << " and threshold " << real_collision_threshold;
                                }
                                return true;
                            }
                        }
                    }
                }
            }
            return false;
        }

        inline std::map<std::pair<size_t, size_t>, Eigen::Vector3d> ExtractSelfCollidingPoints(const Robot& previous_robot, const Robot& current_robot, const std::vector<std::pair<std::string, std::shared_ptr<EigenHelpers::VectorVector4d>>>& robot_links_points, const std::vector<std::pair<size_t, size_t>>& candidate_points, const std::map<size_t, double>& link_masses, const double time_interval) const
        {
            if (candidate_points.size() > 1)
            {
                // Now, we separate the points by link
                std::map<size_t, std::vector<size_t>> point_self_collision_check_map;
                for (size_t idx = 0; idx < candidate_points.size(); idx++)
                {
                    const std::pair<size_t, size_t>& point = candidate_points[idx];
                    point_self_collision_check_map[point.first].push_back(point.second);
                }
                //std::cout << "Considering " << point_self_collision_check_map.size() << " separate links with self-colliding points" << std::endl;
                // Let's see how many links we have - we only care if multiple links are involved
                if (point_self_collision_check_map.size() >= 2)
                {
                    // For each link, figure out which *other* links it is colliding with
                    std::map<size_t, std::vector<size_t>> link_collisions;
                    for (auto fitr = point_self_collision_check_map.begin(); fitr != point_self_collision_check_map.end(); ++fitr)
                    {
                        for (auto sitr = point_self_collision_check_map.begin(); sitr != point_self_collision_check_map.end(); ++sitr)
                        {
                            if (fitr != sitr)
                            {
                                const size_t fitr_link = fitr->first;
                                const size_t sitr_link = sitr->first;
                                const bool self_collision_allowed = current_robot.CheckIfSelfCollisionAllowed(fitr_link, sitr_link);
                                if (self_collision_allowed == false)
                                {
                                    //const std::string msg = "Self collisions not allowed between " + std::to_string(fitr_link) + " and " + std::to_string(sitr_link);
                                    //std::cout << msg << std::endl;
                                    link_collisions[fitr_link].push_back(sitr_link);
                                }
                            }
                        }
                    }
                    if (link_collisions.size() < 2)
                    {
                        return std::map<std::pair<size_t, size_t>, Eigen::Vector3d>();
                    }
                    //std::cout << "Self collisions: " << PrettyPrint::PrettyPrint(link_collisions) << std::endl;
                    // We go through each link and compute an "input momentum" vector that reflects the contribution of the particular link to the collision
                    // We can assume that point motion has occurred over unit time, so motion = velocity, and that each point has unit mass, so momentum = velocity.
                    // Thus, the momentum = motion for each particle
                    // !!! FIX TO INCLUDE LINK & FURTHER LINK(S) MASS SO LOWER LINKS MOVE LESS !!!
                    const double time_multiplier = 1.0 / time_interval;
                    std::map<size_t, Eigen::Vector4d> link_momentum_vectors;
                    for (auto link_itr = point_self_collision_check_map.begin(); link_itr != point_self_collision_check_map.end(); ++link_itr)
                    {
                        const size_t link_idx = link_itr->first;
                        // Skip links we already filtered out due to allowed self collision
                        if (link_collisions.find(link_idx) != link_collisions.end())
                        {
                            const std::string& link_name = robot_links_points[link_itr->first].first;
                            const Eigen::Affine3d previous_link_transform = previous_robot.GetLinkTransform(link_name);
                            const Eigen::Affine3d current_link_transform = current_robot.GetLinkTransform(link_name);
                            const std::vector<size_t>& link_points = link_itr->second;
                            Eigen::Vector4d link_momentum_vector(0.0, 0.0, 0.0, 0.0);
                            for (size_t idx = 0; idx < link_points.size(); idx++)
                            {
                                const size_t link_point = link_points[idx];
                                const Eigen::Vector4d& link_relative_point = (*robot_links_points[link_idx].second)[link_point];
                                const Eigen::Vector4d previous_point_location = previous_link_transform * link_relative_point;
                                const Eigen::Vector4d current_point_location = current_link_transform * link_relative_point;
                                const Eigen::Vector4d point_motion = current_point_location - previous_point_location;
                                const Eigen::Vector4d point_velocity = point_motion * time_multiplier; // point motion/interval * interval/sec
                                if (point_velocity.norm() <= std::numeric_limits<double>::epsilon())
                                {
                                    //const std::string msg = "Point motion would be zero (link " + std::to_string(link_idx) + ", point " + std::to_string(link_point) + ")\nPrevious location: " + PrettyPrint::PrettyPrint(previous_point_location) + "\nCurrent location: " + PrettyPrint::PrettyPrint(current_point_location);
                                    //std::cout << msg << std::endl;
                                }
                                link_momentum_vector = link_momentum_vector + point_velocity;
                            }
                            link_momentum_vectors[link_idx] = link_momentum_vector;
                        }
                    }
                    //std::cout << "Link momentum vectors:\n" << PrettyPrint::PrettyPrint(link_momentum_vectors) << std::endl;
                    // Store the corrections we compute
                    std::map<std::pair<size_t, size_t>, Eigen::Vector3d> self_colliding_points_with_corrections;
                    // Now, for each link, we compute a correction for each colliding point on the link
                    for (auto link_itr = link_collisions.begin(); link_itr != link_collisions.end(); ++link_itr)
                    {
                        const size_t link_idx = link_itr->first;
                        const std::string& link_name = robot_links_points[link_idx].first;
                        const Eigen::Affine3d previous_link_transform = previous_robot.GetLinkTransform(link_name);
                        const Eigen::Vector4d& link_point = (*robot_links_points[link_idx].second)[point_self_collision_check_map[link_idx].front()];
                        const Eigen::Vector4d link_point_location = previous_link_transform * link_point;
                        const std::vector<size_t>& colliding_links = link_itr->second;
                        const auto link_momentum_vector_query = link_momentum_vectors.find(link_idx);
                        assert(link_momentum_vector_query != link_momentum_vectors.end());
                        const Eigen::Vector4d link_momentum_vector = link_momentum_vector_query->second;
                        const Eigen::Vector4d link_velocity = link_momentum_vector / (double)point_self_collision_check_map[link_idx].size();
                        // We compute a whole-link correction
                        // For the purposes of simulation, we assume an elastic collision - i.e. momentum must be conserved
                        Eigen::MatrixXd contact_matrix = Eigen::MatrixXd::Zero((ssize_t)((colliding_links.size() + 1) * 3), (ssize_t)(colliding_links.size() * 3));
                        // For each link, fill in the contact matrix
                        for (int64_t link = 1; link <= (int64_t)colliding_links.size(); link++)
                        {
                            int64_t collision_number = link - 1;
                            // Our current link gets -I
                            contact_matrix.block<3, 3>(0, (collision_number * 3)) = (Eigen::MatrixXd::Identity(3, 3) * -1.0);
                            // The other link gets +I
                            contact_matrix.block<3, 3>((link * 3), (collision_number * 3)) = Eigen::MatrixXd::Identity(3, 3);
                        }
                        //std::cout << "Contact matrix:\n" << PrettyPrint::PrettyPrint(contact_matrix) << std::endl;
                        // Generate the contact normal matrix
                        Eigen::MatrixXd contact_normal_matrix = Eigen::MatrixXd::Zero((ssize_t)(colliding_links.size() * 3), (ssize_t)(colliding_links.size()));
                        for (int64_t collision = 0; collision < (int64_t)colliding_links.size(); collision++)
                        {
                            const size_t other_link_idx = colliding_links[(size_t)collision];
                            const std::string& other_link_name = robot_links_points[other_link_idx].first;
                            const Eigen::Affine3d previous_other_link_transform = previous_robot.GetLinkTransform(other_link_name);
                            const Eigen::Vector4d& other_link_point = (*robot_links_points[other_link_idx].second)[point_self_collision_check_map[other_link_idx].front()];
                            const Eigen::Vector4d other_link_point_location = previous_other_link_transform * other_link_point;
                            // Compute the contact normal
                            //const Eigen::Vector3d other_link_velocity = link_momentum_vectors[other_link_idx] / (double)point_self_collision_check_map[other_link_idx].size();
                            //const Eigen::Vector3d current_link_position = link_velocity * -1.0;
                            //const Eigen::Vector3d current_other_link_position = other_link_velocity * -1.0;
                            //const Eigen::Vector3d contact_direction = current_other_link_position - current_link_position;
                            const Eigen::Vector4d contact_direction = other_link_point_location - link_point_location;
                            const Eigen::Vector4d contact_normal = EigenHelpers::SafeNormal(contact_direction);
                            //const std::string msg = "Contact normal: " + PrettyPrint::PrettyPrint(contact_normal) + "\nCurrent link position: " + PrettyPrint::PrettyPrint(link_point_location) + "\nOther link position: " + PrettyPrint::PrettyPrint(other_link_point_location);
                            //std::cout << msg << std::endl;
                            contact_normal_matrix.block<3, 1>((collision * 3), collision) = contact_normal.block<3, 1>(0, 0);
                        }
                        //std::cout << "Contact normal matrix:\n" << PrettyPrint::PrettyPrint(contact_normal_matrix) << std::endl;
                        // Generate the mass matrix
                        Eigen::MatrixXd mass_matrix = Eigen::MatrixXd::Zero((ssize_t)((colliding_links.size() + 1) * 3), (ssize_t)((colliding_links.size() + 1) * 3));
                        // Add the mass of our link
                        const double link_mass = link_masses.find(link_idx)->second; // (double)point_self_collision_check_map[link_idx].size();
                        mass_matrix.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3) * link_mass;
                        // Add the mass of the other links
                        for (int64_t link = 1; link <= (int64_t)colliding_links.size(); link++)
                        {
                            const size_t other_link_idx = colliding_links[(size_t)(link - 1)];
                            const double other_link_mass = link_masses.find(other_link_idx)->second; // (double)point_self_collision_check_map[other_link_idx].size();
                            mass_matrix.block<3, 3>((link * 3), (link * 3)) = Eigen::MatrixXd::Identity(3, 3) * other_link_mass;
                        }
                        //std::cout << "Mass matrix:\n" << PrettyPrint::PrettyPrint(mass_matrix) << std::endl;
                        // Generate the velocity matrix
                        Eigen::MatrixXd velocity_matrix = Eigen::MatrixXd::Zero((ssize_t)((colliding_links.size() + 1) * 3), (ssize_t)1);
                        velocity_matrix.block<3, 1>(0, 0) = link_velocity.block<3, 1>(0, 0);
                        for (int64_t link = 1; link <= (int64_t)colliding_links.size(); link++)
                        {
                            const size_t other_link_idx = colliding_links[(size_t)(link - 1)];
                            const Eigen::Vector4d other_link_velocity = link_momentum_vectors[other_link_idx] / (double)point_self_collision_check_map[other_link_idx].size();
                            velocity_matrix.block<3, 1>((link * 3), 0) = other_link_velocity.block<3, 1>(0, 0);
                        }
                        //std::cout << "Velocity matrix:\n" << PrettyPrint::PrettyPrint(velocity_matrix) << std::endl;
                        // Compute the impulse corrections
                        // Yes, this is ugly. This is to suppress a warning on type conversion related to Eigen operations
                        #pragma GCC diagnostic push
                        #pragma GCC diagnostic ignored "-Wconversion"
                        const Eigen::MatrixXd impulses = (contact_normal_matrix.transpose() * contact_matrix.transpose() * mass_matrix.inverse() * contact_matrix * contact_normal_matrix).inverse() * contact_normal_matrix.transpose() * contact_matrix.transpose() * velocity_matrix;
                        //std::cout << "Impulses:\n" << PrettyPrint::PrettyPrint(impulses) << std::endl;
                        // Compute the new velocities
                        const Eigen::MatrixXd velocity_delta = (mass_matrix.inverse() * contact_matrix * contact_normal_matrix * impulses) * -1.0;
                        //std::cout << "New velocities:\n" << velocity_delta << std::endl;
                        #pragma GCC diagnostic pop
                        // Extract the correction just for our current link
                        const Eigen::Vector3d link_correction_velocity = velocity_delta.block<3, 1>(0, 0);
                        // We then distribute that correction over the points on that link that have contributed to the collision
                        const std::vector<size_t>& link_points = point_self_collision_check_map[link_idx];
                        for (size_t idx = 0; idx < link_points.size(); idx++)
                        {
                            const size_t point_idx = link_points[idx];
                            const std::pair<size_t, size_t> point_id(link_idx, point_idx);
                            //std::cout << "Link correction velocity: " << PrettyPrint::PrettyPrint(link_correction_velocity) << std::endl;
                            const Eigen::Vector3d point_correction = link_correction_velocity / (double)(link_points.size());
                            assert(std::isnan(point_correction.x()) == false);
                            assert(std::isnan(point_correction.y()) == false);
                            assert(std::isnan(point_correction.z()) == false);
                            //std::cout << "Correction (new): " << PrettyPrint::PrettyPrint(point_id) << " - " << PrettyPrint::PrettyPrint(point_correction) << std::endl;
                            self_colliding_points_with_corrections[point_id] = point_correction;
                        }
                    }
                    return self_colliding_points_with_corrections;
                }
                // One link cannot be self-colliding
                else
                {
                    return std::map<std::pair<size_t, size_t>, Eigen::Vector3d>();
                }
            }
            // One point cannot be self-colliding
            else
            {
                return std::map<std::pair<size_t, size_t>, Eigen::Vector3d>();
            }
        }

        inline std::vector<int64_t> LocationToExtendedGridIndex(const Eigen::Vector4d& location, const double extended_grid_resolution) const
        {
            assert(initialized_);
            const Eigen::Vector4d point_in_grid_frame = this->environment_.GetInverseOriginTransform() * location;
            const int64_t x_cell = (int64_t)(point_in_grid_frame(0) / extended_grid_resolution);
            const int64_t y_cell = (int64_t)(point_in_grid_frame(1) / extended_grid_resolution);
            const int64_t z_cell = (int64_t)(point_in_grid_frame(2) / extended_grid_resolution);
            return std::vector<int64_t>{x_cell, y_cell, z_cell};
        }

        inline std::unordered_map<std::pair<size_t, size_t>, Eigen::Vector3d> CollectSelfCollisions(const Robot& previous_robot, const Robot& current_robot, const std::vector<std::pair<std::string, std::shared_ptr<EigenHelpers::VectorVector4d>>>& robot_links_points, const double time_interval) const
        {
            // Note that robots with only one link *cannot* self-collide!
            if (robot_links_points.size() == 1)
            {
                return std::unordered_map<std::pair<size_t, size_t>, Eigen::Vector3d>();
            }
            else if (robot_links_points.size() == 2)
            {
                // If the robot is only two links, and self-collision between them is allowed, we can avoid checks
                if (current_robot.CheckIfSelfCollisionAllowed(0, 1))
                {
                    return std::unordered_map<std::pair<size_t, size_t>, Eigen::Vector3d>();
                }
            }
            // We use a hashtable to detect self-collisions
            std::unordered_map<VoxelGrid::GRID_INDEX, std::vector<std::pair<size_t, size_t>>> self_collision_check_map;
            // Now, go through the links and points of the robot for collision checking
            bool any_candidate_self_collisions = false;
            for (size_t link_idx = 0; link_idx < robot_links_points.size(); link_idx++)
            {
                // Grab the link name and points
                const std::string& link_name = robot_links_points[link_idx].first;
                const EigenHelpers::VectorVector4d& link_points = *(robot_links_points[link_idx].second);
                // Get the transform of the current link
                const Eigen::Affine3d link_transform = current_robot.GetLinkTransform(link_name);
                // Now, go through the points of the link
                for (size_t point_idx = 0; point_idx < link_points.size(); point_idx++)
                {
                    // Transform the link point into the environment frame
                    const Eigen::Vector4d& link_relative_point = link_points[point_idx];
                    const Eigen::Vector4d environment_relative_point = link_transform * link_relative_point;
                    // Get the corresponding index
                    const std::vector<int64_t> index = LocationToExtendedGridIndex(environment_relative_point, this->environment_.GetResolution());
                    assert(index.size() == 3);
                    VoxelGrid::GRID_INDEX point_index(index[0], index[1], index[2]);
                    // Insert the index into the map
                    std::vector<std::pair<size_t, size_t>>& map_cell = self_collision_check_map[point_index];
                    if (map_cell.size() > 1)
                    {
                        any_candidate_self_collisions = true;
                    }
                    else if (map_cell.size() == 1)
                    {
                        const std::pair<size_t, size_t>& current = map_cell[0];
                        if (current.first != link_idx)
                        {
                            any_candidate_self_collisions = true;
                        }
                    }
                    map_cell.push_back(std::pair<size_t, size_t>(link_idx, point_idx));
                }
            }
            if (any_candidate_self_collisions == false)
            {
                return std::unordered_map<std::pair<size_t, size_t>, Eigen::Vector3d>();
            }
            // Compute approximate link masses (in reverse order, since each link's mass is its mass + mass of all further links
            std::map<size_t, double> link_masses;
            double previous_link_masses = 0.0;
            for (int64_t link_idx = ((int64_t)robot_links_points.size() - 1); link_idx >= 0; link_idx--)
            {
                const double link_mass = (double)((*robot_links_points[(size_t)link_idx].second).size());
                link_masses[(size_t)link_idx] = link_mass + previous_link_masses;
                previous_link_masses += link_mass;
            }
            // Now, we go through the map and see if any points overlap
            // Store "true" self-collisions
            std::unordered_map<std::pair<size_t, size_t>, Eigen::Vector3d> self_collisions;
            for (auto itr = self_collision_check_map.begin(); itr != self_collision_check_map.end(); ++itr)
            {
                //const VoxelGrid::GRID_INDEX& location = itr->first;
                const std::vector<std::pair<size_t, size_t>>& candidate_points = itr->second;
                //std::cout << "Candidate points: " << PrettyPrint::PrettyPrint(candidate_points) << std::endl;
                const std::map<std::pair<size_t, size_t>, Eigen::Vector3d> self_colliding_points = ExtractSelfCollidingPoints(previous_robot, current_robot, robot_links_points, candidate_points, link_masses, time_interval);
                //std::cout << "Extracted points: " << PrettyPrint::PrettyPrint(self_colliding_points) << std::endl;
                for (auto spcitr = self_colliding_points.begin(); spcitr != self_colliding_points.end(); ++spcitr)
                {
                    const std::pair<size_t, size_t>& self_colliding_point = spcitr->first;
                    const Eigen::Vector3d& correction = spcitr->second;
                    self_collisions[self_colliding_point] = correction;
                }
            }
            // If we haven't already returned, we are self-collision-free
            return self_collisions;
        }

        inline bool CheckPointsForSelfCollision(const Robot& current_robot, const std::vector<std::pair<size_t, size_t>>& candidate_points) const
        {
            if (candidate_points.size() > 1)
            {
                // Now, we separate the points by link
                std::map<size_t, std::vector<size_t>> point_self_collision_check_map;
                for (size_t idx = 0; idx < candidate_points.size(); idx++)
                {
                    const std::pair<size_t, size_t>& point = candidate_points[idx];
                    point_self_collision_check_map[point.first].push_back(point.second);
                }
                //std::cout << "Considering " << point_self_collision_check_map.size() << " separate links with self-colliding points" << std::endl;
                // Let's see how many links we have - we only care if multiple links are involved
                if (point_self_collision_check_map.size() >= 2)
                {
                    // For each link, figure out which *other* links it is colliding with
                    for (auto fitr = point_self_collision_check_map.begin(); fitr != point_self_collision_check_map.end(); ++fitr)
                    {
                        for (auto sitr = point_self_collision_check_map.begin(); sitr != point_self_collision_check_map.end(); ++sitr)
                        {
                            if (fitr != sitr)
                            {
                                const size_t fitr_link = fitr->first;
                                const size_t sitr_link = sitr->first;
                                const bool self_collision_allowed = current_robot.CheckIfSelfCollisionAllowed(fitr_link, sitr_link);
                                if (self_collision_allowed == false)
                                {
                                    return true;
                                }
                            }
                        }
                    }
                    return false;
                }
                // One link cannot be self-colliding
                else
                {
                    return false;
                }
            }
            // One point cannot be self-colliding
            else
            {
                return false;
            }
        }

        inline bool CheckSelfCollisions(const Robot& current_robot, const std::vector<std::pair<std::string, std::shared_ptr<EigenHelpers::VectorVector4d>>>& robot_links_points, const double check_resolution) const
        {
            // Note that robots with only one link *cannot* self-collide!
            if (robot_links_points.size() == 1)
            {
                return false;
            }
            else if (robot_links_points.size() == 2)
            {
                // If the robot is only two links, and self-collision between them is allowed, we can avoid checks
                if (current_robot.CheckIfSelfCollisionAllowed(0, 1))
                {
                    return false;
                }
            }
            // We use a hashtable to detect self-collisions
            std::unordered_map<VoxelGrid::GRID_INDEX, std::vector<std::pair<size_t, size_t>>> self_collision_check_map;
            // Now, go through the links and points of the robot for collision checking
            bool any_candidate_self_collisions = false;
            for (size_t link_idx = 0; link_idx < robot_links_points.size(); link_idx++)
            {
                // Grab the link name and points
                const std::string& link_name = robot_links_points[link_idx].first;
                const EigenHelpers::VectorVector4d& link_points = *(robot_links_points[link_idx].second);
                // Get the transform of the current link
                const Eigen::Affine3d link_transform = current_robot.GetLinkTransform(link_name);
                // Now, go through the points of the link
                for (size_t point_idx = 0; point_idx < link_points.size(); point_idx++)
                {
                    // Transform the link point into the environment frame
                    const Eigen::Vector4d& link_relative_point = link_points[point_idx];
                    const Eigen::Vector4d environment_relative_point = link_transform * link_relative_point;
                    // Get the corresponding index
                    const std::vector<int64_t> index = LocationToExtendedGridIndex(environment_relative_point, check_resolution);
                    assert(index.size() == 3);
                    VoxelGrid::GRID_INDEX point_index(index[0], index[1], index[2]);
                    // Insert the index into the map
                    std::vector<std::pair<size_t, size_t>>& map_cell = self_collision_check_map[point_index];
                    if (map_cell.size() > 1)
                    {
                        any_candidate_self_collisions = true;
                    }
                    else if (map_cell.size() == 1)
                    {
                        const std::pair<size_t, size_t>& current = map_cell[0];
                        if (current.first != link_idx)
                        {
                            any_candidate_self_collisions = true;
                        }
                    }
                    map_cell.push_back(std::pair<size_t, size_t>(link_idx, point_idx));
                }
            }
            if (any_candidate_self_collisions == false)
            {
                return false;
            }
            // Now, we go through the map and see if any points overlap
            for (auto itr = self_collision_check_map.begin(); itr != self_collision_check_map.end(); ++itr)
            {
                const std::vector<std::pair<size_t, size_t>>& candidate_points = itr->second;
                const bool self_collision_detected = CheckPointsForSelfCollision(current_robot, candidate_points);
                if (self_collision_detected)
                {
                    return true;
                }
            }
            // If we haven't already returned, we are self-collision-free
            return false;
        }

        virtual bool CheckConfigCollision(const Robot& immutable_robot, const Configuration& config, const double inflation_ratio) const
        {
            Robot current_robot = immutable_robot;
            current_robot.UpdatePosition(config);
            const std::vector<std::pair<std::string, std::shared_ptr<EigenHelpers::VectorVector4d>>> robot_links_points = current_robot.GetRawLinksPoints();
            const double environment_collision_distance_threshold = inflation_ratio * this->environment_.GetResolution();
            const double self_collision_check_resolution = (inflation_ratio + 1.0) * this->environment_.GetResolution();
            const bool env_collision = CheckEnvironmentCollision(current_robot, robot_links_points, environment_collision_distance_threshold);
            const bool self_collision = CheckSelfCollisions(current_robot, robot_links_points, self_collision_check_resolution);
            //std::cout << self_collisions.size() << " self-colliding points to resolve" << std::endl;
            if (env_collision || self_collision)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        inline std::pair<bool, std::unordered_map<std::pair<size_t, size_t>, Eigen::Vector3d>> CheckCollision(const Robot& robot, const Configuration& previous_config, const Configuration& current_config, const std::vector<std::pair<std::string, std::shared_ptr<EigenHelpers::VectorVector4d>>>& robot_links_points, const double time_interval) const
        {
            Robot current_robot = robot;
            Robot previous_robot = robot;
            // We need our own copies with a set config to use for kinematics!
            current_robot.UpdatePosition(current_config);
            previous_robot.UpdatePosition(previous_config);
            const bool env_collision = CheckEnvironmentCollision(current_robot, robot_links_points, contact_distance_threshold_);
            const std::unordered_map<std::pair<size_t, size_t>, Eigen::Vector3d> self_collisions = CollectSelfCollisions(previous_robot, current_robot, robot_links_points, time_interval);
            //std::cout << self_collisions.size() << " self-colliding points to resolve" << std::endl;
            if (env_collision || (self_collisions.size() > 0))
            {
                return std::pair<bool, std::unordered_map<std::pair<size_t, size_t>, Eigen::Vector3d>>(true, self_collisions);
            }
            else
            {
                return std::pair<bool, std::unordered_map<std::pair<size_t, size_t>, Eigen::Vector3d>>(false, self_collisions);
            }
        }

        inline double EstimatePointPenetration(const Eigen::Vector4d& point) const
        {
            const std::pair<double, bool> distance_check = this->environment_sdf_.EstimateDistance(point);
            if (distance_check.second)
            {
                const double current_penetration = distance_check.first;
                if (current_penetration < this->resolution_distance_threshold_)
                {
                    return std::abs(this->resolution_distance_threshold_ - current_penetration);
                }
                else
                {
                    return 0.0;
                }
            }
            else
            {
                return std::numeric_limits<double>::infinity();
            }
        }

        inline double ComputeMaxPointPenetration(const Robot& current_robot, const Configuration& previous_config) const
        {
            UNUSED(previous_config);
            const std::vector<std::pair<std::string, std::shared_ptr<EigenHelpers::VectorVector4d>>> robot_links_points = current_robot.GetRawLinksPoints();
            // Find the point on the robot that moves the most
            double max_point_penetration = 0.0;
            for (size_t link_idx = 0; link_idx < robot_links_points.size(); link_idx++)
            {
                // Grab the link name and points
                const std::string& link_name = robot_links_points[link_idx].first;
                const EigenHelpers::VectorVector4d& link_points = *(robot_links_points[link_idx].second);
                // Get the *current* transform of the current link
                const Eigen::Affine3d current_link_transform = current_robot.GetLinkTransform(link_name);
                // Now, go through the points of the link
                for (size_t point_idx = 0; point_idx < link_points.size(); point_idx++)
                {
                    // Transform the link point into the environment frame
                    const Eigen::Vector4d& link_relative_point = link_points[point_idx];
                    // Get the current world position
                    const Eigen::Vector4d current_environment_position = current_link_transform * link_relative_point;
                    const double penetration = EstimatePointPenetration(current_environment_position);
                    if (penetration > max_point_penetration)
                    {
                        max_point_penetration = penetration;
                    }
                }
            }
            return max_point_penetration;
        }

        inline double EstimateMaxControlInputWorkspaceMotion(const Robot& start_robot, const Robot& end_robot) const
        {
            const std::vector<std::pair<std::string, std::shared_ptr<EigenHelpers::VectorVector4d>>> robot_links_points = start_robot.GetRawLinksPoints();
            // Find the point on the robot that moves the most
            double max_point_motion_squared = 0.0;
            for (size_t link_idx = 0; link_idx < robot_links_points.size(); link_idx++)
            {
                // Grab the link name and points
                const std::string& link_name = robot_links_points[link_idx].first;
                const EigenHelpers::VectorVector4d& link_points = *(robot_links_points[link_idx].second);
                // Get the *current* transform of the current link
                const Eigen::Affine3d current_link_transform = start_robot.GetLinkTransform(link_name);
                // Get the *next* transform of the current link
                const Eigen::Affine3d next_link_transform = end_robot.GetLinkTransform(link_name);
                // Now, go through the points of the link
                for (size_t point_idx = 0; point_idx < link_points.size(); point_idx++)
                {
                    // Transform the link point into the environment frame
                    const Eigen::Vector4d& link_relative_point = link_points[point_idx];
                    // Get the current world position
                    const Eigen::Vector4d current_environment_position = current_link_transform * link_relative_point;
                    // Get the next world position
                    const Eigen::Vector4d next_environment_position = next_link_transform * link_relative_point;
                    // Compute the movement of the point
                    const double point_motion_squared = (next_environment_position - current_environment_position).squaredNorm();
                    if (point_motion_squared > max_point_motion_squared)
                    {
                        max_point_motion_squared = point_motion_squared;
                    }
                }
            }
            return sqrt(max_point_motion_squared);
        }

        inline double EstimateMaxControlInputWorkspaceMotion(const Robot& robot, const Configuration& start, const Configuration& end) const
        {
            Robot start_robot = robot;
            Robot end_robot = robot;
            start_robot.UpdatePosition(start);
            end_robot.UpdatePosition(end);
            return EstimateMaxControlInputWorkspaceMotion(start_robot, end_robot);
        }

        inline double EstimateMaxControlInputWorkspaceMotion(const Robot& current_robot, const Eigen::VectorXd& control_input) const
        {
            // Apply the control input
            Robot next_robot = current_robot;
            next_robot.ApplyControlInput(control_input);
            return EstimateMaxControlInputWorkspaceMotion(current_robot, next_robot);
        }

        inline std::pair<Configuration, std::pair<bool, bool>> ResolveForwardSimulation(const Robot& immutable_robot, const Eigen::VectorXd& control_input, const double controller_interval, RNG& rng, const bool use_individual_jacobians, const bool allow_contacts, simple_simulator_interface::ForwardSimulationStepTrace<Configuration, ConfigAlloc>& trace, const bool enable_tracing, const uint64_t call_number, ros::Publisher& display_pub) const
        {
            Robot robot = immutable_robot;
            const Eigen::VectorXd real_control_input = control_input * controller_interval;
            if (this->debug_level_ >= 25)
            {
                const std::string msg1 = "[" + std::to_string(call_number) + "] Resolving control input: " + PrettyPrint::PrettyPrint(control_input) + "\nReal control input: " + PrettyPrint::PrettyPrint(real_control_input);
                std::cout << msg1 << std::endl;
            }
            // Get the list of link name + link points for all the links of the robot
            const std::vector<std::pair<std::string, std::shared_ptr<EigenHelpers::VectorVector4d>>> robot_links_points = robot.GetRawLinksPoints();
            // Step along the control input
            // First, figure out how much workspace motion is actually going to result from the control input
            const double computed_step_motion = EstimateMaxControlInputWorkspaceMotion(robot, real_control_input);
            const double target_microstep_distance = this->GetResolution() * 0.125;
            const double allowed_microstep_distance = this->GetResolution() * 1.0;
            const uint32_t number_microsteps = std::max(1u, ((uint32_t)ceil(computed_step_motion / target_microstep_distance)));
            if (this->debug_level_ >= 2)
            {
                const std::string msg3 = "[" + std::to_string(call_number) + "] Resolving simulation step with computed motion: " + std::to_string(computed_step_motion) + " in " + std::to_string(number_microsteps) + " microsteps";
                std::cout << msg3 << std::endl;
            }
            const Eigen::VectorXd control_input_step = real_control_input / (double)number_microsteps;
            const double computed_microstep_motion = EstimateMaxControlInputWorkspaceMotion(robot, control_input_step);
            if (computed_microstep_motion > allowed_microstep_distance)
            {
                const std::string msg = "[" + std::to_string(call_number) + "] Computed microstep motion: " + std::to_string(computed_microstep_motion) + " > allowed " + std::to_string(allowed_microstep_distance);
                std::cerr << msg << std::endl;
                assert(false);
            }
            if (this->debug_level_ >= 25)
            {
                const std::string msg4 = "[" + std::to_string(call_number) + "] Control input step: " + PrettyPrint::PrettyPrint(control_input_step);
                std::cout << msg4 << std::endl;
            }
            bool collided = false;
            bool fallback_required = false;
            if (enable_tracing)
            {
                trace.resolver_steps.emplace_back();
                trace.resolver_steps.back().control_input = real_control_input;
                trace.resolver_steps.back().control_input_step = control_input_step;
            }
            // Iterate
            for (uint32_t micro_step = 0; micro_step < number_microsteps; micro_step++)
            {
                if (enable_tracing)
                {
                    trace.resolver_steps.back().contact_resolver_steps.emplace_back();
                }
                // Store the previous configuration of the robot
                const Configuration previous_configuration = robot.GetPosition();
                // Update the position of the robot
                robot.ApplyControlInput(control_input_step, rng);
                const Configuration post_action_configuration = robot.GetPosition(rng);
                robot.UpdatePosition(post_action_configuration);
                const double apply_step_max_motion = EstimateMaxControlInputWorkspaceMotion(robot, previous_configuration, post_action_configuration);
                if (this->debug_level_ >= 25)
                {
                    const std::string msg5 = "\x1b[35;1m [" + std::to_string(call_number) + "] Pre-action configuration: " + PrettyPrint::PrettyPrint(previous_configuration) + " \x1b[0m\n\x1b[33;1m [" + std::to_string(call_number) + "] Control input step: " + PrettyPrint::PrettyPrint(control_input_step) + "\n\x1b[33;1m [" + std::to_string(call_number) + "] Post-action configuration: " + PrettyPrint::PrettyPrint(post_action_configuration) + " \x1b[0m\n[" + std::to_string(call_number) + "] Max point motion (apply step) was: " + std::to_string(apply_step_max_motion);
                    std::cout << msg5 << std::endl;
                }
                std::pair<bool, std::unordered_map<std::pair<size_t, size_t>, Eigen::Vector3d>> collision_check = CheckCollision(robot, previous_configuration, post_action_configuration, robot_links_points, controller_interval);
                std::unordered_map<std::pair<size_t, size_t>, Eigen::Vector3d>& self_collision_map = collision_check.second;
                bool in_collision = collision_check.first;
                if (in_collision)
                {
                    collided = true;
                }
                if (enable_tracing)
                {
                    trace.resolver_steps.back().contact_resolver_steps.back().contact_resolution_steps.push_back(post_action_configuration);
                }
                // Now, we know if a collision has happened
                if (in_collision && allow_contacts)
                {
                    Configuration active_configuration = post_action_configuration;
                    uint32_t resolver_iterations = 0;
                    double correction_step_scaling = solver_config_.resolve_correction_initial_step_size;
                    while (in_collision)
                    {
                        const auto point_jacobians_and_corrections = CollectPointCorrectionsAndJacobians(robot, previous_configuration, active_configuration, robot_links_points, self_collision_map, call_number, rng);

                        const Eigen::VectorXd raw_correction_step = (use_individual_jacobians) ? ComputeResolverCorrectionStepIndividualJacobians(point_jacobians_and_corrections.first, point_jacobians_and_corrections.second) : ComputeResolverCorrectionStepStackedJacobian(point_jacobians_and_corrections.first, point_jacobians_and_corrections.second);
                        const double correction_step_motion_estimate = EstimateMaxControlInputWorkspaceMotion(robot, raw_correction_step);
                        double allowed_resolve_distance = allowed_microstep_distance; //std::min(apply_step_max_motion, allowed_microstep_distance * 1.0); //was 0.25
                        if (this->debug_level_ >= 25)
                        {
                            const std::string msg7 = "[" + std::to_string(call_number) + "] Raw Cstep motion estimate: " + std::to_string(correction_step_motion_estimate) + " Allowed correction step motion: " + std::to_string(allowed_resolve_distance) + " Raw Cstep: " + PrettyPrint::PrettyPrint(raw_correction_step);
                            std::cout << msg7 << std::endl;
                        }
                        // Scale down the size of the correction step
                        // This provides additional help debugging, but performance is stable enough without it, and it reduces the number of collision checks significantly
#ifdef USE_CHECKED_RESOLVE_POINT_PENETRATION
                        const double pre_resolve_max_penetration = ComputeMaxPointPenetration(robot, previous_configuration);
                        uint32_t num_micro_ops = 0u;
                        while (num_micro_ops <= 2)
                        {
                            const double step_fraction = std::max((correction_step_motion_estimate / allowed_resolve_distance), 1.0);
                            const Eigen::VectorXd real_correction_step = (raw_correction_step / step_fraction) * std::abs(correction_step_scaling);
                            if (this->debug_level_ >= 25)
                            {
                                const std::string msg8 = "[" + std::to_string(call_number) + "] Real scaled Cstep: " + PrettyPrint::PrettyPrint(real_correction_step);
                                std::cout << msg8 << std::endl;
                            }
                            // Apply correction step
                            robot.ApplyControlInput(real_correction_step);
                            const double post_resolve_max_penetration = ComputeMaxPointPenetration(robot, previous_configuration);
                            num_micro_ops++;
                            if (post_resolve_max_penetration <= pre_resolve_max_penetration)
                            {
                                break;
                            }
                            else
                            {
                                allowed_resolve_distance *= 0.75;
                                robot.UpdatePosition(active_configuration);
                            }
                        }
                        const Configuration post_resolve_configuration = robot.GetPosition();
                        const double post_resolve_max_penetration = ComputeMaxPointPenetration(robot, previous_configuration);
                        assert(post_resolve_max_penetration <= pre_resolve_max_penetration);
                        if (this->debug_level_ >= 25)
                        {
                            const double resolve_step_max_motion = EstimateMaxControlInputWorkspaceMotion(robot, post_action_configuration, post_resolve_configuration);
                            const double iteration_max_motion = EstimateMaxControlInputWorkspaceMotion(robot, previous_configuration, post_resolve_configuration);
                            const std::string msg9 = "\x1b[36;1m [" + std::to_string(call_number) + "] Post-resolve step configuration after " + std::to_string(num_micro_ops) + " micro-ops: " + PrettyPrint::PrettyPrint(post_resolve_configuration) + " \x1b[0m\n[" + std::to_string(call_number) + "] Pre-resolve max penetration was: " + std::to_string(pre_resolve_max_penetration) + " Post-resolve max penetration was: " + std::to_string(post_resolve_max_penetration) + " Max point motion (resolve step) was: " + std::to_string(resolve_step_max_motion) + "\n[" + std::to_string(call_number) + "] Max point motion (iteration) was: " + std::to_string(iteration_max_motion);
                            std::cout << msg9 << std::endl;
                        }
                        if (this->debug_level_ >= 26)
                        {
                            std::cout << "Press ENTER to continue..." << std::endl;
                            std::cin.get();
                        }
#else
                        const double step_fraction = std::max((correction_step_motion_estimate / allowed_resolve_distance), 1.0);
                        const Eigen::VectorXd real_correction_step = (raw_correction_step / step_fraction) * std::abs(correction_step_scaling);
                        if (this->debug_level_ >= 25)
                        {
                            const std::string msg8 = "[" + std::to_string(call_number) + "] Real scaled Cstep: " + PrettyPrint::PrettyPrint(real_correction_step);
                            std::cout << msg8 << std::endl;
                        }
                        // Apply correction step
                        robot.ApplyControlInput(real_correction_step);
                        const Configuration post_resolve_configuration = robot.GetPosition();
#endif
                        active_configuration = post_resolve_configuration;
                        // Check to see if we're still in collision
                        const std::pair<bool, std::unordered_map<std::pair<size_t, size_t>, Eigen::Vector3d>> new_collision_check = CheckCollision(robot, previous_configuration, active_configuration, robot_links_points, controller_interval);
                        // Update the self-collision map
                        self_collision_map = new_collision_check.second;
                        // Update the collision check variable
                        in_collision = new_collision_check.first;
                        resolver_iterations++;
                        // Update tracing
                        if (enable_tracing)
                        {
                            trace.resolver_steps.back().contact_resolver_steps.back().contact_resolution_steps.push_back(active_configuration);
                        }
                        if (resolver_iterations > solver_config_.max_resolver_iterations)
                        {
                            if (this->debug_level_ >= 2)
                            {
                                const std::string msg10 = "\x1b[31;1m [" + std::to_string(call_number) + "] Resolver iterations > " + std::to_string(solver_config_.max_resolver_iterations) + ", terminating microstep+resolver at configuration " + PrettyPrint::PrettyPrint(active_configuration) + " and returning previous configuration " + PrettyPrint::PrettyPrint(previous_configuration) + "\nCollision check results:\n" + PrettyPrint::PrettyPrint(new_collision_check, false, "\n") + " \x1b[0m";
                                std::cout << msg10 << std::endl;
                            }
                            if (enable_tracing)
                            {
                                trace.resolver_steps.back().contact_resolver_steps.back().contact_resolution_steps.push_back(previous_configuration);
                            }
                            unsuccessful_resolves_.fetch_add(1);
                            if (self_collision_map.size() > 0)
                            {
                                unsuccessful_self_collision_resolves_.fetch_add(1);
                            }
                            else
                            {
                                unsuccessful_env_collision_resolves_.fetch_add(1);
                            }
                            if (this->debug_level_ >= 2)
                            {
                                const std_msgs::ColorRGBA config_color = (self_collision_map.size() > 0) ? this->MakeColor(0.5, 0.0, 0.0, 1.0) : this->MakeColor(0.0, 0.0, 0.5, 1.0);
                                visualization_msgs::Marker active_config_marker = this->DrawRobotConfiguration(robot, active_configuration, config_color);
                                active_config_marker.ns = (self_collision_map.size() > 0) ? "failed_self_collision_resolves" : "failed_env_collision_resolves";
                                active_config_marker.id = (self_collision_map.size() > 0) ? (int)unsuccessful_self_collision_resolves_.load() : (int)unsuccessful_env_collision_resolves_.load();
                                visualization_msgs::MarkerArray markers;
                                markers.markers = {active_config_marker};
                                display_pub.publish(markers);
                            }
                            if (this->debug_level_ >= 30)
                            {
                                assert(false);
                            }
                            if (fallback_required)
                            {
                                fallback_resolves_.fetch_add(1u);
                            }
                            return std::make_pair(previous_configuration, std::make_pair(true, true));
                        }
                        if ((resolver_iterations % solver_config_.resolve_correction_step_scaling_decay_iterations) == 0)
                        {
                            if (correction_step_scaling >= 0.0)
                            {
                                correction_step_scaling = correction_step_scaling * solver_config_.resolve_correction_step_scaling_decay_rate;
                                if (correction_step_scaling < solver_config_.resolve_correction_min_step_scaling)
                                {
                                    correction_step_scaling = -solver_config_.resolve_correction_min_step_scaling;
                                }
                            }
                            else
                            {
                                correction_step_scaling = -solver_config_.resolve_correction_min_step_scaling;
                            }
                        }
                    }
                    if (this->debug_level_ >= 25)
                    {
                        const std::string msg11 = "\x1b[31;1m [" + std::to_string(call_number) + "] Colliding microstep " + std::to_string(micro_step + 1u) + " resolved in " + std::to_string(resolver_iterations) + " iterations \x1b[0m";
                        std::cout << msg11 << std::endl;
                    }
                }
                else if (in_collision && (allow_contacts == false))
                {
                    if (this->debug_level_ >= 25)
                    {
                        const std::string msg12 = "\x1b[31;1m [" + std::to_string(call_number) + "] Colliding microstep " + std::to_string(micro_step + 1u) + " trivially resolved in 1 iteration (allow_contacts==false) \x1b[0m";
                        std::cout << msg12 << std::endl;
                    }
                    if (enable_tracing)
                    {
                        trace.resolver_steps.back().contact_resolver_steps.back().contact_resolution_steps.push_back(previous_configuration);
                    }
                    successful_resolves_.fetch_add(1);
                    if (fallback_required)
                    {
                        fallback_resolves_.fetch_add(1u);
                    }
                    return std::make_pair(previous_configuration, std::make_pair(true, false));
                }
                else
                {
                    if (this->debug_level_ >= 25)
                    {
                        const std::string msg13= "\x1b[31;1m [" + std::to_string(call_number) + "] Uncolliding microstep " + std::to_string(micro_step + 1u) + " trivially resolved in 1 iteration \x1b[0m";
                        std::cout << msg13 << std::endl;
                    }
                    continue;
                }
            }
            if (this->debug_level_ >= 2)
            {
                const std::string msg14 = "\x1b[32;1m [" + std::to_string(call_number) + "] Resolved action, post-action resolution configuration: " + PrettyPrint::PrettyPrint(robot.GetPosition()) + " \x1b[0m";
                std::cout << msg14 << std::endl;
            }
            successful_resolves_.fetch_add(1);
            if (fallback_required)
            {
                fallback_resolves_.fetch_add(1u);
            }
            if (collided)
            {
                collision_resolves_.fetch_add(1);
            }
            else
            {
                free_resolves_.fetch_add(1);
            }
            return std::make_pair(robot.GetPosition(), std::make_pair(collided, false));
        }

        inline std::pair<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Matrix<double, Eigen::Dynamic, 1>> CollectPointCorrectionsAndJacobians(const Robot& robot, const Configuration& previous_config, const Configuration& current_config, const std::vector<std::pair<std::string, std::shared_ptr<EigenHelpers::VectorVector4d>>>& robot_links_points, const std::unordered_map<std::pair<size_t, size_t>, Eigen::Vector3d>& self_collision_map, const uint64_t call_number, RNG& rng) const
        {
            UNUSED(rng);
            Robot current_robot = robot;
            Robot previous_robot = robot;
            // We need our own copy with a set config to use for kinematics!
            current_robot.UpdatePosition(current_config);
            previous_robot.UpdatePosition(previous_config);
            // In case a collision has occured, we need to compute a "collision gradient" that will push the robot out of collision
            // The "collision gradient" is of the form qgradient = J(q)+ * xgradient
            // Make space for the xgradient and Jacobian
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> robot_jacobians;
            Eigen::Matrix<double, Eigen::Dynamic, 1> point_corrections;
            // Now, go through the links and points of the robot to build up the xgradient and Jacobian
            for (size_t link_idx = 0; link_idx < robot_links_points.size(); link_idx++)
            {
                // Grab the link name and points
                const std::string& link_name = robot_links_points[link_idx].first;
                const EigenHelpers::VectorVector4d& link_points = (*robot_links_points[link_idx].second);
                // Get the transform of the current link
                const Eigen::Affine3d previous_link_transform = previous_robot.GetLinkTransform(link_name);
                const Eigen::Affine3d current_link_transform = current_robot.GetLinkTransform(link_name);
                // Now, go through the points of the link
                for (size_t point_idx = 0; point_idx < link_points.size(); point_idx++)
                {
                    // Check if we have a self-collision correction
                    std::pair<bool, Eigen::Vector3d> self_collision_correction(false, Eigen::Vector3d(0.0, 0.0, 0.0));
                    auto self_collision_check = self_collision_map.find(std::pair<size_t, size_t>(link_idx, point_idx));
                    if (self_collision_check != self_collision_map.end())
                    {
                        self_collision_correction.first = true;
                        self_collision_correction.second = self_collision_check->second;
                    }
                    // Check against the environment
                    std::pair<bool, Eigen::Vector3d> env_collision_correction(false, Eigen::Vector3d(0.0, 0.0, 0.0));
                    const Eigen::Vector4d& link_relative_point = link_points[point_idx];
                    // Get the Jacobian for the current point
                    const Eigen::Matrix<double, 3, Eigen::Dynamic> point_jacobian = current_robot.ComputeLinkPointJacobian(link_name, link_relative_point);
                    // Transform the link point into the environment frame
                    const Eigen::Vector4d previous_point_location = previous_link_transform * link_relative_point;
                    const Eigen::Vector4d current_point_location = current_link_transform * link_relative_point;
                    // We only work with points in the SDF
                    const std::pair<double, bool> current_sdf_check = this->environment_sdf_.EstimateDistance4d(current_point_location);
                    if (current_sdf_check.second == false)
                    {
                        const std::string msg = "[" + std::to_string(call_number) + "] (Current) Point out of bounds: " + PrettyPrint::PrettyPrint(current_point_location);
                        arc_helpers::ConditionalPrint(msg, 0, this->debug_level_);
                        if (this->debug_level_ >= 5)
                        {
                            assert(false);
                        }
                    }
                    // We only work with points in collision
                    if (current_sdf_check.first < this->resolution_distance_threshold_ && current_sdf_check.second)
                    {
                        // In the future, we might want to consider reversing the point motion if the robot started in contact and we're trying to leave collision
                        // We query the surface normal map for the gradient to move out of contact using the particle motion
                        const Eigen::Vector4d point_motion = current_point_location - previous_point_location;
                        const Eigen::Vector4d normed_point_motion = EigenHelpers::SafeNormal(point_motion);
                        // Query the surface normal map
                        const std::pair<Eigen::Vector3d, bool> surface_normal_query = this->surface_normals_grid_.LookupSurfaceNormal(current_point_location, normed_point_motion);
                        assert(surface_normal_query.second);
                        const Eigen::Vector3d& raw_gradient = surface_normal_query.first;
                        const Eigen::Vector3d normed_point_gradient = EigenHelpers::SafeNormal(raw_gradient);
                        // Compute the penetration weight
                        const double sdf_penetration_distance = std::abs(this->resolution_distance_threshold_ - current_sdf_check.first);
                        // Compute the correction vector
                        const Eigen::Vector3d correction_vector = normed_point_gradient * sdf_penetration_distance;
                        env_collision_correction.first = true;
                        env_collision_correction.second = correction_vector;
                    }
                    // We only add a correction for the point if necessary
                    if (self_collision_correction.first || env_collision_correction.first)
                    {
                        // Append the new point jacobian to the matrix of jacobians
                        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> extended_robot_jacobians;
                        extended_robot_jacobians.resize(robot_jacobians.rows() + 3, point_jacobian.cols());
                        if (robot_jacobians.cols() > 0)
                        {
                            extended_robot_jacobians << robot_jacobians,point_jacobian;
                        }
                        else
                        {
                            extended_robot_jacobians << point_jacobian;
                        }
                        robot_jacobians = extended_robot_jacobians;
                        // Assemble the workspace correction vector
                        Eigen::Vector3d point_correction(0.0, 0.0, 0.0);
                        if (self_collision_correction.first)
                        {
                            if (this->debug_level_ >= 25)
                            {
                                std::cout << "Self-collision correction: " << PrettyPrint::PrettyPrint(self_collision_correction.second) << std::endl;
                            }
                            point_correction = point_correction + self_collision_correction.second;
                        }
                        if (env_collision_correction.first)
                        {
                            if (this->debug_level_ >= 25)
                            {
                                std::cout << "Env-collision correction: " << PrettyPrint::PrettyPrint(env_collision_correction.second) << std::endl;
                            }
                            point_correction = point_correction + env_collision_correction.second;
                        }
                        // Append the new workspace correction vector to the matrix of correction vectors
                        Eigen::Matrix<double, Eigen::Dynamic, 1> extended_point_corrections;
                        extended_point_corrections.resize(point_corrections.rows() + 3, Eigen::NoChange);
                        extended_point_corrections << point_corrections,point_correction;
                        point_corrections = extended_point_corrections;
                        if (this->debug_level_ >= 35)
                        {
                            std::cout << "Point jacobian:\n" << point_jacobian << std::endl;
                            std::cout << "Point correction: " << PrettyPrint::PrettyPrint(point_correction) << std::endl;
                        }
                    }
                }
            }
            return std::make_pair(robot_jacobians, point_corrections);
        }

        inline Eigen::VectorXd ComputeResolverCorrectionStepJacobiansTranspose(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& robot_jacobians, const Eigen::Matrix<double, Eigen::Dynamic, 1>& point_corrections) const
        {
            // In case a collision has occured, we need to compute a "collision gradient" that will push the robot out of collision
            Eigen::VectorXd raw_correction_step;
            // Go through the collected jacobians and corrections
            for (ssize_t idx = 0; idx < robot_jacobians.rows(); idx += 3)
            {
                const Eigen::Matrix<double, 3, Eigen::Dynamic>& point_jacobian = robot_jacobians.block(idx, 0, 3, robot_jacobians.cols());
                const Eigen::Vector3d& point_correction = point_corrections.block<3, 1>(idx, 0);
                // Compute the correction step T = J(q,p).t * F
                const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> raw_correction = point_jacobian.transpose() * point_correction;
                // Extract the c-space correction
                const Eigen::VectorXd point_correction_step = raw_correction.col(0);
                if (raw_correction_step.size() == 0)
                {
                    raw_correction_step = point_correction_step;
                }
                else
                {
                    raw_correction_step = raw_correction_step + point_correction_step;
                }
            }
            return raw_correction_step;
        }

        inline Eigen::VectorXd ComputeResolverCorrectionStepIndividualJacobians(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& robot_jacobians, const Eigen::Matrix<double, Eigen::Dynamic, 1>& point_corrections) const
        {
            // In case a collision has occured, we need to compute a "collision gradient" that will push the robot out of collision
            Eigen::VectorXd raw_correction_step;
            // Go through the collected jacobians and corrections
            for (ssize_t idx = 0; idx < robot_jacobians.rows(); idx += 3)
            {
                const Eigen::Matrix<double, 3, Eigen::Dynamic>& point_jacobian = robot_jacobians.block(idx, 0, 3, robot_jacobians.cols());
                const Eigen::Vector3d& point_correction = point_corrections.block<3, 1>(idx, 0);
                const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> raw_correction = point_jacobian.colPivHouseholderQr().solve(point_correction);
                // Extract the c-space correction
                const Eigen::VectorXd point_correction_step = raw_correction.col(0);
                if (raw_correction_step.size() == 0)
                {
                    raw_correction_step = point_correction_step;
                }
                else
                {
                    raw_correction_step = raw_correction_step + point_correction_step;
                }
            }
            return raw_correction_step;
        }

        inline Eigen::VectorXd ComputeResolverCorrectionStepStackedJacobian(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& robot_jacobians, const Eigen::Matrix<double, Eigen::Dynamic, 1>& point_corrections) const
        {
            // Compute the correction step
            // We could use the naive Pinv(J) * pdot, but instead we solve the Ax = b (Jqdot = pdot) problem directly using one of the solvers in Eigen
            const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> raw_correction = robot_jacobians.colPivHouseholderQr().solve(point_corrections);
            // Extract the c-space correction
            const Eigen::VectorXd raw_correction_step = raw_correction.col(0);
            return raw_correction_step;
        }
    };
}

#endif // SIMPLE_PARTICLE_CONTACT_SIMULATOR_HPP
