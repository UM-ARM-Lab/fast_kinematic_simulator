#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <functional>
#include <chrono>
#include <random>
#include <mutex>
#include <thread>
#include <atomic>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/zlib_helpers.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/simple_hierarchical_clustering.hpp>
#include <arc_utilities/simple_hausdorff_distance.hpp>
#include <arc_utilities/simple_rrt_planner.hpp>
#include <sdf_tools/tagged_object_collision_map.hpp>
#include <sdf_tools/sdf.hpp>
#include <uncertainty_planning_core/simple_pid_controller.hpp>
#include <uncertainty_planning_core/simple_uncertainty_models.hpp>
#include <uncertainty_planning_core/uncertainty_planner_state.hpp>
#include <uncertainty_planning_core/simple_particle_contact_simulator.hpp>
#include <uncertainty_planning_core/execution_policy.hpp>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <uncertainty_planning_core/uncertainty_contact_planning.hpp>
#include <uncertainty_planning_core/simplese2_robot_helpers.hpp>
#include <uncertainty_planning_core/simplese3_robot_helpers.hpp>
#include <uncertainty_planning_core/simplelinked_robot_helpers.hpp>
#include <uncertainty_planning_core/uncertainty_planning_core.hpp>

using namespace uncertainty_planning_core;

bool uncertainty_planning_core::SaveSE2Policy(const SE2Policy& policy, const std::string& filename)
{
    return uncertainty_contact_planning::UncertaintyPlanningSpace<simplese2_robot_helpers::SimpleSE2Robot, simplese2_robot_helpers::SimpleSE2BaseSampler, Eigen::Matrix<double, 3, 1>, simplese2_robot_helpers::EigenMatrixD31Serializer, simplese2_robot_helpers::SimpleSE2Averager, simplese2_robot_helpers::SimpleSE2Distancer, simplese2_robot_helpers::SimpleSE2DimDistancer, simplese2_robot_helpers::SimpleSE2Interpolator, std::allocator<Eigen::Matrix<double, 3, 1>>, std::mt19937_64>::SavePolicy(policy, filename);
}

SE2Policy uncertainty_planning_core::LoadSE2Policy(const std::string& filename)
{
    return uncertainty_contact_planning::UncertaintyPlanningSpace<simplese2_robot_helpers::SimpleSE2Robot, simplese2_robot_helpers::SimpleSE2BaseSampler, Eigen::Matrix<double, 3, 1>, simplese2_robot_helpers::EigenMatrixD31Serializer, simplese2_robot_helpers::SimpleSE2Averager, simplese2_robot_helpers::SimpleSE2Distancer, simplese2_robot_helpers::SimpleSE2DimDistancer, simplese2_robot_helpers::SimpleSE2Interpolator, std::allocator<Eigen::Matrix<double, 3, 1>>, std::mt19937_64>::LoadPolicy(filename);
}

bool uncertainty_planning_core::SaveSE3Policy(const SE3Policy& policy, const std::string& filename)
{
    return uncertainty_contact_planning::UncertaintyPlanningSpace<simplese3_robot_helpers::SimpleSE3Robot, simplese3_robot_helpers::SimpleSE3BaseSampler, Eigen::Affine3d, simplese3_robot_helpers::EigenAffine3dSerializer, simplese3_robot_helpers::SimpleSE3Averager, simplese3_robot_helpers::SimpleSE3Distancer, simplese3_robot_helpers::SimpleSE3DimDistancer, simplese3_robot_helpers::SimpleSE3Interpolator, Eigen::aligned_allocator<Eigen::Affine3d>, std::mt19937_64>::SavePolicy(policy, filename);
}

SE3Policy uncertainty_planning_core::LoadSE3Policy(const std::string& filename)
{
    return uncertainty_contact_planning::UncertaintyPlanningSpace<simplese3_robot_helpers::SimpleSE3Robot, simplese3_robot_helpers::SimpleSE3BaseSampler, Eigen::Affine3d, simplese3_robot_helpers::EigenAffine3dSerializer, simplese3_robot_helpers::SimpleSE3Averager, simplese3_robot_helpers::SimpleSE3Distancer, simplese3_robot_helpers::SimpleSE3DimDistancer, simplese3_robot_helpers::SimpleSE3Interpolator, Eigen::aligned_allocator<Eigen::Affine3d>, std::mt19937_64>::LoadPolicy(filename);
}

bool uncertainty_planning_core::SaveBaxterPolicy(const BaxterPolicy& policy, const std::string& filename)
{
    return uncertainty_contact_planning::UncertaintyPlanningSpace<simplelinked_robot_helpers::SimpleLinkedRobot<baxter_actuator_helpers::BaxterJointActuatorModel>, simplelinked_robot_helpers::SimpleLinkedBaseSampler, simplelinked_robot_helpers::SimpleLinkedConfiguration, simplelinked_robot_helpers::SimpleLinkedConfigurationSerializer, simplelinked_robot_helpers::SimpleLinkedAverager, baxter_actuator_helpers::SimpleLinkedDistancer, baxter_actuator_helpers::SimpleLinkedDimDistancer, simplelinked_robot_helpers::SimpleLinkedInterpolator, std::allocator<simplelinked_robot_helpers::SimpleLinkedConfiguration>, std::mt19937_64>::SavePolicy(policy, filename);
}

BaxterPolicy uncertainty_planning_core::LoadBaxterPolicy(const std::string& filename)
{
    return uncertainty_contact_planning::UncertaintyPlanningSpace<simplelinked_robot_helpers::SimpleLinkedRobot<baxter_actuator_helpers::BaxterJointActuatorModel>, simplelinked_robot_helpers::SimpleLinkedBaseSampler, simplelinked_robot_helpers::SimpleLinkedConfiguration, simplelinked_robot_helpers::SimpleLinkedConfigurationSerializer, simplelinked_robot_helpers::SimpleLinkedAverager, baxter_actuator_helpers::SimpleLinkedDistancer, baxter_actuator_helpers::SimpleLinkedDimDistancer, simplelinked_robot_helpers::SimpleLinkedInterpolator, std::allocator<simplelinked_robot_helpers::SimpleLinkedConfiguration>, std::mt19937_64>::LoadPolicy(filename);
}

bool uncertainty_planning_core::SaveUR5Policy(const UR5Policy& policy, const std::string& filename)
{
    return uncertainty_contact_planning::UncertaintyPlanningSpace<simplelinked_robot_helpers::SimpleLinkedRobot<ur5_actuator_helpers::UR5JointActuatorModel>, simplelinked_robot_helpers::SimpleLinkedBaseSampler, simplelinked_robot_helpers::SimpleLinkedConfiguration, simplelinked_robot_helpers::SimpleLinkedConfigurationSerializer, simplelinked_robot_helpers::SimpleLinkedAverager, ur5_actuator_helpers::SimpleLinkedDistancer, ur5_actuator_helpers::SimpleLinkedDimDistancer, simplelinked_robot_helpers::SimpleLinkedInterpolator, std::allocator<simplelinked_robot_helpers::SimpleLinkedConfiguration>, std::mt19937_64>::SavePolicy(policy, filename);
}

UR5Policy uncertainty_planning_core::LoadUR5Policy(const std::string& filename)
{
    return uncertainty_contact_planning::UncertaintyPlanningSpace<simplelinked_robot_helpers::SimpleLinkedRobot<ur5_actuator_helpers::UR5JointActuatorModel>, simplelinked_robot_helpers::SimpleLinkedBaseSampler, simplelinked_robot_helpers::SimpleLinkedConfiguration, simplelinked_robot_helpers::SimpleLinkedConfigurationSerializer, simplelinked_robot_helpers::SimpleLinkedAverager, ur5_actuator_helpers::SimpleLinkedDistancer, ur5_actuator_helpers::SimpleLinkedDimDistancer, simplelinked_robot_helpers::SimpleLinkedInterpolator, std::allocator<simplelinked_robot_helpers::SimpleLinkedConfiguration>, std::mt19937_64>::LoadPolicy(filename);
}

std::vector<Eigen::Matrix<double, 3, 1>, std::allocator<Eigen::Matrix<double, 3, 1>>> uncertainty_planning_core::DemonstrateSE2Simulator(const OPTIONS& options, const simplese2_robot_helpers::SimpleSE2Robot& robot, const simplese2_robot_helpers::SimpleSE2BaseSampler& sampler, const Eigen::Matrix<double, 3, 1>& start, const Eigen::Matrix<double, 3, 1>& goal, ros::Publisher& display_debug_publisher)
{
    typedef simplese2_robot_helpers::SimpleSE2Robot Robot;
    typedef Eigen::Matrix<double, 3, 1> Configuration;
    typedef std::allocator<Eigen::Matrix<double, 3, 1>> ConfigAlloc;
    typedef std::mt19937_64 PRNG;
    std::shared_ptr<uncertainty_planning_tools::SimulatorInterface<Robot, Configuration, PRNG, ConfigAlloc>> simulator_ptr(new uncertainty_planning_tools::SimpleParticleContactSimulator<Robot, Configuration, PRNG, ConfigAlloc>(options.environment_name, options.environment_resolution, options.simulation_controller_frequency, options.debug_level));
    uncertainty_contact_planning::UncertaintyPlanningSpace<simplese2_robot_helpers::SimpleSE2Robot, simplese2_robot_helpers::SimpleSE2BaseSampler, Eigen::Matrix<double, 3, 1>, simplese2_robot_helpers::EigenMatrixD31Serializer, simplese2_robot_helpers::SimpleSE2Averager, simplese2_robot_helpers::SimpleSE2Distancer, simplese2_robot_helpers::SimpleSE2DimDistancer, simplese2_robot_helpers::SimpleSE2Interpolator, std::allocator<Eigen::Matrix<double, 3, 1>>, PRNG> planning_space(options.clustering_type, false, options.debug_level, options.num_particles, options.step_size, options.step_duration, options.goal_distance_threshold, options.goal_probability_threshold, options.signature_matching_threshold, options.distance_clustering_threshold, options.feasibility_alpha, options.variance_alpha, options.connect_after_first_solution, robot, sampler, simulator_ptr, options.prng_seed);
    const uncertainty_planning_tools::ForwardSimulationStepTrace<Configuration, ConfigAlloc> trace = planning_space.DemonstrateSimulator(start, goal, display_debug_publisher);
    return uncertainty_planning_tools::ExtractTrajectoryFromTrace(trace);
}

std::pair<SE2Policy, std::map<std::string, double>> uncertainty_planning_core::PlanSE2Uncertainty(const OPTIONS& options, const simplese2_robot_helpers::SimpleSE2Robot& robot, const simplese2_robot_helpers::SimpleSE2BaseSampler& sampler, const Eigen::Matrix<double, 3, 1>& start, const Eigen::Matrix<double, 3, 1>& goal, ros::Publisher& display_debug_publisher)
{
    typedef simplese2_robot_helpers::SimpleSE2Robot Robot;
    typedef Eigen::Matrix<double, 3, 1> Configuration;
    typedef std::allocator<Eigen::Matrix<double, 3, 1>> ConfigAlloc;
    typedef std::mt19937_64 PRNG;
    std::shared_ptr<uncertainty_planning_tools::SimulatorInterface<Robot, Configuration, PRNG, ConfigAlloc>> simulator_ptr(new uncertainty_planning_tools::SimpleParticleContactSimulator<Robot, Configuration, PRNG, ConfigAlloc>(options.environment_name, options.environment_resolution, options.simulation_controller_frequency, options.debug_level));
    uncertainty_contact_planning::UncertaintyPlanningSpace<simplese2_robot_helpers::SimpleSE2Robot, simplese2_robot_helpers::SimpleSE2BaseSampler, Eigen::Matrix<double, 3, 1>, simplese2_robot_helpers::EigenMatrixD31Serializer, simplese2_robot_helpers::SimpleSE2Averager, simplese2_robot_helpers::SimpleSE2Distancer, simplese2_robot_helpers::SimpleSE2DimDistancer, simplese2_robot_helpers::SimpleSE2Interpolator, std::allocator<Eigen::Matrix<double, 3, 1>>, PRNG> planning_space(options.clustering_type, false, options.debug_level, options.num_particles, options.step_size, options.step_duration, options.goal_distance_threshold, options.goal_probability_threshold, options.signature_matching_threshold, options.distance_clustering_threshold, options.feasibility_alpha, options.variance_alpha, options.connect_after_first_solution, robot, sampler, simulator_ptr, options.prng_seed);
    const std::chrono::duration<double> planner_time_limit(options.planner_time_limit);
    return planning_space.Plan(start, goal, options.goal_bias, planner_time_limit, options.edge_attempt_count, options.policy_action_attempt_count, options.use_contact, options.use_reverse, options.use_spur_actions, display_debug_publisher);
}

std::pair<SE2Policy, std::pair<std::map<std::string, double>, std::pair<std::vector<int64_t>, std::vector<double>>>> uncertainty_planning_core::SimulateSE2UncertaintyPolicy(const OPTIONS& options, const simplese2_robot_helpers::SimpleSE2Robot& robot, const simplese2_robot_helpers::SimpleSE2BaseSampler& sampler, SE2Policy policy, const Eigen::Matrix<double, 3, 1>& start, const Eigen::Matrix<double, 3, 1>& goal, ros::Publisher& display_debug_publisher)
{
    typedef simplese2_robot_helpers::SimpleSE2Robot Robot;
    typedef Eigen::Matrix<double, 3, 1> Configuration;
    typedef std::allocator<Eigen::Matrix<double, 3, 1>> ConfigAlloc;
    typedef std::mt19937_64 PRNG;
    std::shared_ptr<uncertainty_planning_tools::SimulatorInterface<Robot, Configuration, PRNG, ConfigAlloc>> simulator_ptr(new uncertainty_planning_tools::SimpleParticleContactSimulator<Robot, Configuration, PRNG, ConfigAlloc>(options.environment_name, options.environment_resolution, options.simulation_controller_frequency, options.debug_level));
    uncertainty_contact_planning::UncertaintyPlanningSpace<simplese2_robot_helpers::SimpleSE2Robot, simplese2_robot_helpers::SimpleSE2BaseSampler, Eigen::Matrix<double, 3, 1>, simplese2_robot_helpers::EigenMatrixD31Serializer, simplese2_robot_helpers::SimpleSE2Averager, simplese2_robot_helpers::SimpleSE2Distancer, simplese2_robot_helpers::SimpleSE2DimDistancer, simplese2_robot_helpers::SimpleSE2Interpolator, std::allocator<Eigen::Matrix<double, 3, 1>>, PRNG> planning_space(options.clustering_type, false, options.debug_level, options.num_particles, options.step_size, options.step_duration, options.goal_distance_threshold, options.goal_probability_threshold, options.signature_matching_threshold, options.distance_clustering_threshold, options.feasibility_alpha, options.variance_alpha, options.connect_after_first_solution, robot, sampler, simulator_ptr, options.prng_seed);
    policy.SetPolicyActionAttemptCount(options.policy_action_attempt_count);
    return planning_space.SimulateExectionPolicy(policy, start, goal, options.num_policy_simulations, options.max_exec_actions, display_debug_publisher, true, 0.001);
}

std::pair<SE2Policy, std::pair<std::map<std::string, double>, std::pair<std::vector<int64_t>, std::vector<double>>>> uncertainty_planning_core::ExecuteSE2UncertaintyPolicy(const OPTIONS& options, const simplese2_robot_helpers::SimpleSE2Robot& robot, const simplese2_robot_helpers::SimpleSE2BaseSampler& sampler, SE2Policy policy, const Eigen::Matrix<double, 3, 1>& start, const Eigen::Matrix<double, 3, 1>& goal, const std::function<std::vector<Eigen::Matrix<double, 3, 1>, std::allocator<Eigen::Matrix<double, 3, 1>>>(const Eigen::Matrix<double, 3, 1>&, const Eigen::Matrix<double, 3, 1>&, const double, const double, const bool)>& robot_execution_fn, ros::Publisher& display_debug_publisher)
{
    typedef simplese2_robot_helpers::SimpleSE2Robot Robot;
    typedef Eigen::Matrix<double, 3, 1> Configuration;
    typedef std::allocator<Eigen::Matrix<double, 3, 1>> ConfigAlloc;
    typedef std::mt19937_64 PRNG;
    std::shared_ptr<uncertainty_planning_tools::SimulatorInterface<Robot, Configuration, PRNG, ConfigAlloc>> simulator_ptr(new uncertainty_planning_tools::SimpleParticleContactSimulator<Robot, Configuration, PRNG, ConfigAlloc>(options.environment_name, options.environment_resolution, options.simulation_controller_frequency, options.debug_level));
    uncertainty_contact_planning::UncertaintyPlanningSpace<simplese2_robot_helpers::SimpleSE2Robot, simplese2_robot_helpers::SimpleSE2BaseSampler, Eigen::Matrix<double, 3, 1>, simplese2_robot_helpers::EigenMatrixD31Serializer, simplese2_robot_helpers::SimpleSE2Averager, simplese2_robot_helpers::SimpleSE2Distancer, simplese2_robot_helpers::SimpleSE2DimDistancer, simplese2_robot_helpers::SimpleSE2Interpolator, std::allocator<Eigen::Matrix<double, 3, 1>>, PRNG> planning_space(options.clustering_type, false, options.debug_level, options.num_particles, options.step_size, options.step_duration, options.goal_distance_threshold, options.goal_probability_threshold, options.signature_matching_threshold, options.distance_clustering_threshold, options.feasibility_alpha, options.variance_alpha, options.connect_after_first_solution, robot, sampler, simulator_ptr, options.prng_seed);
    policy.SetPolicyActionAttemptCount(options.policy_action_attempt_count);
    return planning_space.ExecuteExectionPolicy(policy, start, goal, robot_execution_fn, options.num_policy_executions, options.max_policy_exec_time, display_debug_publisher, false, 0.001);
}


EigenHelpers::VectorAffine3d uncertainty_planning_core::DemonstrateSE3Simulator(const OPTIONS& options, const simplese3_robot_helpers::SimpleSE3Robot& robot, const simplese3_robot_helpers::SimpleSE3BaseSampler& sampler, const Eigen::Affine3d& start, const Eigen::Affine3d& goal, ros::Publisher& display_debug_publisher)
{
    typedef simplese3_robot_helpers::SimpleSE3Robot Robot;
    typedef Eigen::Affine3d Configuration;
    typedef Eigen::aligned_allocator<Eigen::Affine3d> ConfigAlloc;
    typedef std::mt19937_64 PRNG;
    std::shared_ptr<uncertainty_planning_tools::SimulatorInterface<Robot, Configuration, PRNG, ConfigAlloc>> simulator_ptr(new uncertainty_planning_tools::SimpleParticleContactSimulator<Robot, Configuration, PRNG, ConfigAlloc>(options.environment_name, options.environment_resolution, options.simulation_controller_frequency, options.debug_level));
    uncertainty_contact_planning::UncertaintyPlanningSpace<simplese3_robot_helpers::SimpleSE3Robot, simplese3_robot_helpers::SimpleSE3BaseSampler, Eigen::Affine3d, simplese3_robot_helpers::EigenAffine3dSerializer, simplese3_robot_helpers::SimpleSE3Averager, simplese3_robot_helpers::SimpleSE3Distancer, simplese3_robot_helpers::SimpleSE3DimDistancer, simplese3_robot_helpers::SimpleSE3Interpolator, Eigen::aligned_allocator<Eigen::Affine3d>, PRNG> planning_space(options.clustering_type, false, options.debug_level, options.num_particles, options.step_size, options.step_duration, options.goal_distance_threshold, options.goal_probability_threshold, options.signature_matching_threshold, options.distance_clustering_threshold, options.feasibility_alpha, options.variance_alpha, options.connect_after_first_solution, robot, sampler, simulator_ptr, options.prng_seed);
    const uncertainty_planning_tools::ForwardSimulationStepTrace<Configuration, ConfigAlloc> trace = planning_space.DemonstrateSimulator(start, goal, display_debug_publisher);
    return uncertainty_planning_tools::ExtractTrajectoryFromTrace(trace);
}

std::pair<SE3Policy, std::map<std::string, double>> uncertainty_planning_core::PlanSE3Uncertainty(const OPTIONS& options, const simplese3_robot_helpers::SimpleSE3Robot& robot, const simplese3_robot_helpers::SimpleSE3BaseSampler& sampler, const Eigen::Affine3d& start, const Eigen::Affine3d& goal, ros::Publisher& display_debug_publisher)
{
    typedef simplese3_robot_helpers::SimpleSE3Robot Robot;
    typedef Eigen::Affine3d Configuration;
    typedef Eigen::aligned_allocator<Eigen::Affine3d> ConfigAlloc;
    typedef std::mt19937_64 PRNG;
    std::shared_ptr<uncertainty_planning_tools::SimulatorInterface<Robot, Configuration, PRNG, ConfigAlloc>> simulator_ptr(new uncertainty_planning_tools::SimpleParticleContactSimulator<Robot, Configuration, PRNG, ConfigAlloc>(options.environment_name, options.environment_resolution, options.simulation_controller_frequency, options.debug_level));
    uncertainty_contact_planning::UncertaintyPlanningSpace<simplese3_robot_helpers::SimpleSE3Robot, simplese3_robot_helpers::SimpleSE3BaseSampler, Eigen::Affine3d, simplese3_robot_helpers::EigenAffine3dSerializer, simplese3_robot_helpers::SimpleSE3Averager, simplese3_robot_helpers::SimpleSE3Distancer, simplese3_robot_helpers::SimpleSE3DimDistancer, simplese3_robot_helpers::SimpleSE3Interpolator, Eigen::aligned_allocator<Eigen::Affine3d>, PRNG> planning_space(options.clustering_type, false, options.debug_level, options.num_particles, options.step_size, options.step_duration, options.goal_distance_threshold, options.goal_probability_threshold, options.signature_matching_threshold, options.distance_clustering_threshold, options.feasibility_alpha, options.variance_alpha, options.connect_after_first_solution, robot, sampler, simulator_ptr, options.prng_seed);
    const std::chrono::duration<double> planner_time_limit(options.planner_time_limit);
    return planning_space.Plan(start, goal, options.goal_bias, planner_time_limit, options.edge_attempt_count, options.policy_action_attempt_count, options.use_contact, options.use_reverse, options.use_spur_actions, display_debug_publisher);
}

std::pair<SE3Policy, std::pair<std::map<std::string, double>, std::pair<std::vector<int64_t>, std::vector<double>>>> uncertainty_planning_core::SimulateSE3UncertaintyPolicy(const OPTIONS& options, const simplese3_robot_helpers::SimpleSE3Robot& robot, const simplese3_robot_helpers::SimpleSE3BaseSampler& sampler, SE3Policy policy, const Eigen::Affine3d& start, const Eigen::Affine3d& goal, ros::Publisher& display_debug_publisher)
{
    typedef simplese3_robot_helpers::SimpleSE3Robot Robot;
    typedef Eigen::Affine3d Configuration;
    typedef Eigen::aligned_allocator<Eigen::Affine3d> ConfigAlloc;
    typedef std::mt19937_64 PRNG;
    std::shared_ptr<uncertainty_planning_tools::SimulatorInterface<Robot, Configuration, PRNG, ConfigAlloc>> simulator_ptr(new uncertainty_planning_tools::SimpleParticleContactSimulator<Robot, Configuration, PRNG, ConfigAlloc>(options.environment_name, options.environment_resolution, options.simulation_controller_frequency, options.debug_level));
    uncertainty_contact_planning::UncertaintyPlanningSpace<simplese3_robot_helpers::SimpleSE3Robot, simplese3_robot_helpers::SimpleSE3BaseSampler, Eigen::Affine3d, simplese3_robot_helpers::EigenAffine3dSerializer, simplese3_robot_helpers::SimpleSE3Averager, simplese3_robot_helpers::SimpleSE3Distancer, simplese3_robot_helpers::SimpleSE3DimDistancer, simplese3_robot_helpers::SimpleSE3Interpolator, Eigen::aligned_allocator<Eigen::Affine3d>, PRNG> planning_space(options.clustering_type, false, options.debug_level, options.num_particles, options.step_size, options.step_duration, options.goal_distance_threshold, options.goal_probability_threshold, options.signature_matching_threshold, options.distance_clustering_threshold, options.feasibility_alpha, options.variance_alpha, options.connect_after_first_solution, robot, sampler, simulator_ptr, options.prng_seed);
    policy.SetPolicyActionAttemptCount(options.policy_action_attempt_count);
    return planning_space.SimulateExectionPolicy(policy, start, goal, options.num_policy_simulations, options.max_exec_actions, display_debug_publisher, true, 0.001);
}

std::pair<SE3Policy, std::pair<std::map<std::string, double>, std::pair<std::vector<int64_t>, std::vector<double>>>> uncertainty_planning_core::ExecuteSE3UncertaintyPolicy(const OPTIONS& options, const simplese3_robot_helpers::SimpleSE3Robot& robot, const simplese3_robot_helpers::SimpleSE3BaseSampler& sampler, SE3Policy policy, const Eigen::Affine3d& start, const Eigen::Affine3d& goal, const std::function<EigenHelpers::VectorAffine3d(const Eigen::Affine3d&, const Eigen::Affine3d&, const double, const double, const bool)>& robot_execution_fn, ros::Publisher& display_debug_publisher)
{
    typedef simplese3_robot_helpers::SimpleSE3Robot Robot;
    typedef Eigen::Affine3d Configuration;
    typedef Eigen::aligned_allocator<Eigen::Affine3d> ConfigAlloc;
    typedef std::mt19937_64 PRNG;
    std::shared_ptr<uncertainty_planning_tools::SimulatorInterface<Robot, Configuration, PRNG, ConfigAlloc>> simulator_ptr(new uncertainty_planning_tools::SimpleParticleContactSimulator<Robot, Configuration, PRNG, ConfigAlloc>(options.environment_name, options.environment_resolution, options.simulation_controller_frequency, options.debug_level));
    uncertainty_contact_planning::UncertaintyPlanningSpace<simplese3_robot_helpers::SimpleSE3Robot, simplese3_robot_helpers::SimpleSE3BaseSampler, Eigen::Affine3d, simplese3_robot_helpers::EigenAffine3dSerializer, simplese3_robot_helpers::SimpleSE3Averager, simplese3_robot_helpers::SimpleSE3Distancer, simplese3_robot_helpers::SimpleSE3DimDistancer, simplese3_robot_helpers::SimpleSE3Interpolator, Eigen::aligned_allocator<Eigen::Affine3d>, PRNG> planning_space(options.clustering_type, false, options.debug_level, options.num_particles, options.step_size, options.step_duration, options.goal_distance_threshold, options.goal_probability_threshold, options.signature_matching_threshold, options.distance_clustering_threshold, options.feasibility_alpha, options.variance_alpha, options.connect_after_first_solution, robot, sampler, simulator_ptr, options.prng_seed);
    policy.SetPolicyActionAttemptCount(options.policy_action_attempt_count);
    return planning_space.ExecuteExectionPolicy(policy, start, goal, robot_execution_fn, options.num_policy_executions, options.max_policy_exec_time, display_debug_publisher, false, 0.001);
}

std::vector<simplelinked_robot_helpers::SimpleLinkedConfiguration, std::allocator<simplelinked_robot_helpers::SimpleLinkedConfiguration>> uncertainty_planning_core::DemonstrateBaxterSimulator(const OPTIONS& options, const simplelinked_robot_helpers::SimpleLinkedRobot<baxter_actuator_helpers::BaxterJointActuatorModel>& robot, const simplelinked_robot_helpers::SimpleLinkedBaseSampler& sampler, const simplelinked_robot_helpers::SimpleLinkedConfiguration& start, const simplelinked_robot_helpers::SimpleLinkedConfiguration& goal, ros::Publisher& display_debug_publisher)
{
    typedef simplelinked_robot_helpers::SimpleLinkedRobot<baxter_actuator_helpers::BaxterJointActuatorModel> Robot;
    typedef simplelinked_robot_helpers::SimpleLinkedConfiguration Configuration;
    typedef std::allocator<simplelinked_robot_helpers::SimpleLinkedConfiguration> ConfigAlloc;
    typedef std::mt19937_64 PRNG;
    std::shared_ptr<uncertainty_planning_tools::SimulatorInterface<Robot, Configuration, PRNG, ConfigAlloc>> simulator_ptr(new uncertainty_planning_tools::SimpleParticleContactSimulator<Robot, Configuration, PRNG, ConfigAlloc>(options.environment_name, options.environment_resolution, options.simulation_controller_frequency, options.debug_level));
    uncertainty_contact_planning::UncertaintyPlanningSpace<Robot, simplelinked_robot_helpers::SimpleLinkedBaseSampler, Configuration, simplelinked_robot_helpers::SimpleLinkedConfigurationSerializer, simplelinked_robot_helpers::SimpleLinkedAverager, baxter_actuator_helpers::SimpleLinkedDistancer, baxter_actuator_helpers::SimpleLinkedDimDistancer, simplelinked_robot_helpers::SimpleLinkedInterpolator, ConfigAlloc, PRNG> planning_space(options.clustering_type, false, options.debug_level, options.num_particles, options.step_size, options.step_duration, options.goal_distance_threshold, options.goal_probability_threshold, options.signature_matching_threshold, options.distance_clustering_threshold, options.feasibility_alpha, options.variance_alpha, options.connect_after_first_solution, robot, sampler, simulator_ptr, options.prng_seed);
    const uncertainty_planning_tools::ForwardSimulationStepTrace<Configuration, ConfigAlloc> trace = planning_space.DemonstrateSimulator(start, goal, display_debug_publisher);
    return uncertainty_planning_tools::ExtractTrajectoryFromTrace(trace);
}

std::pair<BaxterPolicy, std::map<std::string, double>> uncertainty_planning_core::PlanBaxterUncertainty(const OPTIONS& options, const simplelinked_robot_helpers::SimpleLinkedRobot<baxter_actuator_helpers::BaxterJointActuatorModel>& robot, const simplelinked_robot_helpers::SimpleLinkedBaseSampler& sampler, const simplelinked_robot_helpers::SimpleLinkedConfiguration& start, const simplelinked_robot_helpers::SimpleLinkedConfiguration& goal, ros::Publisher& display_debug_publisher)
{
    typedef simplelinked_robot_helpers::SimpleLinkedRobot<baxter_actuator_helpers::BaxterJointActuatorModel> Robot;
    typedef simplelinked_robot_helpers::SimpleLinkedConfiguration Configuration;
    typedef std::allocator<simplelinked_robot_helpers::SimpleLinkedConfiguration> ConfigAlloc;
    typedef std::mt19937_64 PRNG;
    std::shared_ptr<uncertainty_planning_tools::SimulatorInterface<Robot, Configuration, PRNG, ConfigAlloc>> simulator_ptr(new uncertainty_planning_tools::SimpleParticleContactSimulator<Robot, Configuration, PRNG, ConfigAlloc>(options.environment_name, options.environment_resolution, options.simulation_controller_frequency, options.debug_level));
    uncertainty_contact_planning::UncertaintyPlanningSpace<Robot, simplelinked_robot_helpers::SimpleLinkedBaseSampler, Configuration, simplelinked_robot_helpers::SimpleLinkedConfigurationSerializer, simplelinked_robot_helpers::SimpleLinkedAverager, baxter_actuator_helpers::SimpleLinkedDistancer, baxter_actuator_helpers::SimpleLinkedDimDistancer, simplelinked_robot_helpers::SimpleLinkedInterpolator, ConfigAlloc, PRNG> planning_space(options.clustering_type, false, options.debug_level, options.num_particles, options.step_size, options.step_duration, options.goal_distance_threshold, options.goal_probability_threshold, options.signature_matching_threshold, options.distance_clustering_threshold, options.feasibility_alpha, options.variance_alpha, options.connect_after_first_solution, robot, sampler, simulator_ptr, options.prng_seed);
    const std::chrono::duration<double> planner_time_limit(options.planner_time_limit);
    return planning_space.Plan(start, goal, options.goal_bias, planner_time_limit, options.edge_attempt_count, options.policy_action_attempt_count, options.use_contact, options.use_reverse, options.use_spur_actions, display_debug_publisher);
}

std::pair<BaxterPolicy, std::pair<std::map<std::string, double>, std::pair<std::vector<int64_t>, std::vector<double>>>> uncertainty_planning_core::SimulateBaxterUncertaintyPolicy(const OPTIONS& options, const simplelinked_robot_helpers::SimpleLinkedRobot<baxter_actuator_helpers::BaxterJointActuatorModel>& robot, const simplelinked_robot_helpers::SimpleLinkedBaseSampler& sampler, BaxterPolicy policy, const simplelinked_robot_helpers::SimpleLinkedConfiguration& start, const simplelinked_robot_helpers::SimpleLinkedConfiguration& goal, ros::Publisher& display_debug_publisher)
{
    typedef simplelinked_robot_helpers::SimpleLinkedRobot<baxter_actuator_helpers::BaxterJointActuatorModel> Robot;
    typedef simplelinked_robot_helpers::SimpleLinkedConfiguration Configuration;
    typedef std::allocator<simplelinked_robot_helpers::SimpleLinkedConfiguration> ConfigAlloc;
    typedef std::mt19937_64 PRNG;
    std::shared_ptr<uncertainty_planning_tools::SimulatorInterface<Robot, Configuration, PRNG, ConfigAlloc>> simulator_ptr(new uncertainty_planning_tools::SimpleParticleContactSimulator<Robot, Configuration, PRNG, ConfigAlloc>(options.environment_name, options.environment_resolution, options.simulation_controller_frequency, options.debug_level));
    uncertainty_contact_planning::UncertaintyPlanningSpace<Robot, simplelinked_robot_helpers::SimpleLinkedBaseSampler, Configuration, simplelinked_robot_helpers::SimpleLinkedConfigurationSerializer, simplelinked_robot_helpers::SimpleLinkedAverager, baxter_actuator_helpers::SimpleLinkedDistancer, baxter_actuator_helpers::SimpleLinkedDimDistancer, simplelinked_robot_helpers::SimpleLinkedInterpolator, ConfigAlloc, PRNG> planning_space(options.clustering_type, false, options.debug_level, options.num_particles, options.step_size, options.step_duration, options.goal_distance_threshold, options.goal_probability_threshold, options.signature_matching_threshold, options.distance_clustering_threshold, options.feasibility_alpha, options.variance_alpha, options.connect_after_first_solution, robot, sampler, simulator_ptr, options.prng_seed);
    policy.SetPolicyActionAttemptCount(options.policy_action_attempt_count);
    return planning_space.SimulateExectionPolicy(policy, start, goal, options.num_policy_simulations, options.max_exec_actions, display_debug_publisher, false, 0.001);
}

std::pair<BaxterPolicy, std::pair<std::map<std::string, double>, std::pair<std::vector<int64_t>, std::vector<double>>>> uncertainty_planning_core::ExecuteBaxterUncertaintyPolicy(const OPTIONS& options, const simplelinked_robot_helpers::SimpleLinkedRobot<baxter_actuator_helpers::BaxterJointActuatorModel>& robot, const simplelinked_robot_helpers::SimpleLinkedBaseSampler& sampler, BaxterPolicy policy, const simplelinked_robot_helpers::SimpleLinkedConfiguration& start, const simplelinked_robot_helpers::SimpleLinkedConfiguration& goal, const std::function<std::vector<simplelinked_robot_helpers::SimpleLinkedConfiguration, std::allocator<simplelinked_robot_helpers::SimpleLinkedConfiguration>>(const simplelinked_robot_helpers::SimpleLinkedConfiguration&, const simplelinked_robot_helpers::SimpleLinkedConfiguration&, const double, const double, const bool)>& robot_execution_fn, ros::Publisher& display_debug_publisher)
{
    typedef simplelinked_robot_helpers::SimpleLinkedRobot<baxter_actuator_helpers::BaxterJointActuatorModel> Robot;
    typedef simplelinked_robot_helpers::SimpleLinkedConfiguration Configuration;
    typedef std::allocator<simplelinked_robot_helpers::SimpleLinkedConfiguration> ConfigAlloc;
    typedef std::mt19937_64 PRNG;
    std::shared_ptr<uncertainty_planning_tools::SimulatorInterface<Robot, Configuration, PRNG, ConfigAlloc>> simulator_ptr(new uncertainty_planning_tools::SimpleParticleContactSimulator<Robot, Configuration, PRNG, ConfigAlloc>(options.environment_name, options.environment_resolution, options.simulation_controller_frequency, options.debug_level));
    uncertainty_contact_planning::UncertaintyPlanningSpace<Robot, simplelinked_robot_helpers::SimpleLinkedBaseSampler, Configuration, simplelinked_robot_helpers::SimpleLinkedConfigurationSerializer, simplelinked_robot_helpers::SimpleLinkedAverager, baxter_actuator_helpers::SimpleLinkedDistancer, baxter_actuator_helpers::SimpleLinkedDimDistancer, simplelinked_robot_helpers::SimpleLinkedInterpolator, ConfigAlloc, PRNG> planning_space(options.clustering_type, false, options.debug_level, options.num_particles, options.step_size, options.step_duration, options.goal_distance_threshold, options.goal_probability_threshold, options.signature_matching_threshold, options.distance_clustering_threshold, options.feasibility_alpha, options.variance_alpha, options.connect_after_first_solution, robot, sampler, simulator_ptr, options.prng_seed);
    policy.SetPolicyActionAttemptCount(options.policy_action_attempt_count);
    return planning_space.ExecuteExectionPolicy(policy, start, goal, robot_execution_fn, options.num_policy_executions, options.max_policy_exec_time, display_debug_publisher, false, 0.001);
}

std::vector<simplelinked_robot_helpers::SimpleLinkedConfiguration, std::allocator<simplelinked_robot_helpers::SimpleLinkedConfiguration>> uncertainty_planning_core::DemonstrateUR5Simulator(const OPTIONS& options, const simplelinked_robot_helpers::SimpleLinkedRobot<ur5_actuator_helpers::UR5JointActuatorModel>& robot, const simplelinked_robot_helpers::SimpleLinkedBaseSampler& sampler, const simplelinked_robot_helpers::SimpleLinkedConfiguration& start, const simplelinked_robot_helpers::SimpleLinkedConfiguration& goal, ros::Publisher& display_debug_publisher)
{
    typedef simplelinked_robot_helpers::SimpleLinkedRobot<ur5_actuator_helpers::UR5JointActuatorModel> Robot;
    typedef simplelinked_robot_helpers::SimpleLinkedConfiguration Configuration;
    typedef std::allocator<simplelinked_robot_helpers::SimpleLinkedConfiguration> ConfigAlloc;
    typedef std::mt19937_64 PRNG;
    std::shared_ptr<uncertainty_planning_tools::SimulatorInterface<Robot, Configuration, PRNG, ConfigAlloc>> simulator_ptr(new uncertainty_planning_tools::SimpleParticleContactSimulator<Robot, Configuration, PRNG, ConfigAlloc>(options.environment_name, options.environment_resolution, options.simulation_controller_frequency, options.debug_level));
    uncertainty_contact_planning::UncertaintyPlanningSpace<Robot, simplelinked_robot_helpers::SimpleLinkedBaseSampler, Configuration, simplelinked_robot_helpers::SimpleLinkedConfigurationSerializer, simplelinked_robot_helpers::SimpleLinkedAverager, ur5_actuator_helpers::SimpleLinkedDistancer, ur5_actuator_helpers::SimpleLinkedDimDistancer, simplelinked_robot_helpers::SimpleLinkedInterpolator, ConfigAlloc, PRNG> planning_space(options.clustering_type, false, options.debug_level, options.num_particles, options.step_size, options.step_duration, options.goal_distance_threshold, options.goal_probability_threshold, options.signature_matching_threshold, options.distance_clustering_threshold, options.feasibility_alpha, options.variance_alpha, options.connect_after_first_solution, robot, sampler, simulator_ptr, options.prng_seed);
    const uncertainty_planning_tools::ForwardSimulationStepTrace<Configuration, ConfigAlloc> trace = planning_space.DemonstrateSimulator(start, goal, display_debug_publisher);
    return uncertainty_planning_tools::ExtractTrajectoryFromTrace(trace);
}

std::pair<UR5Policy, std::map<std::string, double>> uncertainty_planning_core::PlanUR5Uncertainty(const OPTIONS& options, const simplelinked_robot_helpers::SimpleLinkedRobot<ur5_actuator_helpers::UR5JointActuatorModel>& robot, const simplelinked_robot_helpers::SimpleLinkedBaseSampler& sampler, const simplelinked_robot_helpers::SimpleLinkedConfiguration& start, const simplelinked_robot_helpers::SimpleLinkedConfiguration& goal, ros::Publisher& display_debug_publisher)
{
    typedef simplelinked_robot_helpers::SimpleLinkedRobot<ur5_actuator_helpers::UR5JointActuatorModel> Robot;
    typedef simplelinked_robot_helpers::SimpleLinkedConfiguration Configuration;
    typedef std::allocator<simplelinked_robot_helpers::SimpleLinkedConfiguration> ConfigAlloc;
    typedef std::mt19937_64 PRNG;
    std::shared_ptr<uncertainty_planning_tools::SimulatorInterface<Robot, Configuration, PRNG, ConfigAlloc>> simulator_ptr(new uncertainty_planning_tools::SimpleParticleContactSimulator<Robot, Configuration, PRNG, ConfigAlloc>(options.environment_name, options.environment_resolution, options.simulation_controller_frequency, options.debug_level));
    uncertainty_contact_planning::UncertaintyPlanningSpace<Robot, simplelinked_robot_helpers::SimpleLinkedBaseSampler, Configuration, simplelinked_robot_helpers::SimpleLinkedConfigurationSerializer, simplelinked_robot_helpers::SimpleLinkedAverager, ur5_actuator_helpers::SimpleLinkedDistancer, ur5_actuator_helpers::SimpleLinkedDimDistancer, simplelinked_robot_helpers::SimpleLinkedInterpolator, ConfigAlloc, PRNG> planning_space(options.clustering_type, false, options.debug_level, options.num_particles, options.step_size, options.step_duration, options.goal_distance_threshold, options.goal_probability_threshold, options.signature_matching_threshold, options.distance_clustering_threshold, options.feasibility_alpha, options.variance_alpha, options.connect_after_first_solution, robot, sampler, simulator_ptr, options.prng_seed);
    const std::chrono::duration<double> planner_time_limit(options.planner_time_limit);
    return planning_space.Plan(start, goal, options.goal_bias, planner_time_limit, options.edge_attempt_count, options.policy_action_attempt_count, options.use_contact, options.use_reverse, options.use_spur_actions, display_debug_publisher);
}

std::pair<UR5Policy, std::pair<std::map<std::string, double>, std::pair<std::vector<int64_t>, std::vector<double>>>> uncertainty_planning_core::SimulateUR5UncertaintyPolicy(const OPTIONS& options, const simplelinked_robot_helpers::SimpleLinkedRobot<ur5_actuator_helpers::UR5JointActuatorModel>& robot, const simplelinked_robot_helpers::SimpleLinkedBaseSampler& sampler, UR5Policy policy, const simplelinked_robot_helpers::SimpleLinkedConfiguration& start, const simplelinked_robot_helpers::SimpleLinkedConfiguration& goal, ros::Publisher& display_debug_publisher)
{
    typedef simplelinked_robot_helpers::SimpleLinkedRobot<ur5_actuator_helpers::UR5JointActuatorModel> Robot;
    typedef simplelinked_robot_helpers::SimpleLinkedConfiguration Configuration;
    typedef std::allocator<simplelinked_robot_helpers::SimpleLinkedConfiguration> ConfigAlloc;
    typedef std::mt19937_64 PRNG;
    std::shared_ptr<uncertainty_planning_tools::SimulatorInterface<Robot, Configuration, PRNG, ConfigAlloc>> simulator_ptr(new uncertainty_planning_tools::SimpleParticleContactSimulator<Robot, Configuration, PRNG, ConfigAlloc>(options.environment_name, options.environment_resolution, options.simulation_controller_frequency, options.debug_level));
    uncertainty_contact_planning::UncertaintyPlanningSpace<Robot, simplelinked_robot_helpers::SimpleLinkedBaseSampler, Configuration, simplelinked_robot_helpers::SimpleLinkedConfigurationSerializer, simplelinked_robot_helpers::SimpleLinkedAverager, ur5_actuator_helpers::SimpleLinkedDistancer, ur5_actuator_helpers::SimpleLinkedDimDistancer, simplelinked_robot_helpers::SimpleLinkedInterpolator, ConfigAlloc, PRNG> planning_space(options.clustering_type, false, options.debug_level, options.num_particles, options.step_size, options.step_duration, options.goal_distance_threshold, options.goal_probability_threshold, options.signature_matching_threshold, options.distance_clustering_threshold, options.feasibility_alpha, options.variance_alpha, options.connect_after_first_solution, robot, sampler, simulator_ptr, options.prng_seed);
    policy.SetPolicyActionAttemptCount(options.policy_action_attempt_count);
    return planning_space.SimulateExectionPolicy(policy, start, goal, options.num_policy_simulations, options.max_exec_actions, display_debug_publisher, true, 0.001);
}

std::pair<UR5Policy, std::pair<std::map<std::string, double>, std::pair<std::vector<int64_t>, std::vector<double>>>> uncertainty_planning_core::ExecuteUR5UncertaintyPolicy(const OPTIONS& options, const simplelinked_robot_helpers::SimpleLinkedRobot<ur5_actuator_helpers::UR5JointActuatorModel>& robot, const simplelinked_robot_helpers::SimpleLinkedBaseSampler& sampler, UR5Policy policy, const simplelinked_robot_helpers::SimpleLinkedConfiguration& start, const simplelinked_robot_helpers::SimpleLinkedConfiguration& goal, const std::function<std::vector<simplelinked_robot_helpers::SimpleLinkedConfiguration, std::allocator<simplelinked_robot_helpers::SimpleLinkedConfiguration>>(const simplelinked_robot_helpers::SimpleLinkedConfiguration&, const simplelinked_robot_helpers::SimpleLinkedConfiguration&, const double, const double, const bool)>& robot_execution_fn, ros::Publisher& display_debug_publisher)
{
    typedef simplelinked_robot_helpers::SimpleLinkedRobot<ur5_actuator_helpers::UR5JointActuatorModel> Robot;
    typedef simplelinked_robot_helpers::SimpleLinkedConfiguration Configuration;
    typedef std::allocator<simplelinked_robot_helpers::SimpleLinkedConfiguration> ConfigAlloc;
    typedef std::mt19937_64 PRNG;
    std::shared_ptr<uncertainty_planning_tools::SimulatorInterface<Robot, Configuration, PRNG, ConfigAlloc>> simulator_ptr(new uncertainty_planning_tools::SimpleParticleContactSimulator<Robot, Configuration, PRNG, ConfigAlloc>(options.environment_name, options.environment_resolution, options.simulation_controller_frequency, options.debug_level));
    uncertainty_contact_planning::UncertaintyPlanningSpace<Robot, simplelinked_robot_helpers::SimpleLinkedBaseSampler, Configuration, simplelinked_robot_helpers::SimpleLinkedConfigurationSerializer, simplelinked_robot_helpers::SimpleLinkedAverager, ur5_actuator_helpers::SimpleLinkedDistancer, ur5_actuator_helpers::SimpleLinkedDimDistancer, simplelinked_robot_helpers::SimpleLinkedInterpolator, ConfigAlloc, PRNG> planning_space(options.clustering_type, false, options.debug_level, options.num_particles, options.step_size, options.step_duration, options.goal_distance_threshold, options.goal_probability_threshold, options.signature_matching_threshold, options.distance_clustering_threshold, options.feasibility_alpha, options.variance_alpha, options.connect_after_first_solution, robot, sampler, simulator_ptr, options.prng_seed);
    policy.SetPolicyActionAttemptCount(options.policy_action_attempt_count);
    return planning_space.ExecuteExectionPolicy(policy, start, goal, robot_execution_fn, options.num_policy_executions, options.max_policy_exec_time, display_debug_publisher, true, 0.001);
}

