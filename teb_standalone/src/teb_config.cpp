/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 *********************************************************************/

#include <teb_local_planner/teb_config.h>

namespace teb_local_planner
{
    

void TebConfig::reconfigure(TebLocalPlannerReconfigureConfig& cfg)
{ 
  boost::mutex::scoped_lock l(config_mutex_);
  
  // Trajectory
  trajectory.teb_autosize = cfg.teb_autosize;
  trajectory.dt_ref = cfg.dt_ref;
  trajectory.dt_hysteresis = cfg.dt_hysteresis;
  trajectory.global_plan_overwrite_orientation = cfg.global_plan_overwrite_orientation;
  trajectory.allow_init_with_backwards_motion = cfg.allow_init_with_backwards_motion;
  trajectory.global_plan_viapoint_sep = cfg.global_plan_viapoint_sep;
  trajectory.via_points_ordered = cfg.via_points_ordered;
  trajectory.max_global_plan_lookahead_dist = cfg.max_global_plan_lookahead_dist;
  trajectory.exact_arc_length = cfg.exact_arc_length;
  trajectory.force_reinit_new_goal_dist = cfg.force_reinit_new_goal_dist;
  trajectory.force_reinit_new_goal_angular = cfg.force_reinit_new_goal_angular;
  trajectory.feasibility_check_no_poses = cfg.feasibility_check_no_poses;
  trajectory.feasibility_check_lookahead_distance = cfg.feasibility_check_lookahead_distance;
  trajectory.publish_feedback = cfg.publish_feedback;
  trajectory.control_look_ahead_poses = cfg.control_look_ahead_poses;
  trajectory.prevent_look_ahead_poses_near_goal = cfg.prevent_look_ahead_poses_near_goal;
  
  // Robot     
  robot.max_vel_x = cfg.max_vel_x;
  robot.max_vel_x_backwards = cfg.max_vel_x_backwards;
  robot.max_vel_y = cfg.max_vel_y;
  robot.max_vel_theta = cfg.max_vel_theta;
  robot.acc_lim_x = cfg.acc_lim_x;
  robot.acc_lim_y = cfg.acc_lim_y;
  robot.acc_lim_theta = cfg.acc_lim_theta;
  robot.min_turning_radius = cfg.min_turning_radius;
  robot.wheelbase = cfg.wheelbase;
  robot.cmd_angle_instead_rotvel = cfg.cmd_angle_instead_rotvel;
  robot.use_proportional_saturation = cfg.use_proportional_saturation;
  if (cfg.max_vel_trans == 0.0)
  {
    cfg.max_vel_trans = robot.max_vel_x;
  }
  robot.max_vel_trans = cfg.max_vel_trans;
  
  // GoalTolerance
  goal_tolerance.xy_goal_tolerance = cfg.xy_goal_tolerance;
  goal_tolerance.yaw_goal_tolerance = cfg.yaw_goal_tolerance;
  goal_tolerance.free_goal_vel = cfg.free_goal_vel;
  goal_tolerance.trans_stopped_vel = cfg.trans_stopped_vel;
  goal_tolerance.theta_stopped_vel = cfg.theta_stopped_vel;

  // Obstacles
  obstacles.min_obstacle_dist = cfg.min_obstacle_dist;
  obstacles.inflation_dist = cfg.inflation_dist;
  obstacles.dynamic_obstacle_inflation_dist = cfg.dynamic_obstacle_inflation_dist;
  obstacles.include_dynamic_obstacles = cfg.include_dynamic_obstacles;
  obstacles.include_costmap_obstacles = cfg.include_costmap_obstacles;
  obstacles.legacy_obstacle_association = cfg.legacy_obstacle_association;
  obstacles.obstacle_association_force_inclusion_factor = cfg.obstacle_association_force_inclusion_factor;
  obstacles.obstacle_association_cutoff_factor = cfg.obstacle_association_cutoff_factor;
  obstacles.costmap_obstacles_behind_robot_dist = cfg.costmap_obstacles_behind_robot_dist;
  obstacles.obstacle_poses_affected = cfg.obstacle_poses_affected;
  obstacles.obstacle_proximity_ratio_max_vel = cfg.obstacle_proximity_ratio_max_vel;
  obstacles.obstacle_proximity_lower_bound = cfg.obstacle_proximity_lower_bound;
  obstacles.obstacle_proximity_upper_bound = cfg.obstacle_proximity_upper_bound;
  
  // Optimization
  optim.no_inner_iterations = cfg.no_inner_iterations;
  optim.no_outer_iterations = cfg.no_outer_iterations;
  optim.optimization_activate = cfg.optimization_activate;
  optim.optimization_verbose = cfg.optimization_verbose;
  optim.penalty_epsilon = cfg.penalty_epsilon;
  optim.weight_max_vel_x = cfg.weight_max_vel_x;
  optim.weight_max_vel_y = cfg.weight_max_vel_y;
  optim.weight_max_vel_theta = cfg.weight_max_vel_theta;
  optim.weight_acc_lim_x = cfg.weight_acc_lim_x;
  optim.weight_acc_lim_y = cfg.weight_acc_lim_y;
  optim.weight_acc_lim_theta = cfg.weight_acc_lim_theta;
  optim.weight_kinematics_nh = cfg.weight_kinematics_nh;
  optim.weight_kinematics_forward_drive = cfg.weight_kinematics_forward_drive;
  optim.weight_kinematics_turning_radius = cfg.weight_kinematics_turning_radius;
  optim.weight_optimaltime = cfg.weight_optimaltime;
  optim.weight_shortest_path = cfg.weight_shortest_path;
  optim.weight_obstacle = cfg.weight_obstacle;
  optim.weight_inflation = cfg.weight_inflation;
  optim.weight_dynamic_obstacle = cfg.weight_dynamic_obstacle;
  optim.weight_dynamic_obstacle_inflation = cfg.weight_dynamic_obstacle_inflation;
  optim.weight_velocity_obstacle_ratio = cfg.weight_velocity_obstacle_ratio;
  optim.weight_viapoint = cfg.weight_viapoint;
  optim.weight_adapt_factor = cfg.weight_adapt_factor;
  optim.obstacle_cost_exponent = cfg.obstacle_cost_exponent;
  
  // Homotopy Class Planner
  hcp.enable_multithreading = cfg.enable_multithreading;
  hcp.max_number_classes = cfg.max_number_classes; 
  hcp.max_number_plans_in_current_class = cfg.max_number_plans_in_current_class;
  hcp.selection_cost_hysteresis = cfg.selection_cost_hysteresis;
  hcp.selection_prefer_initial_plan = cfg.selection_prefer_initial_plan;
  hcp.selection_obst_cost_scale = cfg.selection_obst_cost_scale;
  hcp.selection_viapoint_cost_scale = cfg.selection_viapoint_cost_scale;
  hcp.selection_alternative_time_cost = cfg.selection_alternative_time_cost;
  hcp.selection_dropping_probability = cfg.selection_dropping_probability;
  hcp.switching_blocking_period = cfg.switching_blocking_period;
  
  hcp.obstacle_heading_threshold = cfg.obstacle_heading_threshold;
  hcp.roadmap_graph_no_samples = cfg.roadmap_graph_no_samples;
  hcp.roadmap_graph_area_width = cfg.roadmap_graph_area_width;
  hcp.roadmap_graph_area_length_scale = cfg.roadmap_graph_area_length_scale;
  hcp.h_signature_prescaler = cfg.h_signature_prescaler;
  hcp.h_signature_threshold = cfg.h_signature_threshold;
  hcp.viapoints_all_candidates = cfg.viapoints_all_candidates;
  hcp.visualize_hc_graph = cfg.visualize_hc_graph;
  hcp.visualize_with_time_as_z_axis_scale = cfg.visualize_with_time_as_z_axis_scale;
  
  // Recovery
  recovery.shrink_horizon_backup = cfg.shrink_horizon_backup;
  recovery.oscillation_recovery = cfg.oscillation_recovery;
  recovery.divergence_detection_enable = cfg.divergence_detection_enable;
  recovery.divergence_detection_max_chi_squared = cfg.divergence_detection_max_chi_squared;

  
  checkParameters();
}
    
    
void TebConfig::checkParameters() const
{
  // positive backward velocity?
  if (robot.max_vel_x_backwards <= 0)
  
  // bounds smaller than penalty epsilon
  if (robot.max_vel_x <= optim.penalty_epsilon)
  
  if (robot.max_vel_x_backwards <= optim.penalty_epsilon)
  
  if (robot.max_vel_theta <= optim.penalty_epsilon)
  
  if (robot.acc_lim_x <= optim.penalty_epsilon)
  
  if (robot.acc_lim_theta <= optim.penalty_epsilon)
      
  // dt_ref and dt_hyst
  if (trajectory.dt_ref <= trajectory.dt_hysteresis)
    
  // min number of samples
  if (trajectory.min_samples <3)
  
  // costmap obstacle behind robot
  if (obstacles.costmap_obstacles_behind_robot_dist < 0)
    
  // hcp: obstacle heading threshold
  if (hcp.obstacle_keypoint_offset>=1 || hcp.obstacle_keypoint_offset<=0)
  
  // carlike
  if (robot.cmd_angle_instead_rotvel && robot.wheelbase==0)
  
  if (robot.cmd_angle_instead_rotvel && robot.min_turning_radius==0)
  
  // positive weight_adapt_factor
  if (optim.weight_adapt_factor < 1.0)
  
  if (recovery.oscillation_filter_duration < 0)

  // weights
  if (optim.weight_optimaltime <= 0)

  // holonomic check
  if (robot.max_vel_y > 0) {
    if (robot.max_vel_trans < std::min(robot.max_vel_x, robot.max_vel_trans)) {
    }
    
    if (robot.max_vel_trans > std::max(robot.max_vel_x, robot.max_vel_y)) {
    }
  }
  
}    

    
} // namespace teb_local_planner
