/*
 * footstep_planning_vis_node_extract.cpp
 *
 *  Created on: May 15, 2017
 *      Author: jan
 */
#include <multi_contact_point_estimator/uneven_terrain_stand/terrain_model_uneven.h>
#include <multi_contact_point_estimator/uneven_terrain_stand/visualization/footstep_planning_vis_node_uneven.h>
#include <multi_contact_point_estimator/uneven_terrain_stand/visualization/visualization_non_cvx_foot.h>
#include <vigir_footstep_planning_lib/helper.h>
#include <vigir_footstep_planning_msgs/Step.h>
#include <vigir_footstep_planning_plugins/plugin_aggregators/world_model.h>
#include <vector>

using vigir_footstep_planning_msgs::Step;
using vigir_footstep_planning::msgs::publishStepPlanUneven;

/**
 * The original footstep_planning_vis_node.cpp contains a MAIN method.
 * Because the derived node has its own MAIN method the original cpp file cannot be included.
 * TODO when the main method is separated from the original cpp file, this file here can be removed.
 */

namespace vigir_footstep_planning
{

FootstepPlanningVisNode::FootstepPlanningVisNode()
{
  ros::NodeHandle nh;

  // load param
  getFootSize(nh, foot_size);
  getUpperBodySize(nh, upper_body_size);
  getUpperBodyOriginShift(nh, upper_body_origin_shift);

  // subscribe topics
  step_plan_request_vis_sub = nh.subscribe<msgs::StepPlanRequest>("vis/step_plan_request", 1, &FootstepPlanningVisNode::stepPlanRequestVisCallback, this);
  step_plan_vis_sub = nh.subscribe<msgs::StepPlan>("vis/step_plan", 1, &FootstepPlanningVisNode::stepPlanVisCallback, this);
  planning_feedback_sub = nh.subscribe<msgs::PlanningFeedback>("planning_feedback", 1, &FootstepPlanningVisNode::planningFeedbackCallback, this);

  // publish topics
  step_plan_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("vis/step_plan_vis", 1, true);
  upper_body_vis_pub = nh.advertise<visualization_msgs::MarkerArray>("vis/upper_body_vis", 1, true);
  step_plan_path_pub = nh.advertise<nav_msgs::Path>("vis/step_plan_path", 1, true);
  start_feet_pose_pub = nh.advertise<visualization_msgs::MarkerArray>("vis/start_feet_pose", 1, true);
  goal_feet_pose_pub = nh.advertise<visualization_msgs::MarkerArray>("vis/goal_feet_pose", 1, true);
  visited_steps_pub = nh.advertise<sensor_msgs::PointCloud2>("feedback/visited_steps", 1);
  total_visited_steps_pub = nh.advertise<sensor_msgs::PointCloud2>("feedback/total_visited_steps", 1);
  current_step_plan_pub = nh.advertise<visualization_msgs::MarkerArray>("feedback/current_step_plan", 1);

  // start service clients
  transform_foot_pose_client = nh.serviceClient<msgs::TransformFootPoseService>("transform_foot_pose");
  transform_feet_poses_client = nh.serviceClient<msgs::TransformFeetPosesService>("transform_feet_poses");
  transform_step_plan_client = nh.serviceClient<msgs::TransformStepPlanService>("transform_step_plan");

}

FootstepPlanningVisNode::~FootstepPlanningVisNode()
{
}

void FootstepPlanningVisNode::stepPlanRequestVisCallback(const msgs::StepPlanRequestConstPtr& step_plan_request)
{
  // clear old vis
  clearVisualization(step_plan_request->header);

  msgs::Feet start = step_plan_request->start;
  msgs::Feet goal = step_plan_request->goal;

  // transform start pose to planner frame and visualize
  if (!start.header.frame_id.empty())
  {
    transformToPlannerFrame(start, transform_feet_poses_client);
    vis::publishStart(start_feet_pose_pub, start, foot_size);
  }

  // transform goal pose to planner frame and visualize
  if (!goal.header.frame_id.empty())
  {
    transformToPlannerFrame(goal, transform_feet_poses_client);
    vis::publishGoal(goal_feet_pose_pub, goal, foot_size);
  }
}

void FootstepPlanningVisNode::stepPlanVisCallback(const msgs::StepPlanConstPtr& step_plan)
{

	// transform to planner frame
	msgs::StepPlan step_plan_t = *step_plan;
	transformToPlannerFrame(step_plan_t, transform_step_plan_client);

	// visualize plan
	vis::publishStepPlan(step_plan_vis_pub, step_plan_t, foot_size, last_step_plan_vis);
	vis::publishUpperBody(upper_body_vis_pub, step_plan_t, upper_body_size, upper_body_origin_shift, last_upper_body_vis);
	vis::publishPath(step_plan_path_pub, step_plan_t);

}



void FootstepPlanningVisNode::planningFeedbackCallback(const msgs::PlanningFeedbackConstPtr& planning_feedback)
{
  // collect data
  for (std::vector<msgs::Step>::const_iterator itr = planning_feedback->visited_steps.begin(); itr != planning_feedback->visited_steps.end(); itr++)
  {
    msgs::Step step = *itr;
    total_visited_steps.erase(step);
    total_visited_steps.insert(step);
  }

  if (visited_steps_pub.getNumSubscribers() > 0)
    vis::publishRecentlyVistedSteps(visited_steps_pub, planning_feedback->visited_steps, planning_feedback->header);

  if (total_visited_steps_pub.getNumSubscribers() > 0)
    vis::publishVistedSteps(total_visited_steps_pub, total_visited_steps, planning_feedback->header);

  msgs::StepPlan current_step_plan = planning_feedback->current_step_plan;
  transformToPlannerFrame(current_step_plan, transform_step_plan_client);
  vis::publishStepPlan(current_step_plan_pub, current_step_plan, foot_size, last_current_step_plan_vis, false);
}

void FootstepPlanningVisNode::clearVisualization(const std_msgs::Header& header)
{
  total_visited_steps.clear();

  // last start and goal pose from request
  vis::clearFeet(start_feet_pose_pub, header);
  vis::clearFeet(goal_feet_pose_pub, header);

  // last planner feedback
  vis::clearMarkerArray(current_step_plan_pub, last_current_step_plan_vis);
  sensor_msgs::PointCloud2 point_cloud_msgs;
  point_cloud_msgs.header = header;
  visited_steps_pub.publish(point_cloud_msgs);
  total_visited_steps_pub.publish(point_cloud_msgs);

  // last solution
  vis::clearMarkerArray(step_plan_vis_pub, last_step_plan_vis);
  vis::clearMarkerArray(upper_body_vis_pub, last_upper_body_vis);
  vis::clearPath(step_plan_path_pub, header);
}

} // namespace
