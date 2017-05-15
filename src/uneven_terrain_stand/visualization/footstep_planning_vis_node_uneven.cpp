#include <boost/smart_ptr/shared_ptr.hpp>
#include <multi_contact_point_estimator/uneven_terrain_stand/visualization/footstep_planning_vis_node_uneven.h>
#include <multi_contact_point_estimator/uneven_terrain_stand/visualization/visualization_non_cvx_foot.h>
#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/service_client.h>
#include <ros/service_client_options.h>
#include <ros/subscribe_options.h>
#include <ros/transport_hints.h>
#include <rosconsole/macros_generated.h>
#include <vigir_footstep_planning_lib/helper.h>
#include <vigir_footstep_planning_lib/visualization/footstep_planning_vis.h>
#include <vigir_footstep_planning_msgs/Step.h>
#include <vigir_footstep_planning_msgs/StepPlan.h>
#include <vigir_footstep_planning_msgs/UpdateStepPlanService.h>

using vigir_footstep_planning_msgs::Step;
using vigir_footstep_planning::msgs::publishStepPlanUneven;



namespace vigir_footstep_planning
{

FootstepPlanningVisNodeUneven::FootstepPlanningVisNodeUneven(){

	ros::NodeHandle nh;

	// Make use a service (planner offers to update the steps with more data about non-convex feet)
	update_step_plan_client = nh.serviceClient<msgs::UpdateStepPlanService>("update_step_plan_uneven");


	// shutdown the subscriber from the base class and re-subscribe, offer own callback
	step_plan_vis_sub.shutdown();
	step_plan_vis_sub = nh.subscribe<msgs::StepPlan>("vis/step_plan", 1, &FootstepPlanningVisNodeUneven::stepPlanVisCallbackUneven, this);

}

FootstepPlanningVisNodeUneven::~FootstepPlanningVisNodeUneven(){}

void FootstepPlanningVisNodeUneven::stepPlanVisCallbackUneven(const msgs::StepPlanConstPtr& step_plan)
{
  msgs::StepPlan step_plan_t = *step_plan;

  // call the service to update the step plan with more information about the foot steps.
  msgs::UpdateStepPlanService update_service;
  update_service.request.step_plan = step_plan_t;
  if (!update_step_plan_client.call(update_service.request, update_service.response)) {
	  ROS_ERROR("[MULTI_CP] in footstep_planning_vis_node_uneven.cpp: Can't call UpdateStepPlanService for step plan update!");
	  return;
  }
  step_plan_t = update_service.response.step_plan;

  // transform to planner frame
  transformToPlannerFrame(step_plan_t, transform_step_plan_client);

  // visualize plan with MORE DATA
  publishStepPlanUneven(step_plan_vis_pub, step_plan_t, foot_size, last_step_plan_vis, true);

  // visualize rest
  vis::publishUpperBody(upper_body_vis_pub, step_plan_t, upper_body_size, upper_body_origin_shift, last_upper_body_vis);
  vis::publishPath(step_plan_path_pub, step_plan_t);

}

}// namespace


int main(int argc, char **argv)
{
  ros::init(argc, argv, "footstep_planning_vis_node_uneven");


  vigir_footstep_planning::FootstepPlanningVisNodeUneven vis_node;
  ros::spin();

  return 0;
}

