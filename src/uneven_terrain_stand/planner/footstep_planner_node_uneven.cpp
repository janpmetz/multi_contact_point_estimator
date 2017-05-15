/*
 * footstep_planner_node_uneven.cpp
 *
 *  Created on: May 14, 2017
 *      Author: jan
 */

#include <multi_contact_point_estimator/uneven_terrain_stand/planner/footstep_planner_node_uneven.h>
#include <multi_contact_point_estimator/uneven_terrain_stand/terrain_model_uneven.h>
#include <vigir_footstep_planning_msgs/Step.h>
#include <vigir_footstep_planning_plugins/plugins/terrain_model_plugin.h>

using vigir_footstep_planning_msgs::Step;

namespace vigir_footstep_planning {

FootstepPlannerNodeUneven::FootstepPlannerNodeUneven(ros::NodeHandle& nh) : FootstepPlannerNode(nh) {

	update_step_plan_srv_uneven = nh.advertiseService("update_step_plan_uneven", &FootstepPlannerNodeUneven::updateStepPlanService, this);
}

FootstepPlannerNodeUneven::~FootstepPlannerNodeUneven() {
	// TODO Auto-generated destructor stub
}


bool FootstepPlannerNodeUneven::updateStepPlanService(msgs::UpdateStepPlanService::Request &req, msgs::UpdateStepPlanService::Response &resp)
{
	ROS_INFO("[MULTI_CP] Updating step plan with additional information about non-convex feet.");

	std::vector<Step>* steps = &req.step_plan.steps;

	// get a pointer to TerrainModelUneven
	boost::shared_ptr<const TerrainModelPlugin> ptr = WorldModel::instance().getTerrainModel();
	boost::shared_ptr<const TerrainModelUneven> model_ptr = boost::static_pointer_cast<const TerrainModelUneven>(ptr);

	// update each step
	if(model_ptr) {
		for (std::vector<Step>::iterator itr = steps->begin(); itr != steps->end(); itr++)
		{
			Step step = *itr;
			model_ptr->update3DData(step);
			*itr = step;
		}
	}

	resp.step_plan = req.step_plan;

	return true; // return always true so the message is returned
}



} /* namespace vigir_footstep_planning */



int main(int argc, char** argv)
{
  ros::init(argc, argv, "vigir_footstep_planner_uneven");

  ros::NodeHandle nh;

  // ensure that node's services are set up in proper namespace
  if (nh.getNamespace().size() <= 1)
    nh = ros::NodeHandle("~");

  // init parameter and plugin manager
  vigir_generic_params::ParameterManager::initialize(nh);
  vigir_pluginlib::PluginManager::initialize(nh);

  // init footstep planner
  vigir_footstep_planning::FootstepPlannerNodeUneven footstep_planner_node_uneven(nh);

  ros::spin();

  return 0;
}
