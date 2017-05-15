/*
 * FootstepPlanningVisNodeUneven.h
 *
 *  Created on: May 13, 2017
 *      Author: jan
 */

#ifndef FOOTSTEP_PLANNING_VIS_NODE_UNEVEN_H_
#define FOOTSTEP_PLANNING_VIS_NODE_UNEVEN_H_

#include <ros/ros.h>
#include <vigir_footstep_planning_lib/visualization/footstep_planning_vis_node.h>


namespace vigir_footstep_planning {

/*
 * This visualization node is derived from the FootstepPlanningNode using inheritance.
 * The node is extended to subscribe to a topic, offer a new callback and use a service from the footstep_planner_node_uneven.
 */
class FootstepPlanningVisNodeUneven : public FootstepPlanningVisNode{
public:
	FootstepPlanningVisNodeUneven();
	virtual ~FootstepPlanningVisNodeUneven();

	void stepPlanVisCallbackUneven(const msgs::StepPlanConstPtr& step_plan);
protected:
	ros::ServiceClient update_step_plan_client;
};

} /* namespace vigir_footstep_planning */

#endif /* MULTI_CONTACT_POINT_ESTIMATOR_INCLUDE_MULTI_CONTACT_POINT_ESTIMATOR_UNEVEN_TERRAIN_STAND_FOOTSTEP_PLANNING_VIS_NODE_UNEVEN_H_ */
