/*
 * footstep_planner_node_uneven.h
 *
 *  Created on: May 14, 2017
 *      Author: jan
 */

#ifndef MULTI_CONTACT_POINT_ESTIMATOR_INCLUDE_MULTI_CONTACT_POINT_ESTIMATOR_UNEVEN_TERRAIN_STAND_PLANNER_FOOTSTEP_PLANNER_NODE_UNEVEN_H_
#define MULTI_CONTACT_POINT_ESTIMATOR_INCLUDE_MULTI_CONTACT_POINT_ESTIMATOR_UNEVEN_TERRAIN_STAND_PLANNER_FOOTSTEP_PLANNER_NODE_UNEVEN_H_

#include <vigir_footstep_planner/footstep_planner_node.h>

namespace vigir_footstep_planning {

/*
 * This planner node is derived from the FootstepPlannerNode using inheritance.
 * The planner is extended to offer a service to update the finished step plan
 * to add more information about the foot steps (of non-convex foot shapes).
 */
class FootstepPlannerNodeUneven: public FootstepPlannerNode {
public:
	FootstepPlannerNodeUneven(ros::NodeHandle& nh);
	virtual ~FootstepPlannerNodeUneven();

	bool updateStepPlanService(msgs::UpdateStepPlanService::Request &req, msgs::UpdateStepPlanService::Response &resp);


protected:
	ros::ServiceServer update_step_plan_srv_uneven;

};

} /* namespace vigir_footstep_planning */

#endif /* MULTI_CONTACT_POINT_ESTIMATOR_INCLUDE_MULTI_CONTACT_POINT_ESTIMATOR_UNEVEN_TERRAIN_STAND_PLANNER_FOOTSTEP_PLANNER_NODE_UNEVEN_H_ */
