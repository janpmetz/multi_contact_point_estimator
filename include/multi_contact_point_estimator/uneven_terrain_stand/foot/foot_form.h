#ifndef VIGIR_FOOTSTEP_PLANNING_CORE_VIGIR_FOOTSTEP_PLANNING_DEFAULT_PLUGINS_SRC_UNEVEN_TERRAIN_STAND_FOOT_FORM_H_
#define VIGIR_FOOTSTEP_PLANNING_CORE_VIGIR_FOOTSTEP_PLANNING_DEFAULT_PLUGINS_SRC_UNEVEN_TERRAIN_STAND_FOOT_FORM_H_

#include <ros/ros.h>
#include <vigir_footstep_planning_lib/math.h>

using vigir_footstep_planning::Leg;

class FootForm {
public:
	FootForm();
	virtual ~FootForm();

	bool isInFoot(Leg leg, int x, int y, int width, int height);

	// TODO option to load this from black and white image (once)
	static const int shapeWidth= 20;
	static const int shapeHeight= 20;

	const int foot[20][20];
};

#endif /* VIGIR_FOOTSTEP_PLANNING_CORE_VIGIR_FOOTSTEP_PLANNING_DEFAULT_PLUGINS_SRC_UNEVEN_TERRAIN_STAND_FOOT_FORM_H_ */
