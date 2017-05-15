/*
 * FootForm.cpp
 *
 *  Created on: Jan 12, 2017
 *      Author: jan
 */

#include <multi_contact_point_estimator/uneven_terrain_stand/foot/foot_form.h>
#include <vigir_footstep_planning_lib/math.h>
#include <cmath>

using vigir_footstep_planning::Leg;

// TODO: load foot form from file
FootForm::FootForm()
: foot{	{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
		{0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0},
		{0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0},
		{0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0},
		{0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0},
		{0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0},
		{0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0},
		{0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0},
		{0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,0,0},
		{0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0,0},
		{0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,0,0},
		{0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,0,0},
		{0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,0,0,0},
		{0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0},
		{0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0},
		{0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0},
		{0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0},
		{0,0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,0,0},
		{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
}
{
}

FootForm::~FootForm() {
	// TODO Auto-generated destructor stub
}


bool FootForm::isInFoot(Leg leg, int x, int y, int width, int height) {
	if(width == 0 || height == 0) return false;

	double widthFactor = (shapeWidth) / (double)width;
	double heightFactor = (shapeHeight) / (double)height;

	int idx_x = floor(x*widthFactor); // width and height of foot are opposed to x and y of the array
	int idx_y = floor(y*heightFactor);

	// turn shape around
	if(leg == Leg::LEFT) {
		idx_y = (shapeHeight-1) - idx_y;
	}

	if(idx_x >= shapeHeight || idx_x < 0 || idx_y >= shapeWidth || idx_y < 0) {
		ROS_ERROR("[MULTI_CP] Index is out of bounds of Foot Shape array");
		return false;
	}

	bool r = (foot[idx_x][idx_y] == 1);
	return r;
	//return true;
}

