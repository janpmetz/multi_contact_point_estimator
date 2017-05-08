/*
 * MyQHull.h
 *
 *  Created on: Jan 22, 2017
 *      Author: jan
 */

#ifndef VIGIR_FOOTSTEP_PLANNING_CORE_VIGIR_FOOTSTEP_PLANNING_DEFAULT_PLUGINS_SRC_UNEVEN_TERRAIN_STAND_STAND_PREDICT_H_
#define VIGIR_FOOTSTEP_PLANNING_CORE_VIGIR_FOOTSTEP_PLANNING_DEFAULT_PLUGINS_SRC_UNEVEN_TERRAIN_STAND_STAND_PREDICT_H_

#include <ros/ros.h>
#include <mutex>

#include "foot_state_uneven.h"
#include "src/libqhullcpp/Qhull.h"
#include "uneven_terrain_stand.h"

using namespace orgQhull;

class MyQHull {
public:
	MyQHull();
	virtual ~MyQHull();
	FootStateUneven qHullCalculatePoints(std::vector<vec3> const &points, vec3 zmpv);
	FootStateUneven tensorflow_predict(std::vector<vec3> const &points, vec3 zmpv, int dataWidth, int dataHeight, MyPredict* const &myPred);
	FootStateUneven tensorflowPredictPoints(std::vector<vec3> const &points, vec3 zmpv, int dataWidth, int dataHeight, MyPredict* const &myPred, double yaw,vigir_terrain_classifier::HeightGridMap::Ptr height_grid_map, FootForm ff);
	bool pointInTriangle(orgQhull::vec3 p, orgQhull::vec3 p0, orgQhull::vec3 p1, orgQhull::vec3 p2);
	vec3 getTriangleNormal(vec3 a, vec3 b, vec3 c);
	double triangle_area_3d(vec3 a, vec3 b, vec3 c);
	void rviz_plot(vec3 normal, vec3 p1, vec3 p2,vec3 p3, vec3 zmpv, std::map<int, vec3> original_point_map);
};

#endif /* VIGIR_FOOTSTEP_PLANNING_CORE_VIGIR_FOOTSTEP_PLANNING_DEFAULT_PLUGINS_SRC_UNEVEN_TERRAIN_STAND_STAND_PREDICT_H_ */
