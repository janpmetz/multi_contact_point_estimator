/*
 * model_stand.h
 *
 *  Created on: May 7, 2017
 *      Author: jan
 */

#ifndef MULTI_CONTACT_POINT_ESTIMATOR_INCLUDE_MULTI_CONTACT_POINT_ESTIMATOR_UNEVEN_TERRAIN_STAND_MODEL_STAND_H_
#define MULTI_CONTACT_POINT_ESTIMATOR_INCLUDE_MULTI_CONTACT_POINT_ESTIMATOR_UNEVEN_TERRAIN_STAND_MODEL_STAND_H_

#include "foot_state_uneven.h"
#include "src/libqhullcpp/Qhull.h"
#include "uneven_terrain_stand.h"

using namespace orgQhull;

class ModelStand {
public:
	ModelStand();
	virtual ~ModelStand();
	FootStateUneven tensorflow_predict(std::vector<vec3> const &points, vec3 zmpv, int dataWidth, int dataHeight, MultiContactPointModel* const &model, double yaw,vigir_terrain_classifier::HeightGridMap::Ptr height_grid_map, FootForm ff);
private:
	vec3 getTriangleNormal(vec3 a, vec3 b, vec3 c);
	double triangle_area_3d(vec3 a, vec3 b, vec3 c);

};

#endif /* MULTI_CONTACT_POINT_ESTIMATOR_INCLUDE_MULTI_CONTACT_POINT_ESTIMATOR_UNEVEN_TERRAIN_STAND_MODEL_STAND_H_ */
