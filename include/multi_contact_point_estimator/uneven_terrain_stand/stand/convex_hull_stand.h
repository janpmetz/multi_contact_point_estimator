/*
 * ConvexHullStand.h
 *
 *  Created on: May 7, 2017
 *      Author: jan
 */

#ifndef MULTI_CONTACT_POINT_ESTIMATOR_INCLUDE_MULTI_CONTACT_POINT_ESTIMATOR_UNEVEN_TERRAIN_STAND_STAND_CONVEX_HULL_STAND_H_
#define MULTI_CONTACT_POINT_ESTIMATOR_INCLUDE_MULTI_CONTACT_POINT_ESTIMATOR_UNEVEN_TERRAIN_STAND_STAND_CONVEX_HULL_STAND_H_

#include <multi_contact_point_estimator/uneven_terrain_stand/foot/foot_state_uneven.h>
#include <multi_contact_point_estimator/uneven_terrain_stand/utilities/q_hull_extended.h>

using namespace orgQhull;

class ConvexHullStand {
public:
	ConvexHullStand();
	virtual ~ConvexHullStand();
	FootStateUneven getStand(std::vector<vec3> const &points, vec3 zmpv);
private:
	bool pointInTriangle(orgQhull::vec3 p, orgQhull::vec3 p0, orgQhull::vec3 p1, orgQhull::vec3 p2);
	vec3 getTriangleNormal(vec3 a, vec3 b, vec3 c);
};

#endif /* MULTI_CONTACT_POINT_ESTIMATOR_INCLUDE_MULTI_CONTACT_POINT_ESTIMATOR_UNEVEN_TERRAIN_STAND_STAND_CONVEX_HULL_STAND_H_ */
