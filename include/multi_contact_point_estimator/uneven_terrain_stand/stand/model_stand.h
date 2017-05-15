/*
 * model_stand.h
 *
 *  Created on: May 7, 2017
 *      Author: jan
 */

#ifndef MULTI_CONTACT_POINT_ESTIMATOR_INCLUDE_MULTI_CONTACT_POINT_ESTIMATOR_UNEVEN_TERRAIN_STAND_STAND_MODEL_STAND_H_
#define MULTI_CONTACT_POINT_ESTIMATOR_INCLUDE_MULTI_CONTACT_POINT_ESTIMATOR_UNEVEN_TERRAIN_STAND_STAND_MODEL_STAND_H_

#include <multi_contact_point_estimator/uneven_terrain_stand/foot/foot_state_uneven.h>
#include <multi_contact_point_estimator/uneven_terrain_stand/stand/uneven_terrain_stand.h>
#include <multi_contact_point_estimator/uneven_terrain_stand/utilities/q_hull_extended.h>

using namespace orgQhull;	// TODO maybe replace vec3 with other vector implementation

class ModelStand {
public:
	ModelStand();
	virtual ~ModelStand();
	FootStateUneven tensorflow_predict(std::vector<vec3> const &points, vec3 zmpv, int dataWidth, int dataHeight, MultiContactPointModel* const &model, double yaw,vigir_terrain_classifier::HeightGridMap::Ptr height_grid_map, FootForm ff);
private:
	vec3 getTriangleNormal(vec3 a, vec3 b, vec3 c);
	double triangle_area_3d(vec3 a, vec3 b, vec3 c);
	void getMaxMinPoints(std::vector<vec3> const &points, double &max, double &min);
	void flattenToScaledVector(const std::vector<vec3>& points, int dataWidth, std::map<int, std::vector<double> >& original_point_map,std::vector<float>& pointsFlat);
	void checkNormalUpwards(orgQhull::vec3& n, orgQhull::vec3& p1, orgQhull::vec3& p2, orgQhull::vec3& p3);

	std::vector<vec3> getMostLikelyPoints(const std::vector<float>& pred,
			int dataWidth, FootForm ff, int dataHeight,
			const vigir_terrain_classifier::HeightGridMap::Ptr& height_grid_map,
			std::map<int, std::vector<double> >& original_point_map);

	// Template helper method sorts in ascending order
	template <typename T>
	std::vector<size_t> indices_of_max_elements(std::vector<T> const& values) {
	    std::vector<size_t> indices(values.size());
	    std::iota(indices.begin(), indices.end(), 0); // shows eclipse error but compiles

	    std::sort(
	        indices.begin(), indices.end(),
	        [&](size_t a, size_t b) { return values[a] < values[b]; }
	    );
	    return indices;
	}

};

#endif /* MULTI_CONTACT_POINT_ESTIMATOR_INCLUDE_MULTI_CONTACT_POINT_ESTIMATOR_UNEVEN_TERRAIN_STAND_STAND_MODEL_STAND_H_ */
