/*
 * model_stand.cpp
 *
 *  Created on: May 7, 2017
 *      Author: jan
 */

#include <vigir_footstep_planning_lib/math.h>
#include <vector>
#include <math.h>
#include <multi_contact_point_estimator/uneven_terrain_stand/stand/model_stand.h>
#include <map>

ModelStand::ModelStand() {
	// TODO Auto-generated constructor stub

}

ModelStand::~ModelStand() {
	// TODO Auto-generated destructor stub
}

FootStateUneven ModelStand::tensorflow_predict(std::vector<vec3> const &points, vec3 zmpv, int dataWidth, int dataHeight, MultiContactPointModel* const &model, double yaw, vigir_terrain_classifier::HeightGridMap::Ptr height_grid_map, FootForm ff, Leg leg){

	FootStateUneven stand = FootStateUneven();

	// initializations
	std::vector<double> zmpvec= {zmpv.X[0], zmpv.X[1]};
	std::vector<double> pointsFlat(dataWidth*dataHeight, 0.0f);
	std::map<int, std::vector<double>> original_point_map;

	// turn data to a flat vector with data scaled to [0,1] to feed it into the predicting model
	flattenToScaledVector(points, dataWidth, original_point_map, pointsFlat);

	// PREDICT CONTACT POINTS
	// ##################################################
	// flat predict matrix dataWith x dataHeight containing most likely contact points
	std::vector<double> pred = model->make_prediction(pointsFlat, zmpvec, dataWidth, dataHeight);
	// ##################################################

	// get 3 most likely predicted contact points from the map (that are within the foot shape)
	std::vector<vec3> maxPoints = getMostLikelyPoints(pred, dataWidth, ff, dataHeight, height_grid_map, original_point_map);

	if(maxPoints.size() != 3) {
		stand.setValid(-3);
		return stand;	// bad support polygon
	}

	// the contact points
	vec3 p1,p2,p3;
	p1 = maxPoints.at(0);
	p2 = maxPoints.at(1);
	p3 = maxPoints.at(2);

	// surface normal of the foot
	vec3 n = getTriangleNormal(p1, p2, p3);

	// normal should face upwards here (if wrong turn it)
	checkNormalUpwards(n, p1, p2, p3);

	// fill "stand" data structure 	// TODO how to check validity properly?
	stand.setValid(1);
	stand.setFacetArea(triangle_area_3d(p1, p2, p3));
	stand.setP1(p1.getStdVec());
	stand.setP2(p2.getStdVec());
	stand.setP3(p3.getStdVec());

	// normalizing the normal
	double len = sqrt((n.X[0] * n.X[0]) + (n.X[1] * n.X[1]) + (n.X[2] * n.X[2]));
	n.X[0] = n.X[0]/len;
	n.X[1] = n.X[1]/len;
	n.X[2] = n.X[2]/len;

	// TODO check if the prediction is the right orientation w.r.t. x y up down etc

	stand.setNorm(n.getStdVec());
	stand.setOriginalPointMap(original_point_map);

	return stand;
}


/**
 * From the predicted map of contat points, choose the 3 best
 * That are within the foot shape
 */
std::vector<vec3> ModelStand::getMostLikelyPoints(
		const std::vector<double>& pred, int dataWidth, FootForm ff,	int dataHeight,
		const vigir_terrain_classifier::HeightGridMap::Ptr& height_grid_map,
		std::map<int, std::vector<double> >& original_point_map) {


	// get 3 most likely predicted contact points from the map
	std::vector<unsigned long int> big_pts_idxs = indices_of_max_elements(pred);
	unsigned long int lastIdx = big_pts_idxs.size() - 1; // indices in ascending order

	// fill 3 biggest points with original x, y, z coordinates (work with original height not the predicted height)
	// that are within the foot shape
	std::vector<vec3> maxPoints;
	for (int i = 0; i < big_pts_idxs.size(); i++) {
		vec3 p;
		int idx = big_pts_idxs.at(lastIdx - i);
		// at predicted index, take originally recorded point, predicted height is ignored
		std::map<int, std::vector<double> >::iterator original_point =
				original_point_map.find(idx);
		// translate 1d index to 2d coordinates
		p.IDX[0] = floor(idx / dataWidth);
		p.IDX[1] = idx % dataWidth;
		// only if predicted contact point is really within the foot form then use the prediction

		bool inFoot = ff.isInFoot(Leg::RIGHT, p.IDX[0], p.IDX[1], dataWidth, dataHeight);
		// #TODO USE LEG AS PARAM, look in foot_form.cpp how y is flipped

		// if predicted point does not match a point under the sole continue
		if (original_point == original_point_map.end()) {
			continue;
		}
		std::vector<double> op = original_point->second;
		p.X[0] = op[0];
		p.X[1] = op[1];
		if (inFoot) {
			// set original height of terrain to estimated contact point
			bool hasHeight1 = height_grid_map->getHeight(p.X[0], p.X[1],
					p.X[2]);
		} else {
			// reject, bad contact point
			continue;
		}
		maxPoints.push_back(p);
		if (maxPoints.size() == 3) {
			break;
		}
	}
	return maxPoints;
}

void ModelStand::checkNormalUpwards(vec3& n, vec3& p1, vec3& p2, vec3& p3) {
	// normal should face upwards here (more general, should face towards robot, if the robot wants to walk on the ceiling)
	if (n.X[2] < 0) {
		// changing the sign like n*(-1) to switch the direction would work but,
		// its probably better if the order of the points determines the
		// normal correctly
		vec3 tmp = p3;
		p3 = p1;
		p1 = tmp;
		n = getTriangleNormal(p1, p2, p3);
	}
}

void ModelStand::flattenToScaledVector(const std::vector<vec3>& points,
		int dataWidth, std::map<int, std::vector<double> >& original_point_map,
		std::vector<double>& pointsFlat) {

	// turn data to a flat vector with data scaled to [0,1]
	double max, min;
	getMaxMinPoints(points, max, min);
	for (int i = 0; i < points.size(); i++) {
		long int idx = (points.at(i).IDX[0] * dataWidth) + points.at(i).IDX[1];
		double z = points.at(i).X[2];
		z = (z - min) / (max - min);
		pointsFlat.at(idx) = z;
		vec3 originalPoint = vec3(points.at(i).X[0], points.at(i).X[1], points.at(i).X[2]);
		original_point_map[idx] = originalPoint.getStdVec();
	}
}

void ModelStand::getMaxMinPoints(std::vector<vec3> const &points, double &max, double &min) {
	max = points.at(0).X[2];
	min = points.at(0).X[2];
	for(int i = 0; i < points.size(); i++) {
		double z = points.at(i).X[2];
		if(z > max) {
			max = z;
		} else if(z<min) {
			min = z;
		}
	}
}

double ModelStand::triangle_area_3d(vec3 a, vec3 b, vec3 c) {
	// u = AB
	vec3 u = b - a;
	// v = AC
	vec3 v = c - a;

	// u x v
	vec3 w = u.cross(v);
	double area = 0.5*sqrt(w.X[0]*w.X[0] +  w.X[1]*w.X[1] +  w.X[2]*w.X[2]);

	return area;
}

vec3 ModelStand::getTriangleNormal(vec3 p1, vec3 p2, vec3 p3) {
	vec3 u = p2 - p1;
	vec3 v = p3 - p1;
	// normal = u x v
	return u.cross(v);
}
