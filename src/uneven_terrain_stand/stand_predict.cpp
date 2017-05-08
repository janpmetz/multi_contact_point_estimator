#include <multi_contact_point_estimator/uneven_terrain_stand/stand_predict.h>

#include <vector>
#include <math.h>
#include <map>

#include <libqhullcpp/QhullFacetList.h>
#include <visualization_msgs/Marker.h>
#include <vigir_footstep_planning_lib/math.h>

#include <multi_contact_point_estimator/uneven_terrain_stand/src/libqhullcpp/Qhull.h>
#include <multi_contact_point_estimator/uneven_terrain_stand/src/libqhullcpp/QhullVertexSet.h>

MyQHull::MyQHull() {
	// TODO Auto-generated constructor stub
}

MyQHull::~MyQHull() {
	// TODO Auto-generated destructor stub
}


// sorts in ascending order
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

FootStateUneven MyQHull::tensorflowPredictPoints(std::vector<vec3> const &points, vec3 zmpv, int dataWidth, int dataHeight, MyPredict* const &myPred, double yaw, vigir_terrain_classifier::HeightGridMap::Ptr height_grid_map, FootForm ff){

	FootStateUneven stand = FootStateUneven();

	std::vector<float> zmpvec= {zmpv.X[0], zmpv.X[1]};
	std::vector<float> pointsFlat(dataWidth*dataHeight, 0.0f);

	std::map<int, std::vector<double>> original_point_map;

	// get maximum and minimum
	double max = points.at(0).X[2];
	double min = points.at(0).X[2];
	for(int i = 0; i < points.size(); i++) {
		double z = points.at(i).X[2];
		if(z > max) {
			max = z;
		} else if(z<min) {
			min = z;
		}
	}

	// turn data to a flat vector with data scaled to [0,1] to feed it into the predicting model
	for(int i = 0; i < points.size(); i++) {
		long int idx =  (points.at(i).IDX[0]*dataWidth) + points.at(i).IDX[1];
		double z = points.at(i).X[2];
		z = (z-min)/(max-min);
		pointsFlat.at(idx) = z;
		vec3 originalPoint = vec3(points.at(i).X[0], points.at(i).X[1], points.at(i).X[2]);
		original_point_map[idx] = originalPoint.getStdVec();
	}


	// PREDICT CONTACT POINTS
	// ##################################################
	// flat predict matrix dataWith x dataHeight containing most likely contact points
	std::vector<float> pred = myPred->modelPrediction(pointsFlat, zmpvec, dataWidth, dataHeight);
	// ##################################################

	// get 3 most likely predicted contact points
	std::vector<unsigned long int> big_pts_idxs = indices_of_max_elements(pred);
	unsigned long int lastIdx = big_pts_idxs.size()-1; // indices in ascending order

	// fill 3 biggest points with original x, y, z (not the predicted height)
	// that are within the foot shape
	std::vector<vec3> maxPoints;
	for(int i = 0; i < big_pts_idxs.size(); i++) {

		vec3 p;
		int idx = big_pts_idxs.at(lastIdx - i);

		// at predicted index, take originally recorded point, predicted height is ignored
		std::map<int, std::vector<double>>::iterator original_point = original_point_map.find(idx);

		// translate 1d index to 2d coordinates
		p.IDX[0] = floor(idx / dataWidth);
		p.IDX[1] = idx % dataWidth;

		// only if predicted contact point is really within the foot form then use the prediction
		bool inFoot = ff.isInFoot(p.IDX[0], p.IDX[1], dataWidth, dataHeight);

		// if predicted point does not match a point under the sole
		if(original_point == original_point_map.end()){
			continue;
		}

		std::vector<double> op = original_point->second;
		p.X[0] =  op[0];
		p.X[1] =  op[1];

		if(inFoot) {

			// set original height of terrain to estimated contact point
			bool hasHeight1 = height_grid_map->getHeight(p.X[0], p.X[1], p.X[2]);

		} else { // reject, bad contact point
			continue;
		}

		maxPoints.push_back(p);
		if(maxPoints.size() == 3) {
			break;
		}
	}

	if(maxPoints.size() != 3) {
		stand.setValid(-3);
		return stand;
	}

	// the contact points
	vec3 p1,p2,p3;
	p1 = maxPoints.at(0);
	p2 = maxPoints.at(1);
	p3 = maxPoints.at(2);

	// surface normal of the foot
	vec3 n = getTriangleNormal(p1, p2, p3);

	// normal should face upwards here (more general, should face towards robot, if the robot wants to walk on the ceiling)
	if(n.X[2] < 0) {
		// changing the sign like n*(-1) to switch the direction would work but,
		// its probably better if the order of the points determines the
		// normal correctly
		vec3 tmp = p3;
		p3 = p1;
		p1 = tmp;
		n = getTriangleNormal(p1, p2, p3);
	}

	// fill "stand" data structure 	// TODO how to check validity properly?
	stand.setValid(1);
	stand.setFacetArea(triangle_area_3d(p1, p2, p3));
	stand.setP1(p1.getStdVec());
	stand.setP2(p2.getStdVec());
	stand.setP3(p3.getStdVec());

	// get the HEIGHT of the ZMP on the plane that goes through the 3 contact points
	// height of the foot measured at the x,y coordinates of the ZMP
	double A = n.X[0]; double B = n.X[1]; double C = n.X[2];
	double D = -n.dot(p1); //-dot(normal,p1);
	double zmpHeightOnFacet = (A * zmpv.X[0] + B * zmpv.X[1] + D) / (-C); //(A.*x + B.*y + D)./(-C);
	zmpv.X[3] = zmpHeightOnFacet;
	stand.setHeight(zmpHeightOnFacet);

	// TODO check if the prediction is the right orientation w.r.t. x y up down etc

	// normalizing
	double len = sqrt((n.X[0] * n.X[0]) + (n.X[1] * n.X[1]) + (n.X[2] * n.X[2]));
	n.X[0] = n.X[0]/len;
	n.X[1] = n.X[1]/len;
	n.X[2] = n.X[2]/len;

	stand.setNorm(n.getStdVec());
	stand.setOriginalPointMap(original_point_map);

	return stand;
}

/**
 * Use the qHull convex hull algorithm to get the foot stand (not the prediction)
 */
FootStateUneven MyQHull::qHullCalculatePoints(std::vector<vec3> const &points, vec3 zmpv) {

	int dataWidth=10;
	int dataHeight=10;

	std::vector<float> zmpvec= {zmpv.X[0], zmpv.X[1]};
	std::vector<float> pointsFlat(dataWidth*dataHeight, 0.0f);

	std::map<int, std::vector<double>> original_point_map;

	// turn data to a flat vector
	for(int i = 0; i < points.size(); i++) {
		long int idx =  (points.at(i).IDX[0]*dataWidth) + points.at(i).IDX[1];
		double z = points.at(i).X[2];
		//z = (z-min)/(max-min);
		pointsFlat.at(idx) = z;
		vec3 originalPoint = vec3(points.at(i).X[0], points.at(i).X[1], points.at(i).X[2]);
		original_point_map[idx] = originalPoint.getStdVec();
	}

	Qhull qhull;
	qhull.runQhull3D(points, "Qt");

	// for the final z height of the ZMP, the ZMP on the facet (support polygon) and the normal of that polygon.
	double zmpFacetHeight = -DBL_MAX;
	vec3 facetNormal;
	//stand s;
	FootStateUneven stand = FootStateUneven();
	stand.setValid(-1);

    QhullFacetList facets = qhull.facetList();
    for (QhullFacetList::iterator it = facets.begin(); it != facets.end(); ++it)
    {
    	std::vector<vec3> triPoints;

        if (!(*it).isGood()) continue;
        QhullFacet f = *it;
        QhullVertexSet vSet = f.vertices();
        // shows eclipse error but works
        for (QhullVertexSet::iterator vIt = vSet.begin(); vIt != vSet.end(); ++vIt) // shows eclipse error but works
        {
            QhullVertex v = *vIt;
            QhullPoint p = v.point();
            double * coords = p.coordinates();
            vec3 aPoint = vec3(coords[0], coords[1], coords[2]);
            // ...Do what ever you want
            triPoints.push_back(aPoint);
            int madebugpoint=1;
        }

        bool facetContainsZmp = pointInTriangle(zmpv, triPoints.at(0), triPoints.at(1), triPoints.at(2));

        if(facetContainsZmp) {

        	if (f.hyperplane().isValid()) {
				auto coord = f.hyperplane().coordinates();
				double facetArea = f.facetArea();
				vec3 normal(coord[0], coord[1], coord[2]);
				vec3 otherNormal = getTriangleNormal(triPoints.at(0), triPoints.at(1), triPoints.at(2));
				//double offset = f.hyperplane().offset();
				//facetsNormals.push_back(std::pair<vec3, double>(normal, offset));
				//double n = f.hyperplane().norm();
	    		double A = coord[0]; double B = coord[1]; double C = coord[2];
	    		double D = -normal.dot(triPoints.at(0)); //-dot(normal,p1);
	    		double zmpHeight_onFacet = (A * zmpv.X[0] + B * zmpv.X[1] + D) / (-C); //(A.*x + B.*y + D)./(-C);

	    		// select the highest facet
	    		if(zmpHeight_onFacet > zmpFacetHeight) {
	    			zmpFacetHeight = zmpHeight_onFacet;
	    			facetNormal = normal;
	    			stand.setValid(1);
	    			stand.setNorm(facetNormal.getStdVec());
	    			stand.setP1(triPoints.at(0).getStdVec());
	    			stand.setP2(triPoints.at(1).getStdVec());
	    			stand.setP3(triPoints.at(2).getStdVec());
	    			stand.setFacetArea(facetArea);
	    			stand.setHeight(zmpFacetHeight);
	    		}
        	}
        }

    }


	stand.setOriginalPointMap(original_point_map);

    return stand;
}

bool MyQHull::pointInTriangle(vec3 p, vec3 p0, vec3 p1, vec3 p2) {
	    double A = (1.0/2.0) * (-p1.X[1] * p2.X[0] + p0.X[1] * (-p1.X[0] + p2.X[0]) + p0.X[0] * (p1.X[1] - p2.X[1]) + p1.X[0] * p2.X[1]);
	    double sign = A < 0 ? -1 : 1;
	    double s = (p0.X[1] * p2.X[0] - p0.X[0] * p2.X[1] + (p2.X[1] - p0.X[1]) * p.X[0] + (p0.X[0] - p2.X[0]) * p.X[1]) * sign;
	    double t = (p0.X[0] * p1.X[1] - p0.X[1] * p1.X[0] + (p0.X[1] - p1.X[1]) * p.X[0] + (p1.X[0] - p0.X[0]) * p.X[1]) * sign;
	    return s >= 0 && t >= 0 && (s + t) <= 2 * A * sign;
}


double MyQHull::triangle_area_3d(vec3 a, vec3 b, vec3 c) {
	// u = AB
	vec3 u = b - a;
	// v = AC
	vec3 v = c - a;

	// u x v
	vec3 w = u.cross(v);
	double area = 0.5*sqrt(w.X[0]*w.X[0] +  w.X[1]*w.X[1] +  w.X[2]*w.X[2]);

	return area;
}

vec3 MyQHull::getTriangleNormal(vec3 p1, vec3 p2, vec3 p3) {
	vec3 u = p2 - p1;
	vec3 v = p3 - p1;
	// normal = u x v
	return u.cross(v);
}
