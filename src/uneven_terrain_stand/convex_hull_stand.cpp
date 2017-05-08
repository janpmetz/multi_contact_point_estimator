/*
 * ConvexHullStand.cpp
 *
 *  Created on: May 7, 2017
 *      Author: jan
 */

#include <multi_contact_point_estimator/uneven_terrain_stand/convex_hull_stand.h>
#include <vector>
#include <math.h>
#include <map>

#include <libqhullcpp/QhullFacetList.h>
#include <visualization_msgs/Marker.h>
#include <vigir_footstep_planning_lib/math.h>

#include <multi_contact_point_estimator/uneven_terrain_stand/src/libqhullcpp/Qhull.h>
#include <multi_contact_point_estimator/uneven_terrain_stand/src/libqhullcpp/QhullVertexSet.h>

ConvexHullStand::ConvexHullStand() {
	// TODO Auto-generated constructor stub

}

ConvexHullStand::~ConvexHullStand() {
	// TODO Auto-generated destructor stub
}
/**
 * Use the qHull convex hull algorithm to get the foot stand
 */
FootStateUneven ConvexHullStand::getStand(std::vector<vec3> const &points, vec3 zmpv) {

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

bool ConvexHullStand::pointInTriangle(vec3 p, vec3 p0, vec3 p1, vec3 p2) {
	    double A = (1.0/2.0) * (-p1.X[1] * p2.X[0] + p0.X[1] * (-p1.X[0] + p2.X[0]) + p0.X[0] * (p1.X[1] - p2.X[1]) + p1.X[0] * p2.X[1]);
	    double sign = A < 0 ? -1 : 1;
	    double s = (p0.X[1] * p2.X[0] - p0.X[0] * p2.X[1] + (p2.X[1] - p0.X[1]) * p.X[0] + (p0.X[0] - p2.X[0]) * p.X[1]) * sign;
	    double t = (p0.X[0] * p1.X[1] - p0.X[1] * p1.X[0] + (p0.X[1] - p1.X[1]) * p.X[0] + (p1.X[0] - p0.X[0]) * p.X[1]) * sign;
	    return s >= 0 && t >= 0 && (s + t) <= 2 * A * sign;
}

vec3 ConvexHullStand::getTriangleNormal(vec3 p1, vec3 p2, vec3 p3) {
	vec3 u = p2 - p1;
	vec3 v = p3 - p1;
	// normal = u x v
	return u.cross(v);
}
