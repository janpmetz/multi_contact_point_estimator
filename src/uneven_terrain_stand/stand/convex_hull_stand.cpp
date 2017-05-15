/*
 * ConvexHullStand.cpp
 *
 *  Created on: May 7, 2017
 *      Author: jan
 */

#include <iostream>
#include <fstream>

#include <vector>
#include <math.h>
#include <multi_contact_point_estimator/uneven_terrain_stand/stand/convex_hull_stand.h>
#include <map>

#include <vigir_footstep_planning_lib/math.h>

#include "libqhullcpp/QhullFacet.h"
#include "libqhullcpp/QhullFacetList.h"
#include "libqhullcpp/QhullVertex.h"
#include "libqhullcpp/QhullPoint.h"
#include "libqhullcpp/QhullVertexSet.h"

#include <ros/ros.h>
#include <boost/lexical_cast.hpp>
#include <boost/thread.hpp>

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

	FootStateUneven stand;

	int dataWidth=10;
	int dataHeight=10;

	// initializations
	std::vector<double> zmpvec= {zmpv.X[0], zmpv.X[1]};
	std::vector<double> pointsFlat(dataWidth*dataHeight, 0.0f);	// feed only a vector to QHull
	std::map<int, std::vector<double>> original_point_map;
	flattenPoints(points, dataWidth, pointsFlat, original_point_map);	// flatten e.g. a 10x10 matrix -> vector of length 10*10


	// CALCULATE 3D CONVEX HULL
	// QHull inheritance: make QHull 3D! accessible in C++
	QhullExtended qhull;
	qhull.runQhull3D(points, "Qt");

	// Now get the one facet that is the support polygon:
	double A, B, C, D;	//plane coefficients
	double zmpFacetHeight = -DBL_MAX;	// height of the x,y coords of the ZMP (or COP) is a point on the support polygon
	stand.setValid(-1);

	// Check all facets of the convex hull
    QhullFacetList facets = qhull.facetList();
    for (QhullFacetList::iterator it = facets.begin(); it != facets.end(); ++it)
    {

    	QhullFacet f = *it;
        if (!f.isGood()) continue;

        // get edge points of the facet (is a triangle)
    	std::vector<vec3> edgePoints;
		QhullVertexSet vSet = getFacetEdgePoints(f, edgePoints);
        
		bool facetContainsZmp = pointInTriangle(zmpv, edgePoints.at(0), edgePoints.at(1), edgePoints.at(2));
        bool hyperPlaneValid = f.hyperplane().isValid();

        if(facetContainsZmp && hyperPlaneValid) {

        		// get height of the cop on support polygon
        		double* coord = f.hyperplane().coordinates();
	    		double zmpHeight = pointHeightOnPlane(zmpv, edgePoints, coord);

	    		// store everything for the highest facet (bottom facet VS top facet in 3D)
	    		if(zmpHeight > zmpFacetHeight) {
	    			zmpFacetHeight = zmpHeight;
					vec3 facetNormal(coord[0], coord[1], coord[2]);
	    			stand.setNorm(facetNormal.getStdVec());
	    			stand.setP1(edgePoints.at(0).getStdVec());
	    			stand.setP2(edgePoints.at(1).getStdVec());
	    			stand.setP3(edgePoints.at(2).getStdVec());
	    			stand.setFacetArea(f.facetArea());
	    			stand.setHeight(zmpFacetHeight);
	    			stand.setValid(1);
	    		}

        }
    }


    stand.setOriginalPointMap(original_point_map); // useful for e.g. later visualizations

    return stand;
}

/**
 * Some helper functions
 */

void ConvexHullStand::flattenPoints(const std::vector<vec3>& points, int dataWidth,
		std::vector<double>& pointsFlat,
		std::map<int, std::vector<double> >& original_point_map) {
	// turn data to a flat vector
	for (int i = 0; i < points.size(); i++) {
		long int idx = (points.at(i).IDX[0] * dataWidth) + points.at(i).IDX[1];
		double z = points.at(i).X[2];
		pointsFlat.at(idx) = z;
		vec3 originalPoint = vec3(points.at(i).X[0], points.at(i).X[1],
				points.at(i).X[2]);
		original_point_map[idx] = originalPoint.getStdVec();
	}
}

QhullVertexSet ConvexHullStand::getFacetEdgePoints(const QhullFacet& f,
		std::vector<vec3>& triPoints) {
	QhullVertexSet vSet = f.vertices();
	for (QhullVertexSet::iterator vIt = vSet.begin(); vIt != vSet.end();
			++vIt) {
		// might show error in eclipse but works
		QhullVertex v = *vIt;
		QhullPoint p = v.point();
		double* coords = p.coordinates();
		vec3 aPoint = vec3(coords[0], coords[1], coords[2]);
		triPoints.push_back(aPoint);
	}
	return vSet;
}

double ConvexHullStand::pointHeightOnPlane(const vec3& point, std::vector<vec3> edgePoints,	double* coord) {
	double A = coord[0];
	double B = coord[1];
	double C = coord[2];
	vec3 normal(coord[0], coord[1], coord[2]);
	double D = -normal.dot(edgePoints.at(0)); //-dot(normal,p1);
	double pointHeight = (A * point.X[0] + B * point.X[1] + D) / (-C); //(A.*x + B.*y + D)./(-C);
	return pointHeight;
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
