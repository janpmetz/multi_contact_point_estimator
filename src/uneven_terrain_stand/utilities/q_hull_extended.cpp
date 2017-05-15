/*
 * QhullExtended.cpp
 *
 *  Created on: May 8, 2017
 *      Author: jan
 */

#include <multi_contact_point_estimator/uneven_terrain_stand/utilities/q_hull_extended.h>
#include <vector>


namespace orgQhull {

QhullExtended::QhullExtended() {
	// TODO Auto-generated constructor stub

}

QhullExtended::~QhullExtended() {
	// TODO Auto-generated destructor stub
}

void QhullExtended::runQhull3D(const std::vector<vec3> &points, const char* args)
{
	int dimension = 3;
	std::string describe_points = "points 3D";

	m_externalPoints = new PointCoordinates(dimension, describe_points);
    std::vector<double> allPoints;

    for(vec3 p : points) {
            allPoints.push_back(p.X[0]);
            allPoints.push_back(p.X[1]);
            allPoints.push_back(p.X[2]);
    }

    m_externalPoints->append(allPoints); //convert to vector<double>
    runQhullExtPoints(*m_externalPoints, args);
}

void QhullExtended::runQhullExtPoints(const PointCoordinates &points, const char *qhullCommand2)
{
    runQhull(points.comment().c_str(), points.dimension(), points.count(), &*points.coordinates(), qhullCommand2);
}


} /* namespace orgQhull */
