/*
 * QhullExtended.h
 *
 *  Created on: May 8, 2017
 *      Author: jan
 */

#ifndef MULTI_CONTACT_POINT_ESTIMATOR_INCLUDE_MULTI_CONTACT_POINT_ESTIMATOR_UNEVEN_TERRAIN_STAND_Q_HULL_EXTENDED_H_
#define MULTI_CONTACT_POINT_ESTIMATOR_INCLUDE_MULTI_CONTACT_POINT_ESTIMATOR_UNEVEN_TERRAIN_STAND_Q_HULL_EXTENDED_H_

//#include "libqhullcpp/PointCoordinates.h"
//#include "/home/jan/catkin_ws/build/multi_contact_point_estimator/qhull/src/qhull/src/libqhullcpp/PointCoordinates.h"

#include "libqhullcpp/PointCoordinates.h"
#include "libqhullcpp/Qhull.h" // see the README_QHULL, QHULL needs to be installed
#include <vector>

namespace orgQhull {

// typedef and vec3
typedef double vtype;

/* Basic 3D vector implementation */
struct vec3 {
  vec3() { X[0] = X[1] = X[2] = 0; IDX[0] = IDX[1] = -1; }
  vec3(vtype x, vtype y, vtype z) { X[0] = x; X[1] = y; X[2] = z; IDX[0] = IDX[1] = -1; }

  /* 3D cross product */
  vec3 operator*(const vec3& v) const {
	return vec3(X[1] * v.X[2] - X[2] * v.X[1],
				X[2] * v.X[0] - X[0] * v.X[2],
				X[0] * v.X[1] - X[1] * v.X[0]);
  }

  vec3 operator-(const vec3& v) const {
	return vec3(X[0] - v.X[0], X[1] - v.X[1], X[2] - v.X[2]);
  }

  vec3 operator-() const {
	return vec3(-X[0], -X[1], -X[2]);
  }

  vec3 div(double d) const {
	return vec3(X[0]/(double)d, X[1]/(double)d, X[2]/(double)d);
  }

  vtype dot(const vec3& v) const {
	return X[0] * v.X[0] + X[1] * v.X[1] + X[2] * v.X[2];
  }

  vec3 cross(const vec3& v) const {
	double px = X[1] * v.X[2] - X[2] * v.X[1];
	double py = X[2] * v.X[0] - X[0] * v.X[2];
	double pz = X[0] * v.X[1] - X[1] * v.X[0];
	vec3 crs(px, py, pz);
	return crs;
  }

  std::vector<double> getStdVec() const {
	  std::vector<double> v = {X[0], X[1], X[2]};
	  return v;
  }

  vtype X[3];
  int IDX[2];
};


/*
 * This class extends QHULL and adds methods to run Qhull3D over the C++ interface.
 */
class QhullExtended : public Qhull {
public:
	QhullExtended();
	virtual ~QhullExtended();

    void runQhull3D(const std::vector<vec3> &points, const char* args);
    void runQhullExtPoints(const PointCoordinates &points, const char *qhullCommand2);
private:
	PointCoordinates *m_externalPoints;
};

} /* namespace orgQhull */

#endif /* MULTI_CONTACT_POINT_ESTIMATOR_INCLUDE_MULTI_CONTACT_POINT_ESTIMATOR_UNEVEN_TERRAIN_STAND_Q_HULL_EXTENDED_H_ */
