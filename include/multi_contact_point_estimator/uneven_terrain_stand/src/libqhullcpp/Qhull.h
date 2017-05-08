/****************************************************************************
**
** Copyright (c) 2008-2015 C.B. Barber. All rights reserved.
** $Id: //main/2015/qhull/src/libqhullcpp/Qhull.h#3 $$Change: 2066 $
** $DateTime: 2016/01/18 19:29:17 $$Author: bbarber $
**
****************************************************************************/

#ifndef QHULLCPP_H
#define QHULLCPP_H

#include "../../../../multi_contact_point_estimator/uneven_terrain_stand/src/libqhullcpp/PointCoordinates.h"
#include "../../../../multi_contact_point_estimator/uneven_terrain_stand/src/libqhullcpp/QhullFacet.h"
#include "../../../../multi_contact_point_estimator/uneven_terrain_stand/src/libqhullcpp/QhullPoint.h"
#include "../../../../multi_contact_point_estimator/uneven_terrain_stand/src/libqhullcpp/QhullVertex.h"
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

	  /*vec3 operator/(double d) const {
		return vec3(X[0]/(double)d, X[1]/(double)d, X[2]/(double)d);
	  }
	  */

	  vec3 div(double d) const {
		return vec3(X[0]/(double)d, X[1]/(double)d, X[2]/(double)d);
	  }

	  vtype dot(const vec3& v) const {
		return X[0] * v.X[0] + X[1] * v.X[1] + X[2] * v.X[2];
	  }

	  /*
	  vec3 cross(const vec3& u, const vec3& v) const {
		double px = u[1] * v[2] - u[2] * v[1];
		double py = u[2] * v[0] - u[0] * v[2];
		double pz = u[0] * v[1] - u[1] * v[0];
		vec3 crs(px, py, pz);
		return crs;
	  }
	   */
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




/***
   Compile qhullcpp and libqhull with the same compiler.  setjmp() and longjmp() must be the same.

   #define QHULL_NO_STL
      Do not supply conversions to STL
      Coordinates.h requires <vector>.  It could be rewritten for another vector class such as QList
   #define QHULL_USES_QT
      Supply conversions to QT
      qhulltest requires QT.  It is defined in RoadTest.h

  #define QHULL_ASSERT
      Defined by QhullError.h
      It invokes assert()
*/

#//!\name Used here
    class QhullFacetList;
    class QhullPoints;
    class QhullQh;
    class RboxPoints;

#//!\name Defined here
    class Qhull;

//! Interface to Qhull from C++
class Qhull {


private:
#//!\name Members and friends
    QhullQh *           qh_qh;          //! qhT for this instance
    Coordinates         origin_point;   //! origin for qh_qh->hull_dim.  Set by runQhull()
    bool                run_called;     //! True at start of runQhull.  Errors if call again.
    Coordinates         feasible_point;  //! feasible point for half-space intersection (alternative to qh.feasible_string for qh.feasible_point)

    PointCoordinates *m_externalPoints; // added by me

public:
#//!\name Constructors
                        Qhull();      //!< call runQhull() next
                        Qhull(const RboxPoints &rboxPoints, const char *qhullCommand2);
                        Qhull(const char *inputComment2, int pointDimension, int pointCount, const realT *pointCoordinates, const char *qhullCommand2);
                        ~Qhull() throw();
private:                //! Disable copy constructor and assignment.  Qhull owns QhullQh.
                        Qhull(const Qhull &);
    Qhull &             operator=(const Qhull &);

private:
    void                allocateQhullQh();

public:

#//!\name GetSet
    void                checkIfQhullInitialized();
    int                 dimension() const { return qh_qh->input_dim; } //!< Dimension of input and result
    void                disableOutputStream() { qh_qh->disableOutputStream(); }
    void                enableOutputStream() { qh_qh->enableOutputStream(); }
    countT              facetCount() const { return qh_qh->num_facets; }
    Coordinates         feasiblePoint() const; 
    int                 hullDimension() const { return qh_qh->hull_dim; } //!< Dimension of the computed hull
    bool                hasOutputStream() const { return qh_qh->hasOutputStream(); }
    bool                initialized() const { return (qh_qh->hull_dim>0); }
    const char *        inputComment() const { return qh_qh->rbox_command; }
    QhullPoint          inputOrigin();
                        //! non-const due to QhullPoint
    QhullPoint          origin() { QHULL_ASSERT(initialized()); return QhullPoint(qh_qh, origin_point.data()); }
    QhullQh *           qh() const { return qh_qh; };
    const char *        qhullCommand() const { return qh_qh->qhull_command; }
    const char *        rboxCommand() const { return qh_qh->rbox_command; }
    int                 rotateRandom() const { return qh_qh->ROTATErandom; } //!< Return QRn for repeating QR0 runs
    void                setFeasiblePoint(const Coordinates &c) { feasible_point= c; } //!< Sets qh.feasible_point via initializeFeasiblePoint
    countT              vertexCount() const { return qh_qh->num_vertices; }

#//!\name Delegated to QhullQh
    double              angleEpsilon() const { return qh_qh->angleEpsilon(); } //!< Epsilon for hyperplane angle equality
    void                appendQhullMessage(const std::string &s) { qh_qh->appendQhullMessage(s); }
    void                clearQhullMessage() { qh_qh->clearQhullMessage(); }
    double              distanceEpsilon() const { return qh_qh->distanceEpsilon(); } //!< Epsilon for distance to hyperplane
    double              factorEpsilon() const { return qh_qh->factorEpsilon(); }  //!< Factor for angleEpsilon and distanceEpsilon
    std::string         qhullMessage() const { return qh_qh->qhullMessage(); }
    bool                hasQhullMessage() const { return qh_qh->hasQhullMessage(); }
    int                 qhullStatus() const { return qh_qh->qhullStatus(); }
    void                setErrorStream(std::ostream *os) { qh_qh->setErrorStream(os); }
    void                setFactorEpsilon(double a) { qh_qh->setFactorEpsilon(a); }
    void                setOutputStream(std::ostream *os) { qh_qh->setOutputStream(os); }

#//!\name ForEach
    QhullFacet          beginFacet() const { return QhullFacet(qh_qh, qh_qh->facet_list); }
    QhullVertex         beginVertex() const { return QhullVertex(qh_qh, qh_qh->vertex_list); }
    void                defineVertexNeighborFacets(); //!< Automatically called if merging facets or Voronoi diagram
    QhullFacet          endFacet() const { return QhullFacet(qh_qh, qh_qh->facet_tail); }
    QhullVertex         endVertex() const { return QhullVertex(qh_qh, qh_qh->vertex_tail); }
    QhullFacetList      facetList() const;
    QhullFacet          firstFacet() const { return beginFacet(); }
    QhullVertex         firstVertex() const { return beginVertex(); }
    QhullPoints         points() const;
    QhullPointSet       otherPoints() const;
                        //! Same as points().coordinates()
    coordT *            pointCoordinateBegin() const { return qh_qh->first_point; }
    coordT *            pointCoordinateEnd() const { return qh_qh->first_point + qh_qh->num_points*qh_qh->hull_dim; }
    QhullVertexList     vertexList() const;

#//!\name Methods
    double              area();
    void                outputQhull();
    void                outputQhull(const char * outputflags);
    void                runQhull(const RboxPoints &rboxPoints, const char *qhullCommand2);
    void                runQhull(const char *inputComment2, int pointDimension, int pointCount, const realT *pointCoordinates, const char *qhullCommand2);
    double              volume();

    void runQhull3D(const std::vector<vec3> &points, const char* args); // added by me
    void runQhull(const PointCoordinates &points, const char *qhullCommand2); // added by me

#//!\name Helpers
private:
    void                initializeFeasiblePoint(int hulldim);
};//Qhull

}//namespace orgQhull

#endif // QHULLCPP_H
