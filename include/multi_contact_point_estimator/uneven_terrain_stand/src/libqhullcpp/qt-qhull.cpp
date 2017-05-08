/****************************************************************************
**
** Copyright (c) 2009-2015 C.B. Barber. All rights reserved.
** $Id: //main/2015/qhull/src/qt-qhull.cpp#1 $$Change: 1981 $
** $DateTime: 2015/09/28 20:26:32 $$Author: bbarber $
**
****************************************************************************/

#include <QList>

#include "../../../../multi_contact_point_estimator/uneven_terrain_stand/src/qhulltest/RoadTest.h"

#ifndef QHULL_USES_QT
#define QHULL_USES_QT 1
#endif

#include "../../../../multi_contact_point_estimator/uneven_terrain_stand/src/libqhullcpp/Coordinates.h"
#include "../../../../multi_contact_point_estimator/uneven_terrain_stand/src/libqhullcpp/QhullFacetList.h"
#include "../../../../multi_contact_point_estimator/uneven_terrain_stand/src/libqhullcpp/QhullFacetSet.h"
#include "../../../../multi_contact_point_estimator/uneven_terrain_stand/src/libqhullcpp/QhullHyperplane.h"
#include "../../../../multi_contact_point_estimator/uneven_terrain_stand/src/libqhullcpp/QhullPoint.h"
#include "../../../../multi_contact_point_estimator/uneven_terrain_stand/src/libqhullcpp/QhullPoints.h"
#include "../../../../multi_contact_point_estimator/uneven_terrain_stand/src/libqhullcpp/QhullPointSet.h"
#include "../../../../multi_contact_point_estimator/uneven_terrain_stand/src/libqhullcpp/QhullVertex.h"
#include "../../../../multi_contact_point_estimator/uneven_terrain_stand/src/libqhullcpp/QhullVertexSet.h"

namespace orgQhull {

#//!\name Conversions

QList<coordT> Coordinates::
toQList() const
{
    CoordinatesIterator i(*this);
    QList<coordT> cs;
    while(i.hasNext()){
        cs.append(i.next());
    }
    return cs;
}//toQList

QList<QhullFacet> QhullFacetList::
toQList() const
{
    QhullLinkedListIterator<QhullFacet> i(*this);
    QList<QhullFacet> vs;
    while(i.hasNext()){
        QhullFacet f= i.next();
        if(isSelectAll() || f.isGood()){
            vs.append(f);
        }
    }
    return vs;
}//toQList

//! Same as PrintVertices
QList<QhullVertex> QhullFacetList::
vertices_toQList() const
{
    QList<QhullVertex> vs;
    QhullVertexSet qvs(qh(), first().getFacetT(), NULL, isSelectAll());
    for(QhullVertexSet::iterator i=qvs.begin(); i!=qvs.end(); ++i){
        vs.push_back(*i);
    }
    return vs;
}//vertices_toQList

QList<QhullFacet> QhullFacetSet::
toQList() const
{
    QhullSetIterator<QhullFacet> i(*this);
    QList<QhullFacet> vs;
    while(i.hasNext()){
        QhullFacet f= i.next();
        if(isSelectAll() || f.isGood()){
            vs.append(f);
        }
    }
    return vs;
}//toQList

#ifdef QHULL_USES_QT
QList<coordT> QhullHyperplane::
toQList() const
{
    QhullHyperplaneIterator i(*this);
    QList<coordT> fs;
    while(i.hasNext()){
        fs.append(i.next());
    }
    fs.append(hyperplane_offset);
    return fs;
}//toQList
#endif //QHULL_USES_QT

QList<coordT> QhullPoint::
toQList() const
{
    QhullPointIterator i(*this);
    QList<coordT> vs;
    while(i.hasNext()){
        vs.append(i.next());
    }
    return vs;
}//toQList

QList<QhullPoint> QhullPoints::
toQList() const
{
    QhullPointsIterator i(*this);
    QList<QhullPoint> vs;
    while(i.hasNext()){
        vs.append(i.next());
    }
    return vs;
}//toQList

/******
QList<QhullPoint> QhullPointSet::
toQList() const
{
    QhullPointSetIterator i(*this);
    QList<QhullPoint> vs;
    while(i.hasNext()){
        vs.append(i.next());
    }
    return vs;
}//toQList
*/
}//orgQhull

