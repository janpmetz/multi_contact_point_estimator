#ifndef DATAPOINTS_H_
#define DATAPOINTS_H_

#include <vector>

#include "src/libqhullcpp/Qhull.h"


class DataPoints {
public:
	DataPoints(std::vector<orgQhull::vec3>);
	virtual ~DataPoints();
	std::vector<orgQhull::vec3> getPoints();

private:
	std::vector<orgQhull::vec3> points;
};

#endif /* DATAPOINTS_H_ */
