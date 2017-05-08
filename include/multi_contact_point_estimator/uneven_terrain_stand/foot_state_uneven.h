#ifndef VIGIR_FOOTSTEP_PLANNING_CORE_VIGIR_FOOTSTEP_PLANNING_DEFAULT_PLUGINS_SRC_UNEVEN_TERRAIN_STAND_FOOT_STATE_UNEVEN_H_
#define VIGIR_FOOTSTEP_PLANNING_CORE_VIGIR_FOOTSTEP_PLANNING_DEFAULT_PLUGINS_SRC_UNEVEN_TERRAIN_STAND_FOOT_STATE_UNEVEN_H_

#include <vector>
#include <map>

class FootStateUneven {
public:
	FootStateUneven(){};
	//FootStateUneven(int valid, std::vector<double> angles, std::vector<double> norm, std::vector<double> p1, std::vector<double> p2, std::vector<double> p3, std::vector<double> zmp, double facetArea, double support, double height, std::map<int, std::vector<double>> original_point_map);
	//virtual ~FootStateUneven();

	const std::vector<double>& getAngles() const {
		return angles;
	}

	void setAngles(const std::vector<double>& angles) {
		this->angles = angles;
	}

	double getFacetArea() const {
		return facetArea;
	}

	void setFacetArea(double facetArea) {
		this->facetArea = facetArea;
	}

	double getHeight() const {
		return height;
	}

	void setHeight(double height) {
		this->height = height;
	}



	const std::vector<double>& getNormal() const {
		return norm;
	}

	void setNorm(const std::vector<double>& norm) {
		this->norm = norm;
	}

	const std::vector<double>& getP1() const {
		return p1;
	}

	void setP1(const std::vector<double>& p1) {
		this->p1 = p1;
	}

	const std::vector<double>& getP2() const {
		return p2;
	}

	void setP2(const std::vector<double>& p2) {
		this->p2 = p2;
	}

	const std::vector<double>& getP3() const {
		return p3;
	}

	void setP3(const std::vector<double>& p3) {
		this->p3 = p3;
	}

	double getSupport() const {
		return support;
	}

	void setSupport(double support) {
		this->support = support;
	}

	int getValid() const {
		return valid;
	}

	void setValid(int valid) {
		this->valid = valid;
	}

	const std::vector<double>& getZmp() const {
		return zmp;
	}

	void setZmp(const std::vector<double>& zmp) {
		this->zmp = zmp;
	}

	const std::map<int, std::vector<double> >& getOriginalPointMap() const {
		return original_point_map;
	}

	void setOriginalPointMap(
			const std::map<int, std::vector<double> >& originalPointMap) {
		original_point_map = originalPointMap;
	}

	std::vector<int> getMapKeys() {
		std::vector<int> keyList;
		for(std::map<int,std::vector<double> >::iterator iter = original_point_map.begin(); iter != original_point_map.end(); ++iter)
		{
			int k =  iter->first;
			//ignore value
			//Value v = iter->second;
			keyList.push_back(k);
		}
		return keyList;
	}

	std::vector<double> getValueSet(int idx) {
		std::vector<double> valueList;
		for(std::map<int,std::vector<double> >::iterator iter = original_point_map.begin(); iter != original_point_map.end(); ++iter)
		{
			std::vector<double> k =  iter->second;
			//ignore value
			//Value v = iter->second;
			valueList.push_back(k[idx]);
		}
		return valueList;
	}

private:
	int valid;
	std::vector<double> angles;
	std::vector<double> norm;
	std::vector<double> p1;	// support polygon
	std::vector<double> p2;
	std::vector<double> p3;
	std::vector<double> zmp;
	double facetArea;
	double support;
	double height;
	std::map<int, std::vector<double> > original_point_map;
};



#endif /* VIGIR_FOOTSTEP_PLANNING_CORE_VIGIR_FOOTSTEP_PLANNING_DEFAULT_PLUGINS_SRC_UNEVEN_TERRAIN_STAND_FOOT_STATE_UNEVEN_H_ */
