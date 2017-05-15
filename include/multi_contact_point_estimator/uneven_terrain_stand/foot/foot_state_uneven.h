#ifndef VIGIR_FOOTSTEP_PLANNING_CORE_VIGIR_FOOTSTEP_PLANNING_DEFAULT_PLUGINS_SRC_UNEVEN_TERRAIN_STAND_FOOT_STATE_UNEVEN_H_
#define VIGIR_FOOTSTEP_PLANNING_CORE_VIGIR_FOOTSTEP_PLANNING_DEFAULT_PLUGINS_SRC_UNEVEN_TERRAIN_STAND_FOOT_STATE_UNEVEN_H_

#include <unistd.h>
#include <vector>
#include <map>
#include <boost/serialization/access.hpp>
#include <boost/serialization/vector.hpp>
#include <vigir_footstep_planning_msgs/Step.h>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>


struct FootStateStruct {
	FootStateStruct(){};

	int valid_uneven;
	std::vector<double> p1;	// support polygon
	std::vector<double> p2;	// support polygon
	std::vector<double> p3;	// support polygon
	std::vector<double> normal;
	std::vector<int> original_point_map_keys;	// points under the foot sole
	std::vector<double> original_point_map_x;
	std::vector<double> original_point_map_y;
	std::vector<double> original_point_map_z;

	friend class boost::serialization::access;
		template<class Archive>
		void serialize(Archive & ar, const unsigned int version)
		{
			// Simply list all the fields to be serialized/deserialized.
			ar & valid_uneven;
			ar & normal;
			ar & p1;	// support polygon
			ar & p2;
			ar & p3;
			ar & original_point_map_keys;
			ar & original_point_map_x;
			ar & original_point_map_y;
			ar & original_point_map_z;
		};

		// additional data in "Step" can be stored as std::vector<uint8_t>
		// First serialize the struct to a string and then store it there
		static void serialize_step_data(FootStateStruct& footStep, vigir_footstep_planning_msgs::Step& step) {
			std::ostringstream archive_stream;
			boost::archive::text_oarchive archive(archive_stream);
			archive << footStep;
			std::string str = archive_stream.str();
			std::vector<uint8_t> data_vec(str.begin(), str.end());
			step.data = data_vec;
		}
		// First get the data as a string and then deserialize it to the struct
		static void deserialize_step_data(vigir_footstep_planning_msgs::Step& step, FootStateStruct& footStep) {
			std::vector<uint8_t> vec = step.data;
			std::string data_str(vec.begin(), vec.end());
		    std::istringstream iss(data_str);
		    {
		        boost::archive::text_iarchive ia(iss);
		        ia >> footStep;
		    }
		}
};



class FootStateUneven {
public:

	FootStateUneven(){
		facetArea = 0.0;
		height = 0.0;
		support = 0.0;
		valid = 0;
	};

	virtual ~FootStateUneven(){};

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

	FootStateStruct getFootStateStruct() {
		FootStateStruct step;
	    step.valid_uneven = getValid();
	    step.p1 = getP1();
	    step.p2 = getP2();
	    step.p3 = getP3();
	    step.normal = getNormal();
	    step.original_point_map_keys = getMapKeys();
	    step.original_point_map_x = getValueSet(0);
	    step.original_point_map_y = getValueSet(1);
	    step.original_point_map_z = getValueSet(2);
	    return step;
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
