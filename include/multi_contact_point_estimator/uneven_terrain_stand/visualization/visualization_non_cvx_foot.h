/*
 * VisualizationNonCvxFoot.h
 *
 *  Created on: May 14, 2017
 *      Author: jan
 */

#ifndef MULTI_CONTACT_POINT_ESTIMATOR_INCLUDE_MULTI_CONTACT_POINT_ESTIMATOR_UNEVEN_TERRAIN_STAND_VISUALIZATION_VISUALIZATION_NON_CVX_FOOT_H_
#define MULTI_CONTACT_POINT_ESTIMATOR_INCLUDE_MULTI_CONTACT_POINT_ESTIMATOR_UNEVEN_TERRAIN_STAND_VISUALIZATION_VISUALIZATION_NON_CVX_FOOT_H_
#include <multi_contact_point_estimator/uneven_terrain_stand/terrain_model_uneven.h>
#include <vigir_footstep_planning_msgs/Step.h>

#include <boost/archive/binary_oarchive.hpp>
#include "boost/archive/binary_iarchive.hpp"
#include "boost/archive/binary_oarchive.hpp"
#include "boost/iostreams/device/array.hpp"
#include "boost/iostreams/device/back_inserter.hpp"
#include "boost/iostreams/stream_buffer.hpp"
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <multi_contact_point_estimator/uneven_terrain_stand/foot/foot_state_uneven.h>

/**
 * Methods to visualize the step plan with some markers for non-convex foot shapes.
 */

using vigir_footstep_planning_msgs::Step;

namespace vigir_footstep_planning
{
namespace msgs
{
void publishStepPlanUneven(ros::Publisher& pub, const msgs::StepPlan& step_plan, const geometry_msgs::Vector3& foot_size, visualization_msgs::MarkerArray& last_step_plan_vis, bool add_step_index);
void stepPlanToFootMarkerArrayNonConvex(const msgs::StepPlan& step_plan, const geometry_msgs::Vector3& foot_size, visualization_msgs::MarkerArray& marker_array, bool add_step_index);
void nonConvexFootMarkers(const FootStateStruct& footState, Step& original_step, const geometry_msgs::Vector3& foot_size, const std_msgs::ColorRGBA& color, visualization_msgs::Marker& marker, visualization_msgs::MarkerArray& marker_array);


void publishStepPlanUneven(ros::Publisher& pub, const msgs::StepPlan& step_plan, const geometry_msgs::Vector3& foot_size, visualization_msgs::MarkerArray& last_step_plan_vis, bool add_step_index)
{
  visualization_msgs::MarkerArray step_plan_vis;
  stepPlanToFootMarkerArrayNonConvex(step_plan, foot_size, step_plan_vis, add_step_index);

  // resize marker array
  if (last_step_plan_vis.markers.size() < step_plan_vis.markers.size())
    last_step_plan_vis.markers.resize(step_plan_vis.markers.size());

  // overwrite old markers
  for (size_t i = 0; i < step_plan_vis.markers.size(); i++)
    last_step_plan_vis.markers[i] = step_plan_vis.markers[i];

  // set needless markers to be deleted
  for (size_t i = step_plan_vis.markers.size(); i < last_step_plan_vis.markers.size(); i++)
    last_step_plan_vis.markers[i].action = visualization_msgs::Marker::DELETE;

  // finally publish new markers
  pub.publish(last_step_plan_vis);

  // delete old markers from array
  last_step_plan_vis.markers.resize(step_plan_vis.markers.size());
}


void stepPlanToFootMarkerArrayNonConvex(const msgs::StepPlan& step_plan, const geometry_msgs::Vector3& foot_size, visualization_msgs::MarkerArray& marker_array, bool add_step_index)
{
  std_msgs::ColorRGBA color;
  color.r = 0.0;
  color.g = 0.0;
  color.b = 0.0;
  color.a = 0.6;

  visualization_msgs::Marker marker;

  marker_array.markers.clear();

  std::vector<Step> steps = step_plan.steps;

  for (std::vector<Step>::const_iterator itr = steps.begin(); itr != steps.end(); itr++)
  {
    // colorize
    color.r = itr->foot.foot_index == Foot::LEFT ? 1.0 : 0.0;
    color.g = itr->foot.foot_index == Foot::LEFT ? 0.0 : 1.0;

    /*de-serialize the step data to the footStateStruct (TODO make a method)
    Step step = (*itr);
	std::vector<uint8_t> vec = step.data;
	std::string data_str(vec.begin(), vec.end());
	FootStateStruct footStep;
    std::istringstream iss(data_str);
    {
        boost::archive::text_iarchive ia(iss);
        ia >> footStep;
    }*/

	FootStateStruct footStep;
	Step step = (*itr);
	FootStateStruct::deserialize_step_data(step, footStep);


    // visualize the steps
    if(footStep.valid_uneven) {
    	nonConvexFootMarkers(footStep, step, foot_size, color, marker, marker_array);
		marker.id++;
    } else {
    	stepToFootMarker(*itr, foot_size, color, marker);
		marker_array.markers.push_back(marker);
		marker.id++;
    }

  }
}

// Assemble all the markers to present a foot step with contact points, normal, etc
void nonConvexFootMarkers(const FootStateStruct& step, Step& original_step, const geometry_msgs::Vector3& foot_size, const std_msgs::ColorRGBA& color, visualization_msgs::Marker& marker, visualization_msgs::MarkerArray& marker_array)
{

	if(step.valid_uneven != 1) {
		return;
	}

	std::vector<double> p1 = step.p1;
	std::vector<double> p2 = step.p2;
	std::vector<double> p3 = step.p3;
	std::vector<double> normal = step.normal;

	//ROS_INFO("VECTOR SIZE: %d ", p1.size());
	if(p1.size() == 0){
		return;
	}

	// assemble the point map again
	std::map<int, std::vector<double>> original_point_map;
	for(int i = 0; i < step.original_point_map_keys.size(); i++) {
		std::vector<double> v = {step.original_point_map_x[i], step.original_point_map_y[i], step.original_point_map_z[i]};
		original_point_map[step.original_point_map_keys[i]] = v;
	}

	//axis-angle rotation
	tf::Vector3 axis(normal[0],normal[1],normal[2]);
	tf::Vector3 marker_axis(1, 0, 0);
	tf::Quaternion qt(marker_axis.cross(axis.normalize()), marker_axis.angle(axis.normalize()));
	geometry_msgs::Quaternion quat_msg;
	tf::quaternionTFToMsg(qt, quat_msg);


	// 3 contact points
	visualization_msgs::Marker marker1;
	marker1.id = marker.id;
	marker1.header = original_step.foot.header;
	//marker.header.stamp = ros::Time();
	marker1.type = visualization_msgs::Marker::POINTS;
	marker1.action = visualization_msgs::Marker::ADD;
	marker1.scale.x = 0.02;
	marker1.scale.y = 0.02;
	marker1.scale.z = 0.02;
	marker1.color.a = 1.0;
	marker1.color.r = 0.0;
	marker1.color.g = 1.0;
	marker1.color.b = 0.0;
	marker1.ns = "vigir_footstep_planning/step_plan";

	geometry_msgs::Point p_1;
	geometry_msgs::Point p_2;
	geometry_msgs::Point p_3;

	p_1.x = p1[0];
	p_1.y = p1[1];
	p_1.z = p1[2];

	p_2.x = p2[0];
	p_2.y = p2[1];
	p_2.z = p2[2];

	p_3.x = p3[0];
	p_3.y = p3[1];
	p_3.z = p3[2];

	std::vector<geometry_msgs::Point> my_points;
	my_points.push_back(p_1);
	my_points.push_back(p_2);
	my_points.push_back(p_3);

	for (int ii = 0; ii < my_points.size(); ++ii)
	{
		std_msgs::ColorRGBA c;
			c.r = 1.0;
		c.a = 1;

		marker1.points.push_back(my_points[ii]);
		// Here, the field colors is populated with a specific color per point.
		marker1.colors.push_back(c);
	}

	marker_array.markers.push_back(marker1);
	marker.id++;

	// 3 contact points TRIANGLE
	visualization_msgs::Marker marker4;
	marker4.id = marker.id;
	marker4.ns = "vigir_footstep_planning/step_plan";
	marker4.header = original_step.foot.header;
	marker4.type = visualization_msgs::Marker::TRIANGLE_LIST;
	marker4.action = visualization_msgs::Marker::ADD;

	marker4.scale.x = 0.01;
	marker4.scale.y = 0.01;
	marker4.scale.z = 0.01;
	marker4.color.a = 1.0;
	marker4.color.r = 1.0;
	marker4.color.g = 0.0;
	marker4.color.b = 0.0;

	marker4.points.push_back(p_1);
	marker4.points.push_back(p_2);
	marker4.points.push_back(p_3);

	std_msgs::ColorRGBA c;
	c.r = 0;
	c.g = 1;
	c.b = 0;

	marker4.colors.push_back(c);
	marker4.colors.push_back(c);
	marker4.colors.push_back(c);


	marker_array.markers.push_back(marker4);
	marker.id++;

	// arrow
	visualization_msgs::Marker marker2;
	marker2.id = marker.id;
	marker2.header = original_step.foot.header;
	//arrow.header.stamp = ros::Time::now();
	marker2.ns = "vigir_footstep_planning/step_plan";

	marker2.type = visualization_msgs::Marker::ARROW;
	marker2.action = visualization_msgs::Marker::ADD;

	geometry_msgs::Point pos;
	pos.x = p1[0];
	pos.y = p1[1];
	pos.z = p1[2];
	marker2.pose.position = pos;
	marker2.pose.orientation = quat_msg;

	marker2.scale.x=0.1;
	marker2.scale.y=0.01;
	marker2.scale.z = 0.01;

	marker2.color.g = 1.0f;
	marker2.color.a = 1.0;
	marker2.color.r = 0.0f;
	marker2.color.b = 0.0f;

	marker_array.markers.push_back(marker2);
	marker.id++;
	// arrow



	// foot points
	visualization_msgs::Marker marker3;
	marker3.id = marker.id;

	marker3.header = original_step.foot.header;
	//foot.header.stamp = ros::Time();
	marker3.type = visualization_msgs::Marker::POINTS;
	marker3.action = visualization_msgs::Marker::ADD;
	marker3.scale.x = 0.01;
	marker3.scale.y = 0.01;
	marker3.scale.z = 0.01;
	marker3.color.a = 1.0;
	marker3.color.r = 1.0;
	marker3.color.g = 0.0;
	marker3.color.b = 0.0;

	marker3.ns = "vigir_footstep_planning/step_plan";

	for (std::map<int, std::vector<double>>::iterator iter = original_point_map.begin();
											iter != original_point_map.end();
											++iter){
		std::vector<double> original_point = iter->second;


		geometry_msgs::Point p;
		p.x = original_point[0];
		p.y = original_point[1];
		p.z = original_point[2];

		std_msgs::ColorRGBA c;

		c.g = 1.0;
		c.a = 1;

		marker3.points.push_back(p);
		// Here, the field colors is populated with a specific color per point.
		marker3.colors.push_back(c);
	}

	marker_array.markers.push_back(marker3);

	return;

}

} // namespace msgs
} // namespace

#endif /* MULTI_CONTACT_POINT_ESTIMATOR_INCLUDE_MULTI_CONTACT_POINT_ESTIMATOR_UNEVEN_TERRAIN_STAND_VISUALIZATION_VISUALIZATION_NON_CVX_FOOT_H_ */
