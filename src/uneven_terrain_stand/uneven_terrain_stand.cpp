#include <thread>
#include <chrono>
#include <cmath>
#include <locale>
#include <multi_contact_point_estimator/uneven_terrain_stand/uneven_terrain_stand.h>
#include <multi_contact_point_estimator/uneven_terrain_stand/data_points.h>
#include <multi_contact_point_estimator/uneven_terrain_stand/convex_hull_stand.h>
#include <multi_contact_point_estimator/uneven_terrain_stand/model_stand.h>


using namespace orgQhull;

UnevenTerrainStand::UnevenTerrainStand(vigir_footstep_planning::State s, geometry_msgs::Vector3 foot_size, vigir_terrain_classifier::HeightGridMap::Ptr height_grid_map, FootForm ff, MultiContactPointModel* const &model)
: foot_size(foot_size), height_grid_map(height_grid_map), ff(ff), model(model)
{
	x = s.getX();
	y = s.getY();
	p = s.getPose();
	yaw = s.getYaw();
}


UnevenTerrainStand::~UnevenTerrainStand() {
	// TODO Auto-generated destructor stub
}

FootStateUneven UnevenTerrainStand::getStand() {

	/*
	// TIME MEASUREMENT, TODO remove later
	auto start_time = std::chrono::high_resolution_clock::now(); // ERROR ONLY IN ECLIPSE, BUT IT BUILDS AND RUNS
	*/

	std::vector<orgQhull::vec3> points;
	get_points_under_foot(points);

	double foot_size_half_x = 0.5*foot_size.x;
	double foot_size_half_y = 0.5*foot_size.y;

	// TODO could select ZMP to choose best support
	tf::Vector3 middle_pos;
	middle_pos.setX(0);
	middle_pos.setY(0);
	middle_pos.setZ(0);
	double ground_height_at_zmp = 0.0;
	const tf::Vector3 &zmp_pos = p * middle_pos;
	std::vector<double> zmp = {zmp_pos.getX(), zmp_pos.getY(), zmp_pos.getZ()};

	FootStateUneven s = predictStand(points, zmp);

	/*
	//TIME MEASUREMENT, TODO remove later
	// show time (shows error in eclipse but really compiles and runs)
	auto end_time = std::chrono::high_resolution_clock::now(); // ERROR ONLY IN ECLIPSE, BUT IT BUILDS AND RUNS
	auto time = end_time - start_time;
	//ROS_INFO_STREAM("TIMING: " << std::chrono::duration_cast<std::chrono::microseconds>(time).count());

	std::ofstream outfile;
	outfile.open("/home/jan/Desktop/test.txt", std::ios_base::app);
	outfile << std::chrono::duration_cast<std::chrono::microseconds>(time).count() << "\n"; // ERROR ONLY IN ECLIPSE, BUT IT BUILDS AND RUNS
	*/

	return s;

}



void UnevenTerrainStand::get_points_under_foot(std::vector<orgQhull::vec3> &points) {

	tf::Vector3 relative_pos;
	relative_pos.setZ(0.0);

	double foot_size_half_x = 0.5*foot_size.x;
	double foot_size_half_y = 0.5*foot_size.y;

	double sampling_step_x = foot_size.x/(double)(sampling_steps_x-1);
	double sampling_step_y = foot_size.y/(double)(sampling_steps_y-1);

	int y_idx = sampling_steps_y;
	int x_idx = sampling_steps_x;

	for (double y = -foot_size_half_y; y <= foot_size_half_y; y+=sampling_step_y)
	{
		y_idx--;
		relative_pos.setY(y);
		x_idx = sampling_steps_x;

		for (double x = -foot_size_half_x; x <= foot_size_half_x; x+=sampling_step_x)
		{

			// determine point in world frame and get height at this point
			x_idx--;
			relative_pos.setX(x);

			const tf::Vector3 &trans_pos = p * relative_pos;

			double height = 0.0;
			bool hasHeight = height_grid_map->getHeight(trans_pos.getX(), trans_pos.getY(), height);

			bool isInFootShape = ff.isInFoot(x_idx, y_idx, sampling_steps_x, sampling_steps_y);

			if (isInFootShape && hasHeight)
			{
				orgQhull::vec3 v = orgQhull::vec3(trans_pos.getX(), trans_pos.getY(), height);
				v.IDX[0] = x_idx; // ml prediction works with indices
				v.IDX[1] = y_idx;
				points.push_back(v);
			}
		}
	}

}

FootStateUneven UnevenTerrainStand::predictStand(std::vector<orgQhull::vec3> const &points, std::vector<double> zmp) {

	orgQhull::vec3 zmpv(zmp[0], zmp[1], zmp[2]);

	// Ground Truth or Prediction:
	ConvexHullStand hullStand = ConvexHullStand();
	FootStateUneven s = hullStand.getStand(points, zmpv);

	// TF prediction
	//ModelStand modelStand = ModelStand();
	//FootStateUneven s = modelStand.tensorflow_predict(points, zmpv, sampling_steps_x, sampling_steps_y, myPred, yaw, height_grid_map, ff);

	if(s.getValid() != 1){
		return s;
	}

	double footScale = 0.3;	// TODO get from foot form
	double maxFootArea = foot_size.x * foot_size.y * footScale; // foot form is smaller...
	s.setSupport(s.getFacetArea() / maxFootArea);

	return s;
}

