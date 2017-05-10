# multi_contact_point_estimator

To select either the convex hull algorithm or the tensorflow prediction change the parameter use_tensorflow_model in the planner_multi_contact_points.launch file.

During the build the qhull source is downloaded from qhull.org and included into the project. We need the qhull source for the convex hull algorithm. 

Launch with:
roslaunch multi_contact_point_estimator footstep_planner_test.launch
