# multi_contact_point_estimator

![Screenshot](https://github.com/janpmetz/multi_contact_point_estimator/blob/master/path.png?raw=true)

To select either the convex hull algorithm or the tensorflow prediction change the parameter use_tensorflow_model in the planner_multi_contact_points.launch file.

During the build the qhull source is downloaded from qhull.org and included into the project. We need the qhull source for the convex hull algorithm. 

A point cloud file with uneven terrain can be found here:
https://github.com/janpmetz/foot_pose_estimator

Launch with:
roslaunch multi_contact_point_estimator footstep_planner_test.launch
