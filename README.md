<h1> Robotic Search and Fetch </h1>

Robot helps searching for known objects around the house, manipulating and delivering it if asked to.


<h2> Setup  </h2>
For this project, several packages from the sensible-robots group is needed. These were developed by the LASR (Leeds Autonomous Service Robots) team. The packages are:

* lasr_object_detection_yolo

* lasr_pcl

* jeff_segment_objects

* ycb_objects

* custom_worlds

You may need to move some of the model folders to '~/.gazebo/models/' if necessary.


<h2> Installation </h2>

Clone the packages to your catkin workspace 'catkin_ws/src' and build it:
~~~
catkin build
~~~


<h2> Usage </h2>

`butler.launch` launches the robot in the environment, loads the objects, the map and parameters supplied in the yaml files:
~~~
roslaunch butler_erl butler.launch
~~~

`butler_auxiliary.launch` runs all of the necessary service and action server nodes:
~~~
roslaunch butler_erl butler_auxiliary.launch
~~~

Run the base node, which launches the state machine:
~~~
rosrun butler_erl butler.py
~~~

MoveIt expects point cloud at a specific rate from a specific topic. Run this to utilize it:
~~~
rosrun topic_tools throttle messages /xtion/depth_registered/points 2  /throttle_filtering_points/filtered_points
~~~
