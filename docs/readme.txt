In src/description/urdf/body.xacro at line 7 and 184-
replace directory with whatever the directory is in your system

Once done, don't forget to catkin_make in parent directory

In simulation/launch you'll find steer.launch that starts RViz and Gazebo

To view laserscan feed in RViz-
Note the topic where laser data is being published: /IEEE_sample_bot/laser/scan
In Rviz, click on add in the bottom left corner and select LaserScan
Expand the LaserScan tab and set the topic as above
Change size to something bigger like 0.1 or whatever to see the laser visualisation better
Obviously, to see something, you must add some objects in gazebo (like a sphere or cube)

To view camera feed in RViz-
Note the topic where camera feed is being published: /IEEE_sample_bot/camera/image_raw
In Rviz, click on add in the bottom left corner and select Image
Expand the Image tab and set the topic as above

In control/scripts you'll find two files:
publisher
keyboard driver

Run these in two separate terminals. Use keys i, j, k, l, m
in the terminal where keyboard driver is running to control the bot

If you want to try out a maze with the robot, roslaunch steer_maze.launch from simulation

Don't forget to source the setup.bash file in every terminal you open!

SLAM_GMAPPING 
in src/mapping launch , the g_mapping.launch file will spawn a slam_gmapping node for SLAM it subscribes from the /tf and laser sensor data and publishes to /map topic

AMCL 
Check in simulation/launch , simulation/param and simulation/src 


Mazze karo
