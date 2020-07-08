# Autonomous Traversal
Gazebo simulation of a 4 wheeled skid steer bot that can perform simple autonomous traversal as well as traversal with obstacle avoidance using a GPS, an IMU, ultrasonics and a Kinect.

## How to Run
   1. Install:
      ```bash
         sudo apt-get install ros-melodic-depthimage-to-laserscan
      ```
   2. Clone repo 
      ```bash
         git clone https://github.com/aditirao7/auto_trav.git
      ```
   3. On 2 separate terminals (in mybot_ws), run:
      ```bash
         catkin_make && source devel/setup.bash
      ```
   4. Then run:
      ```bash
         roslaunch mybot_gazebo mybot_world.launch
         
         cd src/mybot_description
         chmod +x auto_trav.py
         rosrun mybot_description auto_trav.py 49.9000534303 8.89991622116
         
         cd src/mybot_description
         chmod +x simple_trav.py
         rosrun mybot_description simple_trav.py 49.9000534303 8.89991622116
      ```
   5. Give the GPS location for auto_trav.py or simple_trav.py accordingly by using the /fix topic.
