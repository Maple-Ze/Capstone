# Introduction:

In the final lab, we will run a competition-based experiment in which the various functions of navigation and visual recognition throughout the semester will be integrated into a complete system. The map and missions are shown below. 

![](C:\Users\15270\Desktop\移动机器人\image\mission.png)

<center><b>Map and missions</b></center>

Four blue circles represent locations (pit stops or PS’s) where the robot needs to make a stop. Each pit stop is exactly 50cm from the two adjacent walls. There are AR code markers (scenic spots or SS’s) placed somewhere on two walls and their numbers are not known a priori. There are also two black lines that lead your robot through (1) the narrow channel from the room of PS1 to the room of PS2, and (2) the interior area of the largest room of the environment where LiDAR data may not be reliable for localization.



# **Tasks:**

Our task is when the TurtleBot reaches any PS, it need to stop and make sure that any part of the car body can cover the PS. At the same time, when passing the SS, the TurtleBot should send a signal to the computer to make the corresponding sound for the AR tag. The competition is divided into two rounds:

### Round 1：

Robot should start from PS1 and visit PS2, PS3, and PS4 in turn, as well as searching for the two SS’s and reporting/signaling their numbers when they are found. Each of the five tasks (three PS’s and two SS’s) is worth 10% (SP1 is the initial location and not worth any points). The order to visit PS2 and SS1 is arbitrary, as well as PS3 and SS2.

### Round 2：

Robot will have a total of three minutes to compete. Also starting at PS1, it can attempt any of the PS’s and/or SS’s in turn, to earn points. Each perfectly completed task is worth 10 points. However, the tasks should be exchanged. We cannot complete the tasks in one side invariably.



# Experimental Platform

### Lenovo Legion Y7000P

The computer used in this experiment is Lenovo Legion Y7000P with 16GB RAM. CPU is Intel Core i7-10875H. GPU is NVIDIA GeForce RTX 2060.

<img src="C:\Users\15270\Desktop\移动机器人\image\computer.jpg" style="zoom:15%;" />

<center><b>Lenovo Legion Y7000P</b></center>

### TurtleBot3

The robot used in this experiment is TurtleBot3. [TurtleBot](https://www.TurtleBot.com/) is a [ROS](http://www.ros.org/about-ros/) standard platform robot. There are 3 versions of the TurtleBot model. TurtleBot3 is a small, affordable, programmable, ROS-based mobile robot for use in education, research, hobby, and product prototyping.

<img src="C:\Users\15270\Desktop\移动机器人\image\turtlebot.jpg" style="zoom:8%;" />

<center><b>TurtleBot3</b></center>

### ROS

ROS is the abbreviation of Robot Operating System. ROS is a flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms. 

ROS is not an operating system in the traditional sense of process management and scheduling; rather, it provides a structured communications layer above the host operating systems of a heterogenous compute cluster.

![](C:\Users\15270\Desktop\移动机器人\image\ros.png)

<center><b>ROS</b></center>



# Tactics：

Due to the problem of visual transmission delay, I did not use the black lines for line following. In the previous experiments, I found that there was a significant delay between the current position of the robot and the image from the camera, which could have caused our TurtleBot to crash into a wall or veer off course. As a result, we rely entirely on lidar for navigation.

Firstly, use SLAM to construct a map for the competition terrain. 

![](C:\Users\15270\Desktop\移动机器人\image\map.png)

<center><b>Map constructed by SLAM</b></center>

Then match the location of TurtleBot and the map. 

![](C:\Users\15270\Desktop\移动机器人\image\match.png)

<center><b>Match the location</b></center>

Finally we can get the coordinate position and orientation of the robot through the topic **/amcl_pose**.

```
rostopic echo /amcl_pose
```

![](C:\Users\15270\Desktop\移动机器人\image\amcl.png)

<center><b>Get the position and orientation</b></center>

Move the robot to five task points, record the coordinate position of each point and set the robot facing AR tag at SS's for identification.

Write the coordinates and orientation of the five task points into the program. Let TurtleBot execute the five positions in sequence, and stop for a second after each task is completed. The coordinates and orientations of the five task points in this project are shown in the figure below.

```python
self.locations = dict()  
self.locations['one']   = Pose(Point(4.28564075035,  3.7944754537, 0.000), Quaternion(0.000, 0.000, -0.999187940384, 0.0402921802795))
self.locations['two']   = Pose(Point(2.80089789076,  3.75163288353, 0.000), Quaternion(0.000, 0.000, -0.999421186545,  0.034018993002))
self.locations['three'] = Pose(Point(4.2581847411,  7.83543252813, 0.000), Quaternion(0.000, 0.000, -0.999114356415, 0.0420773431252))
self.locations['four']  = Pose(Point(4.15707936356, 6.85442485998, 0.000), Quaternion(0.000, 0.000, 0.997792352715, 0.0664109995628))
self.locations['five']  = Pose(Point(0.200589099314, 7.76813161799, 0.000), Quaternion(0.000, 0.000,0.985441957522,0.170012200606))
```

Let the program execute in the above order, the robot will arrive at each task point in turn and complete the parking. After each task is completed, **target_num + 1**. Then TurtleBot moves to the next task point when the stop time is over.

```python	
while not rospy.is_shutdown():
    location = sequeue[target_num]
    rospy.loginfo("target_num_value:"+str(target_num)) 
    rospy.loginfo("Going to: " + str(location))
    time.sleep(1)
    self.send_goal(location)

    finished_within_time = self.move_base.wait_for_result(rospy.Duration(90))
    if not finished_within_time:
        self.move_base.cancel_goal()
        rospy.logerr("ERROR:Timed out achieving goal")
    else:
        state = self.move_base.get_state()

    target_num += 1
```

Send_goal is defined as:

```python	
def send_goal(self, locate):
    self.goal = MoveBaseGoal() 
    self.goal.target_pose.pose = self.locations[locate]
    self.goal.target_pose.header.frame_id = 'map'
    self.goal.target_pose.header.stamp = rospy.Time.now()
    self.move_base.send_goal(self.goal) #send goal to move_base
```

For AR tag identification, a subscriber named **ar_detect** is used to subscribe information on the  topic **/ar_pose_marker** at all times.

```c++
int main(int argc, char **argv)
{
    ros::init(argc, argv, "ar_detect");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/ar_pose_marker", 10, chatterCallback);
    ros::spin();
    return 0;
}
```

Using callback function mechanism, if the camera of the robot sees AR Tag, the information of this AR Tag will appear on the topic of **/ar_pose_marker**, including id, pose and so on. At the same time, create a Sound_Play: : SoundClient object **sc**. Use **sc** to make a sound of the English words of corresponding numbers for the id of markers to indicate that the robot has successfully identified the AR tag.

```c++
void chatterCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
{
    sound_play::SoundClient sc;
    sleep(2);
    if (msg->markers.size() > 0)
    {
        ROS_INFO("[%d]", msg->markers[0].id);
        id_num = msg->markers[0].id;
        switch (msg->markers[0].id)
        {
        case 0:
            sc.say("Zero");
            sleep(2);
            break;
```

If there is no recognizable AR tag, print "Cannot find AR tag." in the terminal to inform the user and to prove that the program is running properly. If no information is displayed when there is no AR tag identified, it is impossible to determine whether the program is working properly.

```c++
else
{
    ROS_INFO("Cannot find AR tag");
}
```



### Round 1:

The program executes each position in order of PS2-->SS1-->PS3-->SS2--> PS4 and emits the corresponding number when reaching AR tags.

### Round 2:

In 3 minutes, in the order of PS2àSS1àPS3àSS2à PS2àSS1àPS3àSS2 to execute. When finish any task，**target_num + 1**. TurtleBot will execute the next task. When the task for SS2 is finished, set target_num to zero and make TurtleBot go to PS2 again.

```python
target_num += 1
if target_num > 3:
    target_num = 0
```



# Results

### Round 1:

TurtleBot successfully reached each mission point and perfectly covering the circle at PS's. At SS's, the sound corresponding to AR Tag was successfully emitted. The time to complete all tasks is 2 '30 ". The results of reaching each task point are shown below.

<img src="C:\Users\15270\Desktop\移动机器人\image\1_0.jpg" style="zoom:10%;" />

<center><b>Go through the narrow channel</b></center>

<img src="C:\Users\15270\Desktop\移动机器人\image\1_1.jpg" style="zoom:10%;" />

<center><b>Stop at PS2</b></center>

<img src="C:\Users\15270\Desktop\移动机器人\image\1_2.jpg" style="zoom:10%;" />

<center><b>Stop at SS1</b></center>

<img src="C:\Users\15270\Desktop\移动机器人\image\1_3.jpg" style="zoom:10%;" />

<center><b>Stop at PS3</b></center>

<img src="C:\Users\15270\Desktop\移动机器人\image\1_4.jpg" style="zoom:10%;" />

<center><b>Stop at SS2</b></center>

<img src="C:\Users\15270\Desktop\移动机器人\image\1_5.jpg" style="zoom:10%;" />

<center><b>Stop at PS4</b></center>

### Round 2：

TurtleBot is also successful as in Round 1. And after completing the task at SS2, it returns to PS2 again to execute the task. In the end, 6 tasks were successfully completed within 3 minutes. The result is similar to **Round 1**. The detailed results are shown in the attached videos.



# Summary

The commands of this lab are shown as follows:

```txt
roscore

******************* on TurtleBot *********************
roslaunch TurtleBot3_bringup TurtleBot3_robot.launch
roslaunch raspicam_node camerav2_410x308_30fps.launch
******************* on TurtleBot *********************

rosrun image_transport republish compressed in:=/raspicam_node/image raw out:=/raspicam_node/image

roslaunch ar_track_alvar alvar.launch

rostopic echo /ar_pose_marker

rosrun sound_play soundplay_node.py

rosrun line_follower_TurtleBot ar_detect

roslaunch TurtleBot3_navigation TurtleBot3_navigation.launch map_file:=$HOME/map.yaml

rosrun simple_navigation_goals move.py
```



In this lab, we sucessfully complete the competions. In **Round 1**, we get the full mark. In **Round 2**, we finished 6 tasks and get 58 points. Due to the network problem, when we proceed **Round 2**, TurtleBot exprienced disconnection from the computer for a period of time. This caused the positioning of the robot location deviated for a little. And when TurtleBot reached PS2 at the second time, it did not cover the circle perfectly, which caused a 2 points deduction.



# Contribution

Xuheng Gao (100%)



# Acknowledgement

Thanks for Professor Zhang's careful guidance in this semester and the help of all the TAs.

Thanks for the author of [this website](https://www.corvin.cn/892.html).



