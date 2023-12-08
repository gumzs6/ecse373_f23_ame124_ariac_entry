<h1>Final: ARIAC (Agile Robotics for Industrial Automation Competition) 2019</h1>

<h2>Overview</h2>
    <p>This package can launch the ARIAC 2019 competition environment and run a node which outputs the first product of the first shipment of the first order, locates parts of received order within a bin, transforms that part's pose from the camera frame to the robot frame, and moves to them accordingly.</p>

<h2>Prior Setup</h2>
    <p>Open new terminal and configure ROS.</p>
    <p>Make sure to have the ARIAC 2019 simulation environment and ARIAC node installed on your device. Those can be found in the following repositories:</p>
        <blockquote>
            <p><link>https://github.com/cwru-eecs-373/cwru_ariac_2019.git</link></p>
            <p><link>https://github.com/cwru-eecs-373/ecse_373_ariac.git</link></p>
        </blockquote>
    <p>The ik_service node also needs to be pulled and in the same catkin workspace as this ariac_entry package which can be found at <link>https://github.com/gumzs6/ecse373_f23_ame124_ik_service</link>.</p>
    <p>Information about ARIAC 2019 Competition and Documentation can be found here: <link>https://bitbucket.org/osrf/ariac/wiki2019/Home</link>.</p>

<h2>Instructions</h2>
    <p>Create a catkin workspace and clone this ariac_entry package into the src subdirectory.</p>
    <p>Launch the node with the command <code>roslaunch ariac_entry entry.launch &</code> </p>
    <p>Once the ARIAC environment is open, press the play button and wait a few moments for the simulation to begin.</p>
    <p>Use the command <code>killall gzserver gzclient roslaunch</code> to end and quit the simulation.</p>

<h2>Launch Files</h2>
    <p>The launch file for this node is entry.launch.</p>
    <p>There is python argument which is to account for an em.py error due to changes in ROS Noetic.</p>
        <ul>
            <li><code>python:=false</code> is recommended.</li>
        </ul>

<h2>Tag Descriptions</h2>
    <p>The laboratory_5 tag is the submission which finds the first product from the first shipment of the first order. It also finds the corresponding bin of parts from the order and performs a transform to get the pose of that part in the frame of the robot.</p>
    <p>The trajectory_topic tag includes publishing to a topic which communicates with the follow_joint_trajectory node that in turn moves the UR10 using the a service which implements inverse kinematics.</p>
    <p>The final_submission tag has everything from the previously mentioned tags with the addition of the arm touching a part.</p>
