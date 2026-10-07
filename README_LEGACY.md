# F1-Tenth-Duke
The repository contains all documentation belonging to the F1 Tenth team at Duke University, located in NC, USA. Copyright is held by Duke University, specifically for instructions and other accompanying materials. However, the code is open-source. :)

This repo is bulit for students who would use the car we currently using the F1 Tenth car which is belonged Mechanical Engineering and Material Science department in Duke University, NC, USA. However, it is more than welcome to all other users to use our code!

Contact: Dr.Siobhan Oca. Email: siobhan.rigby@duke.edu

This project is based on F1 Tenth project from X-LAB, University of Pennsylvania. Check https://f1tenth.org/index.html for more information. STRONGLY RECOMMENDED everyone to finish the "Learn" section on the website.

### **Please read this repo carefully, all details might help you to avoid error!**

# **Things to Keep in Mind For All Future Users**
1. SLOW DOWN! At the time point we built the car, this car values around $4000 and 100+ hours work. The Hokuyo LiDar is $1600, Jetson without SSD and Wifi module is $660, VESC is $300. Therefore, SLOW DOWN and be CAREFUL! We fully understand robotics guys are "crazy" like us. :)
2. Check the environment first. Avoid GLASS, CHAIRS, ANYTHING IT MIGHT GO UNDER! Those things are LiDar killer, because LiDar is using laser and it is almost a 2D sensor.
3. Understand what are you doing and what will happen. Simulation is your best friend! If something didn't work on simulation, it would not work in reality. If the behavior of the car is totally different with your expection, you need to study more!

## Version of Software and Hardware
OS: Ubuntu 20.04LTS
ROS: ROS2 Foxy
Processor on car: Jetson Xavier NX 8GB (J2021)
JetPack: 5.1.2
LiDar: Hokuyo 10LX
Chassis: Traxxas Slash 4x4 Platinum Edition; 1/10 Scale Brushless Pro 4WD
Power board: V4
ESC：FSESC based on VESC6

## Achievements and Future Work
Since we spent most of our time on hardware, we only selected some of tasks to do and we left all documentations we have. We try to make this project is fairly easy to take over. You don't have to follow the same track we did, and we suggest everyone follow the order we listed.

Achieved:
- Drive a distance
- AEB
- Simulator Setup
- SLAM
- Particle Filter
- MPC in simulation
- Vision detection

Future Work:
- PID wall follower
- Pure pursuit
- RRT
- Camera communication in ROS
- Raceline optimization
- MPC in real track


## Instruction Navigation
### All packages in "Code" can be used directly. If you are using the car we were using, please check what do you have firstly. We left the instruction for all users, so they can rebuild the system from 0. We did both simulation and physical testing. Please use the correct launch file, or make one by yourselves! STRONGLY RECOMMENDED USING DOCKER FOR YOUR SIMULATION PART!!!
### Vehicle Setup
- [Build](/Pages/Build.md)
- [Configuring the NVIDIA Jetson NX](/Pages/configuring_nx.rst)
- [Connecting the Pit/Host and the NVIDIA Jetson NX](/Pages/connecting_host.rst)
- [Hokuyo 10LX Ethernet Connection Setup](/Pages/Hokuyo_Lidar/Hokuyo.md)
- [Configuring the VESC](/Pages/VESC/VESC_config.md)
- [Driver Stack Setup](/Pages/driver_stack_setup.rst)
- [Manual Control](/Pages/drive_manual.rst)
- [Calibrating the Odometry](/Pages/drive_calib_odom.rst)
- [Autonomous Control](/Pages/drive_autonomous.rst)
- [Camera Setup](/Pages/Depth_Camera.rst)
### Tasks
- [Drive A Distance](/Pages/drive_autonomous.rst)
- [AEB](/Pages/Automatic_Emergency_Braking.rst)
- [Simulator](/Pages/simulator_install.md)
- [PID wall follower]()
- [SLAM](/Pages/SLAM.md)
- [Particle Filter](/Pages/particle_filter.md)
- [Pure Pursuit]()
- [RRT]()
- [Vision Detection](/Pages/D415_Yolov5.rst)
- [Raceline Optimization](/Pages/Raceline_opt.md)
- [MPC](/Pages/MPC.rst)
### Helper pages
- [Git Command](/Pages/git_command.md)
- [SSD Setup](/Pages/SSD.md)
- [Software Recommendation](/Pages/software_setup.md)
