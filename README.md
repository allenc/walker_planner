# walker_planner
Planner for Wheeled Walker.

Setup
------

1. Clone this repo (unified_planner branch) in catkin_ws/src
Also clone following :
2. git clone https://github.com/shivamvats/smpl.git -b mrmha
3. git clone https://github.com/aurone/leatherman
4. git clone https://github.com/SBPL-Cruz/wheeled_walker

In separate folder :
```
git clone https://github.com/shivamvats/sbpl -b mrmha
mkdir build
mkdir install
cd build
cmake -DCMAKE_INSTALL_PREFIX=../install ..
make install
```

4. Change ~/lolocal in smpl/smpl/CmakeLists.txt and smpl/smpl/smpl-config.cmake.in to install path inside sbpl folder created above

5. catkin build


Generating Taj
--------------

1. roslaunch walker_planner generate_map.launch. Generates map in ~/.ros/-multi_room_map.env. Copy this to env folder and rename to planning_proj.env
2. roslaunch walker_planner generate_start_goals.launch (generates 500 start/goal pairs in ~/.ros/goal_* ~/.ros/start_*. Copy these to environments folder
2. roslaunch walker_planner mrmhaplanner.launch
