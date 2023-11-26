# AnywareInterview

I used an OMPL RRT* solver to solve this task. The collision checking is done using `hpp-fcl` and the forward kinematics is computed using `pinocchio`. The code is written in C++ and the visualization is done using `pybullet` in python. I made one assumption -- the starting joint configuration is given. We can relax this assumption by using the same motion planner but finding a collision-free path from a known collision-free configuration to the starting pose and use the solved joint angles as the starting joint angles.

## Running the Code (Ubuntu + Intel CPU)

To run the code please build the docker image using the following command:

```bash
sudo docker build -t interview .
```

Then run the docker image using the following command:

```bash
sudo docker run -it --name interview_container interview:latest
```

Inside the docker container, go to the folder `/home/AnywareInterview/cpp/build` and run

```bash
cmake ..
cmake --build . -j 4
```

Then run the executable `interview` to see the results

```bash
./interview
```

## C++ Version Visualization

After running the exectuable `interview`, the console output would be similar to the following:

```bash
There are 0 solutions
Info:    RRTstar: Started planning with 1 states. Seeking a solution better than 0.00000.
Info:    RRTstar: Initial k-nearest value of 45
Info:    RRTstar: Found an initial solution with a cost of 4.01 in 5910 iterations (4699 vertices in the graph)
Info:    RRTstar: Created 4999 new states. Checked 2352225 rewire options. 1 goal states in tree. Final solution cost 4.002
Found solution:
Geometric path with 9 states
RealVectorState [0.611 0.215 -0.826]
RealVectorState [0.749545 0.102163 -0.888568]
RealVectorState [1.34436 -0.474563 -1.26863]
RealVectorState [1.72432 -0.827904 -1.3305]
RealVectorState [1.82204 -0.926193 -1.46885]
RealVectorState [1.78629 -1.04777 -1.54173]
RealVectorState [1.59634 -1.10498 -1.51541]
RealVectorState [0.808817 -1.28022 -1.06487]
RealVectorState [0.0708581 -1.3445 -0.532422]
```

Please copy the states and rearange them into a list of lists in `python/visualize.py` as follows:

```python
path = [
    [0.611, 0.215, -0.826],
    [0.749545, 0.102163, -0.888568],
    [1.34436, -0.474563, -1.26863],
    [1.72432, -0.827904, -1.3305],
    [1.82204, -0.926193, -1.46885],
    [1.78629, -1.04777, -1.54173],
    [1.59634, -1.10498, -1.51541],
    [0.808817, -1.28022, -1.06487],
    [0.0708581, -1.3445, -0.532422]
]
```

Then run the command

```bash
python3 visualize.py
```

to visualize the results.

## Sample Trajectories

https://github.com/BolunDai0216/AnywareInterview/assets/36321182/8894b5f6-6905-454f-b794-409bc1db4e2f

https://github.com/BolunDai0216/AnywareInterview/assets/36321182/30d0ea69-b0d2-4127-8ab6-ed6a68e8de20

https://github.com/BolunDai0216/AnywareInterview/assets/36321182/9ec235dc-3e9b-4a9a-a30f-836b51aac2d0

## C++ Version Changing the Environment

To change the environment, please modify the file `cpp/config.yaml`. This allows the user to change the start joint angles, goal tooltip position and the poses of the boxes. To change the size of the robot and boxes, please change the following variables in `cpp/src/state_checker.cpp`:

```cpp
double link1_l = 0.5;  // link 1 length
double link2_l = 0.5;  // link 2 length
double link3_l = 0.5;  // link 3 length
double link_w = 0.1;   // link width
double box_l = 0.4;    // target box length
double box_w = 0.3;    // target box width
double box1_l = 0.4;   // box 1 length
double box1_w = 0.3;   // box 1 width
double box2_l = 0.4;   // box 2 length
double box2_w = 0.3;   // box 2 width
double box3_l = 0.4;   // box 3 length
double box3_w = 0.3;   // box 3 width
```

Additionally, you would also need to change the corresponding values in `python/robots/box.urdf` and `python/robots/robot.urdf`. This is essential due to Pinocchio using the URDF files to compute the forward kinematics. This process can be automated using `xacro`, but it is not yet implemented.

## Running the Code (Arm CPU)

Due to the lack of support for Arm-based CPUs in the C++ versions of `pinocchio` and `hpp-fcl` (building these libraries from source was not ideal due to the time constraint), please refer to the python code in the folder `python` folder for a simplified version of the motion planner where we hardcode the start and goal joint configuration and a RRT* algorithm is used to find a collision-free path.

```bash
python3 -m pip install pybullet pin hpp-fcl rtree numpy
```

Then, run the command

```bash
python3 rrt_star.py
```

This will generate a pickle file in the `data` folder of the root directory. To visualize the results, run the command

```bash
python3 visualize.py --from-pickle
```
