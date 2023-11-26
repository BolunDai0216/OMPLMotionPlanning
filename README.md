# AnywareInterview

## Running the Code (Ubuntu + Intel CPU)

To run the code please build the docker image using the following command:

```bash
sudo docker build -t ompl .
```

Then run the docker image using the following command:

```bash
sudo docker run -it --name ompl_container ompl:latest
```

Inside the docker container, go to the folder `/home/AnywareInterview/cpp/build` and run

```bash
cmake ..
cmake --build . -j 2
```

Then run the executable `ompl_hello_world` to see the results

```bash
./ompl_hello_world
```

## Running the Code (Arm CPU)

Due to the lack of support for Arm-based CPUs in the C++ versions of `pinocchio` and `hpp-fcl`, please refer to the python code in the folder `python` folder to see the results (building these libraries from source was not ideal due to the time constraint).

```bash
python3 -m pip install pybullet pin hpp-fcl rtree numpy
```

Then, run the command

```bash
python3 rrt_star.py
```

This will generate a pickle file in the `data` folder of the root directory. To visualize the results, run the command

```bash
python3 visualize.py
```