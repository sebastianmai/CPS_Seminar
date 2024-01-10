# CPS_Seminar
This repo includes relevant code and pictures for the Seminar in Cyber Physical System.
It uses vrx (https://github.com/osrf/vrx) to simulate a PID controlled vessel.


To launch to Simulation:

Navigate to the vrx folder:
```
cd vrx_ws/
```

Install:
```
. install/setup.bash
```

Setup ros (also needed for pid_contorller execution):
```
. ~/ros2_humble/install/local_setup.bash
```

Launch custom world:
```
ros2 launch vrx_gz competition.launch.py world:=/home/bastimai/Seminar_code/sydney_regatta_custom
```

In order to use the pid_controller make sure the the simulation is running and Ros2 is stup in the new terminal as well. Then you can normally execute the python file.
