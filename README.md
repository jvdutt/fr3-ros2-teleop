# fr3-ros2-teleop
hebi-py for teleop , cartesian impedance for control

The changes made to the [original code](https://github.com/frankaemika/franka_ros2/tree/humble) are:
- cartesian_impedance_example_controller
  - ros1 [code](https://github.com/frankaemika/franka_ros2/pull/51) to ros2
  -  subscibes to /equilbrium_pose topic for the desired location
  - can change the impedance parameters in franka_ros2/franka_bringup/config/controllers.yaml
- cartesian_pose_example_controller
  - added filter for removing the discontinuity errors
  - subscibes to /equilbrium_pose topic for the desired location
- teleop.py
  - uses Hebi, Mobile-IO for input 
  - publishes to /equilibrium_pose topic
 
### Download franka_ros2.zip and unzip.

```bash
unzip ~/Downloads/franka_ros2.zip -d ~/franka_ros2
cd franka_ros2
```

#### Docker Compose (if docker is not installed in your system check [this](https://docs.docker.com/engine/install/ubuntu/)) 

  2. **Save the current user id into a file:**
      ```bash
      echo -e "USER_UID=$(id -u $USER)\nUSER_GID=$(id -g $USER)" > .env
      ```
      It is needed to mount the folder from inside the Docker container.

  3. **Build the container:**
      ```bash
      docker compose build
      ```
  4. **Run the container:**
      ```bash
      docker compose up -d
      ```
  5. **Open a shell inside the container:**
      ```bash
      docker exec -it franka_ros2 /bin/bash
      ```
  6. **Clone the latests dependencies:**
      ```bash
      vcs import src < src/franka.repos --recursive --skip-existing
      ```
  7. **Build the workspace:**
      ```bash
      colcon build 
      ```
  7. **Source the built workspace:**
      ```bash
      source install/setup.bash
      ```
  8. **When you are done, you can exit the shell and delete the container**:
      ```bash
      docker compose down -t 0
      ```
#### NOTE : Make sure that move_to_start_example_controller is working , for general documentation of franka_ros2 [check this website](https://frankaemika.github.io/docs/franka_ros2.html)



### HEBI-IO for the teleop

Download the [app](https://docs.hebi.us/tools.html#mobile-io) in the phone , and make sure that both the system and phone are connected to the same network (Ex: same Wifi).
for examples on how to us the api check [this repository](https://github.com/HebiRobotics/hebi-python-examples).

Open the app and use the phone as a joy stick

installing hebi-py

```bash
sudo apt-get update
sudo apt-get install python3-pip
pip install hebi-py
```


## Teleop with cartesian impedance controller

```bash
ros2 launch franka_bringup franka.launch.py arm_id:=fr3 robot_ip:=<fci-ip>
```

open a new terminal and use the below command for accessing the docker container

```bash
docker exec -it franka_ros2 /bin/bash
```
once you are inside the container ,source setup file and load the controller 

```bash
source install/setup.bash
ros2 control load_controller --set-state active cartesian_impedance_example_controller
```
then for teleop
```bash
python3 src/teleop.py
```


## Teleop with cartesian pose controller

```bash
ros2 launch franka_bringup franka.launch.py arm_id:=fr3 robot_ip:=<fci-ip>
```

open a new terminal and use the below command for accessing the docker container

```bash
docker exec -it franka_ros2 /bin/bash
```
once you are inside the container ,source setup file and load the controller 

```bash
source install/setup.bash
ros2 control load_controller --set-state active cartesian_pose_example_controller
```
then for teleop
```bash
python3 src/teleop.py
```

### Tested on 

- Distributor ID:	Ubuntu
- Description:	Ubuntu 22.04.5 LTS
- Release:	22.04
- Codename:	jammy
