# Reachy 2023 with Docker
Small example about using reachy in a Docker environment


# Install Docker

**What is Docker?**

Docker is a platform that simplifies the process of creating, deploying, and running applications using containers. Containers are lightweight and portable, allowing you to package your application and its dependencies into a single unit that can run consistently across different environments.

**Why Use Docker?**

Using Docker ensures that your development environment is consistent across different machines and operating systems. It eliminates issues related to software dependencies and environment setup, making it easier to onboard new users of Reachy environment.

More info in `https://docs.docker.com/engine/install/ubuntu/`

TL;DR : 

Apt repo
```commandline
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Add the repository to Apt sources:
echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update
```
Install 
```commandline
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

(Optional) If you don't want to put sudo before every Docker command, we recommend : 
```commandline
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

Test it
```commandline
docker run hello-world
```

# Reachy 2023
## Start
You may launch the `reachy_2023` container, if not already installed then
```
git clone https://github.com/pollen-robotics/reachy_2023.git
```
Move to `reachy_2023/docker` then
```
docker-compose up -d
```
Then connect to it with
```
docker exec -it reachy_2023 bash
```

Once in `reachy_2023` environment, you can launch the whole stack, either in fake mode 
```
ros2 launch reachy_bringup reachy.launch.py start_sdk_server:=true fake:=true start_rviz:=true
```
or with gazebo backend for simulation
```
ros2 launch reachy_bringup reachy.launch.py start_sdk_server:=true gazebo:=true start_rviz:=true
```
Go to the A simple SDK test section to try it out!

## Development
Alternatively, you can use the `reachy_2023` container as a development environment.
Launch it with 
```
docker-compose -f mode/dev.yaml up -d
```
`reachy_2023` sources will be mounted as volumes into your container, from the `reachy_2023` repository on your host machine to `/opt/reachy_ws/src/reachy_2023` in your container

### Build
You can build the sources with
```
build # build sources
builds # build sources and source the whole ROS environment
cbuilds # clean build sources and source the ROS environment
```
These commands must be done inside the container, so in a terminal where you connected with "docker exec -it reachy_2023 /bin/bash"

**If in doubt, you should recompile after any source update (also this should not be needed if the modified files are only python)**

## Stop
You can stop the `reachy_2023` container with
```
docker-compose down
```
or
```
docker-compose -f mode/dev.yaml down # if you launched the dev mode

```

## A simple SDK test
Log in
```commandline
docker exec -it reachy_2023 /bin/bash
```
then
```commandline
python3
```
```python
from reachy_sdk import ReachySDK
reachy = ReachySDK(host='127.0.0.1')
reachy.r_arm.r_elbow_pitch.goal_position = -90
```
### From local machine
You can use the same steps to access your `reachy_2023` environment from your machine, outside of docker.
Use `127.0.0.1` as an adress and mind having set a `ROS_DOMAIN_ID` env variable *before* launching the compose and your python shell with the Reachy SDK
