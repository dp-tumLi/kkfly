# README ALLWAYS FIRST!
In case ensure that you will have access to `github.com`, you should first verify if you have already generated `ssh-key` on you local machine.

A way to check if you have `ssh-key` is to check the directory in `/home/<your-user-name>/.ssh/`. If **NOT**, enter command in an arbitrary terminal:

```
ssh-keygen
```

and press **ENTER** for whatever it shows or you could customize your own key with your wish.

Also, notice in [Setup](#Setup) you will build the docker image with your local generated ssh-key, we define the default path to your key, and it names `id_ras`, if **NOT**, please do not forget to specify this argument with your own.


## Docker
Download the Docker with following step in a terminal:
```
sudo apt update
```
```
sudo apt install apt-transport-https ca-certificates curl gnupg-agent software-properties-common
```
```
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
```
```
sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"
```
```
sudo apt update
```
```
sudo apt install docker-ce docker-ce-cli containerd.io
```

## Setup <a name="Setup"></a>
Navigate inside `autsys-projects-kkfly/Docker/` , Build the image with this command:
```
sudo docker build -t kkfly . --build-arg ssh_prv_key="$(cat ~/.ssh/id_rsa)" --no-cache 
```
and run
```
sudo docker run -it -d --network=host --name kkfly kkfly:latest bash
```

## Starting and interacting
Start the docker container:
```
sudo docker start kkfly
```

Open an interactive shell:
```
sudo docker exec -it kkfly bash
```

To exit the docker shell type `exit`.

## Stopping and deleting
Stop the container:
```
sudo docker stop kkfly
```
Delete the container:
```
sudo docker rm kkfly
```


### on host machine:
Open a 1st new terminal under `kkfly/catkin_ws` and run: 
```
source devel/setup.bash
roslaunch simulation simulation.launch
```
if you compile the code with `catkin_make_isolated`, then run:
```
source devel_isolated/setup.bash
roslaunch simulation simulation.launch
```
Open a 2nd new terminal under `kkfly/catkin_ws`, and run:
```
source devel/setup.bash
roslaunch controller_pkg control.launch
```
Do the same if you compile with `catkin_make_isolated` as 1st terminal source step

### on docker image:
open extra 3 terminals with `sudo docker exec -it kkfly bash`, now you have 4 terminals in docker to launch the project.
  on 1st docker terminal:
  ```
  source devel/setup.bash
  roslaunch filter filter.launch
  ```
  on 2nd docker terminal:
  ```
  source devel/setup.bash
  roslaunch path_planner octomap_mapping.launch
  ```
  on 3rd docker terminal:
  ```
  source devel/setup.bash
  roslaunch path_planner octomap_mapping3d.launch
  ```
  on 4th docker terminal:
  ```
  source devel/setup.bash
  roslaunch path_planner waypoint_mission.launch
  ```
