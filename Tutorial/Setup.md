# Setup Verification

## Update the Repository
Ensure your repository is up-to-date if there are new commits:
```
$ cd ~/Deep_learning_d415_ROS2
git pull # Pulls the latest commits to your local repository
git submodule update --init --recursive # Updates the repository submodules
```
## Update Docker Images
To ensure the latest dependencies and libraries are installed, update your Docker image:
```
$ docker pull 
```
## Build your workspace in docker
When the ros2_ws add new rospackage or new c++ code, you need to rebuild your workspace for using the functions.
```
cd ~/Deep_learning_d415_ROS2
cb
```