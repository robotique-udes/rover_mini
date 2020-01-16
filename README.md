
# ROVER_MINI
##  Robotique UdeS
### Steps to control the mini rover :

In a terminal :
```
cd ~/
mkdir -p rover_mini_ws/src
cd rover_mini_ws/src
git clone https://github.com/robotique-udes/rover_mini.git
cd ..
catkin_make
echo "source ~/rover_mini_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
roslaunch osr_bringup osr.launch
```
