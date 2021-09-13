source /opt/ros/noetic/setup.bash
sudo apt update

rosdep update
rosdep install --from-paths src --ignore-src -y

sudo rm -r devel
sudo rm -r install

catkin_make

source devel/setup.bash
