
1、创建 ros1 工作空间
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws
catkin_make


2、下载功能包ymbot_amp_deploy， 将其移动到 ~/catkin_ws/src 下

3、编译
cd ~/catkin_ws
catkin_make

4、刷新环境变量
source devel/setup.bash
rospack list

4、启动（ g1 和 ymbot_e ）
roslaunch ymbot_amp_devel g1_amp_sim2sim.launch
roslaunch ymbot_amp_devel ymbot_e_amp_sim2sim.launch
roslaunch ymbot_amp_devel ymbot_e_sim2sim.launch

roslaunch ymbot_amp_devel ymbot_e_rl_sim2sim.launch
