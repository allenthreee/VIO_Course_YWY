手写VIO ch2

ros_sim_ws用于产生静态IMU仿真数据，对应作业第一题。运行后将得到一个rosbag，之后再用imu_utils 或者 kalibr_allan进行标定即可。

vio_sim_ws用于产生IMU动态数据，对应作业第二题，用中值积分替换欧拉积分。