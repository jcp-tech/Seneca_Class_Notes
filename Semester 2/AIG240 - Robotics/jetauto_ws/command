# data:2022/12/19 by aiden
# 本文档只包含启动需要的指令，部分指令还需要配合其他设置才能生效
# 请结合教程文档使用, 特别说明，每行指令需要单独开一个终端运行，
# 且有先后之分, 标有JetAutoPro的表示需要以JetAutoPro形态才能完成

#1 关闭app自启功能
sudo systemctl disable start_app_node.service 

#2 停止app功能
sudo systemctl stop start_app_node.service

#3 开启app自启功能
sudo systemctl enable start_app_node.service

#4 开启app功能
sudo systemctl start start_app_node.service

#5 重启app功能 
sudo systemctl restart start_app_node.service

#6 查看app后台自启状态
sudo systemctl status start_app_node.service

#7 深度摄像头红外标定
roslaunch jetauto_calibration astrapro_ir_camera_calibration.launch

#8 深度摄像头RGB标定
roslaunch jetauto_calibration astrapro_rgb_camera_calibration.launch

#9 角速度校准
roslaunch jetauto_calibration calibrate_angular.launch

#10 线速度校准
roslaunch jetauto_calibration calibrate_linear.launch

#11 imu校准
roslaunch jetauto_calibration calibrate_imu.launch

#12 usb_cam标定
roslaunch jetauto_calibration usb_camera_calibration.launch

#13 apriltag检测
roslaunch jetauto_example apriltag_detect_demo.launch use_depth_cam:=true
#roslaunch jetauto_example apriltag_detect_demo.launch use_depth_cam:=false

#14 ar检测
roslaunch jetauto_example ar_detect_demo.launch use_depth_cam:=true
#roslaunch jetauto_example ar_detect_demo.launch use_depth_cam:=false

#15 深度摄像头红外可视化
roslaunch jetauto_example astrapro_ir_view.launch

#16 深度摄像头点云可视化
roslaunch jetauto_example astrapro_point_cloud_view.launch

#17 深度摄像头RGB图像可视化
roslaunch jetauto_example astrapro_rgb_view.launch

#18 单目orb_slam2
roslaunch jetauto_example orb_slam2_mono.launch

#19 深度orb_slam2
roslaunch jetauto_example orb_slam2_rgbd.launch

#20 单目orb_slam3
roslaunch jetauto_example orb_slam3_mono.launch

#21 深度orb_slam3
roslaunch jetauto_example orb_slam3_rgbd.launch

#22 肢体姿态融合RGB控制
roslaunch jetauto_example body_and_rgb_control.launch

#23 肢体姿态控制
roslaunch jetauto_example body_control.launch

#24 人体跟踪
roslaunch jetauto_example body_track.launch

#25 跌倒检测 
roslaunch jetauto_example fall_down_detect.launch

#26 颜色识别
roscd jetauto_example/scripts/color_detect && python3 color_detect.py

#27 颜色分拣(JetAutoPro)
roslaunch jetauto_example color_sorting_node.launch

#28 颜色追踪
roslaunch jetauto_example color_track_node.launch use_depth_cam:=true
#roslaunch jetauto_example color_track_node.launch use_depth_cam:=false

#29 垃圾分类(JetAutoPro)
roslaunch jetauto_example garbage_classification.launch

#30 手部跟随
roslaunch jetauto_example hand_track_node.launch use_depth_cam:=true
#roslaunch jetauto_example hand_track_node.launch use_depth_cam:=false

#31 循线清障(JetAutoPro)
roslaunch jetauto_example line_follow_clean_node.launch

#32 颜色追踪拾取 
roslaunch jetauto_example automatic_pick.launch

# 抓取
rosservice call /automatic_pick/pick 
# 放置
rosservice call /automatic_pick/place

#33 导航搬运
roslaunch jetauto_example navigation_transport.launch map:=地图名称

#34 二维码生成
roscd jetauto_example/scripts/qrcode && python3 qrcode_creater.py

#35 二维码检测
roscd jetauto_example/scripts/qrcode && python3 qrcode_detecter.py

#36 无人驾驶(JetAuto)
roslaunch jetauto_example self_driving.launch

#37 物体跟踪
roslaunch jetauto_example object_tracking.launch

#38 人脸检测 
roscd jetauto_example/scripts/mediapipe_example && python3 face_detect.py

#39 人脸网格
roscd jetauto_example/scripts/mediapipe_example && python3 face_mesh.py

#40 指尖轨迹
roslaunch jetauto_example hand_trajectory_node.launch

#41 手关键点检测
roscd jetauto_example/scripts/mediapipe_example && python3 hand.py

#42 肢体关键点检测
roscd jetauto_example/scripts/mediapipe_example && python3 pose.py

#43 背景分割
roscd jetauto_example/scripts/mediapipe_example && python3 self_segmentation.py

#44 整体检测
roscd jetauto_example/scripts/mediapipe_example && workon mediapipe && python3 holistic.py
#(使用完毕后需要退出虚拟环境，输入指令deactivate)

#45 3D物体检测
roscd jetauto_example/scripts/mediapipe_example && workon mediapipe && python3 objectron.py
#(使用完毕后需要退出虚拟环境，输入指令deactivate)

#46 建图
# 建图
roslaunch jetauto_slam slam.launch
# rviz查看建图效果
roslaunch jetauto_slam rviz_slam.launch
# 键盘控制(可选)
roslaunch jetauto_peripherals teleop_key_control.launch

#47 保存地图
roscd jetauto_slam/maps && rosrun map_server map_saver map:=/jetauto_1/map -f 保存名称

#48 导航
# 导航
roslaunch jetauto_navigation navigation.launch map:=地图名称
# rviz发布导航目标
roslaunch jetauto_navigation rviz_navigation.launch

# 多点导航
roslaunch jetauto_navigation publish_point.launch

#49 3D建图
# 3D建图
roslaunch jetauto_slam slam.launch slam_methods:=rtabmap
# rviz查看建图效果
roslaunch jetauto_slam rviz_slam.launch slam_methods:=rtabmap

#50 3D导航
# 3D导航
roslaunch jetauto_navigation rtabmap_navigation.launch
# rviz发布导航目标
roslaunch jetauto_navigation rviz_rtabmap_navigation.launch

# 多点导航
roslaunch jetauto_navigation publish_point.launch

#51 多机群控
roslaunch jetauto_multi multi_control.launch

#52 多机建图
#master 
roscd jetauto_multi/launch/multi_slam && roslaunch master_node.launch
roslaunch jetauto_multi multi_slam_rviz.launch

#host   
roscd jetauto_multi/launch/multi_slam && roslaunch slave_node.launch

#53 多机导航
#master 
roscd jetauto_multi/launch/multi_navigation && roslaunch master_node.launch map:=地图名称
roslaunch jetauto_multi multi_navigation_rviz.launch

#host   
roscd jetauto_multi/launch/multi_navigation && roslaunch slave_node.launch

#55 多机编队
#master 
roscd jetauto_multi/launch/multi_formation && roslaunch master_node.launch map:=地图名称
roslaunch jetauto_multi multi_formation_rviz.launch

#host   
roscd jetauto_multi/launch/multi_formation && roslaunch slave_node.launch

#56 多机环绕
#master 
roscd jetauto_multi/launch/multi_surround && roslaunch master_node.launch map:=地图名称
roslaunch jetauto_multi multi_surround_rviz.launch

#host   
roscd jetauto_multi/launch/multi_surround && roslaunch slave_node.launch

#57 urdf可视化
roslaunch jetauto_description display.launch

#58 gazebo可视化
roslaunch jetauto_gazebo worlds.launch

#59 moveit(JetAutoPro)
# 仅仿真 
roslaunch jetauto_moveit_config demo.launch

# 和真实联动
roslaunch jetauto_moveit_config demo.launch fake_execution:=false

# 和gazebo联动
roslaunch jetauto_moveit_config demo_gazebo.launch

#60 仿真建图
# gazebo仿真
roslaunch jetauto_gazebo room_worlds.launch
roslaunch jetauto_slam slam.launch sim:=true
roslaunch jetauto_slam rviz_slam.launch sim:=true
# 键盘控制(可选)
roslaunch jetauto_peripherals teleop_key_control.launch

#61 仿真导航
roslaunch jetauto_gazebo room_worlds.launch
roslaunch jetauto_navigation navigation.launch sim:=true map:=地图名称
roslaunch jetauto_navigation rviz_navigation.launch sim:=true

# 多点导航
roslaunch jetauto_navigation publish_point.launch

#62 语音控制移动
roslaunch xf_mic_asr_offline voice_control_move.launch

#63 语音控制导航
roslaunch xf_mic_asr_offline voice_control_navigation.launch map:=地图名称

#64 语音控制颜色检测
roslaunch xf_mic_asr_offline voice_control_color_detect.launch use_depth_cam:=true
#roslaunch xf_mic_asr_offline voice_control_color_detect.launch use_depth_cam:=false

#65 语音控制颜色分拣(JetAutoPro)
roslaunch xf_mic_asr_offline voice_control_color_sorting.launch

#66 语音控制颜色跟踪
roslaunch xf_mic_asr_offline voice_control_color_track.launch use_depth_cam:=true
#roslaunch xf_mic_asr_offline voice_control_color_track.launch use_depth_cam:=false

#67 语音控制机械臂(JetAutoPro)
roslaunch xf_mic_asr_offline voice_control_arm.launch

#68 语音控制垃圾分类(JetAutoPro)
roslaunch xf_mic_asr_offline voice_control_garbage_classification.launch

#69 语音控制导航搬运(JetAutoPro)
roslaunch xf_mic_asr_offline voice_control_navigation_transport.launch map:=地图名称

#70 手势控制(JetAutoPro)
roslaunch jetauto_example hand_gesture_control_node.launch
