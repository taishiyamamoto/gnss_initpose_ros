# gnss_initpose_ros  
## 概要  
* [utm_odometry_node](http://wiki.ros.org/gps_common)を用いてUTM座標を取得し，任意の座標を原点として座標変換を行います．  

## ノード
### utm_initpose
#### Subscribed topics  
* fix(sensor_msgs/NavSatFix)  
#### Published topics  
* gps/odom(nav_msgs/Odometry)  
* gps/position(geometry_msgs/PoseWithCovarianceStamped)  
* initialpose(geometry_msgs/PoseWithCovarianceStamped)  
#### Parameters  
* ~map_frame(string, default: map)  
* ~robot_frame(string, default: base_link)  
* ~datum(double[3], default: NULL)  
#### Services  
* utm_initpose(std_srvs/Trigger)

## 環境構築  
```
sudo apt-get install ros-${ROS_DISTRO}-gps-common
```  
```
ca ~/catkin_ws/src
git clone https://github.com/taishiyamamoto/gnss_initpose_ros.git
catkin build gnss_initpose_ros
```

## 実行方法
```
roslaunch gnss_initpose_ros utm_initpose.launch
```

## 使用方法  
* まず，map_frameのUTM座標を[パラメータファイル](https://github.com/taishiyamamoto/gnss_initpose_ros/blob/master/config/utm_initpose_node.yaml)に入力する必要があります．配列になっていて，先頭からUTMのx座標[m],UTMのy座標[m],offset[rad]になっています．このパラメータを地図のデータに合わせて変更してください．  
* 変更したら，launchファイルを実行します．rvizを実行し，gps/position(geometry_msgs/PoseWithCovarianceStamped)が想定した挙動をすることを確認してください．
```
roslaunch gnss_initpose_ros utm_initpose.launch  
rviz
```  
* 本パッケージにはrvizを実行するとサブスクライブされる/initialposeトピックにGNSSの位置情報を送るサービス通信を実装しています． [amcl](http://wiki.ros.org/amcl)のように/initialposeトピックをサブスクライブできる機能があれば， GNSSの位置を反映することができます．
```
rosservice call utm_initpose {0}
```
