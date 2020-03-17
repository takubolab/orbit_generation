# orbit_generation
## 軌道生成を行う手順

MoveItを立ち上げる．
```
roslanch daihen_ur5_moveit_config demo.launch
```
軌道生成を行う
```
rosrun orbit_generation orbit_generation.cpp
```
## プログラムの説明

棚の位置を変更する場合は87,88行目の
```
private_nh.param<double>("x", shelf_pose.pose.position.x, 1.2 + 0.1 - (h * 0.01));
private_nh.param<double>("y", shelf_pose.pose.position.y, -0.25 + (g * 0.01));
```
の数値を変更する

目標位置の変更は119〜135行目までのtarget_pose1.positionの値を変更する
