# teleop_dual_arm

sample.rosinstall見ながら環境構成&`rosdep install -y --from-paths . --ignore-src`

# Gazebo起動
```
reset; pkill -9 gzserver; roslaunch teleop_dual_arm gazebo_iiwa14d_no_controllers.launch
```

# hrpsys起動
```
reset; rtmlaunch teleop_dual_arm iiwa14d_hrpsys_bringup.launch
```
現状存在しないRTCなどに対してWait for *** が出るので，しばらく待つ必要がある

# RViz起動
```
rviz -d `rospack find teleop_dual_arm`/iiwa14d.rviz
```

# IKデモ起動
```
rosrun teleop_dual_arm sample_ik_call.py
```

# xacro編集＆確認
改造したiiwa14dのxacroはiiwa_descriptionに置いてある
```
roslaunch urdf_tutorial display.launch model:=`rospack find iiwa_description`/urdf/iiwa14d.urdf.xacro gui:=false
```

# モデル再生成
iiwa_descriptionのxacroから諸々生成しているので
```
catkin bt --force-cmake
```
でteleop_dual_arm内の.urdfと.daeが手動で再生成できる

# 注意事項
- iiwa7   = KUKA LBR iiwa  7kg 可搬モデル
- iiwa14  = KUKA LBR iiwa 14kg 可搬モデル
- iiwa14d = iiwa14をdualに取り付けたJSK改造モデル
- iiwad   = dualなiiwaに関する設定のために追加
- iiwa_descriptionの慣習により，**_joint_0は土台としてのfixedジョイント

# TODO
- OpenHRPのModelLoader系関数を独立で呼ぶ
- iiwa14dのxacroを完全新規作成でなく，片手ずつスマートに継承して書けないか
- EffortJointControlへ移行＋トルク司令
- カメラ追加
- 机とか対象物とか
- OpenPoseとかでノートPCのインカムでも両手Pose入力できないか
