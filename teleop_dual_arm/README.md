# teleop_dual_arm

sample.rosinstall見ながら環境構成&`rosdep install -y --from-paths . --ignore-src`

OpenHRPの運動学・動力学関数を使うために，iiwa_stackからxacro→urdf→colladaとOpenHRPで読める形式に変換
```
roscd teleop_dual_arm
rosrun xacro xacro `rospack find iiwa_description`/urdf/iiwa14.urdf.xacro > iiwa14.urdf
rosrun collada_urdf urdf_to_collada iiwa14.urdf iiwa14.dae
```

Gazebo起動
```
reset; pkill -9 gzserver; roslaunch teleop_dual_arm demo.launch
```

test (IKは解いてない)
```
rosrun teleop_dual_arm ik_solver
```

hrpsys起動
```
reset; rtmlaunch teleop_dual_arm iiwa14.launch
```

TODO
- hrpsysとGazeboはまだ独立に動いてるだけ
- OpenHRPのModelLoader系関数を独立で呼ぶ
- jointGroupが設定しきれてないからsetTargetPose()が呼べない
- 両手iiwa14モデルの名前をiiwa14-dualとかに移行
- 両手iiwa14のxacroを完全新規作成でなく，片手ずつスマートに継承して書けないか
- catkin bt でiiwa_stackのxacro→urdf→colladaを配置(権利的にポリゴンモデルを置きたくない)
- EffortJointControlへ移行＋トルク司令
- カメラ追加
- 机とか対象物とか
- OpenPoseとかでノートPCのインカムでも両手Pose入力できないか
