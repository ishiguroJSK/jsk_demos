# teleop_dual_arm

Gazebo起動
```
reset; pkill -9 gzserver; roslaunch teleop_dual_arm iiwa14d_gazebo.launch
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
- EffortJointControlへ移行＋トルク司令
- カメラ追加
- 机とか対象物とか
- OpenPoseとかでノートPCのインカムでも両手Pose入力できないか