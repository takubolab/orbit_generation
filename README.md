# orbit_generation
## はじめに
このパッケージは[base_placement_plannner](https://github.com/takubolab/base_placement_planner)で用いる参照軌道の生成を行う

その際、生成したbagファイルは自動的にbagファイル内にフォルダーを作成して保存される
## 軌道生成を行う手順
MoveItを立ち上げる．
```
roslaunch daihen_ur5_moveit_config demo.launch
```
軌道生成を行う
```
rosrun orbit_generation orbit_generation
```
## プログラム内で書き換える可能性のある箇所について説明
### 棚の位置、目標位置の変更方法
#### 棚の位置の変更について
87,88行目の
```
private_nh.param<double>("x", shelf_pose.pose.position.x, 1.2 + 0.1 - (h * 0.01));
private_nh.param<double>("y", shelf_pose.pose.position.y, -0.25 + (g * 0.01));
```
の数値を変更することによって、棚の位置は変更できる

初期設定ではｘが1.1 〜 1.3の範囲で、yが−2.5 〜 2.5の範囲内で移動するように設定している

(x，yの範囲は拡大してもいいが、これよりも大きくなると軌道生成が非常に困難である。また、xは1.0より小さくなると姿勢や棚の形によっては貫通する可能性があるため注意）

#### 目標位置の変更について
107〜121行目までのtarget_pose1.positionの値（目標位置）のx,y,zをそれぞれ変更することによって目標位置を決定する

棚内部の作業領域の寸法は松本さんの修論より、奥行き 400[mm]× 幅 900[mm]× 高さ 500[mm]であるため、範囲内で収まるように設定する
（仮に範囲を超えた場合も貫通はせずにplanが失敗するだけであるだけである）

また初期の目標位置の設定は[base_placement_plannner](https://github.com/takubolab/base_placement_planner)の目標位置のみに設定してあり、棚を動かして擬似的に様々な位置から目標位置に軌道生成を行うようになっている

### bagファイルの保存先の設定
生成したbagファイルは自動的にbagファイル内にプログラム内のstrの名前のフォルダーが生成される

そのため保存先のファイル名を変更する際には31行目の
```
 std::string str("trajectory"); 
```
の""内の名前を変更する

また保存先のパッケージ自体を変更する場合は68行目の
```
  directory_path << ros::package::getPath("orbit_generation") + "/bag/"+ str;
```
を変更すればいい

### ループについての説明
棚の位置や目標位置の変更のため73行目以降に多くのforループが設定されている

それらはそれぞれ用途が異なるため説明する

#### 73,75行目のfor文のh,gについて
hは棚の奥行きの移動回数のループであり、gは棚の左右の移動回数のループである

また、h,gの回数は87,88行目の
```
private_nh.param<double>("x", shelf_pose.pose.position.x, 1.2 + 0.1 - (h * 0.01));
private_nh.param<double>("y", shelf_pose.pose.position.y, -0.25 + (g * 0.01));
```
の大きさに依存し、hはｘが1.1 〜 1.3の範囲で、gはyが−2.5 〜 2.5の範囲内で移動できるような大きさの数にする

#### 94,96,98行目のfor文のi,j,kについて
iは目標位置のx座標（奥行き）で、jは目標位置のy座標（左右）で、kは目標位置のz座標（高さ）の移動回数のループである

またこちらも107〜121行目までのtarget_pose1.positionの値（目標位置）の依存し、棚内部の作業領域の寸法は松本さんの修論より奥行き 400[mm]× 幅 900[mm]× 高さ 500[mm]であるため、範囲内で収まるように設定する

（仮に範囲を超えた場合も貫通はせずにplanが失敗するだけであるだけである）

### 初期姿勢についての説明
軌道生成を行う際に初期姿勢によって軌道が変化する
よって別の姿勢で軌道生成を行う場合は102行目の
```
move_group.setNamedTarget("masita4"); 
```
を変更したい姿勢に変更する

また姿勢変更を行う際、180,184行目は手先が目標位置にある状態で棚の位置変更を行った場合、棚と貫通してしまう可能性があるため、demo.launchの初期姿勢に戻しているので、変更はしなくても特に問題はありません

### plan,executeについての説明
このプログラムでは126行目のplanで軌道を生成し、134行目の
```
handler.saveTrajectoryToBag(plan, directory_path.str());
```
で軌道を保存している

これは[trajectory_data_handler](https://github.com/takubolab/trajectory_data_handler)の関数を用いている

そして137行目のexecuteで軌道を再生しているが、executeはなくても軌道の保存は可能であるため、executeの軌道の確認なしでの軌道生成のほうが動作が早い

