# harurobo2022
春ロボ2022用リポジトリ\
[春ロボHP](https://kantouharurobo.com/haru/entry)\
![robotmodel](/image/robot.png)
## 環境構築
```bash
sudo apt-get install ros-melodic-effort-controllers
sudo apt-get install ros-melodic-joint-state-controller
sudo apt-get install ros-melodic-position-controllers
sudo apt install ros-melodic-ros-control
sudo apt install ros-melodic-ros-controllers
```
```bash
sudo apt-get install ros-melodic-teleop-twist-keyboard
```
## キーボード操作
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
## Serial port 確認
```bash
ls -l /dev/serial/by-id/
```
## USB Serial
```bash
rosrun rosserial_python serial_node.py /dev/ttyUSB0 _baud:=115200
```
## IMU
```bash
rosrun um7 um7_driver _port:=/dev/ttyACM0
```
## LiDAR
```bash
rosrun urg_node urg_node _serial_port:=/dev/ttyACM0
```
## Gazebo
スタートゾーンの色が赤の場合
```bash
roslaunch harurobo2022 gazebo.launch Field:=Red
```
スタートゾーンの色が青の場合
```bash
roslaunch harurobo2022 gazebo.launch Field:=Blue
```
### オブジェクトありの場合
```bash
roslaunch harurobo2022 gazebo.launch Field:=RedObj
```
```bash
roslaunch harurobo2022 gazebo.launch Field:=BlueObj
```
## オムニ三輪コントローラー
gazebo.launchで実行される
```bash
rosrun harurobo2022 omuni3controller.py
```
## モデルの表示
xacroディレクトリに移動し以下のコマンドを実行
```bash
roslaunch urdf_tutorial display.launch model:=OmuniRobot.xacro
```
## navigation
スタートゾーンの色が赤の場合
```bash
roslaunch harurobo2022 navi.launch Field:=Red
```
スタートゾーンの色が青の場合
```bash
roslaunch harurobo2022 navi.launch Field:=Blue
```
## 経路生成
スタートゾーンの色が赤の場合
```bash
roslaunch harurobo2022 PathGenerator.launch Field:=Red
```
スタートゾーンの色が青の場合
```bash
roslaunch harurobo2022 PathGenerator.launch Field:=Blue
```
## csvの経路配信
スタートゾーンの色が赤の場合
```bash
rosrun harurobo2022 PathPublisher.py r
```
スタートゾーンの色が青の場合
```bash
rosrun harurobo2022 PathPublisher.py b
```
## mapping
```bash
 roslaunch harurobo2022 gmapping.launch
```
保存
```bash
rosrun map_server map_saver -f ファイル名
```
## 問題対処法メモ
フィールドを指定した場合にworldに何も表示されないときは以下のコマンドを入力
```bash
export GAZEBO_RESOURCE_PATH=`pwd`
```
