# ydlidar_slam_toolbox
ydlidar Tmini Proを使ってSlamを動かすデモ

# ネイティブ環境

- OS: Ubuntu 22.04
- ROS: ROS 2 Humble Hawksbill

# 使用するLiDAR
[YDLIDAR T-mini Plus](https://www.switch-science.com/products/9751)

## YDLidarの設定
```bash
sudo apt install cmake pkg-config
sudo apt-get install python3 swig
sudo apt-get install python3-pip

git clone https://github.com/YDLIDAR/YDLidar-SDK.git
mkdir YDLidar-SDK/build
cd YDLidar-SDK/build
cmake ..
make
sudo make install

cd ~/YDLidar-SDK/
pip3 install .
```
[[ROS 2 Humble] YDLIDAR T-mini Plusを使う (備忘録)](https://zenn.dev/robohase01/articles/2d3886a4c100c7)


## リポジトリのクローンと依存関係のインストール

```bash
cd ~/
source /opt/ros/humble/setup.bash
sudo apt-get update
sudo apt install python3-rosdep2 git python3-colcon-common-extensions
rosdep update
git clone git@github.com:robohase/ydlidar_slam_toolbox.git
cd ydlidar_slam_toolbox

# 依存ROSパッケージインストール
vcs import --input depends.rosinstall --recursive src
rosdep install -r -y -i --from-paths .

sudo rosdep update

chmod 0777 src/ydlidar_ros2_driver/startup/*
sudo sh src/ydlidar_ros2_driver/startup/initenv.sh
```

## ビルド
```bash
colcon build --symlink-install
source install/setup.bash
```

# 起動方法
```bash
ros2 launch ydlidar_slam_toolbox cartgrapher.launch.py
```
---