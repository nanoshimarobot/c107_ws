# 自走式3D LiDARによるお手軽Graph-based SLAM配布用ROS 2 ws

## 概要

本リポジトリは，コミックマーケット107 サークル「やる気出ないンズ」にて頒布された技術合同誌の記事「自走式3D LiDARによるお手軽Graph-based SLAM」で用いるROS 2ワークスペースです．  
内容は不定期的に更新されます．

## ROS 2 install

[こちら](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)を参考に，ROS 2 jazzyをインストール

## セットアップ

ライブラリのインストール

```bash
# 依存関係の追加
sudo apt install libomp-dev libboost-all-dev libmetis-dev \
                 libfmt-dev libspdlog-dev \
                 ros-jazzy-teleop-twist-keyboard

# GTSAMのインストール
git clone https://github.com/borglab/gtsam
cd gtsam && git checkout 4.3a0
mkdir build && cd build
cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DGTSAM_BUILD_TESTS=OFF \
         -DGTSAM_WITH_TBB=OFF \
         -DGTSAM_USE_SYSTEM_EIGEN=ON \
         -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
make -j$(nproc)
sudo make install

# gtsam_pointsのインストール
git clone https://github.com/koide3/gtsam_points
mkdir gtsam_points/build && cd gtsam_points/build
cmake .. -DBUILD_WITH_CUDA=OFF # CUDAある人はONにしよう
make -j$(nproc)
sudo make install

sudo ldconfig
```

本リポジトリのビルド

```bash
cd ~
git clone https://github.com/nanoshimarobot/c107_ws
cd c107_ws
vcs import . < dependency.repos
rosdep install --from-paths . -y --ignore-src
colcon build --symlink-install
```

## 実行

ロボット通信モジュール，SLAMモジュールの起動
```bash
cd ~/c107_ws
. install/local_setup.bash
ros2 launch c107_bringup slam_demo.launch
```

別ターミナルでキーボード操作用ノードの起動(rosbag再生時は不要です)
```bash
ros2 run keyboard_controller keyboard_controller
```

## テストデータのdownload

[ここから](https://drive.google.com/drive/folders/1v3QyJXd6Q2tEKzYespL7Z3T1UbK3b7xD?usp=sharing)テスト用rosbagデータをダウンロードできます．

- rosbag2_2025_12_30-23_14_46
  どこかの実験室

- rosbag2_2025_12_30-23_14_46
  友人宅で取った廊下データ