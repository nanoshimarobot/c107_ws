# 自走式3D LiDARによるお手軽Graph-based SLAM配布用ROS 2 ws

## 概要

本リポジトリは，コミックマーケット107 サークル「やる気出ないンズ」にて頒布された技術合同誌の記事「自走式3D LiDARによるお手軽Graph-based SLAM」で用いるROS 2ワークスペースです．  
内容は不定期的に更新されます．

## ROS 2 install

[こちら](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)を参考に，ROS 2 jazzyをインストール

## セットアップ

依存関係のインストール

```bash
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

```bash
cd ~/c107_ws
. install/local_setup.bash
ros2 launch c107_bringup slam_demo.launch
```