FROM osrf/ros:noetic-desktop-full

WORKDIR /root/

# キーボード設定の選択を聞かれないようにする
ARG DEBIAN_FRONTEND=noninteractive

# apt-getのアップデートとアップグレード
RUN apt-get update -y && apt-get install -y \
    vim \
    wget \
    tmux \
    terminator && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install ROS packages
# RUN apt-get install -y \
#   ros-noetic-

# Create ROS Workspace
# ROSのセットアップシェルスクリプトを.bashrcファイルに追記
RUN echo "source /opt/ros/noetic/setup.sh" >> .bashrc
# 自分のワークスペース作成のためにフォルダを作成
RUN mkdir -p catkin_ws/src
# srcディレクトリまで移動して，catkin_init_workspaceを実行．
RUN cd catkin_ws && . /opt/ros/noetic/setup.sh && catkin_make
# 自分のワークスペースが反映されるように，.bashrcファイルに追記．
RUN echo "source ./catkin_ws/devel/setup.bash" >> .bashrc