# 题三（以及题一/题二）的容器环境：Isaac Sim 4.5 + ROS 2 Humble。
#
# 别人复刻时只需要这一条命令构建（无需本仓库之外的文件）：
#   docker build -f src/question3_project/Dockerfile -t bajie/sim:humble45 src/question3_project
#
# 为什么以官方 Isaac Sim 镜像为底座：Isaac Sim 的 ROS 2 Bridge 是针对镜像自带的
# Python 3.10 / Omniverse 运行时预编译的，自己装一遍 Isaac Sim 只会引入版本错配。
FROM nvcr.io/nvidia/isaac-sim:4.5.0

USER root
SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8

# 官方镜像的 ENTRYPOINT 是 /isaac-sim/runheadless.sh，会把 docker run 的命令吞掉
# （docker run image bash 实际执行 runheadless.sh bash）。清空后由启动器显式调用
# /isaac-sim/python.sh。
ENTRYPOINT []

RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg lsb-release ca-certificates locales \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

# ---- ROS 2 Humble ----
# 两处妥协来自实测：容器内访问 https://packages.ros.org 会被中间证书拦截，
# 且官方 ros.key 已过期（EXPKEYSIG），因此走 http + trusted=yes。
# 网络允许时可改回 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg 的 https 源。
RUN echo "deb [trusted=yes arch=amd64] http://packages.ros.org/ros2/ubuntu jammy main" \
        > /etc/apt/sources.list.d/ros2.list \
    && apt-get update -o Acquire::Retries=3 && apt-get install -y --no-install-recommends \
        ros-humble-ros-base \
        ros-humble-xacro \
        ros-humble-urdf \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        ros-humble-tf2-tools \
        ros-humble-rmw-fastrtps-cpp \
        ros-humble-rosbag2 \
        python3-colcon-common-extensions \
        build-essential cmake pkg-config \
        python3-numpy \
        python3-scipy \
        git \
    && rm -rf /var/lib/apt/lists/*

# python3-scipy 是题三规划的真实依赖（scipy.optimize / scipy.spatial / Rotation）。
# Isaac Sim 自带 Python 已有 scipy 1.10.1，但容器里的系统 Python 没有；
# 缺了它 run_tests.sh 的题三用例会整片报 ModuleNotFoundError。

RUN printf '%s\n' \
    'source /opt/ros/humble/setup.bash' \
    'export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}' \
    > /etc/profile.d/99-ros-humble.sh \
    && echo '[ -f /etc/profile.d/99-ros-humble.sh ] && . /etc/profile.d/99-ros-humble.sh' >> /root/.bashrc

WORKDIR /workspace
CMD ["/bin/bash"]
