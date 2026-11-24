cmake_minimum_required(VERSION 3.8)
project(bajie_sim)

find_package(ament_cmake REQUIRED)

# 四个目录必须保持同层相对结构：scene/abr_scene.usd 内部用 ../models/... 与
# ../machine/... 引用资源，任何一层被拍平都会导致场景打不开。
# scripts/ 是官方提供的光心坐标转换脚本，一并安装以便 ros2 run 之外的直接调用。
install(DIRECTORY scene machine models scripts
        DESTINATION share/${PROJECT_NAME})

ament_package()
