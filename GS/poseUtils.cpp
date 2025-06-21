//poseUtils.cpp
#include <cmath>
#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip> 
#include <unordered_set>
#include <random>
#include <Eigen/Geometry> 
#include "poseUtils.h"



Eigen::Matrix3f eulerToRotMatrix(float roll, float pitch, float yaw) {
    roll *= M_PI / 180.0f;
    pitch *= M_PI / 180.0f;
    yaw *= M_PI / 180.0f;

    Eigen::Matrix3f Rz, Ry, Rx;
    Rz << cos(yaw), -sin(yaw), 0,
        sin(yaw), cos(yaw), 0,
        0, 0, 1;
    Ry << cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch);
    Rx << 1, 0, 0,
        0, cos(roll), -sin(roll),
        0, sin(roll), cos(roll);
    return Rz * Ry * Rx;
}

Eigen::Matrix4f poseToMatrix(const Pose& pose) {
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3, 3>(0, 0) = eulerToRotMatrix(pose.roll, pose.pitch, pose.yaw);
    T.block<3, 1>(0, 3) = Eigen::Vector3f(pose.x, pose.y, pose.z);
    return T;
}

std::vector<Pose> loadPoses(const std::string& path) {
    std::vector<Pose> poses;
    std::ifstream file(path);
    if (!file) {
        std::cerr << "无法打开位姿文件: " << path << std::endl;
        return poses;
    }

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream iss(line);
        Pose p;
        std::string timestamp;
        iss >> p.frame_id >> timestamp >> p.x >> p.y >> p.z >> p.roll >> p.pitch >> p.yaw;
        poses.push_back(p);
    }

    return poses;
}

void printPoseQuaternions(const std::vector<Pose>& poses, const std::string& output_path ) {
    std::ofstream out(output_path);

    Eigen::Matrix3f R_cl;
    Eigen::Vector3f P_cl;

    R_cl << -0.0303, -0.9995, -0.0047,
        0.4332, -0.0089, -0.9012,
        0.9008, -0.0293, 0.4333;
    P_cl << 0.0475, -0.09528, 0.01258;

    Eigen::Matrix4f T_cam_to_global = Eigen::Matrix4f::Identity();
    T_cam_to_global.block<3, 3>(0, 0) = R_cl;
    T_cam_to_global.block<3, 1>(0, 3) = P_cl;


    if (!out) {
        std::cerr << "无法创建 images.txt 文件: " << output_path << std::endl;
        return;
    }

 
    out << "# IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID IMAGE_NAME\n";

    for (size_t i = 0; i < poses.size(); ++i) {
        const Pose& pose = poses[i];

        int image_id = static_cast<int>(i + 1);
        int camera_id = 1;
        std::string image_name = "image_" + pose.frame_id + ".png";

        Eigen::Matrix4f T_g_to_lidar = poseToMatrix(pose);
        //Eigen::Matrix4f T = T_g_to_lidar * T_cam_to_global;  //修改后
        Eigen::Matrix4f T = T_cam_to_global * T_g_to_lidar;
        Eigen::Matrix3f R = T.block<3, 3>(0, 0);
        Eigen::Vector3f t = T.block<3, 1>(0, 3);
        Eigen::Quaternionf q(R);

        // 设置输出精度
        out << std::fixed << std::setprecision(17);

        out << image_id << " "
            << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << " "
            << t.x() << " " << t.y() << " " << t.z() << " "
            << camera_id << " "
            << image_name << "\n";
    }

    std::cout << "位姿数据已成功写入: " << output_path << std::endl;
}



