//poseUtils.h
#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <unordered_set>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>



struct Pose {
    std::string frame_id;
    float x, y, z;
    float roll, pitch, yaw;
};

std::vector<Pose> loadPoses(const std::string& path);

Eigen::Matrix3f eulerToRotMatrix(float roll, float pitch, float yaw);
Eigen::Matrix4f poseToMatrix(const Pose& pose);

void printPoseQuaternions(const std::vector<Pose>& poses, const std::string& output_path);


