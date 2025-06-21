// processImage.cpp

#include <vector>
#include <cmath>
#include <limits>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <unordered_set>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "poseUtils.h"  

using PointT = pcl::PointXYZRGB;
using CloudT = pcl::PointCloud<PointT>;

// 相机内参
float scale = 1.0f;
float cam_fx = 466.02369f * scale;
float cam_fy = 466.29222f * scale;
float cam_cx = 629.79974f * scale;
float cam_cy = 510.74402f * scale;

// 相机-雷达外参
Eigen::Matrix3f R_cl;
Eigen::Vector3f P_cl;

int main() {
    // 路径配置
    std::string camera_input = "D:/3DGS/Light/cam.txt";
    std::string camera_output = "D:/3DGS/2DGS/2d-gaussian-splatting/data9/sparse/0/cameras.txt";
    std::string image_dir = "D:/3DGS/2DGS/2d-gaussian-splatting/data9/images/";
    std::string pointcloud_file = "D:/3DGS/Scene/1.pcd";
    std::string pose_file = "D:/3DGS/Scene/pose_data4.txt";
    std::string image_output_file = "D:/3DGS/Scene/images.txt";
    std::string mapping_file_path = "D:/3DGS/Scene/pixel_point_map.txt";
    std::string points3D_file_path = "D:/3DGS/2DGS/2d-gaussian-splatting/data9/sparse/0/points3D.txt";

    std::ofstream mapping_file(mapping_file_path);
    std::ofstream points3D_file(points3D_file_path);

    // 初始化相机-雷达外参
    R_cl << -0.0303, -0.9995, -0.0047,
        0.4332, -0.0089, -0.9012,
        0.9008, -0.0293, 0.4333;
    P_cl << 0.0475, -0.09528, 0.01258;

    // 转换 camera 文件
    void convertCameraFile(const std::string & input_path, const std::string & output_path);
    convertCameraFile("D:/3DGS/Light/cam.txt", "D:/3DGS/2DGS/2d-gaussian-splatting/data9/sparse/0/cameras.txt");

    // 加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pointcloud_file, *cloud_raw) == -1) {
        std::cerr << "点云加载失败: " << pointcloud_file << std::endl;
        return -1;
    }

    // 加载位姿
    std::vector<Pose> poses = loadPoses(pose_file);
    if (poses.empty()) {
        std::cerr << "没有读取到有效位姿数据" << std::endl;
        return -1;
    }

    CloudT::Ptr merged_cloud(new CloudT);
    printPoseQuaternions(poses, image_output_file);

    std::unordered_set<size_t> written_point_indices;
    std::string current_frame_id;
    bool first_pixel = true;

    std::unordered_map<size_t, int> point_index_map;  // 原始点索引 → 合并点云索引
    for (const auto& pose : poses) {
        std::string image_path = image_dir + "image_" + pose.frame_id + ".png";
        int img_w, img_h, channels;
        unsigned char* img_data = stbi_load(image_path.c_str(), &img_w, &img_h, &channels, 3);
        if (!img_data) {
            std::cerr << "图像加载失败: " << image_path << std::endl;
            continue;
        }

        Eigen::Matrix4f T_global_to_lidar = poseToMatrix(pose);
        Eigen::Matrix4f T_lidar_to_global = T_global_to_lidar.inverse();

        Eigen::Vector4f P_cl_h; P_cl_h << P_cl, 1.0f;
        Eigen::Vector3f cam_pos = (T_global_to_lidar * P_cl_h).head<3>();

        std::vector<float> min_depth(img_w * img_h, std::numeric_limits<float>::max());
        std::vector<int> pixel_to_point(img_w * img_h, -1);

        if (pose.frame_id != current_frame_id) {
            if (!current_frame_id.empty()) mapping_file << "\n";
            current_frame_id = pose.frame_id;
            mapping_file << current_frame_id;
            first_pixel = true;
        }

        for (size_t i = 0; i < cloud_raw->points.size(); ++i) {
            if (written_point_indices.count(i)) continue;

            const auto& pt = cloud_raw->points[i];
            Eigen::Vector3f pt_world(pt.x, pt.y, pt.z);
            float dist = (pt_world - cam_pos).norm();

            Eigen::Vector4f pt_global(pt.x, pt.y, pt.z, 1.0f);
            Eigen::Vector3f pt_lidar = (T_lidar_to_global * pt_global).head<3>();
            Eigen::Vector3f pt_cam = R_cl * pt_lidar + P_cl;

            if (pt_cam.z() <= 0) continue;

            int u = static_cast<int>(std::round(pt_cam.x() / pt_cam.z() * cam_fx + cam_cx));
            int v = static_cast<int>(std::round(pt_cam.y() / pt_cam.z() * cam_fy + cam_cy));

            int target_width = 150;   // 宽度范围
            int target_height = 200;  // 高度范围

            // 中心点
            int center_u = img_w / 2;
            int center_v = img_h / 2;

            // 区域边界
            int u_min = center_u - target_width / 2;
            int u_max = center_u + target_width / 2;
            int v_min = center_v - target_height / 2;
            int v_max = center_v + target_height / 2;

            // 判断像素点是否落在范围内
            if (u < u_min || u >= u_max || v < v_min || v >= v_max) continue;

            int pixel_idx = v * img_w + u;

            if (dist < min_depth[pixel_idx]) {
                min_depth[pixel_idx] = dist;

                int global_index;
                auto it = point_index_map.find(i);
                if (it == point_index_map.end()) {
                    PointT colored;
                    colored.x = pt.x;
                    colored.y = pt.y;
                    colored.z = pt.z;

                    int offset = (v * img_w + u) * 3;
                    colored.r = img_data[offset];
                    colored.g = img_data[offset + 1];
                    colored.b = img_data[offset + 2];

                    global_index = static_cast<int>(merged_cloud->points.size());
                    point_index_map[i] = global_index;
                    merged_cloud->points.push_back(colored);

                    points3D_file << global_index << " " << pt.x << " " << pt.y << " " << pt.z << " "
                        << static_cast<int>(colored.r) << " " << static_cast<int>(colored.g) << " "
                        << static_cast<int>(colored.b) << " 0\n";
                }
                else {
                    global_index = it->second;
                }

                if (!first_pixel) mapping_file << " ";
                mapping_file << " " << u << " " << v << " " << global_index;
                first_pixel = false;
            }
        }

        stbi_image_free(img_data);
        std::cout << "处理完成帧: " << pose.frame_id
            << "，当前累计点数: " << merged_cloud->points.size() << std::endl;
    }

    merged_cloud->width = merged_cloud->points.size();
    merged_cloud->height = 1;
    merged_cloud->is_dense = false;
    pcl::io::savePCDFileBinary("D:/3DGS/Scene/6_20.pcd", *merged_cloud);
    std::cout << "完整点云保存为 610.pcd" << std::endl;

    mapping_file.close();
    points3D_file.close();

    // 合并 images.txt 与 pixel_point_map.txt
    std::ifstream imagesFile(image_output_file);
    std::ifstream mappingFile(mapping_file_path);
    std::ofstream outputFile("D:/3DGS/2DGS/2d-gaussian-splatting/data9/sparse/0/images.txt");

    if (!imagesFile || !mappingFile || !outputFile) {
        std::cerr << "文件打开失败，请检查路径。\n";
        return -1;
    }

    std::string imageLine, mappingLine;
    if (std::getline(imagesFile, imageLine)) {
        outputFile << imageLine << "\n";
    }

    while (std::getline(imagesFile, imageLine) && std::getline(mappingFile, mappingLine)) {
        outputFile << imageLine << "\n";
        std::istringstream iss(mappingLine);
        std::string frame_id, rest;
        iss >> frame_id;
        std::getline(iss, rest);
        rest.erase(0, rest.find_first_not_of(" \t"));
        outputFile << rest << "\n";
    }

    std::cout << "合并完成，输出文件为 images.txt\n";
    return 0;
}
