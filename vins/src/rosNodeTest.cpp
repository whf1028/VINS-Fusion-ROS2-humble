/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"
#include "logger/logger.h"

Estimator estimator;

queue<sensor_msgs::msg::Imu::ConstPtr> imu_buf;
queue<sensor_msgs::msg::PointCloud::ConstPtr> feature_buf;
queue<sensor_msgs::msg::Image::ConstPtr> img0_buf;
queue<sensor_msgs::msg::Image::ConstPtr> img1_buf;
std::mutex m_buf;

// header: 1403715278
void img0_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    static int img0_count = 0;
    img0_count++;
    m_buf.lock();
    size_t buf_size = img0_buf.size();

    // ========================================================================
    // 2025-11-19 VINS发散修复: 添加缓冲区大小限制
    // ========================================================================
    // 问题: 当处理速度<图像到达速度时, 队列无限增长 (观察到43/43峰值)
    // 影响: 内存暴涨 + 处理延迟累积 + 特征跟踪失败
    // 解决: 限制队列最大长度为8帧, 超出则丢弃最旧帧
    // 原理: 8帧 = 8/15fps = 0.53秒延迟上限, 避免处理过时数据
    // ========================================================================
    const size_t MAX_BUFFER_SIZE = 8;  // 最大缓冲区大小
    if (img0_buf.size() >= MAX_BUFFER_SIZE) {
        img0_buf.pop();  // 丢弃最老的帧
        RCLCPP_WARN(rclcpp::get_logger("vins"),
                    "LEFT buffer overflow (%zu frames), dropping oldest frame to prevent divergence",
                    img0_buf.size());
    }
    // ========================================================================
    // 原始代码 (无缓冲区限制, 已保留上下文)
    // ========================================================================
    // img0_buf.push(img_msg);  // 直接push, 无大小检查
    // 问题: 队列可无限增长, 导致缓冲=43/43时内存和延迟失控
    // ========================================================================

    if (img0_count % 30 == 0) {
        RCLCPP_INFO(rclcpp::get_logger("vins"), "Received LEFT image #%d, timestamp: %d.%09d, buffer size: %zu, img size: %dx%d",
                    img0_count, img_msg->header.stamp.sec, img_msg->header.stamp.nanosec,
                    buf_size, img_msg->width, img_msg->height);
    }
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_callback(const sensor_msgs::msg::Image::SharedPtr img_msg)
{
    static int img1_count = 0;
    img1_count++;
    m_buf.lock();
    size_t buf_size = img1_buf.size();

    // ========================================================================
    // 2025-11-19 VINS发散修复: 添加缓冲区大小限制 (与左相机一致)
    // ========================================================================
    const size_t MAX_BUFFER_SIZE = 8;  // 最大缓冲区大小
    if (img1_buf.size() >= MAX_BUFFER_SIZE) {
        img1_buf.pop();  // 丢弃最老的帧
        RCLCPP_WARN(rclcpp::get_logger("vins"),
                    "RIGHT buffer overflow (%zu frames), dropping oldest frame to prevent divergence",
                    img1_buf.size());
    }
    // ========================================================================

    if (img1_count % 30 == 0) {
        RCLCPP_INFO(rclcpp::get_logger("vins"), "Received RIGHT image #%d, timestamp: %d.%09d, buffer size: %zu, img size: %dx%d",
                    img1_count, img_msg->header.stamp.sec, img_msg->header.stamp.nanosec,
                    buf_size, img_msg->width, img_msg->height);
    }
    img1_buf.push(img_msg);
    m_buf.unlock();
}


// cv::Mat getImageFromMsg(const sensor_msgs::msg::Image::SharedPtr img_msg)
cv::Mat getImageFromMsg(const sensor_msgs::msg::Image::ConstPtr &img_msg)
{
    try {
        cv_bridge::CvImageConstPtr ptr;
        if (img_msg->encoding == "8UC1")
        {
            sensor_msgs::msg::Image img;
            img.header = img_msg->header;
            img.height = img_msg->height;
            img.width = img_msg->width;
            img.is_bigendian = img_msg->is_bigendian;
            img.step = img_msg->step;
            img.data = img_msg->data;
            img.encoding = "mono8";
            ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
        }
        else
            ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

        cv::Mat img = ptr->image.clone();
        if (img.empty()) {
            RCLCPP_ERROR(rclcpp::get_logger("vins"), "getImageFromMsg: converted image is empty!");
        }
        return img;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("vins"), "getImageFromMsg exception: %s", e.what());
        return cv::Mat();
    }
}

// extract images with same timestamp from two topics
void sync_process()
{
    static int sync_count = 0;
    while(1)
    {
        if(STEREO)
        {
            cv::Mat image0, image1;
            std_msgs::msg::Header header;
            double time = 0;
            m_buf.lock();
            size_t buf0_size = img0_buf.size();
            size_t buf1_size = img1_buf.size();

            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.sec + img0_buf.front()->header.stamp.nanosec * (1e-9);
                double time1 = img1_buf.front()->header.stamp.sec + img1_buf.front()->header.stamp.nanosec * (1e-9);
                double time_diff = time0 - time1;

                // 0.003s sync tolerance
                if(time0 < time1 - 0.003)
                {
                    img0_buf.pop();
                    RCLCPP_WARN(rclcpp::get_logger("vins"), "throw img0, time_diff: %.6f, buf sizes: img0=%zu img1=%zu",
                                time_diff, buf0_size, buf1_size);
                }
                else if(time0 > time1 + 0.003)
                {
                    img1_buf.pop();
                    RCLCPP_WARN(rclcpp::get_logger("vins"), "throw img1, time_diff: %.6f, buf sizes: img0=%zu img1=%zu",
                                time_diff, buf0_size, buf1_size);
                }
                else
                {
                    sync_count++;
                    time = img0_buf.front()->header.stamp.sec + img0_buf.front()->header.stamp.nanosec * (1e-9);
                    header = img0_buf.front()->header;

                    if (sync_count % 10 == 0) {
                        RCLCPP_INFO(rclcpp::get_logger("vins"), "Sync #%d: Converting images, time: %.6f, time_diff: %.6f",
                                    sync_count, time, time_diff);
                    }

                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop();

                    if (sync_count % 10 == 0) {
                        RCLCPP_INFO(rclcpp::get_logger("vins"), "Sync #%d: Images converted, img0 size: %dx%d, img1 size: %dx%d",
                                    sync_count, image0.cols, image0.rows, image1.cols, image1.rows);
                        VINS_LOG_THROTTLE(INFO, IMAGE_PROC, 1.0, "图像同步: #%d, t=%.6f, 双目图像大小=%dx%d, 时间差=%.6fs, 缓冲=%zu/%zu",
                                          sync_count, time, image0.cols, image0.rows, time_diff, buf0_size, buf1_size);
                    }
                }
            }
            m_buf.unlock();

            if(!image0.empty())
            {
                if (sync_count % 10 == 0) {
                    RCLCPP_INFO(rclcpp::get_logger("vins"), "Sync #%d: Calling estimator.inputImage() with time %.6f",
                                sync_count, time);
                }
                try {
                    estimator.inputImage(time, image0, image1);
                    if (sync_count % 10 == 0) {
                        RCLCPP_INFO(rclcpp::get_logger("vins"), "Sync #%d: estimator.inputImage() completed successfully",
                                    sync_count);
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(rclcpp::get_logger("vins"), "Sync #%d: estimator.inputImage() exception: %s",
                                 sync_count, e.what());
                } catch (...) {
                    RCLCPP_ERROR(rclcpp::get_logger("vins"), "Sync #%d: estimator.inputImage() unknown exception",
                                 sync_count);
                }
            }
        }
        else
        {
            cv::Mat image;
            std_msgs::msg::Header header;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.sec + img0_buf.front()->header.stamp.nanosec * (1e-9);
                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());
                img0_buf.pop();
            }
            m_buf.unlock();
            if(!image.empty())
            {
                try {
                    estimator.inputImage(time, image);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(rclcpp::get_logger("vins"), "Mono: estimator.inputImage() exception: %s", e.what());
                } catch (...) {
                    RCLCPP_ERROR(rclcpp::get_logger("vins"), "Mono: estimator.inputImage() unknown exception");
                }
            }
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


void imu_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    static int imu_count = 0;
    imu_count++;

    double t = imu_msg->header.stamp.sec + imu_msg->header.stamp.nanosec * (1e-9);
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);

    if (imu_count % 100 == 0) {
        RCLCPP_INFO(rclcpp::get_logger("vins"), "Received IMU #%d, timestamp: %.6f, acc: (%.3f, %.3f, %.3f)",
                    imu_count, t, dx, dy, dz);
        VINS_LOG_THROTTLE(INFO, IMU, 1.0, "IMU数据: #%d, t=%.6f, acc=(%.3f,%.3f,%.3f) m/s², gyr=(%.3f,%.3f,%.3f) rad/s",
                          imu_count, t, dx, dy, dz, rx, ry, rz);
    }
    estimator.inputIMU(t, acc, gyr);
    return;
}


void feature_callback(const sensor_msgs::msg::PointCloud::SharedPtr feature_msg)
{
    std::cout << "feature cb" << std::endl;
    std::cout << "Feature: " << feature_msg->points.size() << std::endl;


    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    for (unsigned int i = 0; i < feature_msg->points.size(); i++)
    {
        int feature_id = feature_msg->channels[0].values[i];
        int camera_id = feature_msg->channels[1].values[i];
        double x = feature_msg->points[i].x;
        double y = feature_msg->points[i].y;
        double z = feature_msg->points[i].z;
        double p_u = feature_msg->channels[2].values[i];
        double p_v = feature_msg->channels[3].values[i];
        double velocity_x = feature_msg->channels[4].values[i];
        double velocity_y = feature_msg->channels[5].values[i];
        if(feature_msg->channels.size() > 5)
        {
            double gx = feature_msg->channels[6].values[i];
            double gy = feature_msg->channels[7].values[i];
            double gz = feature_msg->channels[8].values[i];
            pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
            //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
        }
        assert(z == 1);
        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
        featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
    }
    double t = feature_msg->header.stamp.sec + feature_msg->header.stamp.nanosec * (1e-9);
    estimator.inputFeature(t, featureFrame);
    return;
}

void restart_callback(const std_msgs::msg::Bool::SharedPtr restart_msg)
{
    if (restart_msg->data == true)
    {
        ROS_WARN("restart the estimator!");
        estimator.clearState();
        estimator.setParameter();
    }
    return;
}

void imu_switch_callback(const std_msgs::msg::Bool::SharedPtr switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use IMU!");
        estimator.changeSensorType(1, STEREO);
    }
    else
    {
        //ROS_WARN("disable IMU!");
        estimator.changeSensorType(0, STEREO);
    }
    return;
}

void cam_switch_callback(const std_msgs::msg::Bool::SharedPtr switch_msg)
{
    if (switch_msg->data == true)
    {
        //ROS_WARN("use stereo!");
        estimator.changeSensorType(USE_IMU, 1);
    }
    else
    {
        //ROS_WARN("use mono camera (left)!");
        estimator.changeSensorType(USE_IMU, 0);
    }
    return;
}

int main(int argc, char **argv)
{
    printf("[INFO] [%ld] []: init begins\n", time(NULL));

    // 初始化日志系统
    vins::VinsLogger::getInstance().initialize("", "");
    VINS_LOG_INFO(SYSTEM, "========================================");
    VINS_LOG_INFO(SYSTEM, "VINS-Fusion-ROS2-humble 启动");
    VINS_LOG_INFO(SYSTEM, "========================================");

    rclcpp::init(argc, argv);
	auto n = rclcpp::Node::make_shared("vins_estimator");
    VINS_LOG_INFO(SYSTEM, "ROS2 节点 'vins_estimator' 已创建");
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    // In ROS2, rclcpp::init() does NOT modify argc/argv
    // So we just check if argv[1] exists (config file should be first argument after program name)
    if(argc < 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);
    VINS_LOG_INFO(SYSTEM, "配置文件: %s", argv[1]);

    VINS_LOG_INFO(SYSTEM, "开始读取参数配置...");
    readParameters(config_file);
    VINS_LOG_INFO(SYSTEM, "参数配置读取完成");

    VINS_LOG_INFO(ESTIMATOR, "开始初始化 Estimator...");
    estimator.setParameter();
    VINS_LOG_INFO(ESTIMATOR, "Estimator 初始化完成");

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    ROS_WARN("waiting for image and imu...");

    registerPub(n);


    // QoS配置：适配硬件(PX4/RealSense)发出的实际QoS策略
    // 数据包记录的硬件QoS: history=UNKNOWN, reliability=RELIABLE, durability=VOLATILE/TRANSIENT_LOCAL
    // 使用SystemDefaultsQoS确保与硬件QoS兼容，同时设置合理的depth

    // IMU QoS: 匹配PX4 IMU硬件策略 (RELIABLE, VOLATILE)
    auto imu_qos = rclcpp::SystemDefaultsQoS()
        .keep_last(2000)
        .reliability(rclcpp::ReliabilityPolicy::Reliable)
        .durability(rclcpp::DurabilityPolicy::Volatile);

    // Image QoS: 匹配RealSense相机硬件策略 (RELIABLE, TRANSIENT_LOCAL)
    auto image_qos = rclcpp::SystemDefaultsQoS()
        .keep_last(100)
        .reliability(rclcpp::ReliabilityPolicy::Reliable)
        .durability(rclcpp::DurabilityPolicy::TransientLocal);

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu = NULL;
    if(USE_IMU)
    {
        sub_imu = n->create_subscription<sensor_msgs::msg::Imu>(IMU_TOPIC, imu_qos, imu_callback);
        RCLCPP_INFO(n->get_logger(), "Subscribed to IMU topic: %s with QoS (SystemDefaults + RELIABLE/VOLATILE, depth=2000)", IMU_TOPIC.c_str());
    }
    auto sub_feature = n->create_subscription<sensor_msgs::msg::PointCloud>("/feature_tracker/feature", rclcpp::QoS(rclcpp::KeepLast(2000)), feature_callback);
    auto sub_img0 = n->create_subscription<sensor_msgs::msg::Image>(IMAGE0_TOPIC, image_qos, img0_callback);
    RCLCPP_INFO(n->get_logger(), "Subscribed to IMAGE0 topic: %s with QoS (SystemDefaults + RELIABLE/TRANSIENT_LOCAL, depth=100)", IMAGE0_TOPIC.c_str());

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img1 = NULL;
    if(STEREO)
    {
        sub_img1 = n->create_subscription<sensor_msgs::msg::Image>(IMAGE1_TOPIC, image_qos, img1_callback);
        RCLCPP_INFO(n->get_logger(), "Subscribed to IMAGE1 topic: %s with QoS (SystemDefaults + RELIABLE/TRANSIENT_LOCAL, depth=100)", IMAGE1_TOPIC.c_str());
    }
    
    auto sub_restart = n->create_subscription<std_msgs::msg::Bool>("/vins_restart", rclcpp::QoS(rclcpp::KeepLast(100)), restart_callback);
    auto sub_imu_switch = n->create_subscription<std_msgs::msg::Bool>("/vins_imu_switch", rclcpp::QoS(rclcpp::KeepLast(100)), imu_switch_callback);
    auto sub_cam_switch = n->create_subscription<std_msgs::msg::Bool>("/vins_cam_switch", rclcpp::QoS(rclcpp::KeepLast(100)), cam_switch_callback);

    std::thread sync_thread{sync_process};
    rclcpp::spin(n);

    return 0;
}
