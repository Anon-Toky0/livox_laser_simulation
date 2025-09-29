// 该文件实现Livox激光雷达的Gazebo传感器插件，用于在仿真中模拟Livox雷达的点云输出
// 核心功能：读取CSV文件中的雷达旋转角度信息，通过Gazebo的射线传感器模拟激光发射与障碍物检测，
// 将检测结果转换为ROS点云消息和Gazebo激光扫描消息发布，并广播传感器坐标系TF变换

#include "livox_laser_simulation/livox_points_plugin.h"
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/transport/Node.hh>
#include <livox_laser_simulation/CustomMsg.h>
#include "livox_laser_simulation/csv_reader.hpp"
#include "livox_laser_simulation/livox_ode_multiray_shape.h"
#include "livox_laser_simulation/livox_point_xyzrtl.h"

namespace gazebo {

    GZ_REGISTER_SENSOR_PLUGIN(LivoxPointsPlugin)

        LivoxPointsPlugin::LivoxPointsPlugin() {}

    LivoxPointsPlugin::~LivoxPointsPlugin() {}

    /**
     * @brief 将CSV读取的double类型数据转换为Livox Avia雷达的旋转信息结构体
     * @param datas CSV读取的原始数据，每行包含3个double：时间、方位角（度）、天顶角（度）
     * @param avia_infos 输出的旋转信息向量，存储转换后的时间、方位角（弧度）、天顶角（弧度）
     */
    void convertDataToRotateInfo(const std::vector<std::vector<double>>& datas, std::vector<AviaRotateInfo>& avia_infos) {
        avia_infos.reserve(datas.size());
        double deg_2_rad = M_PI / 180.0;
        int i = 0;
        for (auto& data : datas) {
            if (data.size() == 3) {
                avia_infos.emplace_back();
                avia_infos.back().time = data[0];
                avia_infos.back().azimuth = data[1] * deg_2_rad;
                avia_infos.back().zenith = data[2] * deg_2_rad - M_PI_2;  //转化成标准的右手系角度
                avia_infos.back().line = i % 4; i++;
            } else {
                ROS_INFO_STREAM("data size is not 3!");
            }
        }
    }

    /**
     * @brief Gazebo插件核心加载函数，在插件初始化时被调用
     * 功能：读取SDF配置、加载CSV角度文件、初始化ROS/Gazebo通信、创建射线形状、配置传感器参数
     * @param _parent 插件附着的父传感器（此处为RaySensor）
     * @param sdf SDF配置文件元素，包含插件的参数（如CSV文件名、ROS话题名等）
     */
    void LivoxPointsPlugin::Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr sdf) {
        // -------------------------- 1. 读取SDF配置参数 --------------------------
        // 读取CSV文件路径（存储雷达旋转角度信息）
        std::vector<std::vector<double>> datas;
        std::string file_name = sdf->Get<std::string>("csv_file_name");

        std::string filePath(__FILE__);
        size_t found = filePath.find_last_of("/\\");
        file_name = std::string(filePath.substr(0, found)) + "/../scan_mode/" + file_name;

        ROS_INFO_STREAM("load csv file name:" << file_name);
        if (!CsvReader::ReadCsvFile(file_name, datas)) {
            ROS_INFO_STREAM("cannot get csv file!" << file_name << "will return !");
            return;
        }
        sdfPtr = sdf;

        // 解析SDF中的激光雷达参数（射线、扫描、距离范围）
        auto rayElem = sdfPtr->GetElement("ray");
        auto scanElem = rayElem->GetElement("scan");
        auto rangeElem = rayElem->GetElement("range");

        // 初始化ROS节点和点云发布者
        int argc = 0;
        char** argv = nullptr;
        auto curr_scan_topic = sdf->Get<std::string>("ros_topic");
        ROS_INFO_STREAM("ros topic name:" << curr_scan_topic);
        ros::init(argc, argv, curr_scan_topic);
        rosNode.reset(new ros::NodeHandle);

        publishPointCloudType = static_cast<PointCloudType>(sdfPtr->Get<int>("publish_pointcloud_type"));

        switch (publishPointCloudType) {
        case PointCloudType::SENSOR_MSG_POINT_CLOUD:
            rosPointPub = rosNode->advertise<sensor_msgs::PointCloud>(curr_scan_topic, 5);
            break;
        case PointCloudType::SENSOR_MSG_POINT_CLOUD2_POINTXYZ:
        case PointCloudType::SENSOR_MSG_POINT_CLOUD2_LIVOXPOINTXYZRTLT:
            rosPointPub = rosNode->advertise<sensor_msgs::PointCloud2>(curr_scan_topic, 5);
            break;
        case PointCloudType::livox_laser_simulation_CUSTOM_MSG:
            rosPointPub = rosNode->advertise<livox_laser_simulation::CustomMsg>(curr_scan_topic, 5);
            break;
        default:
            break;
        }

        visualize = sdfPtr->Get<bool>("visualize");

        // 初始化Gazebo传感器和父实体
        raySensor = _parent;
        auto sensor_pose = raySensor->Pose();
        SendRosTf(sensor_pose, raySensor->ParentName(), raySensor->Name()); // 发送传感器相对于父实体的TF变换（ROS坐标系关系）

        // 初始化Gazebo传输节点（用于发布Gazebo原生LaserScanStamped消息）
        node = transport::NodePtr(new transport::Node());
        node->Init(raySensor->WorldName());
        scanPub = node->Advertise<msgs::LaserScanStamped>(_parent->Topic(), 50);

        // 解析CSV旋转数据为AviaRotateInfo结构体
        aviaInfos.clear();
        convertDataToRotateInfo(datas, aviaInfos);
        ROS_INFO_STREAM("scan info size:" << aviaInfos.size());
        maxPointSize = aviaInfos.size();

        // 加载父类RayPlugin的功能（Gazebo射线传感器插件基类）
        RayPlugin::Load(_parent, sdfPtr);

        // 9. 初始化Gazebo激光扫描消息的基本信息
        laserMsg.mutable_scan()->set_frame(_parent->ParentName());
        // parentEntity = world->GetEntity(_parent->ParentName());

        // 10. 获取传感器父实体和物理引擎，创建自定义多线激光碰撞体与形状
        parentEntity = this->world->EntityByName(_parent->ParentName());
        auto physics = world->Physics();

        // 创建多线激光碰撞体：类型为"multiray"，父实体为传感器父实体
        laserCollision = physics->CreateCollision("multiray", _parent->ParentName());
        laserCollision->SetName("ray_sensor_collision");
        laserCollision->SetRelativePose(_parent->Pose());
        laserCollision->SetInitialRelativePose(_parent->Pose());

        // 创建自定义Livox多线激光形状（继承自MultiRayShape，适配Livox扫描模式）
        rayShape.reset(new gazebo::physics::LivoxOdeMultiRayShape(laserCollision));
        laserCollision->SetShape(rayShape);
        samplesStep = sdfPtr->Get<int>("samples"); // 读取采样点数（每帧发射的射线总数）
        downSample = sdfPtr->Get<int>("downsample"); // 读取下采样系数（每隔downSample个点保留一个，减少计算量）
        if (downSample < 1) {
            downSample = 1;
        }
        ROS_INFO_STREAM("sample:" << samplesStep);
        ROS_INFO_STREAM("downsample:" << downSample);

        // 12. 预分配射线数组内存，加载并初始化射线形状
        rayShape->RayShapes().reserve(samplesStep / downSample);
        rayShape->Load(sdfPtr);
        rayShape->Init();

        // 13. 读取距离范围参数（最小/最大探测距离）
        minDist = rangeElem->Get<double>("min");
        maxDist = rangeElem->Get<double>("max");

        // 14. 生成初始射线：根据CSV旋转信息计算每个射线的起点和终点
        auto offset = laserCollision->RelativePose();
        ignition::math::Vector3d start_point, end_point;
        for (int j = 0; j < samplesStep; j += downSample) {
            int index = j % maxPointSize;
            auto& rotate_info = aviaInfos[index];
            ignition::math::Quaterniond ray;
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            start_point = minDist * axis + offset.Pos();
            end_point = maxDist * axis + offset.Pos();
            rayShape->AddRay(start_point, end_point);
        }
    }

    /**
     * @brief 激光扫描数据更新回调函数：Gazebo每次生成新扫描数据时调用
     * 功能：更新射线状态、计算点云坐标、发布Gazebo激光消息和ROS点云消息
     */
    void LivoxPointsPlugin::OnNewLaserScans() {
        // 检查射线形状是否初始化成功
        if (!rayShape) {
            return;
        }

        // 1. 初始化射线：更新本次扫描的射线起点/终点，并记录射线索引与旋转信息的对应关系
        std::vector<std::pair<int, AviaRotateInfo>> points_pair;
        InitializeRays(points_pair, rayShape);

        // 2. 更新射线形状状态（触发Gazebo物理引擎计算每条射线的碰撞距离）
        rayShape->Update();

        // 3. 设置Gazebo激光扫描消息的时间戳（使用Gazebo仿真时间）
        msgs::Set(laserMsg.mutable_time(), world->SimTime());
        switch (publishPointCloudType) {
        case PointCloudType::SENSOR_MSG_POINT_CLOUD:
            PublishPointCloud(points_pair);
            break;
        case PointCloudType::SENSOR_MSG_POINT_CLOUD2_POINTXYZ:
            PublishPointCloud2XYZ(points_pair);
            break;
        case PointCloudType::SENSOR_MSG_POINT_CLOUD2_LIVOXPOINTXYZRTLT:
            PublishPointCloud2XYZRTLT(points_pair);
            break;
        case PointCloudType::livox_laser_simulation_CUSTOM_MSG:
            PublishLivoxROSDriverCustomMsg(points_pair);
            break;
        default:
            break;
        }
    }


    /**
     * @brief 初始化本次扫描的射线：更新每条射线的起点/终点，并建立射线索引与旋转信息的映射
     * @param points_pair 输出参数，存储<射线索引, 旋转信息>的对应关系
     * @param ray_shape 多线激光形状指针（自定义LivoxOdeMultiRayShape）
     */
    void LivoxPointsPlugin::InitializeRays(std::vector<std::pair<int, AviaRotateInfo>>& points_pair,
        boost::shared_ptr<physics::LivoxOdeMultiRayShape>& ray_shape) {
        auto& rays = ray_shape->RayShapes();
        ignition::math::Vector3d start_point, end_point;
        ignition::math::Quaterniond ray;
        auto offset = laserCollision->RelativePose();
        int64_t end_index = currStartIndex + samplesStep;
        int ray_index = 0;
        auto ray_size = rays.size();
        points_pair.reserve(rays.size());
        for (int k = currStartIndex; k < end_index; k += downSample) {
            auto index = k % maxPointSize;
            auto& rotate_info = aviaInfos[index];
            ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
            auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
            start_point = minDist * axis + offset.Pos();
            end_point = maxDist * axis + offset.Pos();
            if (ray_index < ray_size) {
                rays[ray_index]->SetPoints(start_point, end_point);
                points_pair.emplace_back(ray_index, rotate_info);
            }
            ray_index++;
        }
        currStartIndex += samplesStep;
    }

    /**
     * @brief 初始化Gazebo激光扫描消息（msgs::LaserScan）的参数
     * @param scan 待初始化的激光扫描消息指针
     */
    void LivoxPointsPlugin::InitializeScan(msgs::LaserScan*& scan) {
        // Store the latest laser scans into laserMsg
        msgs::Set(scan->mutable_world_pose(), raySensor->Pose() + parentEntity->WorldPose());
        scan->set_angle_min(AngleMin().Radian());
        scan->set_angle_max(AngleMax().Radian());
        scan->set_angle_step(AngleResolution());
        scan->set_count(RangeCount());

        scan->set_vertical_angle_min(VerticalAngleMin().Radian());
        scan->set_vertical_angle_max(VerticalAngleMax().Radian());
        scan->set_vertical_angle_step(VerticalAngleResolution());
        scan->set_vertical_count(VerticalRangeCount());

        scan->set_range_min(RangeMin());
        scan->set_range_max(RangeMax());

        scan->clear_ranges();
        scan->clear_intensities();

        unsigned int rangeCount = RangeCount();
        unsigned int verticalRangeCount = VerticalRangeCount();

        for (unsigned int j = 0; j < verticalRangeCount; ++j) {
            for (unsigned int i = 0; i < rangeCount; ++i) {
                scan->add_ranges(0);
                scan->add_intensities(0);
            }
        }
    }

    /**
     * @brief 获取水平扫描最小角度
     * @return 水平最小角度（ignition::math::Angle类型）
     */
    ignition::math::Angle LivoxPointsPlugin::AngleMin() const {
        if (rayShape)
            return rayShape->MinAngle();
        else
            return -1;
    }

    /**
     * @brief 获取水平扫描最大角度
     * @return 水平最大角度（ignition::math::Angle类型）
     */
    ignition::math::Angle LivoxPointsPlugin::AngleMax() const {
        if (rayShape) {
            return ignition::math::Angle(rayShape->MaxAngle().Radian());
        } else
            return -1;
    }

    /**
     * @brief 获取最小探测距离（封装函数）
     * @return 最小探测距离（米）
     */
    double LivoxPointsPlugin::GetRangeMin() const { return RangeMin(); }

    /**
     * @brief 获取最小探测距离（核心函数）
     * @return 最小探测距离（米）
     */
    double LivoxPointsPlugin::RangeMin() const {
        if (rayShape)
            return rayShape->GetMinRange();
        else
            return -1;
    }

    /**
     * @brief 获取最大探测距离（封装函数）
     * @return 最大探测距离（米）
     */
    double LivoxPointsPlugin::GetRangeMax() const { return RangeMax(); }

    /**
     * @brief 获取最大探测距离（核心函数）
     * @return 最大探测距离（米）
     */
    double LivoxPointsPlugin::RangeMax() const {
        if (rayShape)
            return rayShape->GetMaxRange();
        else
            return -1;
    }

    /**
     * @brief 获取水平角度分辨率（封装函数）
     * @return 水平角度分辨率（弧度/点）
     */
    double LivoxPointsPlugin::GetAngleResolution() const { return AngleResolution(); }

    double LivoxPointsPlugin::AngleResolution() const { return (AngleMax() - AngleMin()).Radian() / (RangeCount() - 1); }

    /**
     * @brief 获取距离分辨率（封装函数）
     * @return 距离分辨率（米）
     */
    double LivoxPointsPlugin::GetRangeResolution() const { return RangeResolution(); }

    double LivoxPointsPlugin::RangeResolution() const {
        if (rayShape)
            return rayShape->GetResRange();
        else
            return -1;
    }

    int LivoxPointsPlugin::GetRayCount() const { return RayCount(); }

    int LivoxPointsPlugin::RayCount() const {
        if (rayShape)
            return rayShape->GetSampleCount();
        else
            return -1;
    }

    int LivoxPointsPlugin::GetRangeCount() const { return RangeCount(); }

    int LivoxPointsPlugin::RangeCount() const {
        if (rayShape)
            return rayShape->GetSampleCount() * rayShape->GetScanResolution();
        else
            return -1;
    }

    int LivoxPointsPlugin::GetVerticalRayCount() const { return VerticalRayCount(); }

    int LivoxPointsPlugin::VerticalRayCount() const {
        if (rayShape)
            return rayShape->GetVerticalSampleCount();
        else
            return -1;
    }

    /**
     * @brief 获取垂直扫描总点数（封装函数）
     * @return 垂直扫描总点数
     */
    int LivoxPointsPlugin::GetVerticalRangeCount() const { return VerticalRangeCount(); }

    /**
     * @brief 获取垂直扫描总点数（核心函数）
     * @return 垂直点数 = 垂直射线数量 × 垂直扫描分辨率
     */
    int LivoxPointsPlugin::VerticalRangeCount() const {
        if (rayShape)
            return rayShape->GetVerticalSampleCount() * rayShape->GetVerticalScanResolution();
        else
            return -1;
    }

    /**
     * @brief 获取垂直扫描最小角度
     * @return 垂直最小角度（ignition::math::Angle类型）
     */
    ignition::math::Angle LivoxPointsPlugin::VerticalAngleMin() const {
        if (rayShape) {
            return ignition::math::Angle(rayShape->VerticalMinAngle().Radian());
        } else
            return -1;
    }

    /**
     * @brief 获取垂直扫描最大角度
     * @return 垂直最大角度（ignition::math::Angle类型）
     */
    ignition::math::Angle LivoxPointsPlugin::VerticalAngleMax() const {
        if (rayShape) {
            return ignition::math::Angle(rayShape->VerticalMaxAngle().Radian());
        } else
            return -1;
    }

    /**
     * @brief 获取垂直角度分辨率（封装函数）
     * @return 垂直角度分辨率（弧度/点）
     */
    double LivoxPointsPlugin::GetVerticalAngleResolution() const { return VerticalAngleResolution(); }

    /**
     * @brief 获取垂直角度分辨率（核心函数）
     * @return 垂直角度分辨率 = (垂直最大角度 - 垂直最小角度) / (垂直点数 - 1)
     */
    double LivoxPointsPlugin::VerticalAngleResolution() const {
        return (VerticalAngleMax() - VerticalAngleMin()).Radian() / (VerticalRangeCount() - 1);
    }

    /**
     * @brief 发送ROS TF变换：建立父子坐标系之间的姿态关系
     * @param pose 子坐标系相对于父坐标系的姿态
     * @param father_frame 父坐标系名称
     * @param child_frame 子坐标系名称
     */
    void LivoxPointsPlugin::SendRosTf(const ignition::math::Pose3d& pose, const std::string& father_frame,
        const std::string& child_frame) {
        if (!tfBroadcaster) {
            tfBroadcaster.reset(new tf::TransformBroadcaster);
        }
        tf::Transform tf;
        auto rot = pose.Rot();
        auto pos = pose.Pos();
        tf.setRotation(tf::Quaternion(rot.X(), rot.Y(), rot.Z(), rot.W()));
        tf.setOrigin(tf::Vector3(pos.X(), pos.Y(), pos.Z()));
        tfBroadcaster->sendTransform(
            tf::StampedTransform(tf, ros::Time::now(), raySensor->ParentName(), raySensor->Name()));
    }

    void LivoxPointsPlugin::PublishPointCloud(std::vector<std::pair<int, AviaRotateInfo>>& points_pair) {
        auto rayCount = RayCount();
        auto verticalRayCount = VerticalRayCount();
        auto angle_min = AngleMin().Radian();
        auto angle_incre = AngleResolution();
        auto verticle_min = VerticalAngleMin().Radian();
        auto verticle_incre = VerticalAngleResolution();

        msgs::LaserScan* scan = laserMsg.mutable_scan();
        InitializeScan(scan);
        // SendRosTf(parentEntity->WorldPose(), world->Name(), raySensor->ParentName());

        sensor_msgs::PointCloud scan_point;
        scan_point.header.stamp = ros::Time::now();
        scan_point.header.frame_id = frameName;
        scan_point.header.frame_id = "livox";
        auto& scan_points = scan_point.points;
        for (auto& pair : points_pair) {
            int verticle_index = roundf((pair.second.zenith - verticle_min) / verticle_incre);
            int horizon_index = roundf((pair.second.azimuth - angle_min) / angle_incre);
            if (verticle_index < 0 || horizon_index < 0) {
                continue;
            }
            if (verticle_index < verticalRayCount && horizon_index < rayCount) {
                auto index = (verticalRayCount - verticle_index - 1) * rayCount + horizon_index;
                auto range = rayShape->GetRange(pair.first);
                auto intensity = rayShape->GetRetro(pair.first);
                if (range >= maxDist || range <= minDist || range <= 1e-5) {
                    range = 0.0;
                }
                scan->set_ranges(index, range);
                scan->set_intensities(index, intensity);

                auto rotate_info = pair.second;
                ignition::math::Quaterniond ray;
                ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));

                auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
                auto point = range * axis;
                scan_points.emplace_back();
                scan_points.back().x = point.X();
                scan_points.back().y = point.Y();
                scan_points.back().z = point.Z();
            }
        }
        rosPointPub.publish(scan_point);
        ros::spinOnce();
        if (scanPub && scanPub->HasConnections() && visualize) {
            scanPub->Publish(laserMsg);
        }
    }

    void LivoxPointsPlugin::PublishPointCloud2XYZ(std::vector<std::pair<int, AviaRotateInfo>>& points_pair) {
        auto rayCount = RayCount();
        auto verticalRayCount = VerticalRayCount();
        auto angle_min = AngleMin().Radian();
        auto angle_incre = AngleResolution();
        auto verticle_min = VerticalAngleMin().Radian();
        auto verticle_incre = VerticalAngleResolution();

        msgs::LaserScan* scan = laserMsg.mutable_scan();
        InitializeScan(scan);

        sensor_msgs::PointCloud2 scan_point;

        pcl::PointCloud<pcl::PointXYZ> pc;
        pc.points.resize(points_pair.size());
        ros::Time timestamp = ros::Time::now();
        int pt_count = 0;
#pragma omp parallel for
        for (int i = 0; i < points_pair.size(); ++i) {
            std::pair<int, gazebo::AviaRotateInfo>& pair = points_pair[i];
            int verticle_index = roundf((pair.second.zenith - verticle_min) / verticle_incre);
            int horizon_index = roundf((pair.second.azimuth - angle_min) / angle_incre);
            if (verticle_index < 0 || horizon_index < 0) {
                continue;
            }
            if (verticle_index < verticalRayCount && horizon_index < rayCount) {
                auto index = (verticalRayCount - verticle_index - 1) * rayCount + horizon_index;
                auto range = rayShape->GetRange(pair.first);
                auto intensity = rayShape->GetRetro(pair.first);
                if (range >= maxDist || range <= minDist || range <= 1e-5) {
                    range = 0.0;
                }

                scan->set_ranges(index, range);
                scan->set_intensities(index, intensity);

                auto rotate_info = pair.second;
                ignition::math::Quaterniond ray;
                ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
                //                auto axis = rotate * ray * math::Vector3(1.0, 0.0, 0.0);
                //                auto point = range * axis + world_pose.Pos();
                auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);

                // if (range < 0.3) {
                //     ROS_WARN_STREAM("Small pt: range: " << range << ", axis: " << axis);
                // }
                auto point = range * axis;
                pcl::PointXYZ pt;
                pt.x = point.X();
                pt.y = point.Y();
                pt.z = point.Z();
                // if (pt_count < pc.size() && pt_count > 0)
#pragma omp critical
                {
                    pc[pt_count] = pt;
                    ++pt_count;
                }
            }
        }
        pc.resize(pt_count);
        pcl::toROSMsg(pc, scan_point);
        scan_point.header.stamp = timestamp;
        scan_point.header.frame_id = frameName;
        rosPointPub.publish(scan_point);
        // SendRosTf(parentEntity->WorldPose(), world->Name(), raySensor->ParentName());
        ros::spinOnce();
        if (scanPub && scanPub->HasConnections() && visualize) {
            scanPub->Publish(laserMsg);
        }
    }

    void LivoxPointsPlugin::PublishPointCloud2XYZRTLT(std::vector<std::pair<int, AviaRotateInfo>>& points_pair) {
        auto rayCount = RayCount();
        auto verticalRayCount = VerticalRayCount();
        auto angle_min = AngleMin().Radian();
        auto angle_incre = AngleResolution();
        auto verticle_min = VerticalAngleMin().Radian();
        auto verticle_incre = VerticalAngleResolution();

        msgs::LaserScan* scan = laserMsg.mutable_scan();
        InitializeScan(scan);
        // SendRosTf(parentEntity->WorldPose(), world->Name(), raySensor->ParentName());

        sensor_msgs::PointCloud2 scan_point;

        pcl::PointCloud<pcl::LivoxPointXyzrtlt> pc;
        pc.points.reserve(points_pair.size());
        ros::Time header_timestamp = ros::Time::now();
        auto header_timestamp_sec_nsec = header_timestamp.toNSec();


        // auto start = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        for (int i = 0; i < points_pair.size(); ++i) {
            std::pair<int, AviaRotateInfo>& pair = points_pair[i];
            int verticle_index = roundf((pair.second.zenith - verticle_min) / verticle_incre);
            int horizon_index = roundf((pair.second.azimuth - angle_min) / angle_incre);
            if (verticle_index < 0 || horizon_index < 0) {
                continue;
            }
            if (verticle_index < verticalRayCount && horizon_index < rayCount) {
                auto index = (verticalRayCount - verticle_index - 1) * rayCount + horizon_index;
                auto range = rayShape->GetRange(pair.first);
                auto intensity = rayShape->GetRetro(pair.first);
                if (range >= maxDist || range <= minDist || abs(range) <= 1e-5) {
                    range = 0.0;
                }
                scan->set_ranges(index, range);
                scan->set_intensities(index, intensity);

                auto rotate_info = pair.second;
                ignition::math::Quaterniond ray;
                ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
                //                auto axis = rotate * ray * math::Vector3(1.0, 0.0, 0.0);
                //                auto point = range * axis + world_pose.Pos();

                auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
                auto point = range * axis;
                pcl::LivoxPointXyzrtlt pt;

                pt.x = point.X();
                pt.y = point.Y();
                pt.z = point.Z();
                pt.intensity = static_cast<float>(intensity);
                pt.tag = 0;
                pt.line = pair.second.line;
                pt.timestamp = static_cast<double>(1e9 / 200000 * i) + header_timestamp_sec_nsec;

                pc.push_back(std::move(pt));
            }
        }
        pcl::toROSMsg(pc, scan_point);
        scan_point.header.stamp = header_timestamp;
        scan_point.header.frame_id = frameName;
        rosPointPub.publish(scan_point);
        ros::spinOnce();
        if (scanPub && scanPub->HasConnections() && visualize) {
            scanPub->Publish(laserMsg);
        }
    }

    void LivoxPointsPlugin::PublishLivoxROSDriverCustomMsg(std::vector<std::pair<int, AviaRotateInfo>>& points_pair) {
        auto rayCount = RayCount();
        auto verticalRayCount = VerticalRayCount();
        auto angle_min = AngleMin().Radian();
        auto angle_incre = AngleResolution();
        auto verticle_min = VerticalAngleMin().Radian();
        auto verticle_incre = VerticalAngleResolution();

        msgs::LaserScan* scan = laserMsg.mutable_scan();
        InitializeScan(scan);
        // SendRosTf(parentEntity->WorldPose(), world->Name(), raySensor->ParentName());

        sensor_msgs::PointCloud2 scan_point;

        livox_laser_simulation::CustomMsg msg;
        // msg.header.frame_id = raySensor->ParentName();

        msg.header.frame_id = frameName;

        struct timespec tn;
        clock_gettime(CLOCK_REALTIME, &tn);

        msg.timebase = tn.tv_nsec;
        msg.header.stamp = ros::Time::now();
        ros::Time timestamp = ros::Time::now();
        for (int i = 0; i < points_pair.size(); ++i) {
            std::pair<int, AviaRotateInfo>& pair = points_pair[i];
            int verticle_index = roundf((pair.second.zenith - verticle_min) / verticle_incre);
            int horizon_index = roundf((pair.second.azimuth - angle_min) / angle_incre);
            if (verticle_index < 0 || horizon_index < 0) {
                continue;
            }
            if (verticle_index < verticalRayCount && horizon_index < rayCount) {
                auto index = (verticalRayCount - verticle_index - 1) * rayCount + horizon_index;
                auto range = rayShape->GetRange(pair.first);
                auto intensity = rayShape->GetRetro(pair.first);
                if (range >= maxDist || range <= minDist || abs(range) <= 1e-5) {
                    range = 0.0;
                }
                scan->set_ranges(index, range);
                scan->set_intensities(index, intensity);

                auto rotate_info = pair.second;
                ignition::math::Quaterniond ray;
                ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
                //                auto axis = rotate * ray * math::Vector3(1.0, 0.0, 0.0);
                //                auto point = range * axis + world_pose.Pos(); Convert to world coordinate system

                auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
                auto point = range * axis;
                // pt.intensity = static_cast<float>(intensity);
                livox_laser_simulation::CustomPoint pt;
                pt.x = point.X();
                pt.y = point.Y();
                pt.z = point.Z();
                pt.line = pair.second.line;
                // ROS_INFO_STREAM("offset_time: " << pt.offset_time );
                pt.tag = 0x10;
                pt.reflectivity = 100;
                pt.offset_time = (1e9 / 200000 * i);
                msg.points.push_back(pt);
            }
        }
        // clock_gettime(CLOCK_REALTIME, &tn);
        // uint64_t interval = tn.tv_nsec - msg.timebase;
        // for (int i = 0; i < msg.points.size(); ++i) {
        //     msg.points[i].offset_time = (float)interval / msg.points.size() * i * 10;
        // }
        msg.point_num = msg.points.size();
        rosPointPub.publish(msg);
        ros::spinOnce();
        if (scanPub && scanPub->HasConnections() && visualize) {
            scanPub->Publish(laserMsg);
        }
    }

}
