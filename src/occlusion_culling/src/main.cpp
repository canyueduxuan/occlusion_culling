#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;

class PinholeCamera {
    public:
    float fx = 914.1773681640625f;
    float fy = 912.8531494140625f;
    float cx = 638.7860107421875f;
    float cy = 368.9774169921875f;
    int width = 1280;
    int height = 720;

    void project()
    {

    }

};

struct ProjectedPoint {
        int u, v;
        float depth;
        bool valid;
    };

class PointCloudColorizer {
    public:
    PointCloudColorizer() {
        // 初始化发布者
        pub_ = nh_.advertise<sensor_msgs::PointCloud2>("colored_pointcloud", 10);

        // 设置订阅者
        image_sub_.subscribe(nh_, "/camera/color/image_raw", 10);
        pcl_sub_.subscribe(nh_, "/livox/lidar", 10);

        // 初始化同步器
        sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), image_sub_, pcl_sub_));
        sync_->registerCallback(boost::bind(&PointCloudColorizer::callback, this, _1, _2));

        this->t_cl << -0.000057, -0.999946, -0.010403,0.069724,
                      0.025538,  0.010398, -0.999620,-0.129633,
                      0.999674, -0.000323,  0.025536,-0.067841,
                      0,0,0,1;
        ROS_INFO("???\r\n");
    }

    void callback(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::PointCloud2ConstPtr& pc_msg)
    {
        ROS_INFO("[callback]recived\r\n");
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*pc_msg, *cloud);

        cv::Mat depth_map(cam.height, cam.width, CV_32FC1, cv::Scalar(std::numeric_limits<float>::max()));
        
        std::vector<ProjectedPoint> proj_pts(cloud->points.size());

        for (size_t i = 0; i < cloud->points.size(); ++i) {
            const auto& pt = cloud->points[i];
            Eigen::Vector4f pt_lidar(pt.x, pt.y, pt.z, 1.0f);
            Eigen::Vector4f pt_cam = this->t_cl * pt_lidar;

            proj_pts[i].valid = false;
            if (pt_cam.z() <= 0.1) continue;

            int u = static_cast<int>(cam.fx * pt_cam.x() / pt_cam.z() + cam.cx);
            int v = static_cast<int>(cam.fy * pt_cam.y() / pt_cam.z() + cam.cy);

            if (u >= 0 && u < cam.width && v >= 0 && v < cam.height) {

                // TODO:写入if里面
                proj_pts[i].u = u;
                proj_pts[i].v = v;
                proj_pts[i].depth = pt_cam.z();
                proj_pts[i].valid = true;

                // 写入 Z-buffer
                if (pt_cam.z() < depth_map.at<float>(v, u)) {
                    depth_map.at<float>(v, u) = pt_cam.z();
                }
            }
        }

        cv::Mat eroded_depth;
        int kernel_size = 20; // 窗口大小。如果雷达线数较少(如16线)，可以调大至 7 或 9
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
        // 深度值越小越近。erode 会取邻域内的最小值，等同于让前景向外“长胖”
        cv::erode(depth_map, eroded_depth, element); 

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        colored_cloud->header = cloud->header;
        colored_cloud->reserve(cloud->size());

        // --- 第二遍遍历：根据膨胀后的深度图进行遮挡判断并上色 ---
        float depth_tolerance = 0.2f; // 深度容差(米)。防止同一倾斜平面的点自己遮挡自己

        for (size_t i = 0; i < cloud->points.size(); ++i)
        {
            const auto& pt = cloud->points[i];
        
            if (proj_pts[i].valid) {
                int u = proj_pts[i].u;
                int v = proj_pts[i].v;
                float depth = proj_pts[i].depth;

                // 获取该像素邻域膨胀后的最小深度
                float min_depth = eroded_depth.at<float>(v, u);

                // 遮挡判定：当前点深度显著大于该区域的最小深度
                if (depth > min_depth + depth_tolerance) {
                    // 点被前景遮挡。你可以直接丢弃该点 (取消下一行的注释)：
                    // continue; 
                    
                    // 或者保留该点的空间信息，但赋予一个默认颜色（比如红色或灰色，方便你调试）
                    pcl::PointXYZRGB pt_rgb;
                    pt_rgb.x = pt.x; pt_rgb.y = pt.y; pt_rgb.z = pt.z;
                    pt_rgb.r = 0; pt_rgb.g = 0; pt_rgb.b = 0; // 灰色
                    colored_cloud->points.push_back(pt_rgb);
                } else {
                    // 未被遮挡，正常从图像取色
                    pcl::PointXYZRGB pt_rgb;
                    pt_rgb.x = pt.x; pt_rgb.y = pt.y; pt_rgb.z = pt.z;
                    const cv::Vec3b& color = cv_ptr->image.at<cv::Vec3b>(v, u);
                    pt_rgb.b = color[0];
                    pt_rgb.g = color[1];
                    pt_rgb.r = color[2];
                    colored_cloud->points.push_back(pt_rgb);
                }
            } else {
                // 投影在图像视野之外的点，如果你不需要可以 continue;
                // 如果需要保持点云完整性，给个默认色即可。
            }
        }


        ROS_INFO("[callback]colored\r\n");


        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*colored_cloud, output);
        output.header = pc_msg->header; // 保持坐标系一致
        pub_.publish(output);
    }

    private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    message_filters::Subscriber<sensor_msgs::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub_;
    boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
    Eigen::Matrix4f t_cl;
    PinholeCamera cam;
};   

int main(int argc, char** argv) {
    ros::init(argc, argv, "occlusion_culling_node");
    PointCloudColorizer pc;
    ros::spin();
    return 0;
}