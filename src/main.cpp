/*
 * Optical Flow node for PX4
 * Copyright (C) 2018 Copter Express Technologies
 *
 * Author: Oleg Kalachev <okalachev@gmail.com>
 *
 * Distributed under MIT License (available at https://opensource.org/licenses/MIT).
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */

#include <vector>
#include <cmath>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <mavros_msgs/msg/optical_flow_rad.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "sensor_msgs/msg/range.hpp"
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

using cv::Mat;

class OpticalFlow : public rclcpp::Node
{
public:
    OpticalFlow() : Node("optical_flow"), camera_matrix_(3, 3, CV_64F)
    {
        initialize();
    }

private:
    rclcpp::Publisher<mavros_msgs::msg::OpticalFlowRad>::SharedPtr flow_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velo_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr shift_pub_;
    
    
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr rangeSub;
    void rangeCallback(const sensor_msgs::msg::Range::SharedPtr msg){
        prev_dist_stamp = dist_stamp.clone():
        range = msg;
        dist_stamp = range->header.stamp;
    }

    sensor_msgs::msg::Range::SharedPtr range = std::make_shared<sensor_msgs::msg::Range>();
    rclcpp::Time prev_dist_stamp = rclcpp::Time();
    rclcpp::Time dist_stamp = rclcpp::Time();
    rclcpp::Time prev_stamp_;
    std::string fcu_frame_id_, local_frame_id_;
    image_transport::CameraSubscriber img_sub_;
    image_transport::Publisher img_pub_;
    mavros_msgs::msg::OpticalFlowRad flow_;
    int roi_px_;
    double roi_rad_;
    cv::Rect roi_;
    Mat hann_;
    Mat prev_, curr_;
    Mat camera_matrix_, dist_coeffs_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    bool calc_flow_gyro_;

    
    bool isLidar = false;

    void initialize()
    {
        auto nh = std::shared_ptr<OpticalFlow>(this, [](auto *) {});
        image_transport::ImageTransport it(nh);
        
        rangeSub = this->create_subscription<sensor_msgs::msg::Range>(
        "/rangeLidar", 10,
        std::bind(&OpticalFlow::rangeCallback, this, std::placeholders::_1));
      

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Parameters
        local_frame_id_ = declare_parameter<std::string>("local_frame_id", "map");
        fcu_frame_id_ = declare_parameter<std::string>("fcu_frame_id", "base_link");
        roi_px_ = declare_parameter<int>("roi", 400);
        roi_rad_ = declare_parameter<double>("roi_rad", 0.0);
        calc_flow_gyro_ = declare_parameter<bool>("calc_flow_gyro", false);

        // Subscribers and Publishers
        img_sub_ = it.subscribeCamera(
            "/camera1/image_raw",
            1,
            [this](const sensor_msgs::msg::Image::ConstSharedPtr& img,
                   const sensor_msgs::msg::CameraInfo::ConstSharedPtr& info) {
                this->flow(img, info);
            }
        );
        
        img_pub_ = it.advertise("debug", 1);
        flow_pub_ = create_publisher<mavros_msgs::msg::OpticalFlowRad>("/mavros/px4flow/raw/send", 1);
        velo_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("angular_velocity", 1);
        shift_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/shift", 1);

        // Initialize flow message
        flow_.integrated_xgyro = NAN;
        flow_.integrated_ygyro = NAN;
        flow_.integrated_zgyro = NAN;
        if (isLidar){
            if (range->range > 1.3){
                flow_.distance = 0;        
                flow_.time_delta_distance_us = 0;
            }else{
                flow_.distance = range->range;
                rclcpp::Duration dist_integration_time = dist_stamp - prev_dist_stamp;
                uint32_t dist_integration_time_us = dist_integration_time.seconds() * 1.0e6;
                flow_.time_delta_distance_us = dist_integration_time_us
            }
        }else{ 
            flow_.distance = 0; //range->range;
        }
        flow_.temperature = 0;

        RCLCPP_INFO(get_logger(), "Optical Flow initialized");
    }

    void parseCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cinfo) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                camera_matrix_.at<double>(i, j) = cinfo->k[3 * i + j];
            }
        }
        dist_coeffs_ = cv::Mat(cinfo->d, true);
    }

    void drawFlow(Mat& frame, double x, double y, double quality) const
    {
        double brightness = (1 - quality) * 25;
        cv::Scalar color(brightness, brightness, brightness);
        double radius = std::sqrt(x * x + y * y);

        cv::Point center(frame.cols >> 1, frame.rows >> 1);
        cv::circle(frame, center, (int)(radius*5), color, 3, cv::LINE_AA);
        cv::line(frame, center, cv::Point(center.x + (int)(x*5), center.y + (int)(y*5)), color, 3, cv::LINE_AA);
        cv::imshow("flow", frame);
        cv::waitKey(1);
    }

    void flow(const sensor_msgs::msg::Image::ConstSharedPtr& msg, 
              const sensor_msgs::msg::CameraInfo::ConstSharedPtr& cinfo)
    {
        parseCameraInfo(cinfo);

        auto img = cv_bridge::toCvShare(msg, "mono8")->image;

        if (roi_.width == 0) {
            if (roi_rad_ != 0) {
                std::vector<cv::Point3f> object_points = {
                    cv::Point3f(-sin(roi_rad_ / 2), -sin(roi_rad_ / 2), cos(roi_rad_ / 2)),
                    cv::Point3f(sin(roi_rad_ / 2), sin(roi_rad_ / 2), cos(roi_rad_ / 2)),
                };

                std::vector<double> vec { 0, 0, 0 };
                std::vector<cv::Point2f> img_points;
                cv::projectPoints(object_points, vec, vec, camera_matrix_, dist_coeffs_, img_points);

                roi_ = cv::Rect(cv::Point2i(round(img_points[0].x), round(img_points[0].y)), 
                        cv::Point2i(round(img_points[1].x), round(img_points[1].y)));

                RCLCPP_INFO(get_logger(), "ROI: %d %d - %d %d", roi_.tl().x, roi_.tl().y, roi_.br().x, roi_.br().y);

            } else if (roi_px_ != 0) {
                roi_ = cv::Rect((msg->width / 2 - roi_px_ / 2), (msg->height / 2 - roi_px_ / 2), roi_px_, roi_px_);
            }
        }

        if (roi_.width != 0) {
            img = img(roi_);
        }

        img.convertTo(curr_, CV_32F);

        if (prev_.empty()) {
            prev_ = curr_.clone();
            prev_stamp_ = msg->header.stamp;
            cv::createHanningWindow(hann_, curr_.size(), CV_32F);

        } else {
            double response;
            cv::Point2d shift = cv::phaseCorrelate(prev_, curr_, hann_, &response);

            // Publish raw shift in pixels
            auto shift_vec = std::make_unique<geometry_msgs::msg::Vector3Stamped>();
            shift_vec->header.stamp = msg->header.stamp;
            shift_vec->header.frame_id = msg->header.frame_id;
            shift_vec->vector.x = shift.x;
            shift_vec->vector.y = shift.y;
            auto shift_vec_x = shift.x;
            auto shift_vec_y = shift.y;
            shift_pub_->publish(std::move(shift_vec));

            // Undistort flow in pixels
            uint32_t flow_center_x = msg->width / 2;
            uint32_t flow_center_y = msg->height / 2;
            shift.x += flow_center_x;
            shift.y += flow_center_y;

            std::vector<cv::Point2d> points_dist = { shift };
            std::vector<cv::Point2d> points_undist(1);

            cv::undistortPoints(points_dist, points_undist, camera_matrix_, dist_coeffs_, cv::noArray(), camera_matrix_);
            points_undist[0].x -= flow_center_x;
            points_undist[0].y -= flow_center_y;

            // Calculate flow in radians
            double focal_length_x = camera_matrix_.at<double>(0, 0);
            double focal_length_y = camera_matrix_.at<double>(1, 1);
            double flow_x = atan2(points_undist[0].x, focal_length_x);
            double flow_y = atan2(points_undist[0].y, focal_length_y);

            // Convert to FCU frame
            auto flow_camera = std::make_unique<geometry_msgs::msg::Vector3Stamped>();
            flow_camera->header.frame_id = msg->header.frame_id;
            flow_camera->header.stamp = msg->header.stamp;
            flow_camera->vector.x = flow_x;
            flow_camera->vector.y = -flow_y;

            try {
                auto flow_fcu = *flow_camera;///tf_buffer_->transform(*flow_camera, fcu_frame_id_);
                
                // Calculate integration time
                rclcpp::Time curr_stamp(msg->header.stamp);
                rclcpp::Duration integration_time = curr_stamp - prev_stamp_;
                uint32_t integration_time_us = integration_time.seconds() * 1.0e6;

                if (calc_flow_gyro_) {
                    try {
                        auto flow_gyro_camera = calcFlowGyro(msg->header.frame_id, prev_stamp_, msg->header.stamp);
                        auto flow_gyro_fcu = tf_buffer_->transform(flow_gyro_camera, fcu_frame_id_);
                        flow_.integrated_xgyro = flow_gyro_fcu.vector.x;
                        flow_.integrated_ygyro = flow_gyro_fcu.vector.y;
                        flow_.integrated_zgyro = flow_gyro_fcu.vector.z;
                    } catch (const tf2::TransformException& e) {
                        prev_.release();
                        return;
                    }
                }

                // Publish flow in fcu frame
                flow_.header.stamp = msg->header.stamp;
                flow_.integration_time_us = integration_time_us;
                flow_.integrated_x = flow_camera->vector.x;
                flow_.integrated_y = flow_camera->vector.y;
                flow_.quality = (uint8_t)(response * 255);
                if (isLidar){
                    if (range->range > 1.3){
                        flow_.distance = 0;        
                        flow_.time_delta_distance_us = 0;
                    }else{
                        flow_.distance = range->range;
                        rclcpp::Duration dist_integration_time = dist_stamp - prev_dist_stamp;
                        uint32_t dist_integration_time_us = dist_integration_time.seconds() * 1.0e6;
                        flow_.time_delta_distance_us = dist_integration_time_us;
                    }
                }else{ 
                    flow_.distance = 0; //range->range;
                }

                flow_pub_->publish(flow_);

                // Publish debug image
                if (img_pub_.getNumSubscribers() > 0) {
                    drawFlow(img, shift_vec_x, shift_vec_y, response);
                    auto out_msg = std::make_unique<cv_bridge::CvImage>();
                    out_msg->header.frame_id = msg->header.frame_id;
                    out_msg->header.stamp = msg->header.stamp;
                    out_msg->encoding = sensor_msgs::image_encodings::MONO8;
                    out_msg->image = img;
                    img_pub_.publish(out_msg->toImageMsg());
                }

                // Publish estimated angular velocity
                auto velo = std::make_unique<geometry_msgs::msg::TwistStamped>();
                velo->header.stamp = msg->header.stamp;
                velo->header.frame_id = fcu_frame_id_;
                velo->twist.angular.x = flow_.integrated_x / integration_time.seconds();
                velo->twist.angular.y = flow_.integrated_y / integration_time.seconds();
                velo_pub_->publish(std::move(velo));

                prev_ = curr_.clone();
                prev_stamp_ = msg->header.stamp;

            } catch (const tf2::TransformException& e) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Transform not available: %s", e.what());
                return;
            }
        }
    }

    geometry_msgs::msg::Vector3Stamped calcFlowGyro(const std::string& frame_id, const rclcpp::Time& prev, const rclcpp::Time& curr)
    {
        geometry_msgs::msg::Vector3Stamped flow;
        flow.header.frame_id = frame_id;
        flow.header.stamp = curr;

        try {
            auto prev_tf = tf_buffer_->lookupTransform(frame_id, local_frame_id_, prev);
            auto curr_tf = tf_buffer_->lookupTransform(frame_id, local_frame_id_, curr);

            tf2::Quaternion prev_rot, curr_rot;
            tf2::fromMsg(prev_tf.transform.rotation, prev_rot);
            tf2::fromMsg(curr_tf.transform.rotation, curr_rot);

            auto diff = ((curr_rot - prev_rot) * prev_rot.inverse()) * 2.0f;
            flow.vector.x = -diff.x();
            flow.vector.y = -diff.y();
            flow.vector.z = -diff.z();

        } catch (const tf2::TransformException& e) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Transform error in calcFlowGyro: %s", e.what());
            throw;
        }

        return flow;
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OpticalFlow>());
    rclcpp::shutdown();
    return 0;
}
