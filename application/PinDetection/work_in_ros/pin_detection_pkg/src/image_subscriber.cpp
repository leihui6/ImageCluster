#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


#include "ImageCluster/ImageCluster.h"
#include "PinDetection.h"

using std::cout;
using std::endl;


void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        //cv::waitKey(30);
        ROS_INFO("Got a image");
        cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;
        if (!img.data)
        {
            cout << "Could not open or find the image" << std::endl;
            return ;
        }

        PinDetection pin_detection;

        pin_detection.find_ROI(img, img);

        cv::resize(img, img, cv::Size(480, 680));

        cv::Mat cluster_image;

        // for showing result
        img.copyTo(cluster_image);

        std::vector<Cluster> total_clusters;

        pin_detection.process_image(img, total_clusters);

        std::vector<cv::Point2i> cluster_pixels, min_box_rect, middle_points_of_lines;

        cv::Point2i cp;

        cv::Rect2i max_rect;

        float angle = 0.0;

        // output the information of cluster
        for (int i = 0; i < total_clusters.size(); ++i)
        {
            total_clusters[i].get_cluster_pixels(cluster_pixels);

            total_clusters[i].get_angle(angle);

            std::cout << "i=" << i << " pixel size=" << cluster_pixels.size() << " angle=" << angle << std::endl;
        }

        // get every cluster from total_cluster
        for (int i = 0; i < total_clusters.size(); ++i)
        {
            total_clusters[i].get_min_box(min_box_rect);

            total_clusters[i].get_max_box(max_rect);

            total_clusters[i].get_middle_points_of_lines(middle_points_of_lines);

            PinDetectionResult pin_detection_result;

            pin_detection.detect(img, total_clusters[i], pin_detection_result);

            if (pin_detection_result.pin_status == FACEUP)
            {
                cv::circle(cluster_image, pin_detection_result.opening_position, 4, cv::Scalar(255, 0, 255), -1);

                cv::circle(cluster_image, pin_detection_result.closing_position, 4, cv::Scalar(0, 0, 255), -1);

            }
            else if (pin_detection_result.pin_status == FACESIDE)
            {
                cv::arrowedLine(cluster_image, pin_detection_result.rotate_direction_begin, pin_detection_result.rotate_direction_end, cv::Scalar(0, 255, 255), 2);
            }
            pin_detection.clear();
        }

        cv::imshow("cluster_image", cluster_image);

        cv::waitKey(33);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    //cv::namedWindow("view");
    //cv::startWindowThread();
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
    ros::spin();
    //cv::destroyWindow("view");
}
