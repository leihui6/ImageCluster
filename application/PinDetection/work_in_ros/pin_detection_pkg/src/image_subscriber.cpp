#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "pin_detection_pkg/pin_detection_result.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include "ImageCluster/ImageCluster.h"
#include "PinDetection.h"

using std::cout;
using std::endl;

ros::Publisher pin_detection_result;

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    try
    {
        pin_detection_pkg::pin_detection_result pdr;

        //cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        //cv::waitKey(30);
        ROS_INFO("Got a image");
        cv::Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

        if (img.empty())
        {
            cout << "Could not open or find the image" << std::endl;
            return;
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
            pin_detection_pkg::pin_detection_unit pdr_unit;

            total_clusters[i].get_min_box(min_box_rect);

            total_clusters[i].get_max_box(max_rect);

            total_clusters[i].get_middle_points_of_lines(middle_points_of_lines);

            PinDetectionResult pin_detection_result;

            pin_detection.detect(img, total_clusters[i], pin_detection_result);

            if (pin_detection_result.pin_status == FACEUP)
            {
                pdr_unit.pin_status = 0;

                cv::circle(cluster_image, pin_detection_result.opening_position, 4, cv::Scalar(255, 0, 255), -1);

                cv::circle(cluster_image, pin_detection_result.closing_position, 4, cv::Scalar(0, 0, 255), -1);

                pdr_unit.opening_position.x = pin_detection_result.opening_position.x;
                pdr_unit.opening_position.y = pin_detection_result.opening_position.y;

                pdr_unit.closing_position.x = pin_detection_result.closing_position.x;
                pdr_unit.closing_position.y = pin_detection_result.closing_position.y;
            }
            else if (pin_detection_result.pin_status == FACESIDE)
            {
                cv::arrowedLine(cluster_image, pin_detection_result.rotate_direction_begin, pin_detection_result.rotate_direction_end, cv::Scalar(0, 255, 255), 2);

                pdr_unit.pin_status = 1;
                pdr_unit.rotate_direction_begin.x = pin_detection_result.rotate_direction_begin.x;
                pdr_unit.rotate_direction_begin.y = pin_detection_result.rotate_direction_begin.y;

                pdr_unit.rotate_direction_end.x = pin_detection_result.rotate_direction_end.x;
                pdr_unit.rotate_direction_end.y = pin_detection_result.rotate_direction_end.y;
            }
            pin_detection.clear();

            pdr.pin_detection_result.push_back(pdr_unit);
            //cout << "following are the result of detection: " << endl << pdr_unit << endl;
        }

        pin_detection_result.publish(pdr);

        //cv::imshow("view", cluster_image);

        //cv::waitKey(33);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_subscriber");
    ros::NodeHandle nh;

    //cv::namedWindow("view");
    //cv::startWindowThread();

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);

    pin_detection_result = nh.advertise<pin_detection_pkg::pin_detection_result>("/pin_detection_result", 1000);

    ros::spin();

    //cv::destroyWindow("view");
}
