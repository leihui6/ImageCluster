#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <iostream>
#include <string>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_publisher");

    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    image_transport::Publisher pub = it.advertise("camera/image", 1);

    cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);

    //cv::waitKey(30);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

    ros::Rate loop_rate(5);

    std::string foo;

    while (nh.ok())
    {
        std::cout << "please input enter to continue:" << std::endl;
        std::cin >> foo;
        if (foo == "q")
        {
            break;
        }
        std::cin.sync();

        pub.publish(msg);

        ros::spinOnce();

        ROS_INFO("sent one image");

        loop_rate.sleep();
    }
    std::cout << "quit by user command" << std::endl;
}
