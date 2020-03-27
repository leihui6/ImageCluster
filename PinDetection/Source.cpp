#include "../ImageCluster/ImageCluster.h"
#include "PinDetection.h"

#define _MAIN_DEBUG_

//#define _LOCAL_DEBUG_

#define _ONLINE_DEBUG_

using namespace cv;

using namespace std;


int main()
{
#ifdef _ONLINE_DEBUG_

	cv::VideoCapture cap;

	//cap.open(0);

	cap.open("sample/special_test.mp4");

	if (!cap.isOpened())
	{
		return -2;
	}

	cv::Mat frame;
	
	cap >> frame;

	cv::resize(frame, frame, cv::Size(480, 640));

	//VideoWriter demo_video("./sample/demo.mp4", VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, frame.size(), true);

	Mat img_bin, img_removed_bg;

	ImageCluster image_cluster;

	while (cap.isOpened())
	{
		clock_t begin_time_cluster = clock();

		cap >> frame;

		//cv::blur(frame, frame, cv::Size(3, 3));

		//cv::medianBlur(frame, frame, 5);

		if (frame.empty())
		{
			return 0;
		}

		cv::imshow("original image", frame);

		PinDetection pin_detection;

		if (4 != pin_detection.find_ROI(frame, frame))
		{
			continue;
		}

		std::vector<Cluster> total_clusters;

		pin_detection.process_image(frame, total_clusters);

		clock_t end_time_cluster = clock();

		cv::Mat cluster_image;

		frame.copyTo(cluster_image);

		std::vector<cv::Point2i> cluster_pixels, min_box_rect, middle_points_of_lines;;

		cv::Point2i cp;
		cv::Rect2i max_rect;

		float angle = 0.0;

		// output the information of cluster
		//for (int i = 0; i < total_clusters.size(); ++i)
		//{
		//	total_clusters[i].get_angle(angle);
		//	std::cout << "i=" << i << " pixel size=" << cluster_pixels.size() << " angle=" << angle << std::endl;
		//}

		// get every cluster from total_cluster
		for (int i = 0; i < total_clusters.size(); ++i)
		{
			total_clusters[i].get_cluster_pixels(cluster_pixels);

			total_clusters[i].get_min_box(min_box_rect);

			total_clusters[i].get_middle_points_of_lines(middle_points_of_lines);

			total_clusters[i].get_max_box(max_rect);

			total_clusters[i].get_center_point(cp);

			// draw the cluster
			//for (int j = 0; j < cluster_pixels.size(); ++j)
			//{
			//	// blue green red
			//	cv::Vec3b &color = cluster_image.at<cv::Vec3b>(cluster_pixels[j]);

			//	color[0] = 255;
			//	color[1] = 255;
			//	color[2] = 255;
			//}
			PinDetectionResult pin_detection_result;

			clock_t begin_time_pin = clock();

			pin_detection.detect(frame, total_clusters[i], pin_detection_result);

			clock_t end_time_pin = clock();

			if (pin_detection_result.pin_status == FACEUP)
			{
				cv::circle(cluster_image, pin_detection_result.opening_position, 4, cv::Scalar(255, 0, 255), -1);

				cv::circle(cluster_image, pin_detection_result.closing_position, 4, cv::Scalar(0, 0, 255), -1);

				cv::putText(cluster_image, ((pin_detection_result.is_has_needle == true) ? std::to_string(i) + " YES" : std::to_string(i) + " NO"), pin_detection_result.ceter_position, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255));
			}
			else if (pin_detection_result.pin_status == FACESIDE)
			{
				cv::circle(cluster_image, pin_detection_result.ceter_position, 5, cv::Scalar::all(0), -1);

				cv::circle(cluster_image, pin_detection_result.point_on_base_side, 5, cv::Scalar(255, 0, 0), -1);
			}

			cv::putText(cluster_image, ((pin_detection_result.pin_status == FACEUP) ? "FACEUP" : "FACESIDE"), cv::Point2i(max_rect.x, max_rect.y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0));

			pin_detection.clear();

			//for (int j = 0; j < 4; ++j)
			//{
			//	// draw the minimum box of cluster
			//	line(cluster_image, min_box_rect[j], min_box_rect[(j + 1) % 4], cv::Scalar(0, 255, 0));

			//	// draw the middle points of lines(minimum box)
			//	cv::circle(cluster_image, middle_points_of_lines[j], 5, cv::Scalar(0, 0, 255));
			//}

			// draw the maximum box of cluster
			//cv::rectangle(cluster_image, max_rect, cv::Scalar::all(255), 1);

			// draw the center point of cluster
			//cv::circle(cluster_image, cp, 3, cv::Scalar::all(0), -1);

			// draw the serial number of cluster
			//cv::putText(cluster_image, std::to_string(i), cp, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar::all(0));

			// put the fps on screen
			cv::putText(cluster_image,
				"FPS:" + std::to_string((int(1000 / (end_time_cluster - begin_time_cluster)))),
				cv::Point2i(0, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar::all(255));

			cv::putText(cluster_image,
				"FPS:" + std::to_string((int(1000.0 / (end_time_pin - begin_time_pin)))),
				cv::Point2i(0, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar::all(255));
		}

		//cv::imshow("gray image", img_bin);

		cv::imshow("cluster_image", cluster_image);

		//demo_video << cluster_image;

		cv::waitKey(33);

		image_cluster.clear();
	}

	cap.release();

#endif // _ONLINE_DEBUG_

#ifdef _LOCAL_DEBUG_

	cv::Mat img = cv::imread("sample/photo5.jpg");

	if (!img.data)
	{
		cout << "Could not open or find the image" << std::endl;
		return -1;
	}

	PinDetection pin_detection;
	
	pin_detection.find_ROI(img, img);

	cv::resize(img, img, cv::Size(480, 680));

	cv::Mat cluster_image;

	// for showing result
	img.copyTo(cluster_image);

	std::vector<Cluster> total_clusters;

	pin_detection.process_image(img, total_clusters);

	std::vector<cv::Point2i> cluster_pixels, min_box_rect, middle_points_of_lines;;

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
		//cv::Mat img_cluster;
		//total_clusters[i].get_cluster_pixels(cluster_pixels);
		//get_image_from_specific_indices(img_cluster, cluster_pixels, img);

		total_clusters[i].get_min_box(min_box_rect);

		total_clusters[i].get_max_box(max_rect);

		total_clusters[i].get_middle_points_of_lines(middle_points_of_lines);

		PinDetectionResult pin_detection_result;

		pin_detection.detect(img, total_clusters[i], pin_detection_result);

		if (pin_detection_result.pin_status == FACEUP)
		{
			cv::circle(cluster_image, pin_detection_result.opening_position, 4, cv::Scalar(255, 0, 255), -1);

			cv::circle(cluster_image, pin_detection_result.closing_position, 4, cv::Scalar(0, 0, 255), -1);

			cv::putText(cluster_image, ((pin_detection_result.is_has_needle == true) ? std::to_string(i) + " YES" : std::to_string(i) + " NO"), pin_detection_result.ceter_position, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255));
		}
		else if (pin_detection_result.pin_status == FACESIDE)
		{
			cv::circle(cluster_image, pin_detection_result.ceter_position, 5, cv::Scalar::all(0), -1);

			cv::circle(cluster_image, pin_detection_result.point_on_base_side, 5, cv::Scalar(255, 0, 0), -1);
		}

		cv::putText(cluster_image, ((pin_detection_result.pin_status == FACEUP) ? "FACEUP" : "FACESIDE"), cv::Point2i(max_rect.x, max_rect.y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0));

		pin_detection.clear();

		//for (int j = 0; j < 4; ++j)
		//{
		//	// draw the minimum box of cluster
		//	line(cluster_image, min_box_rect[j], min_box_rect[(j + 1) % 4], cv::Scalar(255, 0, 0));

		//	// draw the middle points of lines(minimum box)
		//	cv::circle(cluster_image, middle_points_of_lines[j], 5, cv::Scalar(0, 0, 255));
		//}

		//total_clusters[i].get_center_point(cp);

		// draw the maximum box of cluster
		//cv::rectangle(cluster_image, max_rect, cv::Scalar(0, 255, 0), 1);

		// draw the center point of cluster
		//cv::circle(cluster_image, cp, 3, cv::Scalar::all(0), -1);
	}

	cv::imshow("cluster_image", cluster_image);

	cv::waitKey();

#endif // _LOCAL_DEBUG_


	return 0;
}
