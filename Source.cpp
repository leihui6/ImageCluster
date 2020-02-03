#include "ImageCluster.h"

#define _MAIN_DEBUG_

#define _LOCAL_DEBUG_

//#define _ONLINE_DEBUG_

using namespace cv;

using namespace std;

void background_removal(cv::Mat & _img, cv::Mat & _res_img,int _r, int _g, int _b, float _threshold);

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

	VideoWriter demo_video("./demo.mp4", VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, frame.size(), true);

	Mat img_bin, img_removed_bg;

	ImageCluster image_cluster;

	while (cap.isOpened())
	{
		clock_t begin_time = clock();

		cap >> frame;

		if (frame.empty())
		{
			return 0;
		}

		background_removal(frame, img_removed_bg, 255, 255, 255, 250.0);

		cv::cvtColor(img_removed_bg, img_bin, COLOR_RGB2GRAY);

		cv::threshold(img_bin, img_bin, 125, 255, cv::THRESH_OTSU);

		cv::bitwise_not(img_bin, img_bin);

		image_cluster.load_image(img_bin.data, img_bin.cols, img_bin.rows);

		image_cluster.init_kernel_size(8, 12);

		image_cluster.cluster(20);

		clock_t end_time = clock();

		//std::cout << "execution time:" << (double)(end_time - begin_time) << "ms" << std::endl;

		cv::Mat cluster_image;

		frame.copyTo(cluster_image);

		std::vector<cv::Point2i> cluster_pixels, min_box_rect, middle_points_of_lines;;

		std::vector<Cluster> total_clusters;

		image_cluster.get_clusters(total_clusters);

		cv::Point2i p1, p2, cp;

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

			total_clusters[i].get_max_box(p1, p2);

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

			for (int j = 0; j < 4; ++j)
			{
				// draw the minimum box of cluster
				line(cluster_image, min_box_rect[j], min_box_rect[(j + 1) % 4], cv::Scalar(0, 255, 0));

				// draw the middle points of lines(minimum box)
				cv::circle(cluster_image, middle_points_of_lines[j], 5, cv::Scalar(0, 0, 255));
			}

			// draw the maximum box of cluster
			cv::rectangle(cluster_image, cv::Rect(p1, p2), cv::Scalar::all(255), 1);

			// draw the center point of cluster
			cv::circle(cluster_image, cp, 3, cv::Scalar::all(0), -1);

			// draw the serial number of cluster
			cv::putText(cluster_image, std::to_string(i), cp, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar::all(0));

			// put the fps on screen
			cv::putText(cluster_image, "FPS:"+std::to_string((int(1000/(end_time-begin_time)))), cv::Point2i(0,20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar::all(0));
		}

		cv::imshow("original image", frame);

		cv::imshow("gray image", img_bin);

		cv::imshow("cluster_image", cluster_image);

		demo_video << cluster_image;

		cv::waitKey(33);

		image_cluster.clear();
	}

	cap.release();

#endif // _ONLINE_DEBUG_

#ifdef _LOCAL_DEBUG_

	cv::Mat img = cv::imread("sample/photo2.jpg");

	//Mat background = imread("sample/background.jpg");
	//img = img - background;

	if (!img.data)
	{
		cout << "Could not open or find the image" << std::endl;
		return -1;
	}

	Mat img_removed_bg;

	Mat img_bin(cv::Size(img.cols, img.rows), CV_8UC1);

	background_removal(img, img_removed_bg, 255, 255, 255, 200.0);

	cv::resize(img, img, cv::Size(480, 640));
	cv::resize(img_removed_bg, img_removed_bg, cv::Size(480, 640));

	std::cout << "width=" << img_removed_bg.cols << " height=" << img_removed_bg.rows << std::endl;
	
	cv::cvtColor(img_removed_bg, img_bin, COLOR_RGB2GRAY);

	cv::imshow("original image", img);

	cv::imshow("gray image", img_bin);

	cv::threshold(img_bin, img_bin, 125, 255, cv::THRESH_OTSU);

	cv::bitwise_not(img_bin, img_bin);

	cv::imshow("threshold image", img_bin);

	ImageCluster image_cluster;
	
	clock_t begin_time = clock();

	image_cluster.load_image(img_bin.data,img_bin.cols,img_bin.rows);

	image_cluster.init_kernel_size(8, 16);

	image_cluster.cluster(10);

	std::cout << "execution time:" << (double)(clock() - begin_time) << "ms" << std::endl;

	cv::Mat cluster_image;

	img.copyTo(cluster_image);

	std::vector<cv::Point2i> cluster_pixels, min_box_rect, middle_points_of_lines;;

	std::vector<Cluster> total_clusters;

	image_cluster.get_clusters(total_clusters);

	cv::Point2i p1, p2, cp;

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
		total_clusters[i].get_cluster_pixels(cluster_pixels);

		total_clusters[i].get_min_box(min_box_rect);

		total_clusters[i].get_middle_points_of_lines(middle_points_of_lines);

		// draw the cluster
		//for (int j = 0; j < cluster_pixels.size(); ++j)
		//{
		//	// blue green red
		//	cv::Vec3b &color = cluster_image.at<cv::Vec3b>(cluster_pixels[j]);

		//	color[0] = 255;
		//	color[1] = 255;
		//	color[2] = 255;
		//}

		for (int j = 0; j < 4; ++j)
		{
			// draw the minimum box of cluster
			line(cluster_image, min_box_rect[j], min_box_rect[(j + 1) % 4], cv::Scalar(255, 0, 0));

			// draw the middle points of lines(minimum box)
			cv::circle(cluster_image, middle_points_of_lines[j], 5, cv::Scalar(0, 0, 255));
		}

		total_clusters[i].get_max_box(p1, p2);

		total_clusters[i].get_center_point(cp);

		// draw the maximum box of cluster
		cv::rectangle(cluster_image, cv::Rect(p1, p2), cv::Scalar(0, 255, 0), 1);

		// draw the center point of cluster
		cv::circle(cluster_image, cp, 3, cv::Scalar::all(0), -1);

		// draw the serial number of cluster
		cv::putText(cluster_image, std::to_string(i), cp, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar::all(0));
	}

	cv::imshow("cluster_image", cluster_image);

	cv::waitKey();

#endif // _LOCAL_DEBUG_


	return 0;
}

void background_removal(cv::Mat & _img, cv::Mat & _res_img, int _r, int _g, int _b, float _threshold)
{
	int r = 0, g = 0, b = 0;

	_res_img.create(cv::Size(_img.cols, _img.rows), CV_8UC3);

	for (int y = 0; y < _img.rows; ++y)
	{
		for (int x = 0; x < _img.cols; ++x)
		{
			cv::Vec3b & c = _img.at<cv::Vec3b>(y, x);

			cv::Vec3b & c2 = _res_img.at<cv::Vec3b>(y, x);

			b = c[0];
			g = c[1];
			r = c[2];
			//cout << (sqrt((b - _b)*(b - _b) + (_g - g)*(_g - g) + (_r - r)*(_r - r))) <<endl;
			if ((sqrt((b - _b)*(b - _b) + (_g - g)*(_g - g) + (_r - r)*(_r - r))) < _threshold)
			{
				c2[0] = 255;
				c2[1] = 255;
				c2[2] = 255;
			}
			else
			{
				c2[0] = 0;
				c2[1] = 0;
				c2[2] = 0;
			}
		}
	}
}
