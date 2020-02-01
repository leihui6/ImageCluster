#include "ImageCluster.h"

#define _MAIN_DEBUG_

using namespace cv;

using namespace std;

int main()
{
	Mat img = imread("sample/photo1.jpg");

	if (!img.data)
	{
		cout << "Could not open or find the image" << std::endl;
		return -1;
	}
	cv::resize(img, img,cv::Size(480,640));

	Mat img_bin(cv::Size(img.rows, img.cols), CV_8UC1);

	cout << "width=" << img.cols << " height=" << img.rows << endl;
	
	cv::cvtColor(img, img_bin, COLOR_RGB2GRAY);

	// cv::equalizeHist(img, img);

	cv::threshold(img_bin, img_bin, 125, 255, cv::THRESH_OTSU);

	bitwise_not(img_bin, img_bin);

	imshow("original image", img);

	imshow("gray image", img_bin);

	ImageCluster image_cluster;
	
	clock_t begin_time = clock();

	image_cluster.load_image(img_bin.data,img_bin.cols,img_bin.rows);

	image_cluster.init_kernel_size(8, 12);

	image_cluster.cluster(20);

	std::cout << "execution time:" << (double)(clock() - begin_time) << "ms" << std::endl;

#ifdef _MAIN_DEBUG_

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

	// 
	for (int i = 0; i < total_clusters.size(); ++i)
	{
		total_clusters[i].get_cluster_pixels(cluster_pixels);

		total_clusters[i].get_min_box(min_box_rect);

		total_clusters[i].get_middle_points_of_lines(middle_points_of_lines);

		// draw the cluster
		for (int j = 0; j < cluster_pixels.size(); ++j)
		{
			// blue green red
			cv::Vec3b &color =  cluster_image.at<cv::Vec3b>(cluster_pixels[j]);
			
			color[0] = 255;
			color[1] = 255;
			color[2] = 255;
		}

		for (int j = 0; j < 4; ++j)
		{
			// draw the minimum box of cluster
			line(cluster_image, min_box_rect[j], min_box_rect[(j + 1) % 4], cv::Scalar(0,255,0));

			// draw the middle points of lines(minimum box)
			cv::circle(cluster_image, middle_points_of_lines[j], 5, cv::Scalar(0,0,255));
		}

		total_clusters[i].get_max_box(p1, p2);

		total_clusters[i].get_center_point(cp);

		// draw the maximum box of cluster
		cv::rectangle(cluster_image, cv::Rect(p1, p2), cv::Scalar::all(255), 1);

		// draw the center point of cluster
		cv::circle(cluster_image, cp, 3, cv::Scalar::all(0), -1);

		// draw the serial number of cluster
		cv::putText(cluster_image, std::to_string(i), cp, cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar::all(0));
	}

	cv::imshow("cluster_image", cluster_image);

	cv::waitKey();

#endif // _MAIN_DEBUG_


	return 1;
}