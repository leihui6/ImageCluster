#pragma once

#include "ImageCluserHeader.h"

#include "Cluster.h"

//#define _IMAGECLUSTER_DEBUG_

// cluster of image
class ImageCluster
{
public:
	ImageCluster();

	ImageCluster(size_t width, size_t height);
	
	~ImageCluster();

	// load image, and read width and height of image
	int load_image(unsigned char * _image, int _width, int _height);

	// set kernel, and slice the image.
	int init_kernel_size(int kernel_width, int kernel_height);

	// run the cluster task
	/*
	\param[in] _threshold limitation of cluster size
	*/
	int cluster(int _threshold);

	void get_clusters(std::vector<Cluster> & _total_clusters);

	void clear();

private:
	
	//check the kernel box
	void check_kernel_vec();

	// check one kernel by x and y
	bool check_one_kernel(int x, int y);

	// get point in image using index in kernel_vec
	void get_start_point_by_index(int _index, int &_x, int &_y);

	// get the neighbor points index by index, and the neighbor size is always 8.
	void get_nearby_points_by_index(int _index, std::vector<int>& _near_points);

	// convert kernel cluster into pixel cluster
	void convert_kernel_cluster(int _threshold);

	// collect pixel from kernels
	void collect_pixels(int _x, int _y, std::vector<cv::Point2i> & _cluster);

	// delete pointer safely
	void delete_pointer(unsigned char ** _ptr);
	
private:
	unsigned char * m_image_data;
	
	int m_width;

	int m_height;

	int m_kernel_width;

	int m_kernel_height;

	int m_kernel_count_col;

	int m_kernel_count_row;

	// the kernel array, it will be scaned to set true or false which means if it contains vaild points
	std::vector<bool> m_kernel_vec;

	// the visited array, all marked false initially
	std::vector<bool> m_visited_vec;

	// the total cluster which contains the single cluster. !!But kernels.
	std::vector<std::vector<int>> m_total_cluster_kernels;

	// the total cluster which contains the single cluster.
	std::vector<Cluster> m_total_clusters;

};

