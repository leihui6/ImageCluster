#pragma once

#include <vector>
#include <iostream>
#include <string>
#include <opencv2\opencv.hpp>

#define _DEBUG_

// cluster of image
class ImageCluster
{
public:
	ImageCluster();
	
	~ImageCluster();

	// load image, and read width and height of image
	int load_image(unsigned char * _image, int _width, int _height);

	// set kernel, and slice the image.
	int init_kernel_size(int kernel_width, int kernel_height);

	// run the cluster task
	int cluster();

private:
	
	//check the kernel box
	void check_kernel_vec();

	// check one kernel by x and y
	bool check_one_kernel(int x, int y);

	// get point in image using index in kernel_vec
	void get_start_point_by_index(int _index, int &_x, int &_y);

	// get the neighbor points index by index, and the neighbor size is always 8.
	void get_nearby_points_by_index(int _index, std::vector<int>& _near_points);

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

	std::vector<bool> m_kernel_vec;

	// the visited array, all marked false first
	std::vector<bool> m_visited_vec;
};

