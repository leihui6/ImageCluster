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
	int load_image(unsigned char * _image, size_t _width, size_t _height);

	// set kernel, and slice the image.
	int init_kernel_size(size_t kernel_width, size_t kernel_height);

	// run the cluster task
	int cluster();

private:
	
	//check the kernel box
	void check_kernel_vec();

	// check one kernel by x and y
	bool check_one_kernel(size_t x, size_t y);

	// get point in image using index in kernel_vec
	void get_start_point_by_index(size_t _index, size_t &_x, size_t &_y);

	// get the neighbor points index by index, and the neighbor size is always 8.
	void get_nearby_points_by_index(size_t _index, std::vector<size_t>& _near_points);

	// delete pointer safely
	void delete_pointer(unsigned char ** _ptr);

private:
	unsigned char * m_image_data;

	size_t m_width;

	size_t m_height;

	size_t m_kernel_width;

	size_t m_kernel_height;

	size_t m_kernel_count_col;

	size_t m_kernel_count_row;

	std::vector<bool> m_kernel_vec;

	// the visited array, all marked false first
	std::vector<bool> m_visited_vec;
};

