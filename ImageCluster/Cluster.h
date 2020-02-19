#pragma once
#include "ImageCluserHeader.h"

class Cluster
{
public:
	Cluster();
	
	~Cluster();

	int init_cluster(std::vector<cv::Point2i> & _points_vec);

	void get_max_box(cv::Rect2i & _rect);

	void get_min_box(std::vector<cv::Point2i> &_min_box_rect);

	void get_min_box_size(float & _width, float & _height);

	void get_min_box_line_segment_function(std::vector<std::vector<float>>& _line_segment);

	void get_middle_points_of_lines(std::vector<cv::Point2i>& _m_min_box_middle_p);

	void get_center_point(cv::Point2i & _p);

	void get_cluster_pixels(std::vector<cv::Point2i> & _cluster_pixels);

	void get_angle(float& _angle);

private:
	void max_box_rect(std::vector<cv::Point2i> & _points_vec);

	void min_box_rect(std::vector<cv::Point2i> & _points_vec);

private:
	cv::Point2i m_center_point;
	
	cv::Rect2i m_max_box_rect;

	//! 4 points
	std::vector<cv::Point2i> m_min_box_rect;
	float m_min_box_width, m_min_box_height;

	//! note this value may not include all cluster pixels because of background substraction.
	std::vector<cv::Point2i> m_cluster_pixels;

	//! the middle points of min box's 4 lines
	std::vector<cv::Point2i> m_min_box_middle_p;

	//! 4 line segment of min box
	/*
	So the size of min_box_line_vec should be 4.
	each of them represented by A,B,C, so the size of element should be 3.
	*/
	std::vector<std::vector<float>> m_min_box_line_vec;

	float m_angle;
};

