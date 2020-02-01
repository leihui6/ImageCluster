#pragma once
#include "ImageCluserHeader.h"

class Cluster
{
public:
	Cluster();
	
	~Cluster();

	int init_cluster(std::vector<cv::Point2i> & _points_vec);

	void get_max_box(cv::Point2i & _p1, cv::Point2i &_p2);

	void get_min_box(std::vector<cv::Point2i> &_min_box_rect);

	void get_middle_points_of_lines(std::vector<cv::Point2i>& _m_min_box_middle_p);

	void get_center_point(cv::Point2i & _p);

	void get_cluster_pixels(std::vector<cv::Point2i> & _cluster_pixels);

	void get_angle(float& _angle);

private:
	void max_box_rect(std::vector<cv::Point2i> & _points_vec);

	void min_box_rect(std::vector<cv::Point2i> & _points_vec);

	void cetner_point_of_cluster(std::vector<cv::Point2i> & _points_vec);

private:
	cv::Point2i m_center_point;
	
	cv::Point2i m_max_box_rect[2];

	std::vector<cv::Point2i> m_cluster_pixels;
	
	std::vector<cv::Point2i> m_min_box_rect;

	// the middle points of min box's 4 lines
	std::vector<cv::Point2i> m_min_box_middle_p;

	float m_angle;
};

