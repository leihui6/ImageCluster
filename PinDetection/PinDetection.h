#pragma once
#include "../ImageCluster/ImageCluster.h"

struct PinDetectionResult
{
	bool is_has_needle;

	cv::Point2i opening_position;

	cv::Point2i fixing_position;

	cv::Point2i  ceter_position;
};

class PinDetection
{
public:

	PinDetection();

	~PinDetection();

	void detect(cv::Mat & image, Cluster & _cluster, PinDetectionResult & pin_detection_result);

private:

	float get_distance(cv::Point2i pointO, cv::Point2i pointA);

	float get_distance_point_to_line(cv::Point2i & p, float a, float b, float c);

	void order_rect_points(std::vector<cv::Point2i>& rect);

	void get_point_along_with_distance(cv::Point2i & p, float distance, cv::Point2i &new_p, cv::Point2i &dir);

	bool is_parallel_and_same_direction(const cv::Point2f & v1, const cv::Point2f & v2, float threshold = 10e-5);

	void collect_pixels_in_min_box(cv::Rect2i & max_rect, float min_box_width, float min_box_height, std::vector<std::vector<float>> & minbox_line_func, std::vector<cv::Point2i>& points);

	void get_max_box_rect(std::vector<cv::Point2i>& _points_vec, cv::Rect2i & max_rect);

	//void identify_side(std::vector<cv::Point2i> & p_a_circle, std::vector<cv::Point2i>& p_b_circle);

private:

	// This is a region of side.
	// to judge where is opening side and fixing side. 
	std::vector<cv::Point2i> m_p_a_circle, m_p_b_circle;

	std::vector<cv::Point2i> m_pixels_in_min_box;

};

