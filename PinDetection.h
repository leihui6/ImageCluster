#pragma once
#include "ImageCluserHeader.h"

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

	void detect(cv::Mat & image, cv::Rect2i pin_region, std::vector<cv::Point2i>& rect, PinDetectionResult & pin_detection_result);

private:

	float get_distance(cv::Point2i pointO, cv::Point2i pointA);

	void order_rect_points(std::vector<cv::Point2i>& rect);

	void point_along_with_distance(cv::Point2i & p, float distance, cv::Point2i &new_p, cv::Point2i &dir);

	bool is_parallel_and_same_direction(const cv::Point2f & v1, const cv::Point2f & v2, float threshold);

	void identify_side(std::vector<cv::Point2i> & p_a_circle, std::vector<cv::Point2i>& p_b_circle);

private:

	// This is a region of side.
	// to judge where is opening side and fixing side. 
	std::vector<cv::Point2i> m_p_a_circle, m_p_b_circle;
};

