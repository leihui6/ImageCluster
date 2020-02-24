#pragma once
#include "../ImageCluster/ImageCluster.h"

enum PinStatus
{
	POSITIVE,
	BESIDE
};

struct PinDetectionResult
{
	PinStatus pin_status;

	bool is_has_needle;

	std::vector<cv::Point2i> opening_position;

	std::vector<cv::Point2i> fixing_position;

	cv::Point2i ceter_position;

	cv::Point2i rotate_direction;
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

	void order_rect_points(std::vector<cv::Point2i>& rect, cv::Size2f & rect_size);

	void order_rect_points(std::vector<cv::Point2i>& rect);

	void get_point_along_with_distance(cv::Point2i & p, float distance, cv::Point2i &new_p, cv::Point2i &dir);

	bool is_parallel_and_same_direction(const cv::Point2f & v1, const cv::Point2f & v2, float threshold = 10e-5);

	void collect_pixels_in_min_box(cv::Rect2f & max_rect, float min_box_width, float min_box_height, std::vector<std::vector<float>> & minbox_line_func, std::vector<cv::Point2i>& points);

	void get_max_box_rect(std::vector<cv::Point2i>& _points_vec, cv::Rect2f & max_rect, int limited_width, int limited_height);

	//! Building line_segment using ordered points
	void get_line_segment_from_points(std::vector<cv::Point2i> & points, std::vector<std::vector<float>> & line_segments);

	void get_line_segment_from_points(std::vector<cv::Point2i> & points, std::vector<float> & line_segment);

	void get_line_segment_from_points(cv::Point2i & point_0, cv::Point2i & point_1, std::vector<float> & line_segment);

	void detecte_middle_part(cv::Mat &img, std::vector<cv::Point2i>& middle_part_points, bool & is_positive);

	void detecte_both_side(cv::Mat &img, std::vector<cv::Point2i> & side, float & is_opening_probability);

	void get_grasp_position(cv::Mat &img, std::vector<cv::Point2i> & side_1, std::vector<cv::Point2i> & side_2, std::vector<cv::Point2i> & grasp_position, bool is_positive);

	void get_is_has_needle(cv::Mat & img, bool & is_has_needle, cv::Point2i & needle_p_0, cv::Point2i & needle_p_1);

	bool is_needle(cv::Vec3b & c, float threshold = 300);

	void detect_middle_orientation(cv::Mat &img, std::vector<cv::Point2i>& middle_part_points, cv::Point2i &line_0_0, cv::Point2i &line_0_1, cv::Point2i &line_1_0, cv::Point2i &line_1_1, cv::Point2i & orientation_point, float threshold = 100);

	inline void rgb_to_gray(cv::Vec3b &c, int &g);

private:

	// This is a region of side.
	// to judge where is opening side and fixing side. 
	std::vector<cv::Point2i> m_p_a_circle, m_p_b_circle;

	std::vector<cv::Point2i> m_pixels_in_min_box;

	/*
	m_min_rect[2]-----m_min_rect[3]
	     |                 |
	     1                 2
	     |                 |
	     3                 0
	     |                 |
	m_min_rect[1]-----m_min_rect[0]
	*/
	std::vector<cv::Point2i> m_min_rect;

	std::vector<cv::Point2i> m_inner_p;

	std::vector <std::vector<cv::Point2i>> m_inner_part_points;

	int m_middle_positive_threshold;

	int m_middle_orientation_threshold;

	int m_both_side_threshold;

	int m_is_needle_threshold;

	float m_is_middle_positive_probaility;
};

