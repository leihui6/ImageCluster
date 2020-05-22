#pragma once
#include "../../ImageCluster/ImageCluster.h"

#include <algorithm>
#include <numeric>
#include <opencv2/aruco.hpp>

enum PinStatus
{
	FACEUP,
	FACESIDE,
	OVERLAPPing
};

struct PinDetectionResult
{
	PinStatus pin_status;

	// face up
	bool is_has_needle;
	cv::Point2i opening_position;
	cv::Point2i closing_position;

	// face side, rotation derection from begin to end
	cv::Point2i rotate_direction_begin;
	cv::Point2i rotate_direction_end;

	// overlapping
	cv::Point2i ceter_position;
};

class PinDetection
{
public:

	PinDetection();

	~PinDetection();

	// find region of interesting in image
	/*
	\param[in] image input image
	\param[out] result ROI image

	returned value is the marker points in this image.
	*/
	int find_ROI(cv::Mat &image, cv::Mat &res);

	void detect(cv::Mat & image, Cluster & _cluster, PinDetectionResult & pin_detection_result);

	// including pre-process image and image cluster
	void process_image(cv::Mat & img, std::vector<Cluster> &total_clusters);

	void clear();

private:

	void background_removal(cv::Mat & _img, cv::Mat & _res_img, int gray_value,/*int _r, int _g, int _b,*/ float _threshold);

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

	void get_grasp_position(cv::Mat & img, std::vector<cv::Point2i>& side_1, std::vector<cv::Point2i>& side_2, cv::Point2i & opening_position, cv::Point2i & closing_position);

	void get_if_has_needle(cv::Mat & img, bool & is_has_needle, cv::Point2i & needle_p_0, cv::Point2i & needle_p_1);

	bool is_needle(cv::Vec3b & c, float threshold = 300);

	void detect_middle_orientation(cv::Mat &img, std::vector<cv::Point2i>& middle_part_points, cv::Point2i &line_0_0, cv::Point2i &line_0_1, cv::Point2i &line_1_0, cv::Point2i &line_1_1,
		cv::Point2i & orientation_point_begin, cv::Point2i & orientation_point_end, float threshold = 100);

	inline void rgb_to_gray(cv::Vec3b &c, int &g);
	
	void get_min_max_point_in_vector(std::vector<cv::Point2i> &points, cv::Point2i & min_p, cv::Point2i & max_p);

private:

	ImageCluster m_image_cluster;

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

