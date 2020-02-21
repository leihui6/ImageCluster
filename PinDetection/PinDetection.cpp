#include "PinDetection.h"

PinDetection::PinDetection()
{
}

PinDetection::~PinDetection()
{
}

void PinDetection::detect(cv::Mat & image, Cluster& _cluster, PinDetectionResult & pin_detection_result)
{
	_cluster.get_min_box(m_min_rect);
	order_rect_points(m_min_rect);

	/*
	0-----1 1-----0 3-----2 2-----3
	|     |	|     |	|     |	|     |
	|     |	|     |	|     |	|     |
	|     |	|     |	|     |	|     |
	3-----2	2-----3	0-----1	1-----0
	*/
	float d = 0.0;
	m_inner_p.resize(4);
	cv::Point2i dir_line;
	
	d = ((get_distance(m_min_rect[1], m_min_rect[2]) + get_distance(m_min_rect[0], m_min_rect[3])) / 2.0f) * (1 / 5.0f);

	dir_line = cv::Point2i(m_min_rect[3].x - m_min_rect[0].x, m_min_rect[3].y - m_min_rect[0].y);

	// [0]---0---2---[3]
	get_point_along_with_distance(m_min_rect[0], d, m_inner_p[0], dir_line);
	get_point_along_with_distance(m_min_rect[1], d, m_inner_p[1], dir_line);

	// [1]---1---3---[2]
	get_point_along_with_distance(m_min_rect[0], 4 * d, m_inner_p[2], dir_line);
	get_point_along_with_distance(m_min_rect[1], 4 * d, m_inner_p[3], dir_line);

	// inner_part_vertex vertex of middle/both side part
	// inner_part_points points in middle/both side part
	std::vector<cv::Point2i> inner_part_vertex[3];
	cv::Size2f inner_size[3];
	cv::Rect2f inner_max_rect[3];

	std::vector<std::vector<float>> inner_line_segments[3];

	/*
	[2]-----[3]
	|        |
	1        2
	|        |
	3        0
	|        |
	[1]-----[0]
	*/
	inner_part_vertex[0] = { m_min_rect[0],m_min_rect[1],m_inner_p[0],m_inner_p[1] };

	inner_part_vertex[1] = { m_inner_p[0],m_inner_p[2],m_inner_p[1],m_inner_p[3] };

	inner_part_vertex[2] = { m_min_rect[2],m_min_rect[3],m_inner_p[3],m_inner_p[2] };

	for (int i = 0; i < 3; ++i)
	{
		order_rect_points(inner_part_vertex[i], inner_size[i]);

		get_max_box_rect(inner_part_vertex[i], inner_max_rect[i]);

		get_line_segment_from_points(inner_part_vertex[i], inner_line_segments[i]);
	}

	float middle_probobility = 0.6;
	bool middle_is_positive = false;

	m_inner_part_points.resize(3);

	for (int i = 0; i < 3; ++i)
	{
		collect_pixels_in_min_box(inner_max_rect[i], inner_size[i].width, inner_size[i].height, inner_line_segments[i], m_inner_part_points[i]);
		
		//for (auto & i : m_inner_part_points[i])
		//{
		//	auto & color = image.at<cv::Vec3b>(i);
		//	color[0] = 255;
		//	color[1] = 255;
		//	color[2] = 0;
		//}
		//cv::imshow("test", image);
		//cv::waitKey(0);
	}

	//return;

	// First thing we need to do is to detect the middle part.

	// direction of needle if the needle inclines.
	cv::Point2i middle_direction;

	detecte_middle_part(image, m_inner_part_points[1], middle_probobility, middle_is_positive, middle_direction);

	std::cout << middle_is_positive << std::endl;
	
	// this needle is not positive.
	if (!middle_is_positive)
	{
		//TODO
		cv::circle(image, middle_direction, 2, cv::Scalar(0, 0, 0), -1);
		//cv::imshow("test", image);
		//cv::waitKey(0);
	}
	
	// this needle is positive, and we need to detecte the opening side.
	if (middle_is_positive)
	{
		std::vector<cv::Point2i> grasp_position;

		get_grasp_position(image, m_inner_part_points[0], m_inner_part_points[2], grasp_position);

		bool is_has_needle = true;

		std::vector<cv::Point2i> line_points(2);

		line_points[0] = (/*m_min_rect[0] + m_min_rect[1] + */m_inner_p[0] + m_inner_p[1]) / 2;

		line_points[1] = (/*m_min_rect[2] + m_min_rect[3] + */m_inner_p[2] + m_inner_p[3]) / 2;

		get_is_has_needle(image, line_points, m_inner_part_points[1], is_has_needle);
	}

	/*
	for (int i = 0; i < 4; i++)
	{
		cv::circle(image, inner_p[i], 2, cv::Scalar(255, 0, 0), 1);
		cv::circle(image, min_rect[i], 2, cv::Scalar(255, 0, 0), 1);
	}

	cv::imshow("test",image);
	cv::waitKey(0);

	collect_pixels_in_min_box(max_rect, min_rect_width, min_rect_height, line_segments, both_side[0]);

	collect_pixels_in_min_box(max_rect, min_rect_width, min_rect_height, line_segments, both_side[1]);

	collect_pixels_in_min_box(max_rect, min_rect_width, min_rect_height, line_segments, middle_side);
	*/
	//collect_pixels_in_min_box(max_rect, min_rect_width, min_rect_height, line_segments, points_in_min_box);
	//for (auto & i: points_in_min_box)
	//{
	//	auto & color = image.at<cv::Vec3b>(i);
	//	color[0] = 255;
	//	color[1] = 255;
	//	color[2] = 255;
	//}
	//cv::imshow("test",image);
	//cv::waitKey(0);
	/*
	cv::Point2i p_a, p_b, p_0;
	p_a = (min_rect[0] + min_rect[1]) / 2;
	p_b = (min_rect[2] + min_rect[3]) / 2;
	p_0 = (min_rect[0] + min_rect[1] + min_rect[2] + min_rect[3]) / 4;
	
	float d = 0.0;
	std::cout << get_distance(min_rect[1], min_rect[2]) << std::endl;
	std::cout << get_distance(min_rect[0], min_rect[3]) << std::endl;

	// distance to figure out where is opening side.
	d = ((get_distance(min_rect[1], min_rect[2]) + get_distance(min_rect[0], min_rect[3])) / 2.0f) * (1 / 5.0f) * (1 / 2.0f);

	cv::Point2i detection_p[5], dir_line;
	dir_line = cv::Point2i(p_0.x - p_a.x, p_0.y - p_a.y);

	float move_d = d;
	for (int i = 0; i < 5; ++i)
	{
		if (i == 0)
		{
			move_d = d;
		}
		else
		{
			move_d = 2 * d + move_d;
		}
		get_point_along_with_distance(p_a, move_d, detection_p[i], dir_line);
	}
	*/
	/*for (int y = pin_region.y; y < pin_region.y + pin_region.height; ++y)
	{
		for (int x = pin_region.x; x < pin_region.x + pin_region.width; ++x)
		{
			cv::Point2i p = cv::Point2i(x, y);

			if (get_distance(p, new_p_a) < d)
			{
				m_p_a_circle.push_back(p);
			}
			else if (get_distance(p, new_p_b) < d)
			{
				m_p_b_circle.push_back(p);
			}
		}
	}*/
	//identify_side();
	/*
	cv::circle(image, p_0, d, cv::Scalar(255, 0, 0), 1);
	cv::circle(image, p_a, 2, cv::Scalar(255, 0, 0), 1);
	cv::circle(image, p_b, 2, cv::Scalar(255, 0, 0), 1);

	for (int i = 0; i < 5; ++i)
	{
		cv::circle(image, detection_p[i], d, cv::Scalar(255, 0, 0), 1);
	}

	//for (auto i : m_p_a_circle)
	//{
	//	//blue green red
	//	cv::Vec3b &color = image.at<cv::Vec3b>(i);

	//	color[0] = 255;
	//	color[1] = 255;
	//	color[2] = 255;
	//}
	cv::imshow("test", image);
	cv::waitKey(0);
	*/
}

float PinDetection::get_distance(cv::Point2i pointO, cv::Point2i pointA)
{
	float distance = 0.0;
	distance = powf(float(pointO.x - pointA.x), 2) + powf(float(pointO.y - pointA.y), 2);
	return sqrtf(distance);
}

float PinDetection::get_distance_point_to_line(cv::Point2i & p, float a, float b, float c)
{
	return abs(a * p.x + b * p.y + c) / sqrt(a*a + b * b);
}

void PinDetection::order_rect_points(std::vector<cv::Point2i>& rect, cv::Size2f & rect_size)
{
	std::vector<cv::Point2i> ordered_rect;
	ordered_rect.push_back(rect.back());
	rect.pop_back();

	while (!rect.empty())
	{
		int min_i = -1;
		float dis = FLT_MAX, d = 0.0;
		for (int i = 0; i < rect.size(); ++i)
		{
			d = get_distance(rect[i], ordered_rect.back());

			if (d < dis)
			{
				dis = d;
				min_i = i;
			}
		}
		if (min_i != -1)
		{
			//std::cout << d << std::endl;
			ordered_rect.push_back(rect[min_i]);
			rect.erase(rect.begin() + min_i);
		}
	}
	rect = ordered_rect;
	rect_size = { get_distance(rect[1],rect[0]),get_distance(rect[1],rect[2]) };
}

void PinDetection::order_rect_points(std::vector<cv::Point2i>& rect)
{
	std::vector<cv::Point2i> ordered_rect;
	ordered_rect.push_back(rect.back());
	rect.pop_back();

	while (!rect.empty())
	{
		int min_i = -1;
		float dis = FLT_MAX, d = 0.0;
		for (int i = 0; i < rect.size(); ++i)
		{
			d = get_distance(rect[i], ordered_rect.back());

			if (d < dis)
			{
				dis = d;
				min_i = i;
			}
		}
		if (min_i != -1)
		{
			//std::cout << d << std::endl;
			ordered_rect.push_back(rect[min_i]);
			rect.erase(rect.begin() + min_i);
		}
	}
	rect = ordered_rect;
}

void PinDetection::get_point_along_with_distance(cv::Point2i & p, float d, cv::Point2i & new_p, cv::Point2i& dir)
{
	float m, n;
	m = (float)dir.x;
	n = (float)dir.y;

	// M,N = x,y
	float
		M = sqrt((d*d*m*m) / (m*m + n * n)),
		N = sqrt((d*d*n*n) / (m*m + n * n));

	float x[2], y[2];

	x[0] = p.x + M;
	x[1] = p.x - M;
	y[0] = p.y + N;
	y[1] = p.y - N;
	
	new_p.x = ((x[0] - p.x) * dir.x) > 0 ? x[0] : x[1];
	new_p.y = ((y[0] - p.y) * dir.y) > 0 ? y[0] : y[1];
}

bool PinDetection::is_parallel_and_same_direction(const cv::Point2f & v1, const cv::Point2f & v2, float threshold)
{
	float
		r1 = v1.x / v2.x,
		r2 = v1.y / v2.y;

	if (v1.x == 0 || v2.x == 0)
	{
		r1 = 0;
	}
	else if (v1.y == 0 || v2.y == 0)
	{
		r2 = 0;
	}

	//std::cout << r1 << " " << r2 << std::endl;
	
	if (v1.x * v2.x + v1.y * v2.y > 0)
	{
		return true;
	}

	if (abs(r1 - r2) > threshold)
	{
		return false;
	}

	if (r1 < 0 && r2 < 0)
	{
		return false;
	}

	return true;
}

void PinDetection::collect_pixels_in_min_box(cv::Rect2f & max_rect, float min_box_width, float min_box_height, std::vector<std::vector<float>> & minbox_line_func, std::vector<cv::Point2i>& points)
{
	max_rect = cv::Rect2i(max_rect);

	cv::Point2i tmp;
	float dis[4] = {0}, dis_sum = 0.0;

	for (int y = max_rect.y; y < max_rect.y + max_rect.height; ++y)
	{
		for (int x = max_rect.x; x < max_rect.x + max_rect.width; ++x)
		{
			tmp.x = x;
			tmp.y = y;
			dis_sum = 0;

			for (int i = 0; i < 4; ++i)
			{
				dis[i] = get_distance_point_to_line(tmp, minbox_line_func[i][0], minbox_line_func[i][1], minbox_line_func[i][2]);
				dis_sum += dis[i];
			}
			if (abs(dis_sum - (min_box_height + min_box_width)) < 2)
			{ 
				points.push_back(cv::Point2i(x, y));
			}
		}
	}
}

void PinDetection::get_max_box_rect(std::vector<cv::Point2i>& _points_vec, cv::Rect2f & max_rect)
{
	std::vector<int> x_vec, y_vec;

	for (auto i : _points_vec)
	{
		x_vec.push_back(i.x);

		y_vec.push_back(i.y);
	}

	auto min_max_x = std::minmax_element(x_vec.begin(), x_vec.end());

	auto min_max_y = std::minmax_element(y_vec.begin(), y_vec.end());

	max_rect = cv::Rect2i(
		cv::Point2i(*min_max_x.first, *min_max_y.first),
		cv::Point2i(*min_max_x.second, *min_max_y.second));
}

void PinDetection::get_line_segment_from_points(std::vector<cv::Point2i>& points, std::vector<std::vector<float>>& line_segments)
{
	line_segments.resize(4);

	float
		A = 0.0, B = 0.0, C = 0.0,
		x1 = 0.0, x2 = 0.0, y1 = 0.0, y2 = 0.0;
	
	for (int i = 0; i < 4; i++)
	{
		x1 = points[i].x;
		y1 = points[i].y;

		x2 = points[(i + 1) % 4].x;
		y2 = points[(i + 1) % 4].y;

		A = y2 - y1;
		B = x1 - x2;
		C = x2 * y1 - x1 * y2;

		line_segments[i] = std::vector<float>{ A, B, C };
	}
}

void PinDetection::get_line_segment_from_points(std::vector<cv::Point2i> & points, std::vector<float> & line_segment)
{
	line_segment.resize(3);

	float
		A = 0.0, B = 0.0, C = 0.0,
		x1 = 0.0, x2 = 0.0, y1 = 0.0, y2 = 0.0;

	x1 = points[0].x;
	y1 = points[0].y;

	x2 = points[1].x;
	y2 = points[1].y;

	A = y2 - y1;
	B = x1 - x2;
	C = x2 * y1 - x1 * y2;

	line_segment = std::vector<float>{ A, B, C };
}

void PinDetection::detecte_middle_part(cv::Mat &img, std::vector<cv::Point2i>& middle_part_points, float probability, bool & is_positive, cv::Point2i & direction)
{
	std::vector<cv::Point2i> valid_points_vec;

	for (int i = 0; i < middle_part_points.size(); ++i)
	{
		cv::Vec3b & c = img.at<cv::Vec3b>(middle_part_points[i]);
		if (sqrt((c[0] - 255)*(c[0] - 255) + (c[1] - 255)*(c[1] - 255) + (c[2] - 255)*(c[2] - 255)) < 100 )
		{
			valid_points_vec.push_back(middle_part_points[i]);
		}
	}

	std::cout << valid_points_vec.size() / (float)middle_part_points.size() << std::endl;

	if (valid_points_vec.size() / (float)middle_part_points.size() > probability)
	{
		is_positive = true;
	}
	else
	{
		is_positive = false;

		auto mean_p = cv::mean(valid_points_vec);

		direction = { cv::Point2i(mean_p.val[0], mean_p.val[1]) };
	}
}

void PinDetection::detecte_both_side(cv::Mat & img, std::vector<cv::Point2i>& side, float & is_opening_probability)
{
	int valid_count = 0;

	for (int i = 0; i < side.size(); ++i)
	{
		cv::Vec3b & c = img.at<cv::Vec3b>(side[i]);
		if (sqrt((c[0] - 255)*(c[0] - 255) + (c[1] - 255)*(c[1] - 255) + (c[2] - 255)*(c[2] - 255)) < 100)
		{
			valid_count++;
		}
	}
	is_opening_probability = valid_count / (float)side.size();
}

void PinDetection::get_grasp_position(cv::Mat & img, std::vector<cv::Point2i>& side_1, std::vector<cv::Point2i>& side_2, std::vector<cv::Point2i>& grasp_position)
{
	float pro_1 = 0.0, pro_2 = 0.0;
	detecte_both_side(img, side_1, pro_1);
	detecte_both_side(img, side_2, pro_2);

	grasp_position.resize(2);

	if (pro_1 > pro_2)
	{
		grasp_position[0] = (m_min_rect[0] + m_inner_p[0]) / 2;
		grasp_position[1] = (m_min_rect[1] + m_inner_p[1]) / 2;
	}
	else
	{
		grasp_position[0] = (m_min_rect[2] + m_inner_p[3]) / 2;
		grasp_position[1] = (m_min_rect[3] + m_inner_p[2]) / 2;
	}
	cv::circle(img, grasp_position[0], 3, cv::Scalar(0, 0, 0), -1);
	cv::circle(img, grasp_position[1], 3, cv::Scalar(0, 0, 0), -1);
	//cv::imshow("test", img);
	//cv::waitKey(0);
}

void PinDetection::get_is_has_needle(cv::Mat & img, std::vector<cv::Point2i> &line_points,std::vector<cv::Point2i>& middle_region_points,  bool & is_has_needle)
{
	// detecting if needle is installed on the pin.
	std::vector<float> line_segment;

	get_line_segment_from_points(line_points, line_segment);

	float A, B, C;
	A = line_segment[0];
	B = line_segment[1];
	C = line_segment[2];

	cv::Rect2f max_rect;

	get_max_box_rect(line_points, max_rect);

	//cv::rectangle(img, max_rect, cv::Scalar::all(0));

	int x = 0, y = 0, consid_count = 3;

	std::vector<cv::Vec3b> c_vec;

	// collect the points in max rect of line

	for (int i=0;i< middle_region_points.size();++i)
	{
		cv::Vec3b & c = img.at<cv::Vec3b>(middle_region_points[i]);
		c_vec.push_back(c);
	}
	auto mean_val_c = cv::mean(c_vec);

	float mean_val_total = (mean_val_c[0] + mean_val_c[1] + mean_val_c[2]) / 3;

	c_vec.clear();
	
	// collect the points on line
	if (max_rect.width > max_rect.height)
	{
		for (x = max_rect.x; x < max_rect.x + max_rect.width; ++x)
		{
			float y = (-C - A * x) / B;

			for (int i = 0; i < consid_count; ++i)
			{
				cv::Vec3b & c = img.at<cv::Vec3b>(y - (consid_count / 2) + i, x);
				c_vec.push_back(c);
			}
		}
	}
	else
	{
		for (y = max_rect.y; y < max_rect.y + max_rect.height; ++y)
		{
			float x = (-C - B * y) / A;

			for (int i = 0; i < consid_count; ++i)
			{
				cv::Vec3b & c = img.at<cv::Vec3b>(y, x - (consid_count / 2) + i);
				c_vec.push_back(c);
			}
		}
	}

	auto mean_val_line_c = cv::mean(c_vec);

	float mean_val_line = (mean_val_line_c[0] + mean_val_line_c[1] + mean_val_line_c[2]) / 3;

	if (mean_val_line < mean_val_total)
	{
		is_has_needle = true;
	}
	else
	{
		is_has_needle = false;
	}
	std::cout << "is_has_needle: " << is_has_needle << " " << mean_val_line << " " << mean_val_total << std::endl;

	cv::imshow("test", img);
	cv::waitKey(0);
}

bool PinDetection::is_needle(cv::Vec3b & c, float threshold)
{
	if (sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2])< threshold)
	{
		return true;
	}
	else
	{
		return false;
	}
}
