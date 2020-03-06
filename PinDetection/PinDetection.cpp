#include "PinDetection.h"

PinDetection::PinDetection()
{
	m_middle_positive_threshold = 125;

	m_middle_orientation_threshold = 125;

	m_both_side_threshold = 125;

	m_is_middle_positive_probaility = 0.75f;

	m_is_needle_threshold = 23;
}

PinDetection::~PinDetection()
{
}

int PinDetection::find_ROI(cv::Mat & image, cv::Mat & res)
{
	cv::Mat gray;

	if (image.type() == CV_8UC1)
	{
		gray = image;
	}
	else
	{
		cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
	}

	cv::medianBlur(gray, gray, 5);

	std::vector<int> markerIds;

	std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;

	std::vector<cv::Point2i> corner_points(4);

	cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

	cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);

	cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);

	if (markerCorners.size() != 4)
	{
		return markerCorners.size();
	}

	for (size_t i = 0; i < markerCorners.size(); i++)
	{
		std::vector<cv::Point2f> &c = markerCorners[i];

		if (markerIds[i] == 1)
		{
			corner_points[i] = c[2];
		}
		else if (markerIds[i] == 2)
		{
			corner_points[i] = c[3];
		}
		else if (markerIds[i] == 3)
		{
			corner_points[i] = c[1];
		}
		else if (markerIds[i] == 4)
		{
			corner_points[i] = c[0];
		}
	}

	order_rect_points(corner_points);

	cv::Point2i min_p, max_p;
	
	get_min_max_point_in_vector(corner_points, min_p, max_p);

	float inner_distance = 10;

	min_p.x += inner_distance;
	min_p.y += inner_distance;

	max_p.x -= inner_distance;
	max_p.y -= inner_distance;

	res = image(cv::Rect2i(min_p, max_p));

	std::cout << "min_p=" << min_p << " max_p=" << max_p << std::endl;

	//cv::imshow("test", res);

	//cv::waitKey(0);
	return markerCorners.size();
}

void PinDetection::detect(cv::Mat & image, Cluster& _cluster, PinDetectionResult & pin_detection_result)
{
	_cluster.get_min_box(m_min_rect);
	_cluster.get_center_point(pin_detection_result.ceter_position);

	order_rect_points(m_min_rect);
	m_inner_p.resize(4);

	/*
	0-----1 1-----0 3-----2 2-----3
	|     |	|     |	|     |	|     |
	|     |	|     |	|     |	|     |
	|     |	|     |	|     |	|     |
	3-----2	2-----3	0-----1	1-----0
	*/
	float d = 0.0;
	d = ((get_distance(m_min_rect[1], m_min_rect[2]) + get_distance(m_min_rect[0], m_min_rect[3])) / 2.0f) * (1 / 5.0f);

	cv::Point2i dir_line;
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

		get_max_box_rect(inner_part_vertex[i], inner_max_rect[i], image.cols, image.rows);

		get_line_segment_from_points(inner_part_vertex[i], inner_line_segments[i]);
	}

	m_inner_part_points.resize(3);

	for (int i = 0; i < 3; ++i)
	{
		collect_pixels_in_min_box(inner_max_rect[i], inner_size[i].width, inner_size[i].height, inner_line_segments[i], m_inner_part_points[i]);
	}

	// First thing we need to do is to detect the middle part.
	bool middle_is_positive = false;

	detecte_middle_part(image, m_inner_part_points[1], middle_is_positive);

	std::cout << "middle_is_positive= " << middle_is_positive << std::endl;
	
	if (middle_is_positive)
	{
		pin_detection_result.pin_status = FACEUP;
	}
	else
	{
		pin_detection_result.pin_status = FACESIDE;
	}

	// This needle is not positive.
	if (!middle_is_positive)
	{
		// in this case, has or doesnt has needle doesnt matter.
		pin_detection_result.is_has_needle = false;
		
		detect_middle_orientation(image, m_inner_part_points[1], m_inner_p[0], m_inner_p[2], m_inner_p[1], m_inner_p[3], pin_detection_result.point_on_base_side);

		pin_detection_result.rotate_direction = pin_detection_result.point_on_base_side - pin_detection_result.ceter_position;
		
		//cv::circle(image, orientation_point, 2, cv::Scalar(0, 0, 0), -1);
	}
	// this needle is positive, and we need to detecte the opening side.
	else
	{
		cv::Point2i needle_p[2];

		get_if_has_needle(image, pin_detection_result.is_has_needle, needle_p[0], needle_p[1]);

		get_grasp_position(image, m_inner_part_points[0], m_inner_part_points[2], pin_detection_result.opening_position, pin_detection_result.closing_position);

		//cv::line(image, needle_p[0], needle_p[1], cv::Scalar::all(0), 2);
		
		//cv::imshow("test", image);
		
		//cv::waitKey(0);
	}
}

void PinDetection::clear()
{
	m_p_a_circle.clear();

	m_p_b_circle.clear();

	m_pixels_in_min_box.clear();

	/*
	m_min_rect[2]-----m_min_rect[3]
		 |                 |
		 1                 2
		 |                 |
		 3                 0
		 |                 |
	m_min_rect[1]-----m_min_rect[0]
	*/
	m_min_rect.clear();

	m_inner_p.clear();

	m_inner_part_points.clear();
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

void PinDetection::get_max_box_rect(std::vector<cv::Point2i>& _points_vec, cv::Rect2f & max_rect, int limited_width, int limited_height)
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

	max_rect.x = (max_rect.x < 0) ? 0 : max_rect.x;

	max_rect.y = (max_rect.y < 0) ? 0 : max_rect.y;

	max_rect.width = ((max_rect.x + max_rect.width) > limited_width) ? limited_width - max_rect.x : max_rect.width;

	max_rect.height = ((max_rect.y + max_rect.height) > limited_height) ? limited_height - max_rect.y : max_rect.height;
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

void PinDetection::get_line_segment_from_points(cv::Point2i & point_0, cv::Point2i & point_1, std::vector<float> & line_segment)
{
	line_segment.resize(3);

	float
		A = 0.0, B = 0.0, C = 0.0,
		x1 = 0.0, x2 = 0.0, y1 = 0.0, y2 = 0.0;

	x1 = point_0.x;
	y1 = point_0.y;

	x2 = point_1.x;
	y2 = point_1.y;

	A = y2 - y1;
	B = x1 - x2;
	C = x2 * y1 - x1 * y2;

	line_segment = std::vector<float>{ A, B, C };
}

void PinDetection::detecte_middle_part(cv::Mat &img, std::vector<cv::Point2i>& middle_part_points, bool & is_positive)
{
	int valid_count = 0, g = 0;

	for (int i = 0; i < middle_part_points.size(); ++i)
	{
		cv::Vec3b & c = img.at<cv::Vec3b>(middle_part_points[i]);

		rgb_to_gray(c, g);

		if (g > m_middle_positive_threshold)
		{
			valid_count++;
		}
	}

	float pro = valid_count / (float)middle_part_points.size();

	std::cout << "[positive]" << pro << std::endl;

	if (pro > m_is_middle_positive_probaility)
	{
		is_positive = true;
	}
	else
	{
		is_positive = false;
	}
}

void PinDetection::detecte_both_side(cv::Mat & img, std::vector<cv::Point2i>& side, float & is_opening_probability)
{
	int valid_count = 0, g = 0;

	for (int i = 0; i < side.size(); ++i)
	{
		cv::Vec3b & c = img.at<cv::Vec3b>(side[i]);

		rgb_to_gray(c, g);

		if (g > m_both_side_threshold)
		{
			valid_count++;
		}
	}
	is_opening_probability = valid_count / (float)side.size();
}

void PinDetection::get_grasp_position(cv::Mat & img, std::vector<cv::Point2i>& side_1, std::vector<cv::Point2i>& side_2, cv::Point2i & opening_position, cv::Point2i & closing_position)
{
	// points probability close to white color
	float pro_1 = 0.0, pro_2 = 0.0;

	detecte_both_side(img, side_1, pro_1);

	detecte_both_side(img, side_2, pro_2);
	auto
		mean_point_1 = cv::mean(side_1),

		mean_point_2 = cv::mean(side_2);

	if (pro_1 > pro_2)
	{
		opening_position = cv::Point2i(mean_point_1[0], mean_point_1[1]);

		closing_position = cv::Point2i(mean_point_2[0], mean_point_2[1]);
	}
	else
	{
		closing_position = cv::Point2i(mean_point_1[0], mean_point_1[1]);

		opening_position = cv::Point2i(mean_point_2[0], mean_point_2[1]);
	}
}

void PinDetection::get_if_has_needle(cv::Mat & img, bool & is_has_needle, cv::Point2i & needle_p_0, cv::Point2i & needle_p_1)
{
	/*
	0 --.--.--.--.--[.]--> 1(skip the last point)


	2 --.--.--.--.--[.]--> 3(skip the last point)
	*/
	std::vector<cv::Point2i> base_line_0, base_line_1;

	const int base_point_size = 5;

	base_line_0.resize(base_point_size);

	base_line_1.resize(base_point_size);

	cv::Point2i dir_line;

	float d0 = get_distance(m_inner_p[1], m_inner_p[0]) / base_point_size;

	dir_line = m_inner_p[1] - m_inner_p[0];
	
	for (int i = 0; i < base_point_size - 1; ++i)
	{
		get_point_along_with_distance(m_inner_p[0], d0*(i + 1), base_line_0[i], dir_line);
	}

	dir_line = m_inner_p[3] - m_inner_p[2];

	for (int i = 0; i < base_point_size - 1; ++i)
	{
		get_point_along_with_distance(m_inner_p[2], d0*(i + 1), base_line_1[i], dir_line);
	}
	
	//test
	//cv::line(img, m_inner_p[0], m_inner_p[1], cv::Scalar::all(0), 2);
	//cv::line(img, m_inner_p[1], m_inner_p[3], cv::Scalar::all(0), 2);
	//cv::line(img, m_inner_p[3], m_inner_p[2], cv::Scalar::all(0), 2);
	//cv::line(img, m_inner_p[2], m_inner_p[0], cv::Scalar::all(0), 2);

	//for (int i = 0; i < base_line_0.size(); ++i)
	//{
	//	cv::circle(img, base_line_0[i], 1, cv::Scalar::all(255));
	//	cv::circle(img, base_line_1[i], 1, cv::Scalar::all(255));
	//}

	float min_line_value = FLT_MAX;

	int min_i = 0, min_j = 0;

	std::vector<float> line_value_vec;

	for (int i = 0; i < base_point_size - 1; ++i)
	{
		for (int j = 0; j < base_point_size - 1; ++j)
		{
			cv::LineIterator it(img, base_line_0[i], base_line_1[j]);

			float line_value = 0.0;

			int g = 0;

			for (int k = 0; k < it.count; k++)
			{
				cv::Vec3b & c = img.at<cv::Vec3b>(it++.pos());

				rgb_to_gray(c, g);

				line_value += (float)g;
			}
			line_value = line_value / it.count;

			if (line_value < min_line_value)
			{
				min_line_value = line_value;
				min_i = i;
				min_j = j;
			}

			line_value_vec.push_back(line_value);
		}
	}
	
	std::sort(line_value_vec.begin(), line_value_vec.end());

	if (line_value_vec.size() < 2)
	{
		line_value_vec.push_back(0);
	}

	float sum_value = std::accumulate(line_value_vec.begin() + 1, line_value_vec.end(), 0);

	float mean_needle_value = sum_value / (line_value_vec.size() - 1);

	std::cout << "abs(min_line_value - mean_needle_value)=" << abs(min_line_value - mean_needle_value) <<std::endl;

	if (abs(min_line_value - mean_needle_value) > m_is_needle_threshold)
	{
		is_has_needle = true;

		needle_p_0 = base_line_0[min_i];

		needle_p_1 = base_line_1[min_j];
	}
	else
	{
		is_has_needle = false;
	}
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

void PinDetection::detect_middle_orientation(cv::Mat & img, std::vector<cv::Point2i>& middle_part_points, cv::Point2i & line_0_0, cv::Point2i & line_0_1, cv::Point2i & line_1_0, cv::Point2i & line_1_1, cv::Point2i & orientation_point, float threshold)
{
	std::vector<float> line_segment_0, line_segment_1;

	get_line_segment_from_points(line_0_0, line_0_1, line_segment_0);

	get_line_segment_from_points(line_1_0, line_1_1, line_segment_1);

	//cv::line(img, line_0_0, line_0_1, cv::Scalar::all(0));
	//cv::line(img, line_1_0, line_1_1, cv::Scalar::all(0));

	int valid_count[2] = { 0,0 };
	
	int g = 0;

	for (int i = 0; i < middle_part_points.size(); i++)
	{
		cv::Point2i &p = middle_part_points[i];

		cv::Vec3b &c = img.at<cv::Vec3b>(p);

		rgb_to_gray(c, g);

		if (g > m_middle_orientation_threshold)
		{
			float d0, d1;
			d0 = get_distance_point_to_line(p, line_segment_0[0], line_segment_0[1], line_segment_0[2]);

			d1 = get_distance_point_to_line(p, line_segment_1[0], line_segment_1[1], line_segment_1[2]);

			if (d0 < d1)
			{
				valid_count[0]++;
			}
			else
			{
				valid_count[1]++;
			}
		}
	}
	if (valid_count[0] > valid_count[1])
	{
		orientation_point = (line_0_0 + line_0_1) / 2;
	}
	else
	{
		orientation_point = (line_1_0 + line_1_1) / 2;
	}
}

inline void PinDetection::rgb_to_gray(cv::Vec3b & c, int & g)
{
	g = 0.21*c[2] + 0.72*c[1] + 0.07*c[0];
}

void PinDetection::get_min_max_point_in_vector(std::vector<cv::Point2i>& points, cv::Point2i & min_p, cv::Point2i & max_p)
{
	int min_x = INT_MAX, min_y= INT_MAX;

	int max_x = INT_MIN, max_y = INT_MIN;

	for (int i = 0; i < points.size(); ++i)
	{
		int t_x = points[i].x,
			t_y = points[i].y;

		if (t_x < min_x)
		{
			min_x = t_x;
		}
		if (t_y < min_y)
		{
			min_y = t_y;
		}

		if (t_x > max_x)
		{
			max_x = t_x;
		}
		if (t_y > max_y)
		{
			max_y = t_y;
		}
	}
	min_p.x = min_x;
	min_p.y = min_y;

	max_p.x = max_x;
	max_p.y = max_y;
}
