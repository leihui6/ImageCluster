#include "PinDetection.h"

PinDetection::PinDetection()
{
}

PinDetection::~PinDetection()
{
}

void PinDetection::detect(cv::Mat & image, Cluster& _cluster, PinDetectionResult & pin_detection_result)
{
	std::vector<cv::Point2i> min_rect, points_in_min_box;
	float min_rect_width = 0.0, min_rect_height = 0.0;
	std::vector<std::vector<float>> line_segment;
	// This rect is different from cluster's max rect, this is a max rect of min rect points
	cv::Rect2i max_rect;

	_cluster.get_min_box(min_rect);
	_cluster.get_min_box_size(min_rect_width, min_rect_height);
	_cluster.get_min_box_line_segment_function(line_segment);
	get_max_box_rect(min_rect, max_rect);

	// Ordering each rect's point
	order_rect_points(min_rect);

	collect_pixels_in_min_box(max_rect, min_rect_width, min_rect_height, line_segment, points_in_min_box);

	//for (auto & i: points_in_min_box)
	//{
	//	auto & color = image.at<cv::Vec3b>(i);
	//	color[0] = 255;
	//	color[1] = 255;
	//	color[2] = 255;
	//}
	//cv::imshow("test",image);
	//cv::waitKey(0);

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
			std::cout << d << std::endl;
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

	std::cout << r1 << " " << r2 << std::endl;
	
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

void PinDetection::collect_pixels_in_min_box(cv::Rect2i & max_rect, float min_box_width, float min_box_height, std::vector<std::vector<float>> & minbox_line_func, std::vector<cv::Point2i>& points)
{
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
			if (abs(dis_sum - (min_box_height + min_box_width)) < 5)
			{ 
				points.push_back(cv::Point2i(x, y));
			}
		}
	}
}

void PinDetection::get_max_box_rect(std::vector<cv::Point2i>& _points_vec, cv::Rect2i & max_rect)
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
