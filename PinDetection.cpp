#include "PinDetection.h"



PinDetection::PinDetection()
{
}


PinDetection::~PinDetection()
{
}

void PinDetection::detect(cv::Mat & image, cv::Rect2i pin_region, std::vector<cv::Point2i>& rect, PinDetectionResult & pin_detection_result)
{
	// get order of every rect's point
	order_rect_points(rect);

	cv::Point2i p_a, p_b, p_0;
	p_a = (rect[0] + rect[1]) / 2;
	p_b = (rect[2] + rect[3]) / 2;
	p_0 = (rect[0] + rect[1] + rect[2] + rect[3]) / 4;
	
	float d = 0.0;
	std::cout << get_distance(rect[1], rect[2]) << std::endl;
	std::cout << get_distance(rect[0], rect[3]) << std::endl;

	// distance to figure out where is opening side.
	d = ((get_distance(rect[1], rect[2]) + get_distance(rect[0], rect[3])) / 2.0) * (1.0 / 5) * (1.0 / 2);

	cv::Point2i new_p_a, new_p_b, dir_line;

	dir_line = cv::Point2i(p_0.x - p_a.x, p_0.y - p_a.y);
	point_along_with_distance(p_a, d, new_p_a, dir_line);

	dir_line = cv::Point2i(p_0.x - p_b.x, p_0.y - p_b.y);
	point_along_with_distance(p_b, d, new_p_b, dir_line);

	for (int y = pin_region.y; y < pin_region.y + pin_region.height; ++y)
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
	}

	identify_side();

	cv::circle(image, p_0, 2, cv::Scalar(255, 0, 0), -1);
	cv::circle(image, p_a, 2, cv::Scalar(255, 0, 0), -1);
	cv::circle(image, p_b, 2, cv::Scalar(255, 0, 0), -1);
	cv::circle(image, new_p_a, d, cv::Scalar(255, 0, 0), 1);
	cv::circle(image, new_p_b, d, cv::Scalar(255, 0, 0), 1);
	for (auto i : m_p_a_circle)
	{
		//blue green red
		cv::Vec3b &color = image.at<cv::Vec3b>(i);

		color[0] = 255;
		color[1] = 255;
		color[2] = 255;
	}
	cv::imshow("test", image);
	cv::waitKey(0);

}

float PinDetection::get_distance(cv::Point2i pointO, cv::Point2i pointA)
{
	float distance;
	distance = powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2);
	return sqrtf(distance);;
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

void PinDetection::point_along_with_distance(cv::Point2i & p, float d, cv::Point2i & new_p, cv::Point2i& dir)
{
	double m, n;
	m = dir.x;
	n = dir.y;

	// M,N,L = x,y,l
	float
		M = sqrt((d*d*m*m) / (m*m + n * n)),
		N = sqrt((d*d*n*n) / (m*m + n * n));

	float x[2], y[2];

	x[0] = p.x + M;
	x[1] = p.x - M;
	y[0] = p.y + N;
	y[1] = p.y - N;

	//cout
	//	<< "p" << endl << p
	//	<< "line_dir" << endl << line_dir << endl;
	//cout
	//	<< "x:" << x[0] << " " << x[1] << endl
	//	<< "y:" << y[0] << " " << y[1] << endl
	//	<< "z:" << z[0] << " " << z[1] << endl;

	cv::Point2f v;

	std::vector<cv::Point2i> tmp_p;

	for (size_t i = 0; i < 2; i++)
	{
		v.x = x[i] - p.x;
		for (size_t j = 0; j < 2; j++)
		{
			v.y = y[j] - p.y;

			if (is_parallel_and_same_direction(v, dir,0.001))
			{
				cv::Point2f p;
				p.x = x[i];
				p.y = y[j];
				tmp_p.push_back(p);
			}
		}
	}

	new_p = tmp_p.back();

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

	if (r1 == 0)
	{
		return true;
	}
	else if (r2 == 0)
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
