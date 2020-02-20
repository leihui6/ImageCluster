#include "Cluster.h"



Cluster::Cluster()
{
}


Cluster::~Cluster()
{
}

int Cluster::init_cluster(std::vector<cv::Point2i>& _points_vec)
{
	m_cluster_pixels = _points_vec;

	max_box_rect(m_cluster_pixels);
	
	min_box_rect(m_cluster_pixels);

	return 0;
}

void Cluster::get_max_box(cv::Rect2i & _rect)
{
	_rect = m_max_box_rect;
}

void Cluster::get_min_box(std::vector<cv::Point2i> &_min_box_rect)
{
	_min_box_rect = m_min_box_rect;
}

void Cluster::get_min_box_size(float & _width, float & _height)
{
	_width = m_min_box_width;
	_height = m_min_box_height;
}

void Cluster::get_middle_points_of_lines(std::vector<cv::Point2i>& _m_min_box_middle_p)
{
	_m_min_box_middle_p = m_min_box_middle_p;
}

void Cluster::get_center_point(cv::Point2i & _p)
{
	_p = m_center_point;
}

void Cluster::get_cluster_pixels(std::vector<cv::Point2i>& _cluster_pixels)
{
	_cluster_pixels = m_cluster_pixels;
}

void Cluster::get_angle(float& _angle)
{
	_angle = m_angle;
}

void Cluster::max_box_rect(std::vector<cv::Point2i>& _points_vec)
{
	std::vector<int> x_vec, y_vec;

	for (auto i : _points_vec)
	{
		x_vec.push_back(i.x);

		y_vec.push_back(i.y);
	}

	auto min_max_x = std::minmax_element(x_vec.begin(), x_vec.end());

	auto min_max_y = std::minmax_element(y_vec.begin(), y_vec.end());

	m_max_box_rect = cv::Rect2i(
		cv::Point2i(*min_max_x.first, *min_max_y.first),
		cv::Point2i(*min_max_x.second, *min_max_y.second));
}

void Cluster::min_box_rect(std::vector<cv::Point2i>& _points_vec)
{
	cv::RotatedRect minRect = cv::minAreaRect(_points_vec);

	cv::Point2f vertex[4];

	minRect.points(vertex);

	m_min_box_rect.assign(vertex, vertex + 4);

	m_min_box_width = minRect.size.width;

	m_min_box_height = minRect.size.height;

	m_center_point = cv::Point2i((int)minRect.center.x, (int)minRect.center.y);

	m_angle = minRect.angle;

	if (minRect.size.width < minRect.size.height)
	{
		m_angle = 90 + m_angle;
	}

	int n_i = 0;

	m_min_box_middle_p.resize(4);

	for (int i = 0; i < 4; ++i)
	{
		m_min_box_middle_p[i] = cv::Point2i((int)(vertex[i].x + vertex[(i + 1) % 4].x) / 2, (int)(vertex[i].y + vertex[(i + 1) % 4].y) / 2);
	}
}

