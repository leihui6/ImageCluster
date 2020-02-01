#include "ImageCluster.h"

ImageCluster::ImageCluster()
	:m_image_data(nullptr)
{

}

ImageCluster::~ImageCluster()
{
	delete_pointer(&m_image_data);
}

int ImageCluster::load_image(unsigned char * _image, int _width, int _height)
{
	if (!_image)
	{
		return 1;
	}
	m_image_data = new unsigned char[_width * _height + 1]();

	memcpy(m_image_data, _image, _width * _height);
	
	m_width		= _width;
	
	m_height	= _height;

	return 0;
}

int ImageCluster::init_kernel_size(int _kernel_width, int _kernel_height)
{
	if (!m_image_data)
	{
		return 1;
	}

	m_kernel_width = _kernel_width;

	m_kernel_height = _kernel_height;

	m_kernel_count_col = m_width / _kernel_width;

	m_kernel_count_row = m_height / _kernel_height;

	if (m_kernel_count_col < (double)m_width / _kernel_width 
		|| 
		m_kernel_count_row < (double)m_height / _kernel_height
		)
	{
		std::cerr << "[warning]" << "kernel's count on row or col is not a integer" << std::endl;
	}

	m_kernel_vec.resize(m_kernel_count_col * m_kernel_count_row, false);

	m_visited_vec.resize(m_kernel_count_col * m_kernel_count_row, false);

#ifdef _IMAGECLUSTER_DEBUG_

	std::cout << "Created kernel size=" << m_kernel_vec.size() << " " << std::endl;

#endif // _IMAGECLUSTER_DEBUG_

	check_kernel_vec();

	return 0;
}

int ImageCluster::cluster(int _threshold)
{

	if (!m_image_data)
	{
		return 1;
	}

	//std::vector<int> near_points;
	//get_nearby_points_by_index(4, near_points);

	bool is_visited = false, is_vailed = false;

	std::deque<int> cluster_index_q;

	std::vector<int> collect_index_vec, near_points;

	for (int i = 0; i < m_kernel_vec.size(); ++i)
	{
		is_visited = m_visited_vec[i];
		
		if (is_visited)
		{
			continue;
		}

		m_visited_vec[i] = true;

		is_vailed = m_kernel_vec[i];

		if (!is_vailed)
		{
			continue;
		}

		cluster_index_q.push_back(i);
		
		int tar_i = 0;
		while(!cluster_index_q.empty())
		{
			tar_i = cluster_index_q.front();
			
			collect_index_vec.push_back(tar_i);

			cluster_index_q.pop_front();

			get_nearby_points_by_index(tar_i, near_points);

//#ifdef _IMAGECLUSTER_DEBUG_
//			for (auto i : near_points)
//				std::cout << i << " ";
//			std::cout << std::endl;
//#endif // _IMAGECLUSTER_DEBUG_

			for (auto j : near_points)
			{
				if (j != -1 && m_kernel_vec[j] && !m_visited_vec[j])
				{
					cluster_index_q.push_back(j);

					//std::cout << cluster_index_q.size() << std::endl;

					m_visited_vec[j] = true;
				}
			}
		}

		m_total_cluster_kernels.push_back(collect_index_vec);

		collect_index_vec.clear();
	}

#ifdef _IMAGECLUSTER_DEBUG_
	
	for (int i = 0; i < m_total_cluster_kernels.size(); ++i)
	{
		std::cout << "i=" << i << " pixel size=" << m_total_cluster_kernels[i].size() << std::endl;
	}

#endif // _IMAGECLUSTER_DEBUG_

	// convert kernel cluster int pixel cluster
	convert_kernel_cluster(_threshold);

	return 0;
}

void ImageCluster::get_clusters(std::vector<Cluster>& _total_cluster)
{
	_total_cluster = m_total_clusters;
}

void ImageCluster::clear()
{
	m_kernel_vec.clear();

	m_visited_vec.clear();

	m_total_cluster_kernels.clear();

	m_total_clusters.clear();
}

void ImageCluster::check_kernel_vec()
{
#ifdef _IMAGECLUSTER_DEBUG_
	cv::Mat kernel_image(cv::Size(m_width, m_height), CV_8UC1, cv::Scalar::all(0));
#endif // _IMAGECLUSTER_DEBUG_

	int x = 0, y = 0;

	bool is_vaild = false;

	for (int i = 0; i < m_kernel_vec.size(); ++i)
	{
		get_start_point_by_index(i, x, y);

		is_vaild = check_one_kernel(x, y);

		if (is_vaild)
		{
			m_kernel_vec[i] = is_vaild;

#ifdef _IMAGECLUSTER_DEBUG_
			kernel_image.at<uchar>(y, x) = 255;
#endif // _IMAGECLUSTER_DEBUG_
		}

	}
#ifdef _IMAGECLUSTER_DEBUG_
	cv::imshow("Check Image Kernel", kernel_image);
	cv::waitKey(200);
#endif // _IMAGECLUSTER_DEBUG_
}

bool ImageCluster::check_one_kernel(int x, int y)
{
	int tar_i = 0;

	for (int j = y; j < y + m_kernel_height; ++j)
	{
		for (int k = x; k < x + m_kernel_width; ++k)
		{
			tar_i = j * m_width + k;

			if (m_image_data[tar_i] == 255)
			{
				return true;
			}
		}
	}
	return false;
}

void ImageCluster::get_start_point_by_index(int _index, int &_x, int &_y)
{
	int
		A1 = _index / m_kernel_count_col,

		B1 = _index % m_kernel_count_col;

	_x = B1 * m_kernel_width;
	
	_y = A1 * m_kernel_height;
}

void ImageCluster::get_nearby_points_by_index(int _index, std::vector<int>& _near_points)
{
	_near_points.resize(8, -1);
	/*
	[0]  [1]  [2]
	[3] index [4]
	[5]  [6]  [7]
	*/
	_near_points[0] = _index - m_kernel_count_col - 1;
	_near_points[1] = _index - m_kernel_count_col;
	_near_points[2] = _index - m_kernel_count_col + 1;
	_near_points[3] = _index - 1;
	//_near_points[4] = _index;
	_near_points[4] = _index + 1;
	_near_points[5] = _index + m_kernel_count_col - 1;
	_near_points[6] = _index + m_kernel_count_col;
	_near_points[7] = _index + m_kernel_count_col + 1;

	// the index of point at left side 
	if (_index % m_kernel_count_col == 0)
	{
		_near_points[0] = - 1;
		_near_points[3] = - 1;
		_near_points[5] = - 1;
	}
	// the index of point at right side 
	else if ((_index + 1) % m_kernel_count_col == 0)
	{
		_near_points[2] = -1;
		_near_points[4] = -1;
		_near_points[7] = -1;
	}

	int all_kernel_size = m_kernel_count_col * m_kernel_count_row;

	// filter the top and bottom index
	for (auto &i : _near_points)
	{
		i = (i < 0) ? -1 : i;

		i = (i >= all_kernel_size) ? -1 : i;
	}
}

void ImageCluster::convert_kernel_cluster(int _threshold)
{
	for (int i = 0; i < m_total_cluster_kernels.size(); ++i)
	{
		int x = 0, y = 0, tar_i = 0;

		std::vector<cv::Point2i> cluster_pixels;

		for (int j = 0; j < m_total_cluster_kernels[i].size(); ++j)
		{
			tar_i = m_total_cluster_kernels[i][j];

			get_start_point_by_index(tar_i, x, y);

			collect_pixels(x, y, cluster_pixels);
		}

		if (cluster_pixels.size() > _threshold)
		{
			Cluster cluster;

			cluster.init_cluster(cluster_pixels);
			
			m_total_clusters.push_back(cluster);
		}
		else
		{
			std::cerr << "[warning] one cluster's pixel are less than" <<  _threshold << ", size= " << cluster_pixels.size() << std::endl;
		}
	}
}

void ImageCluster::collect_pixels(int _x, int _y, std::vector<cv::Point2i>& _cluster)
{
	int tar_i = 0;

	for (int j = _y; j < _y + m_kernel_height; ++j)
	{
		for (int k = _x; k < _x + m_kernel_width; ++k)
		{
			tar_i = j * m_width + k;

			if (m_image_data[tar_i] == 255)
			{
				_cluster.push_back(cv::Point2i(k, j));
			}
		}
	}
}

void ImageCluster::delete_pointer(unsigned char ** _ptr)
{
	if (*_ptr)
	{
		delete[] *_ptr;
	}
	*_ptr = nullptr;
}
