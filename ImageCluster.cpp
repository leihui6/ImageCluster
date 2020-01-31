#include "ImageCluster.h"



ImageCluster::ImageCluster()
	:m_image_data(nullptr)
{

}


ImageCluster::~ImageCluster()
{
	delete_pointer(&m_image_data);
}

int ImageCluster::load_image(unsigned char * _image, size_t _width, size_t _height)
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

int ImageCluster::init_kernel_size(size_t _kernel_width, size_t _kernel_height)
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
		std::cerr << "[warning]" << "kernel count on row or col is not a integer" << std::endl;
	}

	m_kernel_vec.resize(m_kernel_count_col * m_kernel_count_row, false);

	m_visited_vec.resize(m_kernel_count_col * m_kernel_count_row, false);

#ifdef _DEBUG_

	std::cout << "Created kernel size=" << m_kernel_vec.size() << " " << std::endl;

#endif // _DEBUG_

	check_kernel_vec();

	return 0;
}

int ImageCluster::cluster()
{
	if (!m_image_data)
	{
		return 1;
	}



	return 0;
}

void ImageCluster::check_kernel_vec()
{
#ifdef _DEBUG_
	cv::Mat kernel_image(cv::Size(m_width, m_height), CV_8UC1, cv::Scalar::all(0));
#endif // _DEBUG_

	size_t x = 0, y = 0;

	bool is_vaild = false;

	for (size_t i = 0; i < m_kernel_vec.size(); ++i)
	{
		get_start_point_by_index(i, x, y);

		is_vaild = check_one_kernel(x, y);

		if (is_vaild)
		{
			m_kernel_vec[i] = is_vaild;

#ifdef _DEBUG_
			kernel_image.at<uchar>(y, x) = 255;
#endif // _DEBUG_
		}

	}
#ifdef _DEBUG_
	cv::imshow("Check Image Kernel", kernel_image);
	cv::waitKey();
#endif // _DEBUG_
}

bool ImageCluster::check_one_kernel(size_t x, size_t y)
{
	size_t tar_i = 0;

	for (size_t j = y; j < y + m_kernel_height; ++j)
	{
		for (size_t k = x; k < x + m_kernel_width; ++k)
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

void ImageCluster::get_start_point_by_index(size_t _index, size_t &_x, size_t &_y)
{
	size_t
		A1 = _index / m_kernel_count_col,

		B1 = _index % m_kernel_count_col;

	_x = B1 * m_kernel_width;
	
	_y = A1 * m_kernel_height;
}

void ImageCluster::get_nearby_points_by_index(size_t _index, std::vector<size_t>& _near_points)
{
	_near_points.resize(8);
	_near_points[0] = _index - m_kernel_count_col - 1;
	_near_points[1] = _index - m_kernel_count_col - 1;
	_near_points[2] = _index - m_kernel_count_col - 1;
	_near_points[3] = _index - m_kernel_count_col - 1;
	//_near_points[4] = _index - m_kernel_count_col - 1;
	_near_points[5] = _index - m_kernel_count_col - 1;
	_near_points[6] = _index - m_kernel_count_col - 1;
	_near_points[7] = _index - m_kernel_count_col - 1;
	_near_points[8] = _index - m_kernel_count_col - 1;
}

void ImageCluster::delete_pointer(unsigned char ** _ptr)
{
	if (*_ptr)
	{
		delete[] *_ptr;
	}
	*_ptr = nullptr;
}
