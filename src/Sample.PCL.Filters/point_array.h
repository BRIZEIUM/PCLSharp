#pragma once
#include "point3f.h"

/// <summary>
/// 3D���������
/// </summary>
struct PointArray
{
public:

	/// <summary>
	/// ��������
	/// </summary>
	~PointArray()
	{
		delete[] this->Points;
	}

	/// <summary>
	/// �㼯ָ��
	/// </summary>
	Point3F* Points;

	/// <summary>
	/// ����
	/// </summary>
	int Length;
};
