#pragma once
#include "point3f.h"

/// <summary>
/// ����㼯
/// </summary>
struct Point3Fs
{
	/// <summary>
	/// �޲ι�����
	/// </summary>
	Point3Fs() = default;

	/// <summary>
	/// ��������㼯������
	/// </summary>
	/// <param name="points">�㼯ָ��</param>
	/// <param name="length">����</param>
	Point3Fs(Point3F* points, const int& length)
		:Points(points), Length(length)
	{

	}

	/// <summary>
	/// ��������
	/// </summary>
	~Point3Fs()
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
