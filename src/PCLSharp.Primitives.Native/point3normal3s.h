#pragma once
#include "point3normal3.h"

/// <summary>
/// ����㷨�������ṹ��
/// </summary>
struct Point3Normal3s
{
	/// <summary>
	/// �޲ι�����
	/// </summary>
	Point3Normal3s() = default;

	/// <summary>
	/// ��������㷨�������ṹ�幹����
	/// </summary>
	/// <param name="pointNormals">����㷨������ָ��</param>
	/// <param name="length">����</param>
	Point3Normal3s(Point3Normal3* pointNormals, const int& length)
		:PointNormals(pointNormals), Length(length)
	{

	}

	/// <summary>
	/// ��������
	/// </summary>
	~Point3Normal3s()
	{
		delete[] this->PointNormals;
	}

	/// <summary>
	/// ����㷨������ָ��
	/// </summary>
	Point3Normal3* PointNormals;

	/// <summary>
	/// ����
	/// </summary>
	int Length;
};
