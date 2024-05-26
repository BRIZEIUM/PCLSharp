#pragma once
#include "point3color4.h"

/// <summary>
/// �������ɫ��
/// </summary>
struct Point3Color4s
{
	/// <summary>
	/// �޲ι�����
	/// </summary>
	Point3Color4s() = default;

	/// <summary>
	/// �����������ɫ��������
	/// </summary>
	/// <param name="pointColors">�������ɫ��ָ��</param>
	/// <param name="length">����</param>
	Point3Color4s(Point3Color4* pointColors, const int& length)
		:PointColors(pointColors), Length(length)
	{

	}

	/// <summary>
	/// ��������
	/// </summary>
	~Point3Color4s()
	{
		delete[] this->PointColors;
	}

	/// <summary>
	/// �������ɫ��ָ��
	/// </summary>
	Point3Color4* PointColors;

	/// <summary>
	/// ����
	/// </summary>
	int Length;
};
