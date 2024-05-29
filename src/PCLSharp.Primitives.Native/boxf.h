#pragma once
#include "point3f.h"

/// <summary>
/// ������
/// </summary>
struct BoxF
{
	/// <summary>
	/// �޲ι�����
	/// </summary>
	BoxF() = default;

	/// <summary>
	/// ���������幹����
	/// </summary>
	/// <param name="minPoint">��С�����</param>
	/// <param name="maxPoint">��������</param>
	BoxF(const Point3F& minPoint, const Point3F& maxPoint)
		:MinPoint(minPoint), MaxPoint(maxPoint)
	{

	}

	/// <summary>
	/// ��С�����
	/// </summary>
	Point3F MinPoint;

	/// <summary>
	/// ��������
	/// </summary>
	Point3F MaxPoint;
};
