#pragma once

/// <summary>
/// �����
/// </summary>
struct Point3F
{
public:

	/// <summary>
	/// �޲ι�����
	/// </summary>
	Point3F() = default;

	/// <summary>
	/// ��������㹹����
	/// </summary>
	/// <param name="x">X����</param>
	/// <param name="y">Y����</param>
	/// <param name="z">Z����</param>
	Point3F(const float x, const float y, const float z)
		:X(x), Y(y), Z(z)
	{

	}

	/// <summary>
	/// X����
	/// </summary>
	float X;

	/// <summary>
	/// Y����
	/// </summary>
	float Y;

	/// <summary>
	/// Z����
	/// </summary>
	float Z;
};
