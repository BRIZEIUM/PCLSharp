#pragma once

/// <summary>
/// �������ɫ
/// </summary>
struct Point3Color4
{
	/// <summary>
	/// �޲ι�����
	/// </summary>
	Point3Color4() = default;

	/// <summary>
	/// �����������ɫ������
	/// </summary>
	/// <param name="x">X����</param>
	/// <param name="y">Y����</param>
	/// <param name="z">Z����</param>
	/// <param name="r">Rֵ</param>
	/// <param name="g">Gֵ</param>
	/// <param name="b">Bֵ</param>
	/// <param name="a">Aֵ</param>
	Point3Color4(const float& x, const float& y, const float& z, const unsigned char& r, const unsigned char& g, const unsigned char& b, const unsigned char& a)
		:X(x), Y(y), Z(z), R(r), G(g), B(b), A(a == 0 ? 255 : a)
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

	/// <summary>
	/// Rֵ
	/// </summary>
	unsigned char R;

	/// <summary>
	/// Gֵ
	/// </summary>
	unsigned char G;

	/// <summary>
	/// Bֵ
	/// </summary>
	unsigned char B;

	/// <summary>
	/// Aֵ
	/// </summary>
	unsigned char A;
};
