#pragma once

/// <summary>
/// ����㷨����
/// </summary>
struct Point3Normal3
{
	/// <summary>
	/// �޲ι�����
	/// </summary>
	Point3Normal3() = default;

	/// <summary>
	/// ��������㷨����������
	/// </summary>
	/// <param name="x">X����</param>
	/// <param name="y">Y����</param>
	/// <param name="z">Z����</param>
	/// <param name="nx">������X����</param>
	/// <param name="ny">������Y����</param>
	/// <param name="nz">������Z����</param>
	Point3Normal3(const float& x, const float& y, const float& z, const float& nx, const float& ny, const float& nz)
		:X(x), Y(y), Z(z), NX(nx), NY(ny), NZ(nz)
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
	/// ������X����
	/// </summary>
	float NX;

	/// <summary>
	/// ������Y����
	/// </summary>
	float NY;

	/// <summary>
	/// ������Z����
	/// </summary>
	float NZ;
};
