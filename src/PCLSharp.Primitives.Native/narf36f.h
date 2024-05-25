#pragma once

/// <summary>
/// NARF������
/// </summary>
struct Narf36F
{
	/// <summary>
	/// �޲ι�����
	/// </summary>
	Narf36F() = default;

	/// <summary>
	/// ����NARF�����ӹ�����
	/// </summary>
	/// <param name="x">X����</param>
	/// <param name="y">Y����</param>
	/// <param name="z">Z����</param>
	/// <param name="pitch">������(RX)</param>
	/// <param name="yaw">ƫ����(RY)</param>
	/// <param name="roll">������(RZ)</param>
	Narf36F(const float& x, const float& y, const float& z, const float& pitch, const float& yaw, const float& roll)
		:X(x), Y(y), Z(z), Pitch(pitch), Yaw(yaw), Roll(roll), Features{ 0.0f }
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
	/// ������(RX)
	/// </summary>
	float Pitch;

	/// <summary>
	/// ƫ����(RY)
	/// </summary>
	float Yaw;

	/// <summary>
	/// ������(RZ)
	/// </summary>
	float Roll;

	/// <summary>
	/// ��������
	/// </summary>
	float Features[36];
};
