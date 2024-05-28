#pragma once

/// <summary>
/// λ��
/// </summary>
struct Pose
{
	/// <summary>
	/// �޲ι�����
	/// </summary>
	Pose() = default;

	/// <summary>
	/// ����λ�˹�����
	/// </summary>
	/// <param name="x">X��λ��</param>
	/// <param name="y">Y��λ��</param>
	/// <param name="z">Z��λ��</param>
	/// <param name="rx">X����ת�Ƕ�</param>
	/// <param name="ry">Y����ת�Ƕ�</param>
	/// <param name="rz">Z����ת�Ƕ�</param>
	Pose(const float& x, const float& y, const float& z, const float& rx, const float& ry, const float& rz)
		:X(x), Y(y), Z(z), RX(rx), RY(ry), RZ(rz)
	{

	}

	/// <summary>
	/// X��λ��
	/// </summary>
	float X;

	/// <summary>
	/// Y��λ��
	/// </summary>
	float Y;

	/// <summary>
	/// Z��λ��
	/// </summary>
	float Z;

	/// <summary>
	/// X����ת�Ƕ�
	/// </summary>
	float RX;

	/// <summary>
	/// Y����ת�Ƕ�
	/// </summary>
	float RY;

	/// <summary>
	/// Z����ת�Ƕ�
	/// </summary>
	float RZ;
};
