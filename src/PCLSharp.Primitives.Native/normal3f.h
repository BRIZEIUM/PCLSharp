#pragma once

/// <summary>
/// ������
/// </summary>
struct Normal3F
{
	/// <summary>
	/// �޲ι�����
	/// </summary>
	Normal3F() = default;

	/// <summary>
	/// ����������������
	/// </summary>
	/// <param name="nx">������X����</param>
	/// <param name="ny">������Y����</param>
	/// <param name="nz">������Z����</param>
	Normal3F(const float& nx, const float& ny, const float& nz)
		:NX(nx), NY(ny), NZ(nz)
	{

	}

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
