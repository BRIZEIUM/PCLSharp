#pragma once

/// <summary>
/// 3DSC������
/// </summary>
struct ShapeContext1980F
{
	/// <summary>
	/// �޲ι�����
	/// </summary>
	ShapeContext1980F()
		:RF{ 0.0f }, Features{ 0.0f }
	{

	}

	/// <summary>
	/// RF
	/// </summary>
	float RF[9];

	/// <summary>
	/// ��������
	/// </summary>
	float Features[1980];
};
