#pragma once

/// <summary>
/// SHOT������
/// </summary>
struct Shot352F
{
	/// <summary>
	/// �޲ι�����
	/// </summary>
	Shot352F()
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
	float Features[352];
};
