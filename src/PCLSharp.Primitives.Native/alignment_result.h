#pragma once

/// <summary>
/// ��׼���
/// </summary>
struct AlignmentResult
{
	/// <summary>
	/// �޲ι�����
	/// </summary>
	AlignmentResult() = default;

	/// <summary>
	/// ������׼���������
	/// </summary>
	/// <param name="hasConverged">�Ƿ�����</param>
	/// <param name="fitnessScore">��Ϸ���</param>
	AlignmentResult(const bool& hasConverged, const float& fitnessScore)
		:HasConverged(hasConverged), FitnessScore(fitnessScore), Matrix{ 0.0f }
	{

	}

	/// <summary>
	/// �Ƿ�����
	/// </summary>
	bool HasConverged;

	/// <summary>
	/// ��Ϸ���
	/// </summary>
	float FitnessScore;

	/// <summary>
	/// RT����
	/// </summary>
	float Matrix[16];
};
