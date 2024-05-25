#pragma once
#include "narf36f.h"

/// <summary>
/// NARF�����Ӽ��ṹ��
/// </summary>
struct Narf36Fs
{
	/// <summary>
	/// �޲ι�����
	/// </summary>
	Narf36Fs() = default;

	/// <summary>
	/// ����NARF�����Ӽ��ṹ�幹����
	/// </summary>
	/// <param name="descriptors">NARF�����Ӽ�ָ��</param>
	/// <param name="length">����</param>
	Narf36Fs(Narf36F* descriptors, const int& length)
		:Descriptors(descriptors), Length(length)
	{

	}

	/// <summary>
	/// ��������
	/// </summary>
	~Narf36Fs()
	{
		delete[] this->Descriptors;
	}

	/// <summary>
	/// NARF�����Ӽ�ָ��
	/// </summary>
	Narf36F* Descriptors;

	/// <summary>
	/// ����
	/// </summary>
	int Length;
};
