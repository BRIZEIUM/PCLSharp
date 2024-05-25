#pragma once
#include "pfh_signature125f.h"

/// <summary>
/// PFH�����Ӽ��ṹ��
/// </summary>
struct PFHSignature125Fs
{
	/// <summary>
	/// �޲ι�����
	/// </summary>
	PFHSignature125Fs() = default;

	/// <summary>
	/// ����PFH�����Ӽ��ṹ�幹����
	/// </summary>
	/// <param name="descriptors">PFH�����Ӽ�ָ��</param>
	/// <param name="length">����</param>
	PFHSignature125Fs(PFHSignature125F* descriptors, const int& length)
		:Descriptors(descriptors), Length(length)
	{

	}

	/// <summary>
	/// ��������
	/// </summary>
	~PFHSignature125Fs()
	{
		delete[] this->Descriptors;
	}

	/// <summary>
	/// PFH�����Ӽ�ָ��
	/// </summary>
	PFHSignature125F* Descriptors;

	/// <summary>
	/// ����
	/// </summary>
	int Length;
};
