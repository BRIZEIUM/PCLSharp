#pragma once
#include "fpfh_signature33f.h"

/// <summary>
/// FPFH�����Ӽ�
/// </summary>
struct FPFHSignature33Fs
{
	/// <summary>
	/// �޲ι�����
	/// </summary>
	FPFHSignature33Fs() = default;

	/// <summary>
	/// ����FPFH�����Ӽ�������
	/// </summary>
	/// <param name="descriptors">FPFH�����Ӽ�ָ��</param>
	/// <param name="length">����</param>
	FPFHSignature33Fs(FPFHSignature33F* descriptors, const int& length)
		:Descriptors(descriptors), Length(length)
	{

	}

	/// <summary>
	/// ��������
	/// </summary>
	~FPFHSignature33Fs()
	{
		delete[] this->Descriptors;
	}

	/// <summary>
	/// FPFH�����Ӽ�ָ��
	/// </summary>
	FPFHSignature33F* Descriptors;

	/// <summary>
	/// ����
	/// </summary>
	int Length;
};
