#pragma once
#include "shot352f.h"

/// <summary>
/// SHOT�����Ӽ�
/// </summary>
struct Shot352Fs
{
	/// <summary>
	/// �޲ι�����
	/// </summary>
	Shot352Fs() = default;

	/// <summary>
	/// ����SHOT�����Ӽ�������
	/// </summary>
	/// <param name="descriptors">SHOT�����Ӽ�ָ��</param>
	/// <param name="length">����</param>
	Shot352Fs(Shot352F* descriptors, const int& length)
		:Descriptors(descriptors), Length(length)
	{

	}

	/// <summary>
	/// ��������
	/// </summary>
	~Shot352Fs()
	{
		delete[] this->Descriptors;
	}

	/// <summary>
	/// SHOT�����Ӽ�ָ��
	/// </summary>
	Shot352F* Descriptors;

	/// <summary>
	/// ����
	/// </summary>
	int Length;
};
