#pragma once
#include "shape_context1980f.h"

/// <summary>
/// 3DSC�����Ӽ��ṹ��
/// </summary>
struct ShapeContext1980Fs
{
	/// <summary>
	/// �޲ι�����
	/// </summary>
	ShapeContext1980Fs() = default;

	/// <summary>
	/// ����3DSC�����Ӽ��ṹ�幹����
	/// </summary>
	/// <param name="descriptors">3DSC�����Ӽ�ָ��</param>
	/// <param name="length">����</param>
	ShapeContext1980Fs(ShapeContext1980F* descriptors, const int& length)
		:Descriptors(descriptors), Length(length)
	{

	}

	/// <summary>
	/// ��������
	/// </summary>
	~ShapeContext1980Fs()
	{
		delete[] this->Descriptors;
	}

	/// <summary>
	/// 3DSC�����Ӽ�ָ��
	/// </summary>
	ShapeContext1980F* Descriptors;

	/// <summary>
	/// ����
	/// </summary>
	int Length;
};
