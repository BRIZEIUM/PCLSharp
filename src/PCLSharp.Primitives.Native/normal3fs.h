#pragma once
#include "normal3f.h"

/// <summary>
/// ��������
/// </summary>
struct Normal3Fs
{
	/// <summary>
	/// �޲ι�����
	/// </summary>
	Normal3Fs() = default;

	/// <summary>
	/// ������������������
	/// </summary>
	/// <param name="normals">��������ָ��</param>
	/// <param name="length">����</param>
	Normal3Fs(Normal3F* normals, const int& length)
		:Normals(normals), Length(length)
	{

	}

	/// <summary>
	/// ��������
	/// </summary>
	~Normal3Fs()
	{
		delete[] this->Normals;
	}

	/// <summary>
	/// ��������ָ��
	/// </summary>
	Normal3F* Normals;

	/// <summary>
	/// ����
	/// </summary>
	int Length;
};
