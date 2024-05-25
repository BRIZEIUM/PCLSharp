#pragma once

/// <summary>
/// ����������MxN����
/// </summary>
struct DescriptorsMxN
{
	/// <summary>
	/// �޲ι�����
	/// </summary>
	DescriptorsMxN() = default;

	/// <summary>
	/// ��������������MxN��������
	/// </summary>
	/// <param name="matrix">��������</param>
	/// <param name="rowsCount">����</param>
	/// <param name="colsCount">����</param>
	DescriptorsMxN(float** matrix, const int& rowsCount, const int& colsCount)
		:Matrix(matrix), RowsCount(rowsCount), ColsCount(colsCount)
	{

	}

	/// <summary>
	/// ��������
	/// </summary>
	~DescriptorsMxN()
	{
		//�ͷ�ÿ��
		for (int rowIndex = 0; rowIndex < this->RowsCount; rowIndex++)
		{
			delete[] this->Matrix[rowIndex];
		}

		//�ͷ�����
		delete[] this->Matrix;
	}

	/// <summary>
	/// ��������
	/// </summary>
	float** Matrix;

	/// <summary>
	/// ����
	/// </summary>
	int RowsCount;

	/// <summary>
	/// ����
	/// </summary>
	int ColsCount;
};
