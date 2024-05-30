#include "pcl_dispose.h"

/// <summary>
/// �ͷ������
/// </summary>
/// <param name="pointer">ָ��</param>
void disposePoint3F(const Point3F* pointer)
{
	delete pointer;
}

/// <summary>
/// �ͷ�����㼯
/// </summary>
/// <param name="pointer">ָ��</param>
void disposePoint3Fs(const Point3Fs* pointer)
{
	delete pointer;
}

/// <summary>
/// �ͷ�����㼯����
/// </summary>
/// <param name="pointer">����ָ��</param>
/// <param name="groupCount">������</param>
void disposePoint3FsGroup(const Point3Fs** pointer, const int groupCount)
{
	for (int groupIndex = 0; groupIndex < groupCount; groupIndex++)
	{
		delete pointer[groupIndex];
	}
	delete pointer;
}

/// <summary>
/// �ͷŷ�����
/// </summary>
/// <param name="pointer">ָ��</param>
void disposeNormal3F(const Normal3F* pointer)
{
	delete pointer;
}

/// <summary>
/// �ͷŷ�������
/// </summary>
/// <param name="pointer">ָ��</param>
void disposeNormal3Fs(const Normal3Fs* pointer)
{
	delete pointer;
}

/// <summary>
/// �ͷŷ�����������
/// </summary>
/// <param name="pointer">����ָ��</param>
/// <param name="groupCount">������</param>
void disposeNormal3FsGroup(const Point3Fs** pointer, const int groupCount)
{
	for (int groupIndex = 0; groupIndex < groupCount; groupIndex++)
	{
		delete pointer[groupIndex];
	}
	delete pointer;
}

/// <summary>
/// �ͷ�����㷨����
/// </summary>
/// <param name="pointer">ָ��</param>
void disposePoint3Normal3(const Point3Normal3* pointer)
{
	delete pointer;
}

/// <summary>
/// �ͷ�����㷨������
/// </summary>
/// <param name="pointer">ָ��</param>
void disposePoint3Normal3s(const Point3Normal3s* pointer)
{
	delete pointer;
}

/// <summary>
/// �ͷ�����㷨����������
/// </summary>
/// <param name="pointer">����ָ��</param>
/// <param name="groupCount">������</param>
void disposePoint3Normal3sGroup(const Point3Fs** pointer, const int groupCount)
{
	for (int groupIndex = 0; groupIndex < groupCount; groupIndex++)
	{
		delete pointer[groupIndex];
	}
	delete pointer;
}

/// <summary>
/// �ͷ��������ɫ
/// </summary>
/// <param name="pointer">ָ��</param>
void disposePoint3Color4(const Point3Color4* pointer)
{
	delete pointer;
}

/// <summary>
/// �ͷ��������ɫ��
/// </summary>
/// <param name="pointer">ָ��</param>
void disposePoint3Color4s(const Point3Color4s* pointer)
{
	delete pointer;
}

/// <summary>
/// �ͷ��������ɫ������
/// </summary>
/// <param name="pointer">����ָ��</param>
/// <param name="groupCount">������</param>
void disposePoint3Color4sGroup(const Point3Fs** pointer, const int groupCount)
{
	for (int groupIndex = 0; groupIndex < groupCount; groupIndex++)
	{
		delete pointer[groupIndex];
	}
	delete pointer;
}

/// <summary>
/// �ͷ�NARF������
/// </summary>
/// <param name="pointer">ָ��</param>
void disposeNarf36F(const Narf36F* pointer)
{
	delete pointer;
}

/// <summary>
/// �ͷ�NARF�����Ӽ�
/// </summary>
/// <param name="pointer">ָ��</param>
void disposeNarf36Fs(const Narf36Fs* pointer)
{
	delete pointer;
}

/// <summary>
/// �ͷ�PFH������
/// </summary>
/// <param name="pointer">ָ��</param>
void disposePFHSignature125F(const PFHSignature125F* pointer)
{
	delete pointer;
}

/// <summary>
/// �ͷ�PFH�����Ӽ�
/// </summary>
/// <param name="pointer">ָ��</param>
void disposePFHSignature125Fs(const PFHSignature125Fs* pointer)
{
	delete pointer;
}

/// <summary>
/// �ͷ�FPFH������
/// </summary>
/// <param name="pointer">ָ��</param>
void disposeFPFHSignature33F(const FPFHSignature33F* pointer)
{
	delete pointer;
}

/// <summary>
/// �ͷ�FPFH�����Ӽ�
/// </summary>
/// <param name="pointer">ָ��</param>
void disposeFPFHSignature33Fs(const FPFHSignature33Fs* pointer)
{
	delete pointer;
}

/// <summary>
/// �ͷ�3DSC������
/// </summary>
/// <param name="pointer">ָ��</param>
void disposeShapeContext1980F(const ShapeContext1980F* pointer)
{
	delete pointer;
}

/// <summary>
/// �ͷ�3DSC�����Ӽ�
/// </summary>
/// <param name="pointer">ָ��</param>
void disposeShapeContext1980Fs(const ShapeContext1980Fs* pointer)
{
	delete pointer;
}

/// <summary>
/// �ͷ�SHOT������
/// </summary>
/// <param name="pointer">ָ��</param>
void disposeShot352F(const Shot352F* pointer)
{
	delete pointer;
}

/// <summary>
/// �ͷ�SHOT�����Ӽ�
/// </summary>
/// <param name="pointer">ָ��</param>
void disposeShot352Fs(const Shot352Fs* pointer)
{
	delete pointer;
}

/// <summary>
/// �ͷ���׼���
/// </summary>
/// <param name="pointer">ָ��</param>
void disposeAlignmentResult(const AlignmentResult* pointer)
{
	delete pointer;
}
