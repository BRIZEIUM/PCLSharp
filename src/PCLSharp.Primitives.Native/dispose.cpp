#include "dispose.h"

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
