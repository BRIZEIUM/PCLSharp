#pragma once
#define EXPORT_C extern "C" __declspec(dllexport)
#define CALLING_MODE _cdecl
#include <point3f.h>
#include <point3fs.h>
#include <normal3f.h>
#include <normal3fs.h>
#include <point3normal3.h>
#include <point3normal3s.h>
#include <point3color4.h>
#include <point3color4s.h>

/// <summary>
/// �ͷ������
/// </summary>
/// <param name="pointer">ָ��</param>
EXPORT_C void CALLING_MODE disposePoint3F(const Point3F* pointer);

/// <summary>
/// �ͷ�����㼯
/// </summary>
/// <param name="pointer">ָ��</param>
EXPORT_C void CALLING_MODE disposePoint3Fs(const Point3Fs* pointer);

/// <summary>
/// �ͷŷ�����
/// </summary>
/// <param name="pointer">ָ��</param>
EXPORT_C void CALLING_MODE disposeNormal3F(const Normal3F* pointer);

/// <summary>
/// �ͷŷ�������
/// </summary>
/// <param name="pointer">ָ��</param>
EXPORT_C void CALLING_MODE disposeNormal3Fs(const Normal3Fs* pointer);

/// <summary>
/// �ͷ�����㷨����
/// </summary>
/// <param name="pointer">ָ��</param>
EXPORT_C void CALLING_MODE disposePoint3Normal3(const Point3Normal3* pointer);

/// <summary>
/// �ͷ�����㷨������
/// </summary>
/// <param name="pointer">ָ��</param>
EXPORT_C void CALLING_MODE disposePoint3Normal3s(const Point3Normal3s* pointer);

/// <summary>
/// �ͷ��������ɫ
/// </summary>
/// <param name="pointer">ָ��</param>
EXPORT_C void CALLING_MODE disposePoint3Color4(const Point3Color4* pointer);

/// <summary>
/// �ͷ��������ɫ��
/// </summary>
/// <param name="pointer">ָ��</param>
EXPORT_C void CALLING_MODE disposePoint3Color4s(const Point3Color4s* pointer);
