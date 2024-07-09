#pragma once
#ifdef _WIN32
#define EXPORT_C extern "C" __declspec(dllexport)
#define CALLING_MODE _cdecl
#elif __linux__
#define EXPORT_C extern "C"
#define CALLING_MODE __attribute__((__cdecl__))
#endif
#include <point3f.h>
#include <point3fs.h>
#include <normal3f.h>
#include <normal3fs.h>
#include <point3normal3.h>
#include <point3normal3s.h>
#include <point3color4.h>
#include <point3color4s.h>
#include <narf36f.h>
#include <narf36fs.h>
#include <pfh_signature125f.h>
#include <pfh_signature125fs.h>
#include <fpfh_signature33f.h>
#include <fpfh_signature33fs.h>
#include <shape_context1980f.h>
#include <shape_context1980fs.h>
#include <shot352f.h>
#include <shot352fs.h>
#include <alignment_result.h>

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
/// �ͷ�����㼯����
/// </summary>
/// <param name="pointer">����ָ��</param>
/// <param name="groupCount">������</param>
EXPORT_C void CALLING_MODE disposePoint3FsGroup(const Point3Fs** pointer, int groupCount);

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
/// �ͷŷ�����������
/// </summary>
/// <param name="pointer">����ָ��</param>
/// <param name="groupCount">������</param>
EXPORT_C void CALLING_MODE disposeNormal3FsGroup(const Point3Fs** pointer, int groupCount);

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
/// �ͷ�����㷨����������
/// </summary>
/// <param name="pointer">����ָ��</param>
/// <param name="groupCount">������</param>
EXPORT_C void CALLING_MODE disposePoint3Normal3sGroup(const Point3Fs** pointer, int groupCount);

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

/// <summary>
/// �ͷ��������ɫ������
/// </summary>
/// <param name="pointer">����ָ��</param>
/// <param name="groupCount">������</param>
EXPORT_C void CALLING_MODE disposePoint3Color4sGroup(const Point3Fs** pointer, int groupCount);

/// <summary>
/// �ͷ�NARF������
/// </summary>
/// <param name="pointer">ָ��</param>
EXPORT_C void CALLING_MODE disposeNarf36F(const Narf36F* pointer);

/// <summary>
/// �ͷ�NARF�����Ӽ�
/// </summary>
/// <param name="pointer">ָ��</param>
EXPORT_C void CALLING_MODE disposeNarf36Fs(const Narf36Fs* pointer);

/// <summary>
/// �ͷ�PFH������
/// </summary>
/// <param name="pointer">ָ��</param>
EXPORT_C void CALLING_MODE disposePFHSignature125F(const PFHSignature125F* pointer);

/// <summary>
/// �ͷ�PFH�����Ӽ�
/// </summary>
/// <param name="pointer">ָ��</param>
EXPORT_C void CALLING_MODE disposePFHSignature125Fs(const PFHSignature125Fs* pointer);

/// <summary>
/// �ͷ�FPFH������
/// </summary>
/// <param name="pointer">ָ��</param>
EXPORT_C void CALLING_MODE disposeFPFHSignature33F(const FPFHSignature33F* pointer);

/// <summary>
/// �ͷ�FPFH�����Ӽ�
/// </summary>
/// <param name="pointer">ָ��</param>
EXPORT_C void CALLING_MODE disposeFPFHSignature33Fs(const FPFHSignature33Fs* pointer);

/// <summary>
/// �ͷ�3DSC������
/// </summary>
/// <param name="pointer">ָ��</param>
EXPORT_C void CALLING_MODE disposeShapeContext1980F(const ShapeContext1980F* pointer);

/// <summary>
/// �ͷ�3DSC�����Ӽ�
/// </summary>
/// <param name="pointer">ָ��</param>
EXPORT_C void CALLING_MODE disposeShapeContext1980Fs(const ShapeContext1980Fs* pointer);

/// <summary>
/// �ͷ�SHOT������
/// </summary>
/// <param name="pointer">ָ��</param>
EXPORT_C void CALLING_MODE disposeShot352F(const Shot352F* pointer);

/// <summary>
/// �ͷ�SHOT�����Ӽ�
/// </summary>
/// <param name="pointer">ָ��</param>
EXPORT_C void CALLING_MODE disposeShot352Fs(const Shot352Fs* pointer);

/// <summary>
/// �ͷ���׼���
/// </summary>
/// <param name="pointer">ָ��</param>
EXPORT_C void CALLING_MODE disposeAlignmentResult(const AlignmentResult* pointer);
