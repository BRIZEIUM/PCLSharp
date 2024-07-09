#pragma once
#ifdef _WIN32
#define EXPORT_C extern "C" __declspec(dllexport)
#elif __linux__
#define EXPORT_C extern "C"
#endif
#define CALLING_MODE _cdecl
#include <point3f.h>
#include <point3fs.h>
#include <point3normal3.h>
#include <point3normal3s.h>
#include <point3color4.h>
#include <point3color4s.h>

/// <summary>
/// ����PCD�ļ�
/// </summary>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT_C Point3Fs* CALLING_MODE loadPCD(const char* filePath);

/// <summary>
/// ���ط�����PCD�ļ�
/// </summary>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT_C Point3Normal3s* CALLING_MODE loadNormalPCD(const char* filePath);

/// <summary>
/// ������ɫPCD�ļ�
/// </summary>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT_C Point3Color4s* CALLING_MODE loadColorPCD(const char* filePath);

/// <summary>
/// ����PLY�ļ�
/// </summary>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT_C Point3Fs* CALLING_MODE loadPLY(const char* filePath);

/// <summary>
/// ���ط�����PLY�ļ�
/// </summary>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT_C Point3Normal3s* CALLING_MODE loadNormalPLY(const char* filePath);

/// <summary>
/// ������ɫPLY�ļ�
/// </summary>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT_C Point3Color4s* CALLING_MODE loadColorPLY(const char* filePath);

/// <summary>
/// ����OBJ�ļ�
/// </summary>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT_C Point3Fs* CALLING_MODE loadOBJ(const char* filePath);

/// <summary>
/// ���ط�����OBJ�ļ�
/// </summary>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT_C Point3Normal3s* CALLING_MODE loadNormalOBJ(const char* filePath);

/// <summary>
/// ������ɫOBJ�ļ�
/// </summary>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT_C Point3Color4s* CALLING_MODE loadColorOBJ(const char* filePath);

/// <summary>
/// ����PCD�ı��ļ�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT_C void CALLING_MODE saveTextPCD(Point3F points[], int length, const char* filePath);

/// <summary>
/// ����PCD�������ļ�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT_C void CALLING_MODE saveBinaryPCD(Point3F points[], int length, const char* filePath);

/// <summary>
/// ���淨����PCD�ı��ļ�
/// </summary>
/// <param name="pointNormals">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT_C void CALLING_MODE saveNormalTextPCD(Point3Normal3 pointNormals[], int length, const char* filePath);

/// <summary>
/// ���淨����PCD�������ļ�
/// </summary>
/// <param name="pointNormals">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT_C void CALLING_MODE saveNormalBinaryPCD(Point3Normal3 pointNormals[], int length, const char* filePath);

/// <summary>
/// ������ɫPCD�ı��ļ�
/// </summary>
/// <param name="pointColors">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT_C void CALLING_MODE saveColorTextPCD(Point3Color4 pointColors[], int length, const char* filePath);

/// <summary>
/// ������ɫPCD�������ļ�
/// </summary>
/// <param name="pointColors">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT_C void CALLING_MODE saveColorBinaryPCD(Point3Color4 pointColors[], int length, const char* filePath);

/// <summary>
/// ����PLY�ı��ļ�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT_C void CALLING_MODE saveTextPLY(Point3F points[], int length, const char* filePath);

/// <summary>
/// ����PLY�������ļ�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT_C void CALLING_MODE saveBinaryPLY(Point3F points[], int length, const char* filePath);

/// <summary>
/// ���淨����PLY�ı��ļ�
/// </summary>
/// <param name="pointNormals">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT_C void CALLING_MODE saveNormalTextPLY(Point3Normal3 pointNormals[], int length, const char* filePath);

/// <summary>
/// ���淨����PLY�������ļ�
/// </summary>
/// <param name="pointNormals">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT_C void CALLING_MODE saveNormalBinaryPLY(Point3Normal3 pointNormals[], int length, const char* filePath);

/// <summary>
/// ������ɫPLY�ı��ļ�
/// </summary>
/// <param name="pointColors">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT_C void CALLING_MODE saveColorTextPLY(Point3Color4 pointColors[], int length, const char* filePath);

/// <summary>
/// ������ɫPLY�������ļ�
/// </summary>
/// <param name="pointColors">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT_C void CALLING_MODE saveColorBinaryPLY(Point3Color4 pointColors[], int length, const char* filePath);
