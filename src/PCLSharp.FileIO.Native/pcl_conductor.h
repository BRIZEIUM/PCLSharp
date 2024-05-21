#pragma once
#define EXPORT extern "C" __declspec(dllexport)
#define CALLING_MODE _cdecl
#include <format>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <point3f.h>
#include <point3fs.h>
#include <primitives_map.h>

/// <summary>
/// ����PCD�ļ�
/// </summary>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT Point3Fs* CALLING_MODE loadPCD(const char* filePath);

/// <summary>
/// ����PLY�ļ�
/// </summary>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT Point3Fs* CALLING_MODE loadPLY(const char* filePath);

/// <summary>
/// ����OBJ�ļ�
/// </summary>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT Point3Fs* CALLING_MODE loadOBJ(const char* filePath);

/// <summary>
/// ����PCD�ı��ļ�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT void CALLING_MODE saveTextPCD(Point3F points[], int length, const char* filePath);

/// <summary>
/// ����PCD�������ļ�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT void CALLING_MODE saveBinaryPCD(Point3F points[], int length, const char* filePath);

/// <summary>
/// ����PLY�ı��ļ�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT void CALLING_MODE saveTextPLY(Point3F points[], int length, const char* filePath);

/// <summary>
/// ����PLY�������ļ�
/// </summary>
/// <param name="points">�㼯</param>
/// <param name="length">�㼯����</param>
/// <param name="filePath">�ļ�·��</param>
/// <returns>�㼯</returns>
EXPORT void CALLING_MODE saveBinaryPLY(Point3F points[], int length, const char* filePath);

/// <summary>
/// �ͷ���Դ
/// </summary>
/// <param name="pointer">ָ��</param>
EXPORT void CALLING_MODE dispose(const Point3Fs* pointer);
