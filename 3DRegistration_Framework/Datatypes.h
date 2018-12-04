#pragma once
#include<vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include<pcl/PCLPointCloud2.h>
#include<pcl/common/io.h>
#include<pcl/search/search.h>
#include"Config.h"

// typedefs
typedef pcl::PointXYZ PointType;
typedef pcl::PointXYZRGBA PointColorType;
typedef pcl::PointXYZI PointIntensityType;
typedef pcl::PointXYZRGBNormal PointRGBNormalType;
typedef pcl::Normal NormalType;
typedef pcl::PointNormal PointNormalType;

typedef pcl::PCLPointCloud2::Ptr CloudWithoutType;
typedef pcl::PointCloud<PointType>::Ptr CloudPtr;
typedef pcl::PointCloud<PointColorType>::Ptr CloudWithColorPtr;
typedef pcl::PointCloud<PointIntensityType>::Ptr CloudWithIntensityPtr;
typedef pcl::PointCloud<PointRGBNormalType>::Ptr CloudWithRGBNormalPtr;
typedef pcl::PointCloud<NormalType>::Ptr NormalPtr;
typedef pcl::PointCloud<PointNormalType>::Ptr CloudWithNormalPtr;
//typedef pcl::search::KdTree<PointType>::Ptr tree;

#ifdef _DEBUG 
#define MY_LOG std::cout
#else
#define MY_LOG if(false) std::cout
#endif

struct GridBound
{
    float u_max;
    float v_max;
    float u_min;
    float v_min;
    GridBound() {};
    GridBound(float u_max, float v_max, float u_min, float v_min) :
        u_max(u_max), v_max(v_max), u_min(u_min), v_min(v_min)
    {}
    GridBound(const GridBound &temp)
    {
        u_max = temp.u_max;
        v_max = temp.v_max;
        u_min = temp.u_min;
        v_min = temp.v_min;
    }
    bool operator==(const GridBound &temp)const
    {
        return u_max == temp.u_max && v_max == temp.v_max && u_min == temp.u_min && v_min == temp.v_min;
    }
};
template<typename TData>
struct PointData
{
    TData X;
    TData Y;
    TData Z;
    TData nX;
    TData nY;
    TData nZ;
    PointData(TData x, TData y, TData z, TData n_x, TData n_y, TData n_z):X(x),Y(y),
        Z(z),nX(n_x),nY(n_y),nZ(n_z)
    {}

    bool operator==(const PointData &o)const {
        return x == o.x && y == o.y && z == o.z;
    }
    PointData( const PointData& temp)
    {
        X = temp.X;
        Y = temp.Y;
        Z = temp.Z;
        nX = temp.nX;
        nY = temp.nY;
        nZ = temp.nZ;   
    }
    PointData& operator =(const PointData &temp)
    {
        X = temp.X;
        Y = temp.Y;
        Z = temp.Z;
        nX = temp.nX;
        nY = temp.nY;
        nZ = temp.nZ;
        return *this;
    }
    bool operator<(const PointData &o)const {
        return std::tie(X, Y, Z) < std::tie(o.X, o.Y, o.Z); // x < o.x || x == o.x && y < o.y && z < o.z;
    }
    bool isValidPoint()
    {
        if (isfinite((*this).X) && isfinite((*this).Y) && isfinite((*this).Z))
            return true;
        else
            return false;
    }
    PointData operator + (const PointData& point)const
    {
        PointData tempPoint((*this).X + point.X, (*this).Y + point.Y, (*this).Z + point.Z,
            (*this).nX + point.nX, (*this).nY + point.nY, (*this).nZ + point.nZ);
        return tempPoint;
    }
    PointData operator *(float dScalar)const
    {
        PointData tempPoint((*this).X * dScalar, (*this).Y * dScalar, (*this).Z * dScalar,
            (*this).nX * dScalar, (*this).nY* dScalar, (*this).nZ * dScalar);
        return tempPoint;
    }
};

template<typename TData>
struct UVData
{
    TData u;
    TData v;
    UVData() {}
    UVData(TData uIndex, TData vIndex):u(uIndex),v(vIndex)
    {}
    bool operator==(const UVData &o)const {
        return u == o.u && v == o.v;
    }

    bool operator<(const UVData &o)const {
        return u < o.u || (u == o.u && v < o.v);
    }
};
REG3D_API PointRGBNormalType POINT_EIGEN_CAST(const Eigen::Vector3f &vector, const Eigen::Vector3f &normal);
template <typename poinT, typename pointype>
REG3D_API void copypoint(PointData<pointype> &sourcepoint, poinT &targetpoint);
template< typename type1>
REG3D_API Eigen::Vector3f EIGEN_POINT_CAST(const type1 &point);
template< typename type1>
REG3D_API Eigen::Vector3f EIGEN_NORMAL_CAST(const type1 &point);
template< typename type1>
 bool isPointFinite(const type1 &point);

 
  //  template <typename poinT>
  //   void copypoint(PointData<float> &sourcepoint, poinT &targetpoint)
  //  {
  //      
  //      if (typeid(poinT) == typeid(PointType))
  //      {
  //          targetpoint.x = sourcepoint.X;
  //          targetpoint.y = sourcepoint.Y;
  //          targetpoint.z = sourcepoint.Z;
  //        
  //      }
  //      else  if (typeid(poinT) == typeid(PointNormalType))
  //      {
  //          targetpoint.x = sourcepoint.X;
  //          targetpoint.y = sourcepoint.Y;
  //          targetpoint.z = sourcepoint.Z;
  //          targetpoint.normal_x = sourcepoint.nX;
  //          targetpoint.normal_y = sourcepoint.nY;
  //          targetpoint.normal_z = sourcepoint.nZ;

  //      }
  //      else if (typeid(poinT) == typeid(PointRGBNormalType))
  //      {
  //          targetpoint.x = sourcepoint.X;
  //          targetpoint.y = sourcepoint.Y;
  //          targetpoint.z = sourcepoint.Z;
  //          targetpoint.normal_x = sourcepoint.nX;
  //          targetpoint.normal_y = sourcepoint.nY;
  //          targetpoint.normal_z = sourcepoint.nZ;
  //          targetpoint.rgba = 0.0;

  //      }
  //      else
  //      {
  //          throw std::runtime_error("point cloud type could not be determined");
  //      }
  //  }
  //  template< typename type1>
  //  static Eigen::Vector3f EIGEN_POINT_CAST(const type1 &point)
  //  {
  //      Eigen::Vector3f vector;
  //     /* vector(0) = point.x;
  //      vector(1) = point.y;
  //      vector(2) = point.z;*/
  //      vector = point.getVector3fMap();
  //      return vector;
  //  }

  //  static PointRGBNormalType POINT_EIGEN_CAST(const Eigen::Vector3f &vector, const Eigen::Vector3f &normal)
  //  {
  //      PointRGBNormalType point;
  //      point.getVector3fMap() = vector;
  //      point.getNormalVector3fMap() = normal;
  //      /*point.x = vector(0);
  //      point.y = vector(1);
  //      point.z = vector(2);
  //      point.normal_x = normal(0);
  //      point.normal_y = normal(1);
  //      point.normal_z = normal(2);*/
  //      return point;
  //  }

  //  template< typename type1>
  //  static Eigen::Vector3f EIGEN_NORMAL_CAST(const type1 &point)
  //  {
  //      Eigen::Vector3f vector;
  //      /*vector(0) = point.normal_x;
  //      vector(1) = point.normal_y;
  //      vector(2) = point.normal_z;*/
  //      vector = point.getNormalVector3fMap();
  //      return vector;
  //  }
    template< typename type1>
    static bool isPointFinite(const type1 &point)
    {
        bool flag = false;
        if (!isnan(point.x) && !isnan(point.y) && !isnan(point.z))
        {
            flag = true;
        }
        return flag;
  }
