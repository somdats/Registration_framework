#pragma once
#include <Eigen/Core>
#include<vector>
#include<map>
#include"Datatypes.h"
using namespace Eigen;


template <class datatype, class uvtype>
class C3DPointCloud
{
protected:
    // data contained in file
    std::vector<Eigen::VectorBlock<datatype>>		m_acVertexList;		// list of vertex coordinates
    std::vector<Eigen::VectorBlock<uvtype>>		m_acUVList;	        // list of UV coordinates
    std::vector<Eigen::VectorBlock<datatype>>		m_acNormalList;		// list of normal directions
    std::vector<uint32_t>                     m_acColorList;        //color list
    std::map <Eigen::VectorBlock<uvtype>, Eigen::VectorBlock<datatype>> UV3DMap;

public:
  
    C3DPointCloud();
    virtual ~C3DPointCloud();
    C3DPointCloud(const C3DPointCloud &cPoint3D);
    C3DPointCloud operator +( const C3DPointCloud &other)const;
    C3DPointCloud operator - (const C3DPointCloud& vector) const;
    C3DPointCloud operator * (datatype dScalar) const;
    C3DPointCloud operator / (datatype dScalar) const;
    int	GetNumVertices() const;
    int	GetNumUV() const;
    int GetNumNormals() const;

    int CopyData(const C3DPointCloud &cPoint);
    void ReleaseData();

    const Eigen::VectorBlock<datatype>& GetVertex(int nIndex) const;
    const Eigen::VectorBlock<uvtype>& GetUVIndex(int nIndex) const;
    const Eigen::VectorBlock<datatype>& GetNormal(int nIndex) const;

    void SetVertex(const Eigen::VectorBlock<datatype> &cVertex, int nIndex);
    void SetUV(const Eigen::VectorBlock<uvtype> &cPoint, int nIndex);
    void SetNormal(const Eigen::VectorBlock<datatype> &cNormal, int nIndex);
    void setColor(const uint32_t &colorValue, int nIndex);
    void createMap();
    std::map <Eigen::VectorBlock<uvtype>, Eigen::VectorBlock<datatype>> getMap();
    
};
