#include"pch.h"
#include"PointData.h"

template <typename datatype, typename uvtype>
C3DPointCloud<datatype, uvtype>::C3DPointCloud()
{

}

template <typename datatype, typename uvtype>
C3DPointCloud<datatype, uvtype>::~C3DPointCloud()
{
    ReleaseData();
}

template <typename datatype, typename uvtype>
C3DPointCloud<datatype, uvtype>::C3DPointCloud(const C3DPointCloud &cPoint3D)
{
    CopyData(cPoint3D);
}

template <typename datatype, typename uvtype>
int C3DPointCloud<datatype, uvtype>::CopyData(const C3DPointCloud &cPoint)
{
    // clear memory if point is not empty
    ReleaseData();

    // allocate memory
    m_acVertexList.reserve(C3DPointCloud.m_acVertexList.size());
    m_acUVList.reserve(C3DPointCloud.m_acUVList.size());
    m_acNormalList.reserve(C3DPointCloud.m_acNormalList.size());


    // make copy of data arrays
    std::copy(C3DPointCloud.m_acVertexList.begin(), C3DPointCloud.m_acVertexList.end(), m_acVertexList.begin());
    std::copy(C3DPointCloud.m_acUVList.begin(), C3DPointCloud.m_acUVList.end(), m_acUVList.begin());
    std::copy(C3DPointCloud.m_acNormalList.begin(), C3DPointCloud.m_acNormalList.end(), m_acNormalList.begin());

    return 0;
}
//template <typename datatype, typename uvtype>
//C3DPointCloud<datatype, uvtype> :: operator +(const C3DPointCloud &other)const
//{
//    C3DPointCloud cloud;
//    return cloud;
//}
//
//template <typename datatype, typename uvtype>
//C3DPointCloud<datatype, uvtype> ::operator -(const C3DPointCloud& vector) const
//{
//    C3DPointCloud cloud;
//    return cloud;
//}
//template <typename datatype, typename uvtype>
//C3DPointCloud<datatype, uvtype> :: operator *(datatype dScalar) const
//{
//    C3DPointCloud cloud;
//    return cloud;
//}
//
//template <typename datatype, typename uvtype>
//C3DPointCloud<datatype, uvtype> :: operator /(datatype dScalar) const
//{
//    C3DPointCloud cloud;
//    return cloud;
//}

template <typename datatype, typename uvtype>
void C3DPointCloud<datatype, uvtype>::ReleaseData()
{
    m_acVertexList.clear();
    m_acUVList.clear();
    m_acNormalList.clear();
}

template <typename datatype, typename uvtype>
int C3DPointCloud<datatype, uvtype> ::GetNumVertices()const
{
    return(int)m_acVertexList.size();
}

template <typename datatype, typename uvtype>
int C3DPointCloud<datatype, uvtype>::GetNumNormals() const
{
    return (int)m_acNormalList.size();
}

template <typename datatype, typename uvtype>
int C3DPointCloud<datatype, uvtype>::GetNumUV() const
{
    return (int)m_acUVList.size();
}

template <typename datatype, typename uvtype>
const Eigen::VectorBlock<datatype>& C3DPointCloud<datatype, uvtype>::GetVertex(int nIndex) const
{
    return m_acVertexList[nIndex];
}

template <typename datatype, typename uvtype>
const Eigen::VectorBlock<datatype>& C3DPointCloud<datatype, uvtype>::GetNormal(int nIndex) const
{
    return m_acNormalList[nIndex];
}


template <typename datatype, typename uvtype>
const Eigen::VectorBlock<uvtype>& C3DPointCloud<datatype, uvtype>::GetUVIndex(int nIndex) const
{
    return m_acUVList[nIndex];
}

template <typename datatype, typename uvtype>
void C3DPointCloud<datatype, uvtype>::SetVertex(const Eigen::VectorBlock<datatype> &cVertex, int nIndex)
{
    m_acVertexList[nIndex] = cVertex;
}

template <typename datatype, typename uvtype>
void C3DPointCloud<datatype, uvtype>::SetNormal(const Eigen::VectorBlock<datatype> &cNormal, int nIndex)
{
    m_acNormalList[nIndex] = cNormal;
}

template <typename datatype, typename uvtype>
void C3DPointCloud<datatype, uvtype>::SetUV(const Eigen::VectorBlock<uvtype> &UV, int nIndex)
{
    m_acUVList[nIndex] = UV;
}
template <typename datatype, typename uvtype>
void C3DPointCloud<datatype, uvtype>::setColor(const uint32_t &colorValue, int nIndex)
{
    m_acColorList[nIndex] = colorValue;
}

template <typename datatype, typename uvtype>
std::map<Eigen::VectorBlock<uvtype>, Eigen::VectorBlock<datatype>> C3DPointCloud<datatype,uvtype>::getMap()
{
    return UV3DMap;
}

template <typename datatype, typename uvtype>
void C3DPointCloud<datatype, uvtype>::createMap()
{
    if ((this.m_acVertexList.size() == 0) || (this.m_acUVList.size() == 0))
    throw std::runtime_error("Missing Data : cannot create Map");

   
    int num3DPoint = this.GetNumVertices();
    int numUV = this.GetNumUV();
    if(numUV != num3DPoint)
        throw std::runtime_error("Missmatch no. of points : cannot create Map");

    for (int i = 0; i < num3DPoint; i++)
    {
        Eigen::VectorBlock<uvtype> tempUV = GetVertex(i);
        Eigen::VectorBlock<datatype> tempPoint = GetUVIndex(i);
        this.insert(std::pair(Eigen::VectorBlock<uvtype>, Eigen::VectorBlock<datatype>)(tempUV, tempPoint));
    }

}
