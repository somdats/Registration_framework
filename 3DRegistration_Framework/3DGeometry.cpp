#include"pch.h"
/*!
	\file 3DGeometry.cpp
	\brief 3D Vector Geometry Module Implementation
*/

#include <math.h>

#include "3DGeometry.h"


/*
	3D Geomentry : Vector implementation
*/
// initialization
template<typename PointDataType>
C3DVector<PointDataType>::C3DVector ()
{
	m_dX = INF;
	m_dY = INF;
	m_dZ = INF;
}

template<typename PointDataType>
C3DVector<PointDataType>::C3DVector (const C3DVector& a)
{
    
	m_dX = a.m_dX;
	m_dY = a.m_dY;
	m_dZ = a.m_dZ;
}

template<typename PointDataType>
C3DVector<PointDataType>::C3DVector (PointDataType x, PointDataType y, PointDataType z)
{
	m_dX = x;
	m_dY = y;
	m_dZ = z;
}

template<typename PointDataType>
void C3DVector<PointDataType>::SetData (PointDataType x, PointDataType y, PointDataType z)
{
	m_dX = x;
	m_dY = y;
	m_dZ = z;
}

template<typename PointDataType>
void C3DVector< PointDataType>::GetData (PointDataType &x, PointDataType &y, PointDataType &z) const
{
	x = m_dX;
	y = m_dY;
	z = m_dZ;
}

template<typename PointDataType>
PointDataType	C3DVector< PointDataType>::X () const
{
	return m_dX;
}

template<typename PointDataType>
PointDataType	C3DVector<PointDataType>::Y () const
{
	return m_dY;
}

template<typename PointDataType>
PointDataType C3DVector<PointDataType>::Z () const
{
	return m_dZ;
}

template<typename PointDataType>
bool C3DVector< PointDataType>::isValid () const
{
	return !(*this == C3DVector());
}

// binary operators
template<typename PointDataType>
C3DVector<PointDataType> C3DVector< PointDataType>::operator + (const C3DVector<PointDataType>& vector) const
{
	C3DVector<PointDataType> cTempVector ((*this).m_dX + vector.m_dX, (*this).m_dY + vector.m_dY, (*this).m_dZ + vector.m_dZ);
	return cTempVector;
}

template<typename PointDataType>
C3DVector<PointDataType> C3DVector<PointDataType>::operator - (const C3DVector<PointDataType>& vector) const
{
	C3DVector<PointDataType> cTempVector ((*this).m_dX - vector.m_dX, (*this).m_dY - vector.m_dY, (*this).m_dZ - vector.m_dZ);
	return cTempVector;
}

template<typename PointDataType>
C3DVector<PointDataType> C3DVector<PointDataType>::operator * (PointDataType dScalar) const
{
	C3DVector<PointDataType> cTempVector ((*this).m_dX * dScalar,	(*this).m_dY * dScalar, (*this).m_dZ * dScalar);
	return cTempVector;
}

template<typename PointDataType>
C3DVector<PointDataType> C3DVector< PointDataType>::operator / (PointDataType dScalar) const
{
	C3DVector<PointDataType> cTempVector ((*this).m_dX / dScalar, (*this).m_dY / dScalar, (*this).m_dZ / dScalar);
	return cTempVector;
}
template<typename PointDataType>
C3DVector<PointDataType>  operator * (PointDataType dScalar, const C3DVector<PointDataType>& vector)
{
	return (vector * dScalar);
}

// conditional operators
template<typename PointDataType>
bool C3DVector< PointDataType>::operator == (const C3DVector<PointDataType>& vector) const
{
	return (fabs ((*this).m_dX - vector.m_dX) < EPSILON && fabs ((*this).m_dY - vector.m_dY) < EPSILON && fabs ((*this).m_dZ - vector.m_dZ) < EPSILON);
}

template<typename PointDataType>
bool C3DVector< PointDataType>::operator != (const C3DVector<PointDataType>& vector) const
{
	return (!((*this) == vector));
}
template<typename PointDataType>
bool C3DVector< PointDataType>:: operator <(const C3DVector<PointDataType> &vector)const
{
    return std::tie (m_dX, m_dY, m_dZ) < std::tie(vector.m_dX, vector.m_dY, vector.m_dZ);
}
template<typename PointDataType>
bool IsVectorParallel (const C3DVector<PointDataType> &cV1, const C3DVector<PointDataType> &cV2)
{
	if (cV1.Direction () == cV2.Direction () || cV1.Direction () == -1 * cV2.Direction ())
	{
		return true;
	}
	return false;
}

// vector operations
template<typename PointDataType>
PointDataType C3DVector< PointDataType>::Magnitude () const
{
	return (sqrt (m_dX * m_dX + m_dY * m_dY + m_dZ * m_dZ));
}
template<typename PointDataType>
C3DVector<PointDataType> C3DVector<PointDataType>::Direction () const
{
    PointDataType dMagnitude = C3DVector::Magnitude ();
	if (fabs (dMagnitude) < EPSILON)
	{
		return C3DVector (0.0, 0.0, 0.0);
	}
	C3DVector<PointDataType> cTempVector (m_dX / dMagnitude, m_dY / dMagnitude, m_dZ / dMagnitude);
	return cTempVector;
}
template<typename PointDataType>
void C3DVector<PointDataType>::Normalize ()
{
	(*this) = C3DVector::Direction ();
}
template<typename PointDataType>
PointDataType VectorDot (const C3DVector<PointDataType>& a, const C3DVector<PointDataType>& b)
{
	return (a.m_dX * b.m_dX + a.m_dY * b.m_dY + a.m_dZ * b.m_dZ);
}
template<typename PointDataType>
C3DVector<PointDataType> VectorCross(const C3DVector<PointDataType>& a, const C3DVector<PointDataType>& b)
{
	C3DVector<PointDataType> cTempVector (a.m_dY * b.m_dZ - b.m_dY * a.m_dZ, b.m_dX * a.m_dZ - a.m_dX * b.m_dZ, a.m_dX * b.m_dY - b.m_dX * a.m_dY);
	return cTempVector;
}
template<typename PointDataType>
PointDataType VectorAngle (const C3DVector<PointDataType>& a, const C3DVector<PointDataType>& b)
{
	return acos ((a.m_dX * b.m_dX + a.m_dY * b.m_dY + a.m_dZ * b.m_dZ) / 
		((sqrt (a.m_dX * a.m_dX + a.m_dY * a.m_dY + a.m_dZ * a.m_dZ)) * (sqrt (b.m_dX * b.m_dX + b.m_dY * b.m_dY + b.m_dZ * b.m_dZ))));
}

