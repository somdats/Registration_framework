/*!
	\file 2DGeometry.cpp
	\brief 2D Coordinate Geometry Module Implementation
	
*/
#include"pch.h"
#include <stdlib.h>
#include <math.h>
#include "2DGeometry.h"

#define height 1000




/*! 
	2D Geometry : point implementation
*/

/*!
	Initialize all data members with INF
*/
template<typename UVType>
CPoint2D<UVType>::CPoint2D(void)
{
	m_dX = INF;
	m_dY = INF;
}

/*!
	Initialize all data members with given values
*/
template<typename UVType>
CPoint2D<UVType>::CPoint2D(UVType dX, UVType dY)
{
	m_dX = dX;
	m_dY = dY;
}

/*!
	de-allocating the memory
*/
template<typename UVType>
CPoint2D<UVType>::~CPoint2D(void)
{

}

/*!
	\return x-coordinate of the 2D point
*/
template<typename UVType>
UVType CPoint2D<UVType>::getX() const
{
	return m_dX;
}

/*!
	\return y-coordinate of the 2D point
*/
template<typename UVType>
UVType CPoint2D<UVType>::getY() const
{
	return m_dY;
}

/*!
	set x-coordinate of the 2D point
*/
template<typename UVType>
void CPoint2D<UVType>::setX(UVType dX)
{
	m_dX = dX;
}

/*!
	set y-coordinate of the 2D point
*/
template<typename UVType>
void CPoint2D<UVType>::setY(UVType dY)
{
	m_dY = dY;
}

/*!
	init coordinates of the 2D point
*/
template<typename UVType>
void CPoint2D<UVType>::init (UVType dX, UVType dY)
{
	m_dX = dX;
	m_dY = dY;
}

/*!
	initialize with the given 2D point
*/
template<typename UVType>
void CPoint2D<UVType>::init(CPoint2D cPoint)
{
	m_dX = cPoint.getX();
	m_dY = cPoint.getY();
}


