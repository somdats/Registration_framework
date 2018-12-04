/*!
	\file 2DGeometry.h
	\brief 2D Coordinate Geometry Module Header
	
*/

#pragma once



//!CPoint2D class
/*!
	CPoint2D implements the structure required for a 2D point
*/
template<typename UVType>
class CPoint2D
{
protected:
	//!x-coordinate
    UVType	m_dX;
	//!y-coordinate
    UVType	m_dY;

public:
	//!constructor default
	CPoint2D (void);
	//!constructor with two parameters
	CPoint2D (UVType dX, UVType dY);
	//!destructor
	~CPoint2D (void);
	//!return x-coordinate
    UVType getX (void) const;
	//!return y-coordinate
    UVType getY (void) const;
	//!set x-coordinate
	void setX (UVType dX);
	//!set y-coordinate
	void setY (UVType dY);
	//!initialize the 2D point
	void init (UVType dX, UVType dY);
	//!intialize the point
	void init(CPoint2D cPoint);
	//friend double distPointToPoint (CPoint2D cP1, CPoint2D cp2);
};

