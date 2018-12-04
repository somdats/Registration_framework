/*!
	\file 3DGeometry.h
	\brief 3D Vector Geometry Module Header
	
	Contains class declarations of various 3D Vector Geometry constructs and routines.*/


#pragma once
template<typename PointDataType>
#define EPSILON	1E-6
class C3DVector
{
protected:
	// data
    PointDataType m_dX;
    PointDataType m_dY;
    PointDataType m_dZ;

public:
	// initialization
    C3DVector ();
    C3DVector (const C3DVector& a);
    C3DVector (PointDataType x, PointDataType y, PointDataType z);
	void SetData (PointDataType x, PointDataType y, PointDataType z);
	void GetData (PointDataType &x, PointDataType &y, PointDataType &z) const;
    PointDataType	X () const;
    PointDataType	Y () const;
    PointDataType	Z () const;
	bool	isValid () const;

    // binary operators
    C3DVector operator + (const C3DVector& vector) const;
    C3DVector operator - (const C3DVector& vector) const;
    C3DVector operator * (PointDataType dScalar) const;
    C3DVector operator / (PointDataType dScalar) const;
    friend C3DVector operator * (PointDataType dScalar, const C3DVector& vector);
	
	/*
	// unary operators
    C3DVector operator ++ () const;
    C3DVector operator -- () const;

	// assignment operators
    C3DVector& operator += (const C3DVector& vector);
    C3DVector& operator -= (const C3DVector& vector);
    C3DVector& operator *= (double dScalar);
    C3DVector& operator /= (double dScalar);
	*/

	// conditional operators
    bool operator == (const C3DVector &vector) const;
    bool operator != (const C3DVector &vector) const;
    bool operator < (const C3DVector &vector)const;
	friend bool IsVectorParallel (const C3DVector &cV1, const C3DVector &cV2);

	// vector operations
    PointDataType Magnitude () const;
	C3DVector Direction ()  const;
	void Normalize ();
	friend PointDataType VectorDot (const C3DVector &a, const C3DVector &b);
	friend C3DVector VectorCross (const C3DVector &a, const C3DVector &b);
	friend PointDataType VectorAngle (const C3DVector &a, const C3DVector &b);

};

