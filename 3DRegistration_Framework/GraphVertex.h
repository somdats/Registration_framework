/*!
	\file GraphVertex.h
	
	Contains class declaration of graph vertex whose vertex list can be accessed.
	Author: Divya Banda
	Email:	divyareddy@iitkgp.ac.in
*/


# pragma once
#include "Graph.h"
#include <iostream>

class CGraphWithVertex : public CGraph<int>
{
public:
	CGraphVertex<int> GetVertex (int VertexIndex)
	{
		return m_cVertexList[VertexIndex];
	}
};
