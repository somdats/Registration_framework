

#pragma once

#include <vector>
#include <list>
#include "Common.h"

typedef enum _tag_EColor
{
	_WHITE,
	_GREY,
	_BLACK,
	_RED
}EColor;

template<class EdgeDataType>
class CGraphEdge
{
public:
	int				m_nVertexIndex;					// reference to the terminal vertex of the edge
	double			m_dEdgeWeight;
	EdgeDataType	m_cAuxData;

	CGraphEdge ();
	CGraphEdge (int nVertexIndex, double dEdgeWeight, EdgeDataType cAuxData = NULL);
};

template<class EdgeDataType>
class CGraphVertex
{
public:
	int									m_nDataIndex;		// index reference to an external array
	EColor								m_eVertexColor;
	std::list<CGraphEdge<EdgeDataType>>	m_cEdgeList;

	CGraphVertex ();
	~CGraphVertex();
};

template<class EdgeDataType>
class CGraph
{
protected:
	bool									m_fIsDirected;
	int										m_nRootIndex;
	std::vector<CGraphVertex<EdgeDataType>>	m_cVertexList;
	
public:
	CGraph ();
	~CGraph ();
	void ReleaseData ();
	void ClearVertexColor ();

	bool IsGraphEmpty () const;
	int GetRootIndex () const;
	int GetNumVertices () const;
	bool IsValidVertex (int nIndex) const;
	bool IsValidEdge (int nBeginIndex, int nEndIndex) const;
	bool DoesEdgeExists (int nBeginIndex, int nEndIndex) const;

	int Create (int numVertices, bool fIsDirected = true);
	void SetRootIndex (int nRootIndex);
	void SetVertexData (int nVertexIndex, int nDataIndex);
	void SetVertexColor (int nVertexIndex, EColor eColor);
	void AddEdge (int nBeginIndex, int nEndIndex, double dEdgeWeight, EdgeDataType cAuxData);

	int GetVertexData (int nIndex) const;
	EColor GetVertexColor (int nIndex) const;
	CGraphEdge<EdgeDataType> GetNextEdge (int nVertexIndex = -1) const;
	double GetEdgeWeight (int nBeginIndex, int nEndIndex) const;
	EdgeDataType GetEdgeData (int nBeginIndex, int nEndIndex) const;
};

template<class EdgeDataType>
CGraphEdge<EdgeDataType>::CGraphEdge ()
{
	m_nVertexIndex = 0;
	m_dEdgeWeight = 0.0;
}

template<class EdgeDataType>
CGraphEdge<EdgeDataType>::CGraphEdge (int nVertexIndex, double dEdgeWeight, EdgeDataType cAuxData)
{
	m_nVertexIndex = nVertexIndex;
	m_dEdgeWeight = dEdgeWeight;
	m_cAuxData = cAuxData;
}

template<class EdgeDataType>
CGraphVertex<EdgeDataType>::CGraphVertex ()
{
	m_nDataIndex = 0;
	m_eVertexColor = _WHITE;
	m_cEdgeList.clear ();
}

template<class EdgeDataType>
CGraphVertex<EdgeDataType>::~CGraphVertex()
{
	m_cEdgeList.clear ();
}

template<class EdgeDataType>
CGraph<EdgeDataType>::CGraph ()
{
	m_fIsDirected = true;
	m_nRootIndex = -1;
	m_cVertexList.clear ();
}

template<class EdgeDataType>
CGraph<EdgeDataType>::~CGraph ()
{
	ReleaseData ();
}

template<class EdgeDataType>
void CGraph<EdgeDataType>::ReleaseData ()
{
	m_fIsDirected = true;
	m_nRootIndex = -1;
	m_cVertexList.clear ();
}

template<class EdgeDataType>
void CGraph<EdgeDataType>::ClearVertexColor ()
{
	for (int iCount = 0; iCount < m_cVertexList.size (); iCount++)
	{
		m_cVertexList [iCount].m_eVertexColor = _WHITE;
	}
}

template<class EdgeDataType>
bool CGraph<EdgeDataType>::IsGraphEmpty () const
{
	return (0 == (int)m_cVertexList.size ());
}

template<class EdgeDataType>
int CGraph<EdgeDataType>::GetRootIndex () const
{
	return m_nRootIndex;
}

template<class EdgeDataType>
int CGraph<EdgeDataType>::GetNumVertices () const
{
	return ((int)m_cVertexList.size ());
}

template<class EdgeDataType>
bool CGraph<EdgeDataType>::IsValidVertex (int nIndex) const
{
	return (0 <= nIndex && (int)m_cVertexList.size () > nIndex);
}

template<class EdgeDataType>
bool CGraph<EdgeDataType>::IsValidEdge (int nBeginIndex, int nEndIndex) const
{
	std::list<CGraphEdge<EdgeDataType>>::const_iterator	it;
	for (it = m_cVertexList [nBeginIndex].m_cEdgeList.begin (); it != m_cVertexList [nBeginIndex].m_cEdgeList.end (); it++)
	{
		if (it->m_nVertexIndex == nEndIndex)	// edge already exists
		{
			return false;						// not a valid edge
		}
	}

	return true;
}

template<class EdgeDataType>
bool CGraph<EdgeDataType>::DoesEdgeExists (int nBeginIndex, int nEndIndex) const
{
	return (!IsValidEdge (nBeginIndex, nEndIndex));
}

template<class EdgeDataType>
int CGraph<EdgeDataType>::Create (int numVertices, bool fIsDirected)
{
	ReleaseData ();

	m_fIsDirected = fIsDirected;
	m_nRootIndex = 0;
	m_cVertexList.resize (numVertices);

	return 0;
}

template<class EdgeDataType>
void CGraph<EdgeDataType>::SetRootIndex (int nRootIndex)
{
	if (IsValidVertex (nRootIndex))
	{
		m_nRootIndex = nRootIndex;
	}
}

template<class EdgeDataType>
void CGraph<EdgeDataType>::SetVertexData (int nVertexIndex, int nDataIndex)
{
	if (IsValidVertex (nVertexIndex))
	{
		m_cVertexList[nVertexIndex].m_nDataIndex = nDataIndex;
	}
}

template<class EdgeDataType>
void CGraph<EdgeDataType>::SetVertexColor (int nVertexIndex, EColor eColor)
{
	if (IsValidVertex (nVertexIndex))
	{
		m_cVertexList[nVertexIndex].m_eVertexColor = eColor;
	}
}

template<class EdgeDataType>
void CGraph<EdgeDataType>::AddEdge (int nBeginIndex, int nEndIndex, double dEdgeWeight, EdgeDataType cAuxData)
{
	if (IsValidEdge (nBeginIndex, nEndIndex) )
	{
		CGraphEdge<EdgeDataType> cEdge (nEndIndex, dEdgeWeight, cAuxData);
		m_cVertexList[nBeginIndex].m_cEdgeList.push_back(cEdge);
	}

	if (false == m_fIsDirected)
	{
		CGraphEdge<EdgeDataType> cEdge (nBeginIndex, dEdgeWeight, cAuxData);
		if (IsValidEdge (nEndIndex, nBeginIndex))
		{
			m_cVertexList[nEndIndex].m_cEdgeList.push_back(cEdge);
		}
	}
}

template<class EdgeDataType>
int CGraph<EdgeDataType>::GetVertexData (int nIndex) const
{
	return m_cVertexList [nIndex].m_nDataIndex;
}

template<class EdgeDataType>
EColor CGraph<EdgeDataType>::GetVertexColor (int nIndex) const
{
	return m_cVertexList [nIndex].m_eVertexColor;
}

template<class EdgeDataType>
CGraphEdge<EdgeDataType> CGraph<EdgeDataType>::GetNextEdge (int nVertexIndex) const
{
	static int m_nVertexIndex = 0;
	static std::list<CGraphEdge<EdgeDataType>>::const_iterator	it = m_cVertexList[m_nVertexIndex].m_cEdgeList.begin ();

	if (nVertexIndex >= 0)
	{
		// reset iterator
		m_nVertexIndex = nVertexIndex;
		it = m_cVertexList[m_nVertexIndex].m_cEdgeList.begin ();
		return *it; //->m_nVertexIndex;
	}
	else
	{
		it++;
		if (it == m_cVertexList[m_nVertexIndex].m_cEdgeList.end ())
		{
			CGraphEdge<EdgeDataType> cEdge ;
			cEdge.m_nVertexIndex = -1;
			return cEdge;
		}
		
		return *it; //->m_nVertexIndex;
	}
}

template<class EdgeDataType>
double CGraph<EdgeDataType>::GetEdgeWeight (int nBeginIndex, int nEndIndex) const
{
	// return 0 if self edge is requested
	if (nBeginIndex == nEndIndex)
	{
		return 0;
	}

	// try finding edge. if found return edge weight
	std::list<CGraphEdge<EdgeDataType>>::const_iterator	it;
	for (it = m_cVertexList [nBeginIndex].m_cEdgeList.begin (); it != m_cVertexList [nBeginIndex].m_cEdgeList.end (); it++)
	{
		// matching edge found
		if (it->m_nVertexIndex == nEndIndex)
		{
			return it->m_dEdgeWeight;
		}
	}

	// return infinity if edge doesn't exist
	return INF;
}

template<class EdgeDataType>
EdgeDataType CGraph<EdgeDataType>::GetEdgeData (int nBeginIndex, int nEndIndex) const
{
	std::list<CGraphEdge<EdgeDataType>>::const_iterator	it;
	for (it = m_cVertexList [nBeginIndex].m_cEdgeList.begin (); it != m_cVertexList [nBeginIndex].m_cEdgeList.end (); it++)
	{
		// matching edge found
		if (it->m_nVertexIndex == nEndIndex)
		{
			return it->m_cAuxData;
		}
	}
	EdgeDataType	temp;

	return temp;
}
