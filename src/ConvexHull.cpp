#include <xsi_vector3f.h>
#include <xsi_random.h>

#include <vector>
#include <algorithm>

#include "ConvexHull.h"


//---------------------------------------------------------------------------
// Helper functions.

bool SortByBiggestDistance( const Mesh::PointOnFace elem1, const Mesh::PointOnFace elem2 )
{
	return elem1.distance > elem2.distance;
}

inline LONG FindEdge(const Mesh::EdgeArray& edgeArray, Mesh::Edge edge) 
{
	for (ULONG i=0; i<edgeArray.size(); ++i)
		if (edge==edgeArray[i])
			return i;
	return -1;
}


//---------------------------------------------------------------------------
// ConvexHull::GetPointsSetFromArray
//
// Set the points set using a singleton array.
void ConvexHull::GetPointsSet(CDataArray2DVector3f& inPositionsData, const bool IsFlat=false)
{
	CDataArray2DVector3f::Accessor inPositionsAccessor = inPositionsData[0]; // always first element as we are singleton
	m_ulNbPoints = inPositionsAccessor.GetCount();
	if(m_ulNbPoints > 3)
	{
		m_bIsValid = true;
		
		m_positions.resize(inPositionsAccessor.GetCount());
		
		CRandom rx, ry, rz;
		double x,y,z;
		for (ULONG i=0; i<inPositionsAccessor.GetCount(); i++ )
		{
			if(IsFlat)
			{
				rx = CRandom(i);
				ry = CRandom(i+1);
				rz = CRandom(i+2);
				x = static_cast<double>(rx.GetNormalizedValue())*0.001;
				y = static_cast<double>(ry.GetNormalizedValue())*0.001;
				z = static_cast<double>(rz.GetNormalizedValue())*0.001;

				m_positions[i].Set(inPositionsAccessor[i].GetX()+x, inPositionsAccessor[i].GetY()+y, inPositionsAccessor[i].GetZ()+z);
			}
			else
			{
				m_positions[i].Set(inPositionsAccessor[i].GetX(), inPositionsAccessor[i].GetY(), inPositionsAccessor[i].GetZ());
			}
		}
	}
}

//---------------------------------------------------------------------------
// ConvexHull::GetPointsSetFromElememts
//
// Set the points set from the non singleton context.
void ConvexHull::GetPointsSet(CIndexSet & inIndexSet, CDataArrayVector3f & inPositionsData, const bool IsFlat=false)
{
	m_ulNbPoints = inPositionsData.GetCount();
	if(m_ulNbPoints > 3)
	{
		m_bIsValid = true;
		
		m_positions.resize(inPositionsData.GetCount());
		
		CRandom rx, ry, rz;
		double x,y,z;
		for(CIndexSet::Iterator it = inIndexSet.Begin(); it.HasNext(); it.Next())
		{
			if(IsFlat)
			{
				rx = CRandom(it.GetAbsoluteIndex());
				ry = CRandom(it.GetAbsoluteIndex()+1);
				rz = CRandom(it.GetAbsoluteIndex()+2);
				x = static_cast<double>(rx.GetNormalizedValue())*0.001;
				y = static_cast<double>(ry.GetNormalizedValue())*0.001;
				z = static_cast<double>(rz.GetNormalizedValue())*0.001;

				m_positions[it.GetAbsoluteIndex()].Set(inPositionsData[it].GetX()+x, inPositionsData[it].GetY()+y, inPositionsData[it].GetZ()+z);
			}
			else
			{
				m_positions[it.GetAbsoluteIndex()].Set(inPositionsData[it].GetX(), inPositionsData[it].GetY(), inPositionsData[it].GetZ());
			}
		}
	}
}

//---------------------------------------------------------------------------
// ConvexHull::RandomUnitVector
//
// A random and normalized vector used to find the initial point.
inline CVector3 ConvexHull::RandomUnitVector()
{
	float x = static_cast<float>(CRandom(12345));
	float y = static_cast<float>(CRandom(12346));
	float z = static_cast<float>(CRandom(12347));
	
	CVector3 vec(x, y, z);
	vec.NormalizeInPlace();
	return vec;
}

inline void ConvexHull::GetFirstVertex()
{
	// Find the extreme point in some random direction.
	m_vRandomNormal = RandomUnitVector();

	for (m_currentPoint=0; m_currentPoint<m_ulNbPoints; ++m_currentPoint)
	{
		m_dCurrentDot = m_vRandomNormal.Dot(m_positions[m_currentPoint]);
		if (m_currentPoint==0 || m_dCurrentDot>m_dMaxDot)
		{
			m_dMaxDot = m_dCurrentDot;
			m_lVertexA = m_currentPoint;
		}
	}	
}

inline void	ConvexHull::GetSecondVertex()
{
	//  Find another point, m_lVertexB, to use for the initial m_edges.  Choose m_lVertexB
	//  with the smallest angle to the normal plane at m_lVertexA.
	m_dMaxDot = 2;
	m_lVertexB = -1;
	for (m_currentPoint=0; m_currentPoint<m_ulNbPoints; ++m_currentPoint)
	{
		if (m_currentPoint != m_lVertexA)
		{
			m_vLineBA.Sub( m_positions[m_lVertexA], m_positions[m_currentPoint] );
			m_dCurrentDot = m_vRandomNormal.Dot(m_vLineBA) / m_vLineBA.GetLength();
			if (m_dCurrentDot < m_dMaxDot)
			{
				m_dMaxDot = m_dCurrentDot;
				m_lVertexB = m_currentPoint;
			}
		}
	}
}

inline void ConvexHull::GetNextNonColinearPoint(CVector3& out_vTriangleNormal)
{
	for (m_lVertexC=0; m_lVertexC < static_cast<LONG>( m_ulNbPoints ); ++m_lVertexC)
	{
		if (m_lVertexC!=m_lVertexA && m_lVertexC!=m_lVertexB)
		{
			m_vLineBC.Sub( m_positions[m_lVertexC], m_positions[m_lVertexB] );
			out_vTriangleNormal.Cross(m_vLineBA, m_vLineBC);
			// If three points are not collinear
			if (out_vTriangleNormal.GetLength() > EPSILON)
				break;
		}
	}
}

//---------------------------------------------------------------------------
//  ConvexHull::BuildHull
//
//  This computes the edges and faces of the ConvexHull based on the
//  vertices.
LongArray ConvexHull::BuildHull(const bool isTriangulate )
{
	CVector3 l_vPosC, l_vPt2ToPt1, l_vTriangleNormal, l_vMidAB;
	Facet l_facet, l_triangle;
	l_triangle.m_indices.resize(3);
	ULONG l_ulLastvertex, l_ulIndexCounter;

	LONG l_lA1, l_lA2, l_lNbPointsOnPlane;

	GetFirstVertex();
	GetSecondVertex();
	
	// Store first edge
	Edge l_edge(m_lVertexA,m_lVertexB);
	m_edgesToDo.push_back(l_edge);
	
	if(m_lVertexA<m_lVertexB)
	{
		m_edges.push_back(l_edge);
	}
	else
	{
		m_edges.push_back(Edge(m_lVertexB,m_lVertexA));
	}
	
	while (m_ulNbToDo > 0)
	{
		m_lVertexA = m_edgesToDo[m_ulNbToDo-1].m_lNode1;
		m_lVertexB = m_edgesToDo[m_ulNbToDo-1].m_lNode2;

		m_vLineBA.Sub( m_positions[m_lVertexA], m_positions[m_lVertexB] );

		GetNextNonColinearPoint(l_vTriangleNormal);
		
		// Find the third vertex starting from the next non-collinear point
		for (l_ulIndexCounter=0; l_ulIndexCounter<m_ulNbPoints; ++l_ulIndexCounter)
		{
			m_dMaxDot=0.0; 
			l_ulLastvertex=m_lVertexC;
			m_pointsOnSamePlane.clear();

			for (m_currentPoint=0; m_currentPoint<m_ulNbPoints; ++m_currentPoint)
			{
				if (m_currentPoint != m_lVertexA && m_currentPoint != m_lVertexB )
				{
					l_vPt2ToPt1.Sub( m_positions[m_currentPoint], m_positions[m_lVertexB] );
					
					// distance to the current point in direction of the potential l_triangle normal.
					m_dCurrentDot = l_vPt2ToPt1.Dot(l_vTriangleNormal);
					
					// if dot == 0 we are on the l_triangle plane
					if (fabs(m_dCurrentDot) < EPSILON )
					{
						m_pointsOnSamePlane.push_back(PointOnFace((LONG)m_currentPoint));
					}
					else if (m_dCurrentDot > m_dMaxDot)
					{
						m_dMaxDot = m_dCurrentDot;
						m_lVertexC = m_currentPoint;
					}
				}
			}
			
			if (l_ulLastvertex==m_lVertexC) break;         // Success, no exterior points found.

			m_vLineBC.Sub( m_positions[m_lVertexC], m_positions[m_lVertexB] );
			l_vTriangleNormal.Cross(m_vLineBA, m_vLineBC);
			l_vTriangleNormal.NormalizeInPlace();
		}
		
		// we've got a valid l_triangle ABC and a valid Normal. We also got the indices (in m_pointsOnSamePlane) of other points on the plane defined by l_triangle ABC.
		// Lets construct the face using vertex A, vertex B and all the planes points.

		l_lNbPointsOnPlane = static_cast<LONG>( m_pointsOnSamePlane.size() );
		
		if( (l_lNbPointsOnPlane == 0) ) break;


		// Get midpoint between vertex A and vertex B
		l_vMidAB.Add( m_positions[m_lVertexA], m_positions[m_lVertexB] );
		l_vMidAB.ScaleInPlace(0.5);

		for (LONG i=0; i<l_lNbPointsOnPlane; ++i)
		{
			l_vPosC.Sub( m_positions[m_pointsOnSamePlane[i].index], l_vMidAB );
			l_vPosC.NormalizeInPlace();
			m_pointsOnSamePlane[i].distance = m_vLineBA.Dot(l_vPosC);
		}
		
		// We need to get the plane points in the good order to draw a polygon.
		std::sort( m_pointsOnSamePlane.begin(), m_pointsOnSamePlane.end(), SortByBiggestDistance );

		// Lets find the edges of the polygon...
		for (LONG i=-2; i<l_lNbPointsOnPlane; ++i)
		{
			if (i==-2)
			{
				l_lA1 = m_lVertexB; 
				l_lA2 = m_pointsOnSamePlane[l_lNbPointsOnPlane-1].index;
			}
			else if (i==-1)
			{
				l_lA1 = m_lVertexA; 
				l_lA2 = m_lVertexB;
			}
			else
			{
				l_lA1 = m_pointsOnSamePlane[i].index;
				l_lA2 = m_lVertexA;
				m_lVertexA = l_lA1; 
			}
			
			l_facet.m_indices.push_back(l_lA1);

			l_edge.Set(l_lA1,l_lA2);
			if(l_lA1 > l_lA2)
				l_edge = Edge(l_lA2,l_lA1);

			if(FindEdge(m_edges, l_edge) >=0)
			{
				LONG x = FindEdge(m_edgesToDo, Edge(l_lA1,l_lA2));
				if (x >= 0)
					m_edgesToDo.erase(m_edgesToDo.begin()+x);
			}
			else
			{
				m_edges.push_back(l_edge);
				m_edgesToDo.push_back(Edge(l_lA2,l_lA1));
			}
			
		}
		
		l_facet.m_normal = l_vTriangleNormal;
		m_faces.push_back(l_facet);

		l_facet.m_indices.clear();
		
		UpdateNumberToDo();
	}
	
	return SetPolygonalDescription();
 }
 
 inline LongArray ConvexHull::SetPolygonalDescription()
 {
	LongArray lPolyDescription;
	lPolyDescription.reserve(m_faces.size()*4);

	for(ULONG i=0; i<m_faces.size(); i++)
	{
		// number of vertices on the m_faces
		for (ULONG j=0; j< static_cast<ULONG>(m_faces[i].m_indices.size()); ++j)
		{
  			lPolyDescription.push_back(m_faces[i].m_indices[j]);
  		}
  		lPolyDescription.push_back(END_POLYGON);
	}
	
	return lPolyDescription;
 }