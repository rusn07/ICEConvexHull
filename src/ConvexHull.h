#ifndef __CONVEXHULL_H__
#define __CONVEXHULL_H__

#include <vector>

#include <xsi_application.h>
#include <xsi_vector3.h>
#include <xsi_indexset.h>
#include <xsi_dataarray.h>
#include <xsi_dataarray2D.h>

#define EPSILON FLT_EPSILON
#define END_POLYGON -2

using namespace XSI;
using namespace XSI::MATH;

typedef std::vector<CVector3> PositionArray;
typedef std::vector<LONG> LongArray;

class Mesh
{
	public:

	struct Edge
	{
		LONG m_lNode1, m_lNode2;

		Edge(): m_lNode1(0), m_lNode2(0) {}
		Edge(const LONG node1, const LONG node2): m_lNode1(node1), m_lNode2(node2) { }
		void Set(const LONG node1, const LONG node2){ m_lNode1=node1; m_lNode2=node2; } 
		bool operator == (const Edge& edge) { return m_lNode1==edge.m_lNode1 && m_lNode2==edge.m_lNode2; }
	};

	struct Facet
	{ 
		std::vector<LONG> m_indices;
		CVector3 m_normal;
	
		Facet& operator = (const Facet& face) { m_normal=face.m_normal; m_indices=face.m_indices; return *this; }
	};

	// This object store a vertex index and its "distance" in the direction of the edge BA of the triangle
	struct PointOnFace
	{
		LONG index;
		double distance;
	
		PointOnFace(const LONG id=0,const double dist=0.0):index(id), distance(dist) { }
	};

	
	typedef std::vector<Mesh::Edge> EdgeArray;
	typedef std::vector<Mesh::Facet> FaceArray;
	typedef std::vector<Mesh::PointOnFace> PointsOnPlaneArray;
};




class ConvexHull : public Mesh
{
public:
	
	ConvexHull(): m_bIsValid(false), m_lVertexA(-1), m_lVertexB(-1), m_lVertexC(-1), m_ulNbToDo(1)	{	};
	void GetPointsSet(CDataArray2DVector3f& inPositionsData, const bool IsFlat); // Singleton Array of positions
	void GetPointsSet(CIndexSet& inIndexSet, CDataArrayVector3f& inPositionsData, const bool IsFlat); // Per Element position
	LongArray BuildHull(const bool isTriangulate);
	bool IsValid()const{return m_bIsValid;}
	LongArray SetPolygonalDescription();

private:
	
	// Methods
	CVector3 RandomUnitVector();
	void GetFirstVertex();
	void GetSecondVertex();
	void GetNextNonColinearPoint(CVector3& out_vTriangleNormal);
	void UpdateNumberToDo(){ m_ulNbToDo = static_cast<ULONG>( m_edgesToDo.size() ); }

	// Data
	bool				m_bIsValid;
	ULONG				m_ulNbPoints;
	ULONG				m_currentPoint;
	ULONG				m_ulNbToDo;
	LONG				m_lVertexA, m_lVertexB, m_lVertexC;
	double				m_dMaxDot, m_dCurrentDot;
	CVector3			m_vRandomNormal, m_vLineBA, m_vLineBC;
	PositionArray		m_positions;
	EdgeArray			m_edges, m_edgesToDo;
	FaceArray			m_faces;
	PointsOnPlaneArray	m_pointsOnSamePlane;
};


#endif /* __CONVEXHULL_H__ */