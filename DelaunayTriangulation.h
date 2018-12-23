#ifndef _DELAUNAYTRIANGULATION_HEADER_
#define _DELAUNAYTRIANGULATION_HEADER_
#include "..\include\DHalfEdge.h"
#include <time.h>



class DelaunayTriangulation2D {
public:
	DelaunayTriangulation2D();
	~DelaunayTriangulation2D();

	class DelaunayData
	{
	public:
		DTHalfEdge * rightmost;
		DTHalfEdge * leftmost;
	};

	enum POS_Line {
		ON_RIGHT,
		ON_SEG,
		ON_LEFT
	};
	enum POS_Circle {
		INSIDE,
		OUTSIDE,
		ON_CIRCLE
	};
public:
	void Triangulate();
	void AddPoints(std::vector<DTVertex*>& vts);
	void buildface();
public:
	void divide_and_conquer(DelaunayData *del, int start, int end);
	void mergedelaunay(DelaunayData *result, DelaunayData *left, DelaunayData *right);
protected:
	void sortvertex();
	void init_tri(DelaunayData *del, int start);
	void init_seg(DelaunayData *del, int start);
	// decide vertex side
	DelaunayTriangulation2D::POS_Line point_halfedge_position(DTHalfEdge *d, DTVertex *pt);
	// decide vertex side
	DelaunayTriangulation2D::POS_Line point_seg_position(DTVertex *s, DTVertex *e, DTVertex *pt);
	// get lower tangent
	DTHalfEdge* get_lower_tangent(DelaunayData *left, DelaunayData *right);
	DTHalfEdge* findValidBaseLine(DTHalfEdge* base);
	DTHalfEdge* findleftCandinateVertexHalfEdge(DTHalfEdge* base);
	DTHalfEdge* findrightCandinateVertexHalfEdge(DTHalfEdge* base);
	DelaunayTriangulation2D::POS_Circle in_circle(DTVertex *pt0, DTVertex *pt1, DTVertex *pt2, DTVertex *p);
	void RemoveHalfEdge(DTHalfEdge* e);
	// add half-edge
	void addHalfEdge(DTHalfEdge* e, DTVertex *v1, DTVertex *v2) {
		std::pair<DTVertex *, DTVertex *> p = std::make_pair(v1, v2);
		m_edgePairs_cache[p] = e;
	}
	void build_halfedge_face(DTHalfEdge* h);

	void testoutput(DelaunayData* m_del);
protected:
	std::vector<DTVertex *> m_vertexs;// all the vertex
	std::map< std::pair<DTVertex *, DTVertex *>, DTHalfEdge* > m_edgePairs_cache;
	//all the faces
	std::vector<DTFace *> m_faces;
	DelaunayData* m_delaunay;
};
#endif// 2018/12/07