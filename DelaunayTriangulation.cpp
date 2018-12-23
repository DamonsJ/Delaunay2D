#include "..\include\DelaunayTriangulation.h"
#include <assert.h>
DelaunayTriangulation2D::DelaunayTriangulation2D()
{
	m_delaunay = new DelaunayData;
}

DelaunayTriangulation2D::~DelaunayTriangulation2D()
{
}

void DelaunayTriangulation2D::AddPoints(std::vector<DTVertex*>& vts)
{
	for (auto &v :vts)
	{
		m_vertexs.push_back(v);
	}
}

void DelaunayTriangulation2D::Triangulate()
{
	sortvertex();
	int num_points = m_vertexs.size();
	if (num_points >= 3) {
		divide_and_conquer(m_delaunay, 0, num_points - 1);
	}

	buildface();

	std::string filename = "result_ls.off";
	std::ofstream fstrm(filename);
	fstrm << "OFF" << std::endl;
	fstrm << m_vertexs.size() << " " << m_faces.size()-1 << "  0" << std::endl;

	for (auto &ptr : m_vertexs)
	{
		fstrm << ptr->x << " " << ptr->y << " " << ptr->z << std::endl;
	}

	for (int i = 1; i < m_faces.size(); ++i)
	{
		DTFace *f = m_faces[i];
		fstrm << 3;
		DTHalfEdge *curr = f->edge;
		do {
			fstrm << " " << curr->start_vert->id;
			curr = curr->next;
		} while (curr != f->edge);
		fstrm << std::endl;
	}
	
	fstrm.close();
}

void DelaunayTriangulation2D::buildface() {
	// build external face
	build_halfedge_face(m_delaunay->rightmost->pair);
	//////////////////////////////////////////////////////////////////////////

	for (auto &ptr : m_edgePairs_cache)
	{
		DTHalfEdge* curr = ptr.second;
		if(curr->face)
			continue;
		
		build_halfedge_face(curr);
	}
}

void DelaunayTriangulation2D::testoutput(DelaunayData* m_del) {

	build_halfedge_face(m_del->rightmost->pair);
	//////////////////////////////////////////////////////////////////////////

	for (auto &ptr : m_edgePairs_cache)
	{
		DTHalfEdge* curr = ptr.second;
		if (curr->face)
			continue;

		build_halfedge_face(curr);
	}

	std::string filename = "result.off";
	std::ofstream fstrm(filename);
	fstrm << "OFF" << std::endl;
	fstrm << m_vertexs.size() << " " << m_faces.size() - 1 << "  0" << std::endl;

	for (auto &ptr : m_vertexs)
	{
		fstrm << ptr->x << " " << ptr->y << " " << ptr->z << std::endl;
	}

	for (int i = 1; i < m_faces.size(); ++i)
	{
		DTFace *f = m_faces[i];
		fstrm << 3;
		DTHalfEdge *curr = f->edge;
		do {
			fstrm << " " << curr->start_vert->id;
			curr = curr->next;
		} while (curr != f->edge);
		fstrm << std::endl;
	}

	fstrm.close();
}

void DelaunayTriangulation2D::build_halfedge_face(DTHalfEdge* h) {
	if (h->face)
		return;
	DTFace *face = new DTFace;
	m_faces.push_back(face);
	face->edge = h;

	DTHalfEdge* curr = h;
	do {
		curr->face = face;
		(face->number_vertex)++;
		curr = curr->next;
	} while (curr != h);
}

void DelaunayTriangulation2D::divide_and_conquer(DelaunayData *del, int start, int end) {
	DelaunayData	left, right;
	int			i, n;

	n = (end - start + 1);

	if (n > 3) {
		i = (n / 2) + (n & 1);
		divide_and_conquer(&left, start, start + i - 1);
		divide_and_conquer(&right, start + i, end);
		mergedelaunay(del, &left, &right);
	}
	else {
		if (n == 3) {
			init_tri(del, start);
		}
		else {
			if (n == 2) {
				init_seg(del, start);
			}
		}
	}
}

void DelaunayTriangulation2D::mergedelaunay(DelaunayData *result, DelaunayData *left, DelaunayData *right){

	/* save the most right point and the most left point */
	DTVertex *ml = left->leftmost->start_vert;
	DTVertex *mr = right->rightmost->start_vert;
	// get lower tangent edge
	DTHalfEdge* base= get_lower_tangent(left,right);

	DTVertex *R = base->next->pair->start_vert;
	DTVertex *L = base->prev->start_vert;

	while (point_halfedge_position(base, L) == POS_Line::ON_LEFT ||
		   point_halfedge_position(base, R) == POS_Line::ON_LEFT)
	{
		base = findValidBaseLine(base);
		R = base->next->pair->start_vert;
	    L = base->prev->start_vert;
	}

	right->rightmost = mr->edge_out;
	left->leftmost = ml->edge_out;

	//change right most and left most edge
	while (point_halfedge_position(right->rightmost, right->rightmost->pair->next->pair->start_vert) == POS_Line::ON_RIGHT)
		right->rightmost = right->rightmost->pair->next;

	while (point_halfedge_position(left->leftmost, left->leftmost->pair->next->pair->start_vert) == POS_Line::ON_RIGHT)
		left->leftmost = left->leftmost->pair->next;


	result->leftmost = left->leftmost;
	result->rightmost = right->rightmost;

	if (0) {
		testoutput(result);
	}
}

DTHalfEdge* DelaunayTriangulation2D::findValidBaseLine(DTHalfEdge* base){
	//1. find left candinate point
	DTHalfEdge* le = findleftCandinateVertexHalfEdge(base);
	DTVertex* lc = le->start_vert;

	//2. find right candinate point
	DTHalfEdge* re = findrightCandinateVertexHalfEdge(base);
	DTVertex* rc = re->start_vert;

	//3. test left right candinate
	DTVertex* L = base->start_vert;
	DTVertex* R = base->pair->start_vert;

	DTVertex* u = nullptr;
	DTVertex* v = nullptr;

	if (lc != L && rc != R){
		// test circle
		POS_Circle p1 =  in_circle(L,R,lc,rc);
		if(POS_Circle::INSIDE == p1){
			POS_Circle p2 =  in_circle(L,R,rc,lc);
			if(POS_Circle::INSIDE == p2){
				assert(0);
			}
			else{
				u = L;
				v = rc;
			}
		}
		else{
			u = lc;
			v = R;
		}
		
	}
	else if( lc == L && rc!=R){
		// candinate in right
		// create L rc
		u = L;
		v = rc;
	}
	else if( lc != L && rc==R){
		// candinate in right
		// create  lc R
		u = lc;
		v = R;
	}
	else{
	// no candinate
	return base;
	}

	DTHalfEdge *e1 = new DTHalfEdge;
	DTHalfEdge *e2 = new DTHalfEdge;

	if (v == R) {
		e1->start_vert = u;
		e1->prev = base->prev->prev;
		base->prev->prev->next = e1;
		e1->next = base->next;
		base->next->prev = e1;
		e1->pair = e2;

		e2->start_vert = v;
		e2->pair = e1;
		e2->next = base->prev;
		e2->prev = base;
		base->next = e2;
		base->prev->prev = e2;
	}
	else if (u == L) {
		e1->start_vert = u;
		e1->prev = base->prev;
		base->prev->next = e1;

		e1->next = base->next->next;
		base->next->next->prev = e1;
		e1->pair = e2;

		e2->start_vert = v;
		e2->pair = e1;
		e2->next = base;
		e2->prev = base->next;
		base->prev = e2;
		base->next->next = e2;
	}
	
	addHalfEdge(e1, u, v);
	addHalfEdge(e2, v, u);


	return e1;
}

DTHalfEdge* DelaunayTriangulation2D::findleftCandinateVertexHalfEdge(DTHalfEdge* base){

	DTHalfEdge* cur = base->prev;
	DTVertex*   R1 = cur->start_vert;
	DTVertex*   R2 = cur->pair->prev->start_vert;

	DTVertex* lv = base->start_vert;
	DTVertex* rv = base->pair->start_vert;

	DTHalfEdge* candinate_edge = cur->pair;

	if (point_seg_position(lv, rv, R1) == POS_Line::ON_LEFT )
	{
		
		while (R2 != lv && R2 != rv && in_circle(lv, rv, R1, R2) == POS_Circle::INSIDE)
		{
			DTHalfEdge* need_to_remove = cur;
			base->prev = cur->pair->prev;
			cur = base->prev;

			RemoveHalfEdge(need_to_remove);

			R1 = cur->start_vert;
			R2 = cur->pair->prev->start_vert;

			candinate_edge = cur->pair;
		}
	}
	else{
		candinate_edge = base->pair;
	}

	return candinate_edge->pair;
}

DTHalfEdge* DelaunayTriangulation2D::findrightCandinateVertexHalfEdge(DTHalfEdge* base){
	DTHalfEdge* cur = base->next;
	DTVertex*   R1 = cur->pair->start_vert;
	DTVertex*   R2 = cur->pair->next->pair->start_vert;

	DTVertex* lv = base->start_vert;
	DTVertex* rv = base->pair->start_vert;

	
	if (point_seg_position(lv, rv, R1) == POS_Line::ON_LEFT )
	{
		while (R2 != lv && R2 != rv && in_circle(lv, rv, R1, R2) == POS_Circle::INSIDE)
		{
			DTHalfEdge* need_to_remove = cur;
			base->next = cur->pair->next;
			cur = base->next;

			RemoveHalfEdge(need_to_remove);

			R1 = cur->pair->start_vert;
			R2 = cur->pair->next->pair->start_vert;
		}
	}
	else{
		cur = base;
	}

	return cur->pair;
}

void DelaunayTriangulation2D::RemoveHalfEdge(DTHalfEdge* e){
	
	DTVertex *f = e->start_vert;
	DTVertex *s = e->pair->start_vert;

	/* finally free the halfedges */
	std::pair<DTVertex *, DTVertex *> p1 = std::make_pair(f, s);
	m_edgePairs_cache.erase(p1);
	std::pair<DTVertex *, DTVertex *> p2 = std::make_pair(s, f);
	m_edgePairs_cache.erase(p2);

	DTHalfEdge *next  =  e->next;
	DTHalfEdge *prev  =  e->prev;
	DTHalfEdge *pair  =  e->pair;

	prev->next = pair->next;
	next->prev = pair->prev;

	pair->next->prev = prev;
	pair->prev->next = next;

	if (pair)
		pair->pair = nullptr;

	if (e->start_vert->edge_out == e)
		e->start_vert->edge_out = prev->next;

	if (pair->start_vert->edge_out == pair)
		pair->start_vert->edge_out = next;

	e->next = nullptr;
	e->prev = nullptr;
	e->pair = nullptr;
	e->start_vert = nullptr;
		
	pair->next = nullptr;
	pair->prev = nullptr;
	pair->pair = nullptr;
	pair->start_vert = nullptr;

	delete e;
	delete pair;
}

DTHalfEdge* DelaunayTriangulation2D::get_lower_tangent(DelaunayData *left, DelaunayData *right){

	DTHalfEdge *left_d = left->rightmost;
	DTHalfEdge *right_d = right->leftmost;

	DTHalfEdge *left_pair = left->rightmost->pair;
	DTHalfEdge *right_pair = right->leftmost->pair;

	POS_Line sl, sr;
	do {
		DTVertex *pl = left_pair->next->pair->start_vert;
		DTVertex *pr = right_pair->prev->pair->start_vert;

		if ((sl = point_seg_position(left_d->start_vert, right_d->start_vert, pl)) == POS_Line::ON_RIGHT) {
			left_pair = left_pair->next;
			left_d = left_pair->pair;
		}

		if ((sr = point_seg_position(left_d->start_vert, right_d->start_vert, pr)) == POS_Line::ON_RIGHT) {
			right_pair = right_pair->prev;
			right_d = right_pair->pair;
		}

	} while (sl == POS_Line::ON_RIGHT || sr == POS_Line::ON_RIGHT);


	/* create the 2 halfedges */
	DTHalfEdge *new_ld = new DTHalfEdge;
	DTHalfEdge *new_rd = new DTHalfEdge;

	/* setup new_gd and new_dd */
	new_ld->start_vert = left_d->start_vert;
	new_ld->pair = new_rd;
	new_ld->prev = left_d->pair;
	new_ld->next = right_d->pair->next;

	DTHalfEdge *temp = left_d->pair->next;
	left_d->pair->next = new_ld;
	right_d->pair->next->prev = new_ld;

	new_rd->start_vert = right_d->start_vert;
	new_rd->pair = new_ld;
	new_rd->prev = right_d->pair;
	new_rd->next = temp;
	right_d->pair->next = new_rd;
	temp->prev = new_rd;

	addHalfEdge(new_ld, new_ld->start_vert, new_rd->start_vert);
	addHalfEdge(new_rd, new_rd->start_vert, new_ld->start_vert);

	return new_ld;
}

void DelaunayTriangulation2D::init_tri(DelaunayData *del, int start) {
	DTVertex *f = m_vertexs[start];
	DTVertex *s = m_vertexs[start + 1];
	DTVertex *t = m_vertexs[start + 2];

	DTHalfEdge *d0 = new DTHalfEdge;
	DTHalfEdge *d1 = new DTHalfEdge;
	DTHalfEdge *d2 = new DTHalfEdge;
	DTHalfEdge *d3 = new DTHalfEdge;
	DTHalfEdge *d4 = new DTHalfEdge;
	DTHalfEdge *d5 = new DTHalfEdge;

	// whether this three points are cw or ccw
	if (point_seg_position(f, t, s) == POS_Line::ON_LEFT) {//cw
		d0->start_vert = f;
		d1->start_vert = t;
		d2->start_vert = s;

		d3->start_vert = t;
		d4->start_vert = s;
		d5->start_vert = f;

		f->edge_out = d0;
		t->edge_out = d1;
		s->edge_out = d2;

		/* set halfedges pair */
		d0->pair = d3;
		d3->pair = d0;

		d1->pair = d4;
		d4->pair = d1;

		d2->pair = d5;
		d5->pair = d2;

		/* next and next -1 setup */
		d0->next = d1;
		d0->prev = d2;

		d1->next = d2;
		d1->prev = d0;

		d2->next = d0;
		d2->prev = d1;

		d3->next = d5;
		d3->prev = d4;

		d4->next = d3;
		d4->prev = d5;

		d5->next = d4;
		d5->prev = d3;

		addHalfEdge(d0, f, t);
		addHalfEdge(d1, t, s);
		addHalfEdge(d2, s, f);
		addHalfEdge(d3, t, f);
		addHalfEdge(d4, s, t);
		addHalfEdge(d5, f, s);

		del->rightmost = d1;
		del->leftmost = d0;
	}
	else {//ccw

		d0->start_vert = f;
		d1->start_vert = s;
		d2->start_vert = t;

		d3->start_vert = s;
		d4->start_vert = t;
		d5->start_vert = f;

		f->edge_out = d0;
		s->edge_out = d1;
		t->edge_out = d2;

		/* set halfedges pair */
		d0->pair = d3;
		d3->pair = d0;

		d1->pair = d4;
		d4->pair = d1;

		d2->pair = d5;
		d5->pair = d2;

		/* next and next -1 setup */
		d0->next = d1;
		d0->prev = d2;

		d1->next = d2;
		d1->prev = d0;

		d2->next = d0;
		d2->prev = d1;

		d3->next = d5;
		d3->prev = d4;

		d4->next = d3;
		d4->prev = d5;

		d5->next = d4;
		d5->prev = d3;


		addHalfEdge(d0, f, s);
		addHalfEdge(d1, s, t);
		addHalfEdge(d2, t, f);
		addHalfEdge(d3, s, f);
		addHalfEdge(d4, t, s);
		addHalfEdge(d5, f, t);

		del->rightmost = d2;
		del->leftmost = d0;
	}
}

void DelaunayTriangulation2D::init_seg(DelaunayData *del, int start) {
	DTVertex *f = m_vertexs[start];
	DTVertex *s = m_vertexs[start + 1];

	DTHalfEdge *d0 = new DTHalfEdge;
	DTHalfEdge *d1 = new DTHalfEdge;

	d0->start_vert = f;
	d0->next = d0->prev = d1;

	d1->start_vert = s;
	d1->next = d1->prev = d0;
    
	d0->pair = d1;
	d1->pair = d0;

	f->edge_out = d0;
	s->edge_out = d1;

	del->rightmost = d1;
	del->leftmost = d0;

	addHalfEdge(d0, f, s);
	addHalfEdge(d1, s, f);
}

void DelaunayTriangulation2D::sortvertex(){
auto cmp = [](const DTVertex *a, const DTVertex *b)->int {
		if (a->x < b->x)
			return true;
		else if (a->x > b->x)
			return false;
		else if (a->y < b->y)
			return true;
		else if (a->y > b->y)
			return false;
		//assert(0 && "2 or more points share the same exact coordinate");
		return false; /* Should not be given! */
	};
	std::sort(m_vertexs.begin(), m_vertexs.end(), cmp);
	for (int i = 0; i < m_vertexs.size(); ++i)
	{
		m_vertexs[i]->id = i;
	}
}

DelaunayTriangulation2D::POS_Line  DelaunayTriangulation2D::point_halfedge_position(DTHalfEdge *d, DTVertex *pt) {

	DTVertex *s, *e;

	s = d->start_vert;
	e = d->pair->start_vert;

	return point_seg_position(s, e, pt);
}
DelaunayTriangulation2D::POS_Line  DelaunayTriangulation2D::point_seg_position(DTVertex *s, DTVertex *e, DTVertex *pt) {

	double		se_x, se_y, spt_x, spt_y;
	double		res;

	se_x = e->x - s->x;
	se_y = e->y - s->y;

	spt_x = pt->x - s->x;
	spt_y = pt->y - s->y;

	res = ((se_x * spt_y) - (se_y * spt_x));
	if (res < 0.0)
		return POS_Line::ON_RIGHT;
	else if (res > 0.0)
		return POS_Line::ON_LEFT;

	return POS_Line::ON_SEG;
}


DelaunayTriangulation2D::POS_Circle DelaunayTriangulation2D::in_circle(DTVertex *pt0, DTVertex *pt1, DTVertex *pt2, DTVertex *p) {

	// reduce the computational complexity by substracting the last row of the matrix
	// ref: https://www.cs.cmu.edu/~quake/robust.html
	double	p0p_x, p0p_y, p1p_x, p1p_y, p2p_x, p2p_y, p0p, p1p, p2p, res;
	double	m[3][3];

	p0p_x = pt0->x - p->x;
	p0p_y = pt0->y - p->y;

	p1p_x = pt1->x - p->x;
	p1p_y = pt1->y - p->y;

	p2p_x = pt2->x - p->x;
	p2p_y = pt2->y - p->y;

	p0p = p0p_x * p0p_x + p0p_y * p0p_y;
	p1p = p1p_x * p1p_x + p1p_y * p1p_y;
	p2p = p2p_x * p2p_x + p2p_y * p2p_y;

	m[0][0] = p0p_x;
	m[0][1] = p0p_y;
	m[0][2] = p0p;

	m[1][0] = p1p_x;
	m[1][1] = p1p_y;
	m[1][2] = p1p;

	m[2][0] = p2p_x;
	m[2][1] = p2p_y;
	m[2][2] = p2p;

	res = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
		- m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
		+ m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

	res = -res;

	if (res < 0.0)
		return POS_Circle::INSIDE;
	else if (res > 0.0)
		return POS_Circle::OUTSIDE;

	return POS_Circle::ON_CIRCLE;
}