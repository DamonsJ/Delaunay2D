#include <iostream>
#include <time.h>
#include <assert.h>
#include "..\include\delaunay.h"
#include "..\include\DelaunayTriangulation.h"

void testdelaunay() {
	del_point2d_t	points_[10];
	points_[0].x = 1; points_[0].y = 3;
	points_[1].x = 2; points_[1].y = 1;
	points_[2].x = 3; points_[2].y = 2;
	points_[3].x = 2; points_[3].y = 4;
	points_[4].x = 2; points_[4].y = 6;
	points_[5].x = 5; points_[5].y = 5;
	points_[6].x = 4; points_[6].y = 7;
	points_[7].x = 6; points_[7].y = 1;
	points_[8].x = 7; points_[8].y = 2;
	points_[9].x = 7; points_[9].y = 7;


	delaunay2d_from(points_,10);
}

void testselfdelaunay() {
	DelaunayTriangulation2D *d2d = new DelaunayTriangulation2D;
	std::vector<DTVertex *> points;
	points.push_back(new DTVertex(1, 3));
	points.push_back(new DTVertex(2, 1));
	points.push_back(new DTVertex(3, 2));
	points.push_back(new DTVertex(2, 4));
	points.push_back(new DTVertex(2, 6));
	points.push_back(new DTVertex(5, 5));
	points.push_back(new DTVertex(4, 7));
	points.push_back(new DTVertex(6, 1));
	points.push_back(new DTVertex(7, 2));
	points.push_back(new DTVertex(7, 7));

	clock_t t1 = clock();
	d2d->AddPoints(points);
	d2d->SortPoints();
	d2d->Triangulate();
	clock_t t2 = clock();
	std::cout << "eclipse time : " << t2 - t1 << std::endl;
}
int main() {
	testselfdelaunay();
	
	system("pause");
	return 0;
}