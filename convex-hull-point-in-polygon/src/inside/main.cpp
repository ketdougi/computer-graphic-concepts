////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>
////////////////////////////////////////////////////////////////////////////////

typedef std::complex<double> Point;
typedef std::vector<Point> Polygon;

double inline det(const Point &u, const Point &v) {
	return (u.real() * v.imag()) - (v.real() * u.imag());
}

// Return true iff [a,b] intersects [c,d], and store the intersection in ans
bool intersect_segment(const Point &a, const Point &b, const Point &c, const Point &d)//, Point &ans) 
{
	double acbc = det( 	Point( a.real()-c.real(), a.imag()-c.imag() ), 
						Point( b.real()-c.real(), b.imag()-c.imag() ));
	double adbd = det( 	Point( a.real()-d.real(), a.imag()-d.imag() ), 
						Point( b.real()-d.real(), b.imag()-d.imag() ));
	double cada = det( 	Point( c.real()-a.real(), c.imag()-a.imag() ), 
						Point( d.real()-a.real(), d.imag()-a.imag() ));
	double cbdb = det( 	Point( c.real()-b.real(), c.imag()-b.imag() ), 
						Point( d.real()-b.real(), d.imag()-b.imag() ));

	//std::cout << b << " " << c << " " << d << "\t" <<  (acbc>0) << " " << (adbd>0) << " " << (cada>0) << " " << (cbdb>0) << std::endl; 

	//one of acbc & abdb must be positive and one must be negative
	if(((acbc > 0) && (adbd > 0)) || ((acbc < 0) && (adbd < 0)))
	{
		return false;
	}
	//one of cada & cbdb must be positive and one must be negative
	else if(((cada > 0) && (cbdb > 0)) || ((cada < 0) && (cbdb < 0)))
	{
		return false;
	}
	return true;
}

////////////////////////////////////////////////////////////////////////////////

bool is_inside(const Polygon &poly, const Point &query) {
	// 1. Compute bounding box and set coordinate of a point outside the polygon
	Point outside(0, 0);

	// 2. Cast a ray from the query point to the 'outside' point, count number of intersections
	bool first = true;
	Point prev;
	int count = 0;

	for(Point p : poly)
	{
		if(first)
		{
			first = false;
			prev = p;
			continue;
		}

		if(intersect_segment(outside, query, prev, p))
		{
			count++;
		}
		prev = p;
	}

	//check line between last point and first point
	if(intersect_segment(outside, query, prev, poly[0]))
	{
		count++;
	}

	//if intersects even number of times, point is not in polygon
	if( (count%2) == 0 )
	{
		return false;
	}
	return true;
}

////////////////////////////////////////////////////////////////////////////////

std::vector<Point> load_xyz(const std::string &filename) {
	std::vector<Point> points;
	std::ifstream in(filename);
	
	int length;
	std::string buff;

	in >> length;

	for(int i=0; i<length; i++)
	{
		if(!in.eof())
		{
			double real;
			double imag;
			in >> real >> imag;
			in >> buff;
			Point toadd = Point(real, imag);
			points.push_back(toadd);
		}
	}
	
	return points;
}

Polygon load_obj(const std::string &filename) {
	Polygon p;
	std::ifstream in(filename);
	std::string buff;
	
	while(!in.eof())
	{
		in >> buff;
		if(buff == "v")
		{
			double real;
			double imag;
			in >> real >> imag;
			in >> buff;
			Point toadd = Point(real, imag);
			p.push_back(toadd);
		}
	}

	return p;
}

void save_xyz(const std::string &filename, const std::vector<Point> &points) {
	std::ofstream out(filename);
	if (!out.is_open()) {
		throw std::runtime_error("failed to open file " + filename);
		return;
	}
	out << std::fixed;

	// save length
	out << points.size() << "\n";

	// save coordinates
	for( Point p : points)
	{
		out << p.real() << " " << p.imag() << " " << 0 << "\n";
	}

	out << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[]) {
	if (argc <= 3) {
		std::cerr << "Usage: " << argv[0] << " points.xyz poly.obj result.xyz" << std::endl;
		return -1;
	}
	std::vector<Point> points = load_xyz(argv[1]);
	Polygon poly = load_obj(argv[2]);
	std::vector<Point> result;
	for (size_t i = 0; i < points.size(); ++i) {
		if (is_inside(poly, points[i])) {
			result.push_back(points[i]);
		}
	}
	save_xyz(argv[3], result);
	return 0;
}
