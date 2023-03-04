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

struct Compare {
	Point p0; // Leftmost point of the poly
	bool operator ()(const Point &p1, const Point &p2) {
		//return true if p1 is smaller angle than p2
		double angle1 = atan( (p1.imag()-p0.imag()) / (p1.real()-p0.real()) );
		double angle2 = atan( (p2.imag()-p0.imag()) / (p2.real()-p0.real()) );

		if(isnan(angle1))
			return true;
		else if(isnan(angle2))
			return false;

		if( (angle1>0) && (angle2<0))
			return true;
		else if( (angle1<0) && (angle2>0))
			return false;

		return angle1<angle2;
	}
};

bool inline salientAngle(Point &a, Point &b, Point &c) {
	double ab_real = b.real() - a.real();
	double ab_imag = b.imag() - a.imag();

	double bc_real = c.real() - b.real();
	double bc_imag = c.imag() - b.imag();

	Point u = Point(ab_real, ab_imag);
	Point v = Point(bc_real, bc_imag);


	double result = det(u, v);
	
	if(result > 0)
	{
		return true;	//left hand turn
	}
	return false;		//right hand turn
	
}

////////////////////////////////////////////////////////////////////////////////

Point lowest_point(std::vector<Point> &points)
{
	Point lowest = points[0];
	for( Point p : points)
	{
		if(p.imag() < lowest.imag())
		{
			lowest = p;
		}
	}
	
	return lowest;
}

Polygon convex_hull(std::vector<Point> &points) {
	Compare order;

	order.p0 = lowest_point(points);
	std::sort(points.begin(), points.end(), order);
	Polygon hull;


	// use salientAngle(a, b, c) here
	hull.push_back(points[0]);
	hull.push_back(points[1]);

	int ind = 2;
	Point cur = points[ind];

	while(ind < points.size())
	{
		Point last = hull.back();
		hull.pop_back();
		bool result = salientAngle(hull.back(), last, cur);

		if(result)	//left turn, add cur point to hull
		{
			hull.push_back(last);
			hull.push_back(cur);
			ind++;
			cur = points[ind];
		}
		
	}
	return hull;
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

void save_obj(const std::string &filename, Polygon &poly) {
	std::ofstream out(filename);
	if (!out.is_open()) {
		throw std::runtime_error("failed to open file " + filename);
	}
	out << std::fixed;
	for (const auto &v : poly) {
		out << "v " << v.real() << ' ' << v.imag() << " 0\n";
	}
	for (size_t i = 0; i < poly.size(); ++i) {
		out << "l " << i+1 << ' ' << 1+(i+1)%poly.size() << "\n";
	}
	out << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[]) {
	if (argc <= 2) {
		std::cerr << "Usage: " << argv[0] << " points.xyz output.obj" << std::endl;
		return -1;
	}
	std::vector<Point> points = load_xyz(argv[1]);
	Polygon hull = convex_hull(points);
	save_obj(argv[2], hull);
	return 0;
}
