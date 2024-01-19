////////////////////////////////////////////////////////////////////////////////
#include <algorithm>
#include <complex>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>


#include <Eigen/Dense>
// Shortcut to avoid  everywhere, DO NOT USE IN .h
using namespace Eigen;
////////////////////////////////////////////////////////////////////////////////

const std::string root_path = DATA_DIR;

// Computes the determinant of the matrix whose columns are the vector u and v
double inline det(const Vector2d &u, const Vector2d &v)
{
    // TODO
    return 0;
}

// Helper function to compute the orientation of ordered triplet (p, q, r).
int orientation(const Eigen::Vector2d &p, const Eigen::Vector2d &q, const Eigen::Vector2d &r) {
    // Convert 2D points to 3D by adding a zero z-component
    Eigen::Vector3d p3(p.x(), p.y(), 0.0);
    Eigen::Vector3d q3(q.x(), q.y(), 0.0);
    Eigen::Vector3d r3(r.x(), r.y(), 0.0);

    // Compute the cross product of vectors (q - p) and (r - q)
    Eigen::Vector3d pq = q3 - p3;
    Eigen::Vector3d qr = r3 - q3;
    Eigen::Vector3d cross_product = pq.cross(qr);

    // The z-component of the cross product gives the orientation
    double val = cross_product.z();

    if (val == 0) return 0;  // collinear
    if (val > 0) return 1;   // clockwise
    return 2;                // counterclockwise
}


// Given three collinear points p, q, r, the function checks if
// point r lies on line segment 'pq'
bool on_segment(const Vector2d &p, const Vector2d &q, const Vector2d &r) {
    return ((((r.x() >= p.x() && r.x() <= q.x()) || (r.x() <= p.x() && r.x() >= q.x())) &&
        ((r.y() >= p.y() && r.y() <= q.y()) || (r.y() <= p.y() && r.y() >= q.y()))));
}

// Return true iff [a,b] intersects [c,d]
bool intersect_segment(const Vector2d &a, const Vector2d &b, const Vector2d &c, const Vector2d &d)
{
    int o1 = orientation(a, b, c);
    int o2 = orientation(a, b, d);
    int o3 = orientation(c, d, a);
    int o4 = orientation(c, d, b);

    if (o1 != o2 && o3 != o4) return true;

    // if a,b,c are collinear and c lies on segment ab
    if (o1 == 0 && on_segment(a, b, c)) return true;

    // if a,b,d are collinear and d lies on segment ab
    if (o2 == 0 && on_segment(a, b, d)) return true;

    // if c,d,a are collinear and a lies on segment cd
    if (o3 == 0 && on_segment(c, d, a)) return true;

    // if c,d,b are collinear and b lies on segment cd
    if (o4 == 0 && on_segment(c, d, b)) return true;

    return false;
}

////////////////////////////////////////////////////////////////////////////////

bool is_inside(const std::vector<Vector2d> &poly, const Vector2d &query)
{
    // 1. Compute bounding box and set coordinate of a point outside the polygon
    double min_x = poly[0].x();
    double max_x = poly[0].x();
    double min_y = poly[0].y();
    double max_y = poly[0].y();
    for (const auto p: poly) {
        if (p.x() < min_x) min_x = p.x();
        if (p.x() > max_x) max_x = p.x();
        if (p.y() < min_y) min_y = p.y();
        if (p.y() > max_y) max_y = p.y();
    }
    Vector2d outside(max_x +1, query.y());

    int intersections = 0;
    for (int i = 0; i<poly.size() ; i++) {
        Vector2d a = poly[i];
        Vector2d b = poly[(i+1)% poly.size()]; // wrap around
        if (intersect_segment(a, b, query, outside)) {
            if (orientation(a, b, query) == 0) {
                if (on_segment(a, b, query)) {
                    return true;
                }
            }
            intersections++;
        }
    }
    // 2. Cast a ray from the query point to the 'outside' point, count number of intersections
    // TODO
     return (intersections % 2 == 1);
}

////////////////////////////////////////////////////////////////////////////////

std::vector<Vector2d> load_xyz(const std::string &filename)
{
    std::vector<Vector2d> points;
    std::ifstream in(filename);
    if (!in)
    {
        std::cerr << "Cannot open file: " << filename << std::endl;
        return points;
    }

    int num_points;
    in >> num_points; // Read the number of points

    double x, y, z;
    for (int i = 0; i < num_points; ++i)
    {
        in >> x >> y >> z; // Read each point's coordinates
        points.emplace_back(x, y); // Ignore the z coordinate as per the instructions
    }
    return points;
}

void save_xyz(const std::string &filename, const std::vector<Vector2d> &points)
{
    std::ofstream out(filename);
    if (!out)
    {
        std::cerr << "Cannot open file for writing: " << filename << std::endl;
        return;
    }

    // Write the number of points
    out << points.size() << std::endl;

    // Write each point's coordinates
    for (const auto &point : points)
    {
        out << point.x() << " " << point.y() << " 0" << std::endl;
    }
}

std::vector<Vector2d> load_obj(const std::string &filename)
{
    std::ifstream in(filename);
    std::vector<Vector2d> points;
    std::vector<Vector2d> poly;
    char key;
    while (in >> key)
    {
        if (key == 'v')
        {
            double x, y, z;
            in >> x >> y >> z;
            points.push_back(Vector2d(x, y));
        }
        else if (key == 'f')
        {
            std::string line;
            std::getline(in, line);
            std::istringstream ss(line);
            int id;
            while (ss >> id)
            {
                poly.push_back(points[id - 1]);
            }
        }
    }
    return poly;
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    const std::string points_path = root_path + "/points.xyz";
    const std::string poly_path = root_path + "/polygon.obj";

    std::vector<Vector2d> points = load_xyz(points_path);

    ////////////////////////////////////////////////////////////////////////////////
    //Point in polygon
    std::vector<Vector2d> poly = load_obj(poly_path);
    std::vector<Vector2d> result;
    int in = 0;
    for (size_t i = 0; i < points.size(); ++i)
    {   
        
        if (is_inside(poly, points[i]))
        {
            in = in +1;
            // std::cout << in << std::endl;
            result.push_back(points[i]);
        }
    }
    save_xyz("output.xyz", result);

    return 0;
}
