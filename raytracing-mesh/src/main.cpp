////////////////////////////////////////////////////////////////////////////////
// C++ include
#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <fstream>
#include <algorithm>
#include <numeric>
#include <queue>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

////////////////////////////////////////////////////////////////////////////////
// Class to store tree
////////////////////////////////////////////////////////////////////////////////
class AABBTree
{
public:
    class Node
    {
    public:
        AlignedBox3d bbox;
        int parent;   // Index of the parent node (-1 for root)
        int left;     // Index of the left child (-1 for a leaf)
        int right;    // Index of the right child (-1 for a leaf)
        int triangle; // Index of the node triangle (-1 for internal nodes)
    };

    std::vector<Node> nodes;
    int root;

    AABBTree() = default;                           // Default empty constructor
    AABBTree(const MatrixXd &V, const MatrixXi &F); // Build a BVH from an existing mesh
};

////////////////////////////////////////////////////////////////////////////////
// Scene setup, global variables
////////////////////////////////////////////////////////////////////////////////
const std::string data_dir = DATA_DIR;
const std::string filename("raytrace.png");
const std::string mesh_filename(data_dir + "dodeca.off");

//Camera settings
const double focal_length = 2;
const double field_of_view = 0.7854; //45 degrees
const bool is_perspective = true;
const Vector3d camera_position(0, 0, 2);

// Triangle Mesh
MatrixXd vertices; // n x 3 matrix (n points)
MatrixXi facets;   // m x 3 matrix (m triangles)
AABBTree bvh;

//Material for the object, same material for all objects
const Vector4d obj_ambient_color(0.0, 0.5, 0.0, 0);
const Vector4d obj_diffuse_color(0.5, 0.5, 0.5, 0);
const Vector4d obj_specular_color(0.2, 0.2, 0.2, 0);
const double obj_specular_exponent = 256.0;
const Vector4d obj_reflection_color(0.7, 0.7, 0.7, 0);

// Precomputed (or otherwise) gradient vectors at each grid node
const int grid_size = 20;
std::vector<std::vector<Vector2d>> grid;

//Lights
std::vector<Vector3d> light_positions;
std::vector<Vector4d> light_colors;
//Ambient light
const Vector4d ambient_light(0.2, 0.2, 0.2, 0);

//Fills the different arrays
void setup_scene()
{
    //Loads file
    std::ifstream in(mesh_filename);
    std::string token;
    in >> token;
    int nv, nf, ne;
    in >> nv >> nf >> ne;
    vertices.resize(nv, 3);
    facets.resize(nf, 3);
    for (int i = 0; i < nv; ++i)
    {
        in >> vertices(i, 0) >> vertices(i, 1) >> vertices(i, 2);
    }
    for (int i = 0; i < nf; ++i)
    {
        int s;
        in >> s >> facets(i, 0) >> facets(i, 1) >> facets(i, 2);
        assert(s == 3);
    }

    //setup tree
    bvh = AABBTree(vertices, facets);

    //Lights
    light_positions.emplace_back(8, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(6, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(0, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-2, -8, 0);
    light_colors.emplace_back(16, 16, 16, 0);

    light_positions.emplace_back(-4, 8, 0);
    light_colors.emplace_back(16, 16, 16, 0);
}

////////////////////////////////////////////////////////////////////////////////
// BVH Code
////////////////////////////////////////////////////////////////////////////////

AlignedBox3d bbox_from_triangle(const Vector3d &a, const Vector3d &b, const Vector3d &c)
{
    AlignedBox3d box;
    box.extend(a);
    box.extend(b);
    box.extend(c);
    return box;
}

int AABB_helper(const MatrixXd &V, const MatrixXi &F, AABBTree* tree, std::vector<int> SS, MatrixXd centroids)
{
    if(SS.size() == 1)
    {
        //leaf
        int cur_ind = SS[0];
        AABBTree::Node new_node;
        new_node.bbox = bbox_from_triangle(V.row(F(cur_ind, 0)), V.row(F(cur_ind, 1)), V.row(F(cur_ind, 2)));
        new_node.left = -1;
        new_node.right = -1;
        new_node.triangle = cur_ind; // Index of the node triangle (-1 for internal nodes)
        tree->nodes.push_back(new_node);
        return tree->nodes.size()-1;
    }

    // Split each set of primitives into 2 sets of roughly equal size,
    // based on sorting the centroids along one direction or another.

    int ind = 0;
    double distance = 0;

    MatrixXd cur_centroids(SS.size(), centroids.cols());
    for(int i=0; i<SS.size(); i++)
    {
        cur_centroids.row(i) = centroids.row(SS[i]);
    }

    for( int i=0; i<3; i++)
    {
        double max = cur_centroids.col(i).maxCoeff();
        double min = cur_centroids.col(i).minCoeff();

        double cur_dist = max-min;
        if(cur_dist>distance)
        {
            distance = cur_dist;
            ind = i;
        }
    }

    std::sort(SS.begin(), SS.end(), [&centroids, ind](int e1, int e2){ return centroids.col(ind)(e1)<centroids.col(ind)(e1); });

    std::vector<int> S1;
    std::vector<int> S2;
    for(int i = 0; i < ceil(SS.size()/2); i++)
    {
        S1.push_back(SS[i]);
    }

    for(int i = ceil(SS.size()/2); i < SS.size(); i++)
    {
        S2.push_back(SS[i]);
    }

    int ind1 = AABB_helper(V, F, tree, S1, centroids);
    int ind2 = AABB_helper(V, F, tree, S2, centroids);

    AABBTree::Node new_node;
    AlignedBox3d new_box;
    new_box.extend(tree->nodes[ind1].bbox);
    new_box.extend(tree->nodes[ind2].bbox);
    new_node.bbox = new_box;
    new_node.left = ind1;
    new_node.right = ind2;
    new_node.triangle = -1;
    tree->nodes.push_back(new_node);

    tree->nodes[ind1].parent = tree->nodes.size()-1;
    tree->nodes[ind2].parent = tree->nodes.size()-1;

    return tree->nodes.size()-1;
}

AABBTree::AABBTree(const MatrixXd &V, const MatrixXi &F)
{
    // Compute the centroids of all the triangles in the input mesh
     MatrixXd centroids(F.rows(), V.cols());
    centroids.setZero();
    for (int i = 0; i < F.rows(); ++i)
    {
        for (int k = 0; k < F.cols(); ++k)
        {
            centroids.row(i) += V.row(F(i, k));
        }
        centroids.row(i) /= F.cols();
    }

    std::vector<int> indices;
    for(int i = 0; i < F.rows(); i++)
        indices.push_back(i);

    int root_ind = AABB_helper(V, F, this, indices, centroids);

    this->root = root_ind;
}

////////////////////////////////////////////////////////////////////////////////
// Intersection code
////////////////////////////////////////////////////////////////////////////////

double ray_triangle_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const Vector3d &a, const Vector3d &b, const Vector3d &c, Vector3d &p, Vector3d &N)
{
    // TODO
    // Compute whether the ray intersects the given triangle.
    // If you have done the parallelogram case, this should be very similar to it.

    const Vector3d pgram_origin = a;
    const Vector3d pgram_u = b - a;
    const Vector3d pgram_v = c - a;

    Matrix3d AA;
    AA.col(0) = pgram_u;
    AA.col(1) = pgram_v;
    AA.col(2) = -ray_direction;

    Vector3d bb = ray_origin - pgram_origin;//ray_direction - pgram_u;
    Matrix3d A_inv = AA.inverse();
    Vector3d x = A_inv*bb;
    const double u = x[0], v = x[1], t = x[2];

    if ((u > 1) || ((u+v)>1) || (t<0) || (u<0)  || (v<0))
    {
        return -1;
    }

    p = ray_origin + (t*ray_direction);
    N = (pgram_v).cross(pgram_u).normalized();;

    return t;
}

bool ray_box_intersection(const Vector3d &ray_origin, const Vector3d &ray_direction, const AlignedBox3d &box)
{
    // TODO
    // Compute whether the ray intersects the given box.
    // we are not testing with the real surface here anyway.
    double tmin = std::numeric_limits<double>::min(), tmax = std::numeric_limits<double>::max();

    double tx1 = (box.min()[0]-ray_origin(0))/ray_direction(0);
    double tx2 = (box.max()[0]-ray_origin(0))/ray_direction(0);

    tmin = std::max(tmin, std::min(tx1, tx2));
    tmax = std::min(tmax, std::max(tx1, tx2));

    double ty1 = (box.min()[1]-ray_origin(1))/ray_direction(1);
    double ty2 = (box.max()[1]-ray_origin(1))/ray_direction(1);

    tmin = std::max(tmin, std::min(ty1, ty2));
    tmax = std::min(tmax, std::max(ty1, ty2));

    double tz1 = (box.min()[2]-ray_origin(2))/ray_direction(2);
    double tz2 = (box.max()[2]-ray_origin(2))/ray_direction(2);

    tmin = std::max(tmin, std::min(tz1, tz2));
    tmax = std::min(tmax, std::max(tz1, tz2));

    return (tmax >= tmin);
}

//Finds the closest intersecting object returns its index
//In case of intersection it writes into p and N (intersection point and normals)
bool find_nearest_object(const Vector3d &ray_origin, const Vector3d &ray_direction, Vector3d &p, Vector3d &N)
{
    Vector3d tmp_p, tmp_N;
    bool method1 = false;

    // TODO
    // Method (1): Traverse every triangle and return the closest hit.
    // Method (2): Traverse the BVH tree and test the intersection with a
    // triangles at the leaf nodes that intersects the input ray.

    // Method (1)
    if(method1)
    {
        double closest_t = std::numeric_limits<double>::max();
        bool intersects = false;

        for( int i=0; i<facets.rows(); i++ )
        {
            const Vector3d a = vertices.row(facets(i, 0));
            const Vector3d b = vertices.row(facets(i, 1));
            const Vector3d c = vertices.row(facets(i, 2));
            const double t = ray_triangle_intersection(ray_origin, ray_direction, a, b, c, tmp_p, tmp_N);

            if( (t>=0) && (t<closest_t) )
            {
                closest_t = t;
                p = tmp_p;
                N = tmp_N;
                intersects = true;
            }
        }

        return intersects;
    }
    else    
    // Method (2)
    {
        double closest_t = std::numeric_limits<double>::max();
        bool intersects = false;

        std::queue<int> queue;
        queue.push(bvh.root);

        while(!queue.empty())
        {
            int cur_ind = queue.front();
            queue.pop();

            AABBTree::Node cur_node = bvh.nodes[cur_ind];
            if(cur_node.triangle == -1)
            {
                bool result = ray_box_intersection(ray_origin, ray_direction, cur_node.bbox);
                if(result)
                {
                    queue.push(cur_node.left);
                    queue.push(cur_node.right);
                }
            }
            else
            {
                const Vector3d a = vertices.row(facets(cur_node.triangle, 0));
                const Vector3d b = vertices.row(facets(cur_node.triangle, 1));
                const Vector3d c = vertices.row(facets(cur_node.triangle, 2));
                const double t = ray_triangle_intersection(ray_origin, ray_direction, a, b, c, tmp_p, tmp_N);
                if( (t>=0) && (t<closest_t) )
                {
                    closest_t = t;
                    p = tmp_p;
                    N = tmp_N;
                    intersects = true;
                }
            } 
        }
        return intersects;
    }
}

////////////////////////////////////////////////////////////////////////////////
// Raytracer code
////////////////////////////////////////////////////////////////////////////////

Vector4d shoot_ray(const Vector3d &ray_origin, const Vector3d &ray_direction)
{
    //Intersection point and normal, these are output of find_nearest_object
    Vector3d p, N;

    const bool nearest_object = find_nearest_object(ray_origin, ray_direction, p, N);

    if (!nearest_object)
    {
        // Return a transparent color
        return Vector4d(0, 0, 0, 0);
    }

    // Ambient light contribution
    const Vector4d ambient_color = obj_ambient_color.array() * ambient_light.array();

    // Punctual lights contribution (direct lighting)
    Vector4d lights_color(0, 0, 0, 0);
    for (int i = 0; i < light_positions.size(); ++i)
    {
        const Vector3d &light_position = light_positions[i];
        const Vector4d &light_color = light_colors[i];

        Vector4d diff_color = obj_diffuse_color;

        // Diffuse contribution
        const Vector3d Li = (light_position - p).normalized();
        const Vector4d diffuse = diff_color * std::max(Li.dot(N), 0.0);

        // Specular contribution
        const Vector3d Hi = (Li - ray_direction).normalized();
        const Vector4d specular = obj_specular_color * std::pow(std::max(N.dot(Hi), 0.0), obj_specular_exponent);
        // Vector3d specular(0, 0, 0);

        // Attenuate lights according to the squared distance to the lights
        const Vector3d D = light_position - p;
        lights_color += (diffuse + specular).cwiseProduct(light_color) / D.squaredNorm();
    }

    // Rendering equation
    Vector4d C = ambient_color + lights_color;

    //Set alpha to 1
    C(3) = 1;

    return C;
}

////////////////////////////////////////////////////////////////////////////////

void raytrace_scene()
{
    std::cout << "Simple ray tracer." << std::endl;

    int w = 640;
    int h = 480;
    MatrixXd R = MatrixXd::Zero(w, h);
    MatrixXd G = MatrixXd::Zero(w, h);
    MatrixXd B = MatrixXd::Zero(w, h);
    MatrixXd A = MatrixXd::Zero(w, h); // Store the alpha mask

    // The camera always points in the direction -z
    // The sensor grid is at a distance 'focal_length' from the camera center,
    // and covers an viewing angle given by 'field_of_view'.
    double aspect_ratio = double(w) / double(h);
    //TODO
    double image_y = sin(field_of_view/2)*focal_length/cos(field_of_view/2);
    double image_x = image_y*aspect_ratio;

    // The pixel grid through which we shoot rays is at a distance 'focal_length'
    const Vector3d image_origin(-image_x, image_y, camera_position[2] - focal_length);
    const Vector3d x_displacement(2.0 / w * image_x, 0, 0);
    const Vector3d y_displacement(0, -2.0 / h * image_y, 0);

    for (unsigned i = 0; i < w; ++i)
    {
        for (unsigned j = 0; j < h; ++j)
        {
            const Vector3d pixel_center = image_origin + (i + 0.5) * x_displacement + (j + 0.5) * y_displacement;

            // Prepare the ray
            Vector3d ray_origin;
            Vector3d ray_direction;

            if (is_perspective)
            {
                // Perspective camera
                ray_origin = camera_position;
                ray_direction = (pixel_center - camera_position).normalized();
            }
            else
            {
                // Orthographic camera
                ray_origin = pixel_center;
                ray_direction = Vector3d(0, 0, -1);
            }

            const Vector4d C = shoot_ray(ray_origin, ray_direction);
            R(i, j) = C(0);
            G(i, j) = C(1);
            B(i, j) = C(2);
            A(i, j) = C(3);
        }
    }

    // Save to png
    write_matrix_to_png(R, G, B, A, filename);
}

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{
    setup_scene();

    raytrace_scene();
    return 0;
}
