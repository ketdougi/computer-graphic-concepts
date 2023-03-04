// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "utils.h"

// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

// Shortcut to avoid Eigen:: everywhere, DO NOT USE IN .h
using namespace Eigen;

//determinate of matrix = [a c]
//                        [b d]
double det(double a, double b, double c, double d)
{
    return (a*d) - (b*c);
}

//cramer's rule
double cramers(MatrixXd m)
{
    return    m.coeff(0, 0) * det(m.coeff(1, 1), m.coeff(2, 1), m.coeff(1, 2), m.coeff(2, 2))
            + m.coeff(1, 0) * det(m.coeff(0, 2), m.coeff(2, 2), m.coeff(0, 1), m.coeff(2, 1))
            + m.coeff(2, 0) * det(m.coeff(0, 1), m.coeff(1, 1), m.coeff(0, 2), m.coeff(1, 2));
}

//returns array [B, l, t]
double* parallelogram_intersect(const Vector3d pgram_u, const Vector3d pgram_v, const Vector3d ray_direction,
                                    const Vector3d pgram_origin, const Vector3d ray_origin)
{
    //e + td = a + B(b-a) + l(c-a)
    //ray_origin + (t*ray_direction) = pgram_origin + (B*pgram_u) + (l*pgram_v)
    //x = [t u v]

    Vector3d u = -1*pgram_u;
    Vector3d v = -1*pgram_v;
    Vector3d dir = -1*ray_direction;

    MatrixXd m(3, 3);
    m.col(0) = u;
    m.col(1) = v;
    m.col(2) = dir;

    double M = cramers(m);

    Vector3d e = pgram_origin - ray_origin;

    //B
    m.col(0) = e;
    m.col(1) = v;
    m.col(2) = dir;
    double B = cramers(m)/M;

    //gamma
    m.col(0) = dir;
    m.col(1) = u;
    m.col(2) = e;
    double l = cramers(m)/M;

    //t
    m.col(0) = v;
    m.col(1) = u;
    m.col(2) = e;
    double t = cramers(m)/M;

    double* toreturn = new double[3];
    toreturn[0] = B;
    toreturn[1] = l;
    toreturn[2] = t;

    return toreturn;
}

void raytrace_sphere()
{
    std::cout << "Simple ray tracer, one sphere with orthographic projection" << std::endl;

    const std::string filename("sphere_orthographic.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is orthographic, pointing in the direction -z and covering the
    // unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray
            const Vector3d ray_origin = pixel_center;
            const Vector3d ray_direction = camera_view_direction;

            // Intersect with the sphere
            // NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis
            Vector2d ray_on_xy(ray_origin(0), ray_origin(1));
            const double sphere_radius = 0.9;

            if (ray_on_xy.norm() < sphere_radius)
            {
                // The ray hit the sphere, compute the exact intersection point
                Vector3d ray_intersection(
                    ray_on_xy(0), ray_on_xy(1),
                    sqrt(sphere_radius * sphere_radius - ray_on_xy.squaredNorm()));

                // Compute normal at the intersection point
                Vector3d ray_normal = ray_intersection.normalized();

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_parallelogram()
{
    std::cout << "Simple ray tracer, one parallelogram with orthographic projection" << std::endl;

    const std::string filename("plane_orthographic.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is orthographic, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
    const Vector3d pgram_origin(-0.5, -0.5, 0);
    const Vector3d pgram_u(0, 0.7, -10);
    const Vector3d pgram_v(1, 0.4, 0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray
            const Vector3d ray_origin = pixel_center;
            const Vector3d ray_direction = camera_view_direction;

            // TODO: Check if the ray intersects with the parallelogram
            double* result = parallelogram_intersect(pgram_u, pgram_v, ray_direction, pgram_origin, ray_origin);

            double B = result[0]; 
            double l = result[1];
            double t = result[2];

            //ray intersects parallelogram if 0 < B,l < 1
            if ((B < 1) && (l<1) && (t>0) && (l>0)  && (B>0))
            {
                // TODO: The ray hit the parallelogram, compute the exact intersection
                // point
                
                Vector3d ray_intersection = ray_origin + (t*ray_direction);     //evaluate ray equation at calculated t

                // TODO: Compute normal at the intersection point
                //Vector3d ray_normal = ray_intersection.normalized();
                Vector3d ray_normal = (ray_intersection+pgram_v).cross(ray_intersection+pgram_u).normalized(); //normal of parallelogram

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_perspective()
{
    std::cout << "Simple ray tracer, one parallelogram with perspective projection" << std::endl;

    const std::string filename("plane_perspective.png");
    MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / C.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / C.rows(), 0);

    // TODO: Parameters of the parallelogram (position of the lower-left corner + two sides)
    const Vector3d pgram_origin(-0.5, -0.5, 0);
    const Vector3d pgram_u(0, 0.7, -10);
    const Vector3d pgram_v(1, 0.4, 0);

    // Single light source
    const Vector3d light_position(-1, 1, 1);

    for (unsigned i = 0; i < C.cols(); ++i)
    {
        for (unsigned j = 0; j < C.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // TODO: Prepare the ray (origin point and direction)
            // for perspective, ray_origin is at camera and ray_direction is direction from camera to pixel
            const Vector3d ray_origin = camera_origin;
            const Vector3d ray_direction = (pixel_center - camera_origin).normalized();

            // TODO: Check if the ray intersects with the parallelogram
            double* result = parallelogram_intersect(pgram_u, pgram_v, ray_direction, pgram_origin, ray_origin);

            double B = result[0]; 
            double l = result[1];
            double t = result[2];

            if ((B < 1) && (l<1) && (t>0) && (l>0)  && (B>0))
            {
                // TODO: The ray hit the parallelogram, compute the exact intersection
                // point
                Vector3d ray_intersection = ray_origin + (t*ray_direction);

                // TODO: Compute normal at the intersection point
                //Vector3d ray_normal = ray_intersection.normalized();
                Vector3d ray_normal = (ray_intersection+pgram_v).cross(ray_intersection+pgram_u).normalized();

                // Simple diffuse model
                C(i, j) = (light_position - ray_intersection).normalized().transpose() * ray_normal;

                // Clamp to zero
                C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(C, C, C, A, filename);
}

void raytrace_shading()
{
    std::cout << "Simple ray tracer, one sphere with different shading" << std::endl;

    const std::string filename("shading.png");
    //MatrixXd C = MatrixXd::Zero(800, 800); // Store the color
    MatrixXd Red = MatrixXd::Zero(800, 800); // Store red
    MatrixXd Green = MatrixXd::Zero(800, 800); // Store green
    MatrixXd Blue = MatrixXd::Zero(800, 800); // Store blue
    MatrixXd A = MatrixXd::Zero(800, 800); // Store the alpha mask

    const Vector3d camera_origin(0, 0, 3);
    const Vector3d camera_view_direction(0, 0, -1);

    // The camera is perspective, pointing in the direction -z and covering the unit square (-1,1) in x and y
    const Vector3d image_origin(-1, 1, 1);
    const Vector3d x_displacement(2.0 / A.cols(), 0, 0);
    const Vector3d y_displacement(0, -2.0 / A.rows(), 0);

    //Sphere setup
    const Vector3d sphere_center(0, 0, 0);
    const double sphere_radius = 0.9;

    //material params
    const Vector3d diffuse_color(1, 0, 1);
    const double specular_exponent = 100;
    const Vector3d specular_color(0., 0, 1);

    // Single light source
    const Vector3d light_position(-1, 1, 1);
    double ambient = 0.1;

    for (unsigned i = 0; i < A.cols(); ++i)
    {
        for (unsigned j = 0; j < A.rows(); ++j)
        {
            const Vector3d pixel_center = image_origin + double(i) * x_displacement + double(j) * y_displacement;

            // Prepare the ray (for perspective view)
            const Vector3d ray_origin = camera_origin;
            const Vector3d ray_direction = (camera_origin - pixel_center).normalized();

            // Intersect with the sphere
            // NOTE: this is a special case of a sphere centered in the origin and for orthographic rays aligned with the z axis
            //Vector2d ray_on_xy(ray_origin(0), ray_origin(1));
            //const double sphere_radius = 0.9;

            Vector3d e = ray_origin;
            Vector3d d = ray_direction;
            Vector3d c = sphere_center;

            double Aa = d.dot(d);
            double B = 2*d.dot(e-c);
            double C = (e-c).dot(e-c) - pow(sphere_radius, 2);

            double discriminant =  pow(B, 2) - (4*Aa*C);

            //if discriminant is negative => no roots => no intersection
            if (discriminant >= 0)
            {
                // The ray hit the sphere, compute the exact intersection point
                double t = ((-1*B) + sqrt(discriminant)) / (2*Aa);  //use quadratic equation to find t
                Vector3d ray_intersection = e + (t*d);              //evaluate ray equation at t

                // Compute normal at the intersection point
                Vector3d ray_normal = ray_intersection.normalized();

                // TODO: Add shading parameter here
                double I = 1;   //assume intesity 1
                
                Vector3d l = (light_position - ray_intersection).normalized();  //light ray
                Vector3d v = (camera_origin - ray_intersection).normalized();   //view ray

                Vector3d h = (l+v) / (l+v).norm();      //find bisect between light and view ray
                
                const double diffuse  = I * std::max( 0.0, (light_position - ray_intersection).normalized().dot(ray_normal) );
                const double specular = I * pow( std::max( 0.0, (h).normalized().dot(ray_normal) ) , specular_exponent );

                Red(i, j) = ambient + (diffuse_color.coeff(0)*diffuse) + (specular_color.coeff(0)*specular);
                Red(i, j) = std::max(Red(i, j), 0.);

                Green(i, j) = ambient + (diffuse_color.coeff(1)*diffuse) + (specular_color.coeff(1)*specular);
                Green(i, j) = std::max(Green(i, j), 0.);

                Blue(i, j) = ambient + (diffuse_color.coeff(2)*diffuse) + (specular_color.coeff(2)*specular);
                Blue(i, j) = std::max(Blue(i, j), 0.);

                // Simple diffuse model
                //C(i, j) = ambient + diffuse + specular;
                // Clamp to zero
                //C(i, j) = std::max(C(i, j), 0.);

                // Disable the alpha mask for this pixel
                A(i, j) = 1;
            }
        }
    }

    // Save to png
    write_matrix_to_png(Red, Green, Blue, A, filename);
}



int main()
{
    raytrace_sphere();
    raytrace_parallelogram();
    raytrace_perspective();
    raytrace_shading();

    return 0;
}
