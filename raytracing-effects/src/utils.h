#ifndef UTILS_H
#define UTILS_H

#include "stb_image_write.h"
#include <Eigen/Dense>
#include <algorithm>
#include <vector>

unsigned char double_to_unsignedchar(const double d)
{
    return round(std::max(std::min(1., d), 0.) * 255);
}

void write_matrix_to_uint8(
    const Eigen::MatrixXd &R, const Eigen::MatrixXd &G,
    const Eigen::MatrixXd &B, const Eigen::MatrixXd &A,
    std::vector<uint8_t> &image)
{
    assert(R.rows() == G.rows() && G.rows() == B.rows() && B.rows() == A.rows());
    assert(R.cols() == G.cols() && G.cols() == B.cols() && B.cols() == A.cols());

    const int w = R.rows();               // Image width
    const int h = R.cols();               // Image height
    const int comp = 4;                   // 4 Channels Red, Green, Blue, Alpha
    const int stride_in_bytes = w * comp; // Length of one row in bytes
    image.resize(w * h * comp, 0);        // The image itself;

    for (unsigned wi = 0; wi < w; ++wi)
    {
        for (unsigned hi = 0; hi < h; ++hi)
        {
            image[(hi * w * 4) + (wi * 4) + 0] = double_to_unsignedchar(R(wi, hi));
            image[(hi * w * 4) + (wi * 4) + 1] = double_to_unsignedchar(G(wi, hi));
            image[(hi * w * 4) + (wi * 4) + 2] = double_to_unsignedchar(B(wi, hi));
            image[(hi * w * 4) + (wi * 4) + 3] = double_to_unsignedchar(A(wi, hi));
        }
    }
}

void write_matrix_to_png(
    const Eigen::MatrixXd &R, const Eigen::MatrixXd &G,
    const Eigen::MatrixXd &B, const Eigen::MatrixXd &A,
    const std::string &filename)
{
    const int w = R.rows();               // Image width
    const int h = R.cols();               // Image height
    const int comp = 4;                   // 3 Channels Red, Green, Blue, Alpha
    const int stride_in_bytes = w * comp; // Length of one row in bytes

    std::vector<uint8_t> image;
    write_matrix_to_uint8(R, G, B, A, image);
    stbi_write_png(filename.c_str(), w, h, comp, image.data(), stride_in_bytes);
}

//determinate of matrix = [a c]
//                        [b d]
double det(double a, double b, double c, double d)
{
    return (a*d) - (b*c);
}

//cramer's rule
double cramers(Eigen::MatrixXd m)
{
    return    m.coeff(0, 0) * det(m.coeff(1, 1), m.coeff(2, 1), m.coeff(1, 2), m.coeff(2, 2))
            + m.coeff(1, 0) * det(m.coeff(0, 2), m.coeff(2, 2), m.coeff(0, 1), m.coeff(2, 1))
            + m.coeff(2, 0) * det(m.coeff(0, 1), m.coeff(1, 1), m.coeff(0, 2), m.coeff(1, 2));
}

//returns array [B, l, t]
double* parallelogram_intersect(const Eigen::Vector3d pgram_u, const Eigen::Vector3d pgram_v, const Eigen::Vector3d ray_direction,
                                    const Eigen::Vector3d pgram_origin, const Eigen::Vector3d ray_origin)
{
    //e + td = a + B(b-a) + l(c-a)
    //ray_origin + (t*ray_direction) = pgram_origin + (B*pgram_u) + (l*pgram_v)
    //x = [t u v]

    Eigen::Vector3d u = -1*pgram_u;
    Eigen::Vector3d v = -1*pgram_v;
    Eigen::Vector3d dir = -1*ray_direction;

    Eigen::MatrixXd m(3, 3);
    m.col(0) = u;
    m.col(1) = v;
    m.col(2) = dir;

    double M = cramers(m);

    Eigen::Vector3d e = pgram_origin - ray_origin;

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

#endif
