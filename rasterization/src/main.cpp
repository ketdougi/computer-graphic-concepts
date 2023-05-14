// C++ include
#include <iostream>
#include <string>
#include <vector>

// Utilities for the Assignment
#include "raster.h"

#include <gif.h>
#include <fstream>

#include <Eigen/Geometry>
// Image writing library
#define STB_IMAGE_WRITE_IMPLEMENTATION // Do not include this line twice in your project!
#include "stb_image_write.h"

using namespace std;
using namespace Eigen;

//Image height
const int H = 480;

//Camera settings
const double near_plane = 1.5; //AKA focal length
const double far_plane = near_plane * 100;
const double field_of_view = 0.7854; //45 degrees
const double aspect_ratio = 1.5;
const bool is_perspective = true;
const Vector3d camera_position(0, 0, 3);
const Vector3d camera_gaze(0, 0, -1);
const Vector3d camera_top(0, 1, 0);

//Object
const std::string data_dir = DATA_DIR;
const std::string mesh_filename(data_dir + "bunny.off");
MatrixXd vertices; // n x 3 matrix (n points)
MatrixXi facets;   // m x 3 matrix (m triangles)

//Material for the object
const Vector3d obj_diffuse_color(0.5, 0.5, 0.5);
const Vector3d obj_specular_color(0.2, 0.2, 0.2);
const double obj_specular_exponent = 256.0;

//Lights
std::vector<Vector3d> light_positions;
std::vector<Vector3d> light_colors;
//Ambient light
const Vector3d ambient_light(0.3, 0.3, 0.3);

//Fills the different arrays
void setup_scene()
{
    //Loads file
    std::ifstream in(mesh_filename);
    if (!in.good())
    {
        std::cerr << "Invalid file " << mesh_filename << std::endl;
        exit(1);
    }
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

    //Lights
    light_positions.emplace_back(8, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(6, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(4, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(2, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(0, 8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(-2, -8, 0);
    light_colors.emplace_back(16, 16, 16);

    light_positions.emplace_back(-4, 8, 0);
    light_colors.emplace_back(16, 16, 16);
}

void build_uniform(UniformAttributes &uniform)
{
    double image_y = sin(field_of_view/2)*near_plane/cos(field_of_view/2);
    double image_x = image_y*aspect_ratio;
    //TODO: setup uniform
    
    uniform.view << 1, 0, 0, 0,
                    0, 1, 0, 0,
                    0, 0, 1, 0,
                    0, 0, 0, 1;
    
    if (aspect_ratio < 1)
        uniform.view(0, 0) = aspect_ratio;
    else
        uniform.view(1, 1) = 1/aspect_ratio;

    //TODO: setup camera, compute w, u, v

    Vector3d w = -camera_gaze/camera_gaze.norm();
    Vector3d u = camera_top.cross(w) / (camera_top.cross(w)).norm();
    Vector3d v = w.cross(u);
    
    //TODO: compute the camera transformation
    Matrix4d cam = Matrix4d(4, 4);

    cam.row(0) << u[0], v[0], w[0], camera_position[0];
    cam.row(1) << u[1], v[1], w[1], camera_position[1];
    cam.row(2) << u[2], v[2], w[2], camera_position[2];
    cam.row(3) << 0, 0, 0, 1;

    uniform.camera = Matrix4d(4, 4);
    uniform.camera = cam.inverse();

    //TODO: setup projection matrix
    double t = image_y;    //top
    double b = -image_y;  //bottom
    
    double r = image_x;   //right
    double l = -image_x;  //left
    
    double f = -far_plane; //far
    double n = -near_plane; //near

    uniform.projection.row(0) << 2/(r-l), 0, 0, -(r+l)/(r-l);
    uniform.projection.row(1) << 0, 2/(t-b), 0, -(t+b)/(t-b); //made taller
    uniform.projection.row(2) << 0, 0, 2/(n-f), -(n+f)/(n-f);
    uniform.projection.row(3) << 0, 0, 0, 1;

    if (is_perspective)
    {
        //TODO setup prespective camera
        uniform.perspective  << n, 0, 0, 0,
                                0, n, 0, 0,
                                0, 0, (n+f), -f*n,
                                0, 0, 1, 0;
    }
    else
    {
        uniform.perspective  << 1, 0, 0, 0,
                                0, 1, 0, 0,
                                0, 0, 1, 0,
                                0, 0, 0, 1;
    }
}

void simple_render(Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;

    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        VertexAttributes out;
        out.position = uniform.projection * uniform.perspective * uniform.camera * va.position;
        return out;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        return FragmentAttributes(1, 0, 0);
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        //TODO: fill the shader
        return FrameBufferAttributes(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
    };

    std::vector<VertexAttributes> vertex_attributes;
    //TODO: build the vertex attributes from vertices and facets
    for( int i=0; i<facets.rows(); i++ )
    {
        VectorXd a = vertices.row(facets(i, 0));
        VectorXd b = vertices.row(facets(i, 1));
        VectorXd c = vertices.row(facets(i, 2));
        vertex_attributes.push_back(a);
        vertex_attributes.push_back(b);
        vertex_attributes.push_back(c);
    }

    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

Matrix4d compute_rotation(const double alpha)
{
    //TODO: Compute the rotation matrix of angle alpha on the y axis around the object barycenter
    Matrix4d res;

    if(alpha == 0)
    {
        res  << 1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
        return res;
    }

    res  << cos(alpha),  0, sin(alpha), 0,
            0,           1, 0,          0,
            -sin(alpha), 0, cos(alpha), 0,
            0,           0, 0,          1;

    return res;
}

void wireframe_render(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;

    Matrix4d trafo = compute_rotation(alpha);
    uniform.trafo = trafo;

    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        VertexAttributes out;
        out.position = uniform.projection * uniform.perspective * uniform.camera * uniform.trafo * va.position;
        return out;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: fill the shader
        return FragmentAttributes(1, 0, 0);
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        //TODO: fill the shader
        return FrameBufferAttributes(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
    };

    std::vector<VertexAttributes> vertex_attributes;

    //TODO: generate the vertex attributes for the edges and rasterize the lines
    for( int i=0; i<facets.rows(); i++ )
    {
        VectorXd a = vertices.row(facets(i, 0));
        VectorXd b = vertices.row(facets(i, 1));
        VectorXd c = vertices.row(facets(i, 2));
        vertex_attributes.push_back(a);
        vertex_attributes.push_back(b);
        vertex_attributes.push_back(b);
        vertex_attributes.push_back(c);
        vertex_attributes.push_back(c);
        vertex_attributes.push_back(a);
    }
    //TODO: use the transformation matrix

    rasterize_lines(program, uniform, vertex_attributes, 0.5, frameBuffer);
}

void get_shading_program(Program &program)
{
    program.VertexShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: transform the position and the normal
        //TODO: compute the correct lighting
        VertexAttributes out;
        out.position = uniform.projection * uniform.perspective * uniform.camera * uniform.trafo * va.position;
        out.normal   = uniform.trafo * va.normal;
        //TODO: create the correct fragment
        Vector4d lights_color(0, 0, 0, 1);

        const Vector3d normal   = Vector3d(out.normal[0],   out.normal[1],   out.normal[2]);
        const Vector3d position = Vector3d(out.position[0], out.position[1], out.position[2]);
        
        for (int i = 0; i < light_positions.size(); ++i)
        {
            const Vector3d &light_position = light_positions[i];
            const Vector4d &light_color = Vector4d(light_colors[i][0], light_colors[i][1], light_colors[i][2], 0);

            // diffuse
            const Vector3d Li = (light_position - position).normalized();
            Vector4d diff_color = Vector4d(obj_diffuse_color[0], obj_diffuse_color[1], obj_diffuse_color[2], 0);
            const Vector4d diffuse  = diff_color * std::max(Li.dot(normal), 0.0);

            //specular
            Vector3d v = (camera_position - position).normalized(); 
            const Vector3d Hi = ((Li + v) / (Li + v).norm()).normalized();
            Vector4d spec_color = Vector4d(obj_specular_color[0], obj_specular_color[1], obj_specular_color[2], 0);
            const Vector4d specular = spec_color * std::pow(std::max(Hi.dot(normal), 0.0), obj_specular_exponent);

            const Vector3d D = light_position - position;
            
            lights_color += (diffuse + specular).cwiseProduct(light_color) / D.squaredNorm();

        }
        out.color = lights_color;
        out.color[0] += ambient_light[0];
        out.color[1] += ambient_light[1];
        out.color[2] += ambient_light[2];

        return out;
    };

    program.FragmentShader = [](const VertexAttributes &va, const UniformAttributes &uniform) {
        //TODO: create the correct fragment
        FragmentAttributes out(va.color[0], va.color[1], va.color[2], va.color[3]);
        if(is_perspective)
            out.position       = Vector4d(va.position[0], va.position[1], va.position[2], va.position[3]);
        else
            out.position       = Vector4d(va.position[0], va.position[1], -1*va.position[2], va.position[3]);
        return out;
    };

    program.BlendingShader = [](const FragmentAttributes &fa, const FrameBufferAttributes &previous) {
        //TODO: implement the depth check
        if (fa.position[2] < previous.depth)
        {
            FrameBufferAttributes out(fa.color[0] * 255, fa.color[1] * 255, fa.color[2] * 255, fa.color[3] * 255);
            out.depth = fa.position[2];
            return out;
        }
        else
            return previous;
    };
}

void flat_shading(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;
    get_shading_program(program);
    Eigen::Matrix4d trafo = compute_rotation(alpha);
    uniform.trafo = trafo;

    std::vector<VertexAttributes> vertex_attributes;
    for( int i=0; i<facets.rows(); i++ )
    {
        VectorXd a = vertices.row(facets(i, 0));
        VectorXd b = vertices.row(facets(i, 1));
        VectorXd c = vertices.row(facets(i, 2));

        Vector3d e1 = b-a;
        Vector3d e2 = c-a;
        Vector3d n0 = e1.cross(e2).normalized();
        Vector4d n  = Vector4d(n0[0], n0[1], n0[2], 0);

        VertexAttributes a_VA = VertexAttributes(a, n);
        VertexAttributes b_VA = VertexAttributes(b, n);
        VertexAttributes c_VA = VertexAttributes(c, n);

        vertex_attributes.push_back(a_VA);
        vertex_attributes.push_back(b_VA);
        vertex_attributes.push_back(c_VA);
    }
    //TODO: compute the normals
    //TODO: set material colors

    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

void pv_shading(const double alpha, Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> &frameBuffer)
{
    UniformAttributes uniform;
    build_uniform(uniform);
    Program program;
    get_shading_program(program);

    Eigen::Matrix4d trafo = compute_rotation(alpha);
    uniform.trafo = trafo;

    //TODO: compute the vertex normals as vertex normal average
    std::vector<Vector4d> vertex_normals[vertices.size()];

    for( int i=0; i<facets.rows(); i++ )
    {
        VectorXd a = vertices.row(facets(i, 0));
        VectorXd b = vertices.row(facets(i, 1));
        VectorXd c = vertices.row(facets(i, 2));

        Vector3d e1 = b-a;
        Vector3d e2 = c-a;
        Vector3d n0 = e1.cross(e2).normalized();
        Vector4d n  = Vector4d(n0[0], n0[1], n0[2], 0);

        vertex_normals[facets(i, 0)].push_back(n);
        vertex_normals[facets(i, 1)].push_back(n);
        vertex_normals[facets(i, 2)].push_back(n);
    }


    std::vector<VertexAttributes> vertex_attributes;
    //TODO: create vertex attributes

    for( int i=0; i<facets.rows(); i++ )
    {
        VectorXd a = vertices.row(facets(i, 0));
        VectorXd b = vertices.row(facets(i, 1));
        VectorXd c = vertices.row(facets(i, 2));

        Vector4d v_normal[3];
        for(int j=0; j<3; j++)
        {
            for(Vector4d n : vertex_normals[facets(i, j)])
            {
                v_normal[j] += n;
            }
            v_normal[j] /= vertex_normals[facets(i, j)].size();
            v_normal[j] = v_normal[j].normalized();
        }

        VertexAttributes a_VA = VertexAttributes(a, v_normal[0]);
        VertexAttributes b_VA = VertexAttributes(b, v_normal[1]);
        VertexAttributes c_VA = VertexAttributes(c, v_normal[2]);

        vertex_attributes.push_back(a_VA);
        vertex_attributes.push_back(b_VA);
        vertex_attributes.push_back(c_VA);
    }

    //TODO: set material colors

    rasterize_triangles(program, uniform, vertex_attributes, frameBuffer);
}

int main(int argc, char *argv[])
{
    setup_scene();

    int W = H * aspect_ratio;
    Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic> frameBuffer(W, H);
    vector<uint8_t> image;

    simple_render(frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("simple.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    frameBuffer = Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic>::Zero(W,H);
    wireframe_render(0, frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("wireframe.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    frameBuffer = Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic>::Zero(W,H);
    flat_shading(0, frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("flat_shading.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    frameBuffer = Eigen::Matrix<FrameBufferAttributes, Eigen::Dynamic, Eigen::Dynamic>::Zero(W,H);
    pv_shading(0, frameBuffer);
    framebuffer_to_uint8(frameBuffer, image);
    stbi_write_png("pv_shading.png", frameBuffer.rows(), frameBuffer.cols(), 4, image.data(), frameBuffer.rows() * 4);

    //TODO: add the animation
    int delay = 25;
    double pi = 3.14159265;
    GifWriter g;
    
    GifBegin(&g, "wireframe.gif", frameBuffer.rows(), frameBuffer.cols(), delay);

    for(double i=0; i<1; i+=0.05)
    {
        frameBuffer.setConstant(FrameBufferAttributes());
        wireframe_render(i*2*pi, frameBuffer);
        framebuffer_to_uint8(frameBuffer, image);
        GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
    }
    GifEnd(&g);

    GifBegin(&g, "flat_shading.gif", frameBuffer.rows(), frameBuffer.cols(), delay);

    for(double i=0; i<1; i+=0.05)
    {
        frameBuffer.setConstant(FrameBufferAttributes());
        flat_shading(i*2*pi, frameBuffer);
        framebuffer_to_uint8(frameBuffer, image);
        GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
    }
    GifEnd(&g);

    GifBegin(&g, "pv_shading.gif", frameBuffer.rows(), frameBuffer.cols(), delay);

    for(double i=0; i<1; i+=0.05)
    {
        frameBuffer.setConstant(FrameBufferAttributes());
        pv_shading(i*2*pi, frameBuffer);
        framebuffer_to_uint8(frameBuffer, image);
        GifWriteFrame(&g, image.data(), frameBuffer.rows(), frameBuffer.cols(), delay);
    }
    GifEnd(&g);
    
    return 0;
}
