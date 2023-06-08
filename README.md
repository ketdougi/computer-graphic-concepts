# Computer Graphics Concepts
These projects were completed for an Introduction to Computer Graphics course at the University of Victoria. Throughout the assignments, practical programming skills in C++ were developed, utilizing libraries like Simple Directmedia Library (SDL) for graphics programming. The course covered fundamental mathematical concepts, image synthesis algorithms and rendering techniques using raytracing and rasterization.

## Requirements to Run Projects
- CMake
- A C++ compiler

## Projects
[Convex Hull and Point in Polygon Problem](./convex-hull-point-in-polygon)
- Explored algorithms for finding the convex hull of a set of points and determining if a point is inside a polygon.
- Implemented solutions using concepts such as Graham's scan algorithm and ray-casting algorithm.

[Raytracing and Shading](./raytracing-shading)
- Introduced the principles of ray tracing for rendering 3D scenes.
- Implemented ray-object intersection algorithms for spheres, parallelograms, and triangle meshes.
- Applied shading techniques such as ambient, diffuse, and specular reflection to achieve realistic lighting effects.

[Basic Raytracing Effects](./raytracing-effects)
- Extended the ray tracing implementation to incorporate advanced effects such as shadows, reflections, and refractions.
- Implemented techniques like shadow rays to determine if a point is in shadow, and ideal reflection for reflective surfaces.

[Triangle Meshes and AABB Trees](./raytracing-mesh)
- Explored data structures to accelerate ray-triangle intersection calculations.
- Implemented an AABB tree (Axis-Aligned Bounding Box) to efficiently find intersections with triangles in a scene.

[Rasterization](./rasterization)
- Studied the rasterization pipeline for real-time rendering.
- Implemented 2 shading options, flat shading and per-vertex shading, using view transformations, vertex shaders, fragment shaders and blending shader.
- Used concepts such as z-buffering and perspective correction to produce accurate 2D images from 3D scenes.
