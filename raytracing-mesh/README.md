Ray Tracing: Triangle Meshes and AABB Trees
===========================================
## Process
This project implements raytracing of triangle mesh. As this computation can take a long time, a Bounding Volume Hierarchy (BVH) structure will be impplemented to accelerate the computation time. This will be done with an Axis-Aligned Bounding Box tree. Other effects, such as adding other objects, shadows and reflected rays, were also implemented.

**Triangle Mesh:**
- Implemented ray_triangle_intersection() function to calculate the intersection of a ray with a triangle
- Implemented find_nearest_object() function that uses ray_triangle_intersection()  to check if a ray intersects with any triangles in the mesh.

**AABB Tree:**
- Implemented ray_box_intersection() function to check if a ray intersects with an axis-aligned bounding box (AABB).
- Implemented AABB tree construction using the AABBTree constructor and AABB_helper() function, recursively - dividing the set of triangles based on centroid differences.
- Updated find_nearest_object() function to utilize the AABB tree, traversing the tree to check for ray-triangle intersections more efficiently.

**Other Effects:**
- Added other objects (spheres and parallelogram) to the scene by initializing them in the setup_scene() function.
- Added ray_sphere_intersection() and ray_parallelogram_intersection() functions from a previous assignment.
- Updated find_nearest_object() to also check for intersections with spheres and the parallelogram.
- Implemented shadow rays by using the is_light_visible() function to skip shading calculations when the light source is blocked.
- Implemented ideal reflection by calculating the reflected ray using the normal and view vector, and recursively calling the shoot_ray() function with the reflected ray.

## How to run:

```
mkdir build; cd build; cmake ..; make
./assignment4
```

## Results
<div align="center">
    <img src="images/result_dragon.png" alt=images/result-dragon.png>
  <p><i> Resultant image of dragon with other objects and shadow and reflection rays </i></p>
</div>
