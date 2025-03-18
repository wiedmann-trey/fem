## FEM

A simulation is initialized using a config.ini file passed as the single command line argument. This file contains global parameters under the Global header, as well as any number of meshes. The meshes are given headers Object0, Object1, etc. Here are a list of all config parameters. See "inis/" for examples.

Global
- camera_pos (Format: double, double, double) (Default: original stencil code position) -> xyz position of the camera 
- timestep (Format: double) (Default: .0003) -> timestep in seconds
- gravity (Format: double) (Default: 1) -> downwards acceleration of all deformable objects due to gravity
- collision_penalty (Format: double) (Default: 8e7) -> collision penalty scaling
- collision_epsilon (Format: double) (Default: .005) -> tolerance for detecting a collision

Object
- meshfile (Format: string) (Must be provided) -> path to meshfile
- simulate (Format: bool) (Default: false) -> simulate this mesh as a deformable object
- is_collider (Format: bool) (Default: false) -> deformable objects can collide with this mesh
- transform (Format: list of 16 double, row major) (Default: no transformation) -> transformation matrix to be applied to all vertices before start of simulation
- velocity (Format: double, double, double) (Default: 0, 0, 0) -> initial xyz velocities for all vertices in the mesh
- density (Format: double) (Default: 1200)
- incompressibility (Format: double) (Default: 4e4)
- rigidity (Format: double) (Default: 4e4)
- viscosity_1 (Format: double) (Default: 100)
- viscosity_2 (Format: double) (Default: 100)

### Videos
https://github.com/brown-cs-224/fem-wiedmann-trey/blob/master/example-video/simple_cube.mp4

https://github.com/brown-cs-224/fem-wiedmann-trey/blob/master/example-video/static_collider.mp4

https://github.com/brown-cs-224/fem-wiedmann-trey/blob/master/example-video/sphere_collision.mp4

https://github.com/brown-cs-224/fem-wiedmann-trey/blob/master/example-video/stack.mp4
### Implementation Details

- Surface extraction: Loop every face in the mesh, maintaining a set of ones we've seen so far. If the mesh only contains a face once, it's an outside face. I also use this code to ensure that the faces for each tetrahedron point outwards.
- Internal Forces: Every deformable object is represented with a FEMObject, which provides methods to set/get the state of the object, and compute the gradient. On initialization, we assign node masses and precompute the beta matrix for each tetrahedron. Then, when the derivative is computed, for each tetrahedron, we compute the strain and stress, and accumulate the stress forces into each node, and also add gravity as well as collision forces.
- Collision resolution: I have a Collider class, which represents a mesh that objects can collide with. Each one has a method that checks whether a point collides with the mesh, and returns the penalty force for the collision. Every deformable object checks for collisions with all colliders, and applies the penalty forces.
- Explicit integration: I mantain an object called FEMSystem, which offers methods for getting/setting all object's states, as well as getting the derivative of the state. I implement the midpoint method, which is very simple to implement given these methods.
- Extra features: For an extra feature, deformable objects can collide with each other. To do this, meshes can have both a collider and be a deformable object. Its collider will be updated when its vertices are updated each step.

### Implementation Locations

Please list the lines where the implementations of these features start:

- [Surface extraction](https://github.com/brown-cs-224/fem-wiedmann-trey/blob/291e7c7fc2e9689cf3da7fe0e41f2cccb1b17bab/src/extractfaces.cpp#L69)
- [Internal Forces](https://github.com/brown-cs-224/fem-wiedmann-trey/blob/291e7c7fc2e9689cf3da7fe0e41f2cccb1b17bab/src/femobject.cpp#L160)
- [Collision Resolution](https://github.com/brown-cs-224/fem-wiedmann-trey/blob/291e7c7fc2e9689cf3da7fe0e41f2cccb1b17bab/src/collider.cpp#L62)
- [Explicit Integration](https://github.com/brown-cs-224/fem-wiedmann-trey/blob/291e7c7fc2e9689cf3da7fe0e41f2cccb1b17bab/src/midpoint.h#L6)
- Extra: Deformable objects can collide with each other -- [here](https://github.com/brown-cs-224/fem-wiedmann-trey/blob/291e7c7fc2e9689cf3da7fe0e41f2cccb1b17bab/src/collider.cpp#L14) and [here](https://github.com/brown-cs-224/fem-wiedmann-trey/blob/291e7c7fc2e9689cf3da7fe0e41f2cccb1b17bab/src/femobject.cpp#L31) and [here](https://github.com/brown-cs-224/fem-wiedmann-trey/blob/291e7c7fc2e9689cf3da7fe0e41f2cccb1b17bab/src/femobject.cpp#L128)


### Collaboration/References
Worked alone. I referenced DeepSeek for some help with Eigen, linear algebra, and general C++ syntax.

### Known Bugs
None
