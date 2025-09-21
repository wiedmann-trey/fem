## FEM - Finite Element Method Simulator
This is a basic implementation of a simulator for deformable solid objects using the finite element method completed for CS2240: Advanced Computer Graphics, at Brown University. The simulator runs real-time with basic OpenGL graphics.

https://github.com/user-attachments/assets/d4673c08-92a1-44c3-a3f2-dc3b84ca8978

https://github.com/user-attachments/assets/d97d3dba-f687-4e11-9951-5f68fc6fb69d

https://github.com/user-attachments/assets/64f90591-6480-4bbe-8281-60750db3efa4

### Initialising a simulation
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

### Implementation
- [Surface extraction](https://github.com/wiedmann-trey/fem/blob/8d34f2ff7cc44fcd9033da3c33d7489955db4480/src/extractfaces.cpp#L69): Loop every face in the mesh, maintaining a set of ones we've seen so far. If the mesh only contains a face once, it's an outside face. I also use this code to ensure that the faces for each tetrahedron point outwards.
- [Internal Forces](https://github.com/wiedmann-trey/fem/blob/8d34f2ff7cc44fcd9033da3c33d7489955db4480/src/femobject.cpp#L160): Every deformable object is represented with a FEMObject, which provides methods to set/get the state of the object, and compute the gradient. On initialization, we assign node masses and precompute the beta matrix for each tetrahedron. Then, when the derivative is computed, for each tetrahedron, we compute the strain and stress, and accumulate the stress forces into each node, and also add gravity as well as collision forces.
- [Collision Resolution](https://github.com/wiedmann-trey/fem/blob/8d34f2ff7cc44fcd9033da3c33d7489955db4480/src/collider.cpp#L62): I have a Collider class, which represents a mesh that objects can collide with. Each one has a method that checks whether a point collides with the mesh, and returns the penalty force for the collision. Every deformable object checks for collisions with all colliders, and applies the penalty forces.
- [Explicit Integration](https://github.com/wiedmann-trey/fem/blob/8d34f2ff7cc44fcd9033da3c33d7489955db4480/src/midpoint.h#L6): I mantain an object called FEMSystem, which offers methods for getting/setting all object's states, as well as getting the derivative of the state. I implement the midpoint method, which is very simple to implement given these methods.
- Multi-way deformable object collision: Deformable objects can collide with each other. To do this, meshes can have both a collider and be a deformable object. Its collider will be updated when its vertices are updated each step.

### Building + running the project
Make sure to run "git submodule update --init" after cloning. The project can be built in Qt Creator. Open the project via the CMakeList and set the working directory to the base directory of the project. See above for info about config files, or use one provided in "inis/". Pass the path to the config file as a command line argument.
