## NURBS Surfaces

**NURBS (Non-Uniform Rational B-Splines)** surfaces are a powerful mathematical representation used for modeling complex shapes in computer graphics, CAD (Computer-Aided Design), and 3D modeling. NURBS surfaces are defined by a grid of control points, allowing for precise control over the surface geometry and enabling the representation of both simple and intricate shapes.


### Key Features:

- **Flexibility**: NURBS can represent simple shapes like lines and circles as well as complex shapes like freeform surfaces.
- **Control Points**: The shape of a NURBS surface is influenced by a set of control points, which can be moved to alter the surface geometry.
- **Weights**: Each control point can have an associated weight, allowing for more control over the influence of the point on the surface shape.
- **Degree**: NURBS surfaces can be defined with different degrees, affecting the smoothness and continuity of the surface.

### Mathematical Representation:

A NURBS surface is defined using the following formula:

$$
S(u, v) = \frac{\sum_{i=0}^{n} \sum_{j=0}^{m} P_{ij} \cdot N_{i,p}(u) \cdot N_{j,q}(v) \cdot w_{ij}}{\sum_{i=0}^{n} \sum_{j=0}^{m} N_{i,p}(u) \cdot N_{j,q}(v) \cdot w_{ij}}
$$

Where:

- \(S(u, v)\) is the surface point at parameters \(u\) and \(v\).
- \(P_{ij}\) are the control points of the surface.
- \(N_{i,p}(u)\) and \(N_{j,q}(v)\) are the basis functions defined by the B-spline of degree \(p\) and \(q\).
- \(w_{ij}\) are the weights associated with each control point.
- \(n\) and \(m\) are the number of control points in the \(u\) and \(v\) directions, respectively.


  ## Execution Instructions

To compile and run the project, follow these steps:

### Build the Project

1. Open your terminal and navigate to the project directory.
2. Run the following command to compile the project: make
3. After the build process completes, execute the following command to run the application: ./main

### User Controls

Initial View: Displays the surface along with the control polygon.
Surface with Tangents and Normals: Press Z to view the surface along with tangent vectors in the 𝑢 and 𝑣 directions, as well as the normal vectors.
Curvature Visualization: Press E to attempt to visualize the curvature (note: results inaccurate).
Move Vectors: Use U, u, V, and v to move the tangent and normal vectors along the surface.
Return to Initial View: Press A to revert to the first view.
