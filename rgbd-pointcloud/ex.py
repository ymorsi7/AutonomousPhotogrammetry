import open3d as o3d

# Load the point cloud from the PCD file
point_cloud = o3d.io.read_point_cloud("194430109120F01200_1685827071.pcd")
point_cloud.estimate_normals()
# Convert the point cloud to a mesh
mesh, _ = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(point_cloud)

# Save the mesh to an OBJ file
o3d.io.write_triangle_mesh("output_mes3.obj", mesh)

