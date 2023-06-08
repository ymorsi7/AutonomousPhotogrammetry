import open3d as o3d

# Load the point cloud from the PCD file
point_cloud = o3d.io.read_point_cloud("194430109120F01200_1686104757.pcd")

# Estimate normals for the point cloud
point_cloud.estimate_normals()

# Convert the point cloud to a mesh
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(point_cloud)

# Create an empty mesh with the required properties
output_mesh = o3d.geometry.TriangleMesh()
output_mesh.vertices = o3d.utility.Vector3dVector(mesh.vertices)
output_mesh.triangles = o3d.utility.Vector3iVector(mesh.triangles)
output_mesh.vertex_normals = o3d.utility.Vector3dVector(mesh.vertex_normals)
output_mesh.vertex_colors = o3d.utility.Vector3dVector(mesh.vertex_colors)
# Save the mesh to an OBJ file
o3d.io.write_triangle_mesh("omesh.obj", output_mesh)

