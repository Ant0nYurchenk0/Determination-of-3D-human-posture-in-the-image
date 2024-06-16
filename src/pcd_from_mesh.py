import open3d as o3d 

mesh = o3d.io.read_triangle_mesh("media/mesh/human_base_mesh_male.glb")
pcd = o3d.geometry.PointCloud()
pcd.points = mesh.vertices
pcd.normals = mesh.vertex_normals

o3d.visualization.draw_geometries([pcd])
