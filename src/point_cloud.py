import open3d as o3d 
import cv2
import numpy as np
import matplotlib.pyplot as plt
import imageio.v3 as iio


# FX_DEPTH = 5.8262448167737955e+02
# FY_DEPTH = 5.8269103270988637e+02
# CX_DEPTH = 3.1304475870804731e+02
# CY_DEPTH = 2.3844389626620386e+02


# depth_image = iio.imread("media/gymnast/depth/0.mp4", index=None)[0]

# print(f"Image resolution: {depth_image.shape}")
# print(f"Data type: {depth_image.dtype}")
# print(f"Min value: {np.min(depth_image)}")
# print(f"Max value: {np.max(depth_image)}")

# imgplot = plt.imshow(depth_image)
# plt.show()

# pcd = []
# height, width, _ = depth_image.shape
# for i in range(height):
#    for j in range(width):
#        z = depth_image[i][j]
#        x = (j - CX_DEPTH) * z / FX_DEPTH
#        y = (i - CY_DEPTH) * z / FY_DEPTH
#        pcd.append([x, y, z])

# pcd_o3d = o3d.geometry.PointCloud()  # create point cloud object
# pcd_o3d.points = o3d.utility.Vector3dVector(pcd)  # set pcd_np as the point cloud points
# # Visualize:
# o3d.visualization.draw_plotly([pcd_o3d])

camera=o3d.camera.PinholeCameraIntrinsic(650, 480, 525, 525, 320, 240)


trajectory = o3d.io.read_pinhole_camera_trajectory("media/images/trajectory.log")

pcds=[]

for i in range(211):
    color = o3d.io.read_image("media/images/color/{:05d}.jpg".format(i))
    depth = o3d.io.read_image("media/images/depth/{:05d}.png".format(i))
    im = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color, depth, 1000.0, 5.0, False)
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(im, camera, trajectory.parameters[i].extrinsic)
    pcd = pcd.voxel_down_sample(voxel_size=0.02)
    pcd.transform([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, -1]])
    pcds.append(pcd)



pcd_combined = o3d.geometry.PointCloud()
for point_id in range(len(pcds)):
    pcd_combined += pcds[point_id]
pcd_combined = pcd_combined.voxel_down_sample(voxel_size=0.01)


# o3d.visualization.draw_geometries([pcd_combined])



alpha = 0.05
mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd_combined, alpha)


# radii = [0.005, 0.01, 0.02, 0.04]
# mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd_combined, o3d.utility.DoubleVector(radii))


# pcd_combined.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
# mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd_combined, depth=9)
# vertices_to_remove = densities < np.quantile(densities, 0.01)
# mesh.remove_vertices_by_mask(vertices_to_remove)
# mesh.compute_vertex_normals()

o3d.visualization.draw_geometries([mesh])
# o3d.io.write_triangle_mesh("mesh.ply", mesh)