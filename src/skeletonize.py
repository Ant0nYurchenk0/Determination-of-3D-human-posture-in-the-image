import skeletor as sk
import numpy as np
import trimesh
import trimesh.interfaces
import matplotlib.pyplot as plt
from skspatial.objects import Line, Points,Vector

def get_angle(skel, indices_1, indices_2, plot):
  fig = plt.figure(figsize=(100, 100))
  ax = fig.add_subplot(projection='3d')

  x = [point[0] for point in skel.vertices]
  y = [point[1] for point in skel.vertices]
  z = [point[2] for point in skel.vertices]


  ax.scatter(x, y, z)
  ax.set_aspect('equal', adjustable='box')

  for i in range(len(skel.vertices)):
      ax.text(x[i], y[i], z[i], '({})'.format(i))

  points_1 = Points([[x[i], y[i], z[i]] for i in indices_1])
  line_fit = Line.best_fit(points_1)
  p_1_1 = line_fit.to_point(-1)
  p_1_2 = line_fit.to_point(1)

  ax.plot([p_1_1[0], p_1_2[0]], [p_1_1[1], p_1_2[1]], [p_1_1[2], p_1_2[2]], 'ro-')

  points_2 = Points([[x[i], y[i], z[i]] for i in indices_2])
  line_fit = Line.best_fit(points_2)
  p_2_1 = line_fit.to_point(-1)
  p_2_2 = line_fit.to_point(1)

  ax.plot([p_2_1[0], p_2_2[0]], [p_2_1[1], p_2_2[1]], [p_2_1[2], p_2_2[2]], 'ro-')

  angle = Vector([p_1_2[0]-p_1_1[0], p_1_2[1]-p_1_1[1], p_1_2[2]-p_1_1[2]]).angle_between([p_2_2[0]-p_2_1[0], p_2_2[1]-p_2_1[1], p_2_2[2]-p_2_1[2]])
  degrees = np.degrees(angle).round()
  if(plot):
    plt.show()
  return degrees



mesh = trimesh.load("media/mesh/45.stl", force='mesh')
# mesh.show()
simple = sk.pre.simplify(mesh, 0.4)
skel = sk.skeletonize.by_tangent_ball(simple)
skel = sk.post.remove_bristles(skel, mesh)
# skel.show(mesh=True)
degrees = get_angle(skel, [36, 38, 14, 0], [0, 34], True)
print(degrees)


