import numpy as np
import scipy.ndimage as sp
import open3d as o3d
from hole_detection import hole_detection
from edge_swapping import edge_swapping
from center_vertex_making import center_vertex_making
from sklearn.neighbors import KDTree
import random
# mesh load
mesh = o3d.io.read_triangle_mesh("D:/pointcloud\\sphere_hole2.ply")
mesh_old = o3d.io.read_triangle_mesh("D:/pointcloud\\sphere_hole2.ply")

# hole detect
boundary_triangle, boundary_line_i, boundary_vertex_i = hole_detection(mesh)

# hole filling
mesh, added_num = center_vertex_making(mesh)

# Edge swapping
mesh = edge_swapping(mesh, mesh_old, added_num,num=1)
#mesh = edge_swapping(mesh, mesh_old, added_num,num=2)
o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=True) # 출력

#################STEP1#####################33

for i in range(len(boundary_vertex_i)):
    a = np.unique(np.asarray(mesh_old.triangles)[np.where(np.asarray(mesh_old.triangles) == boundary_vertex_i[i])[0]])
    if i == 0:
        b = a
    else:
        b = np.unique(np.hstack([a,b]))
# 해당 점 인덱스로 실제 점 좌표 구하기


b_cood = np.zeros([len(b),3])
for i in range(len(b)):
    b_cood[i] = np.asarray(mesh_old.vertices)[b[i]]



temp_pcd = o3d.geometry.PointCloud()
temp_pcd.points = o3d.utility.Vector3dVector(b_cood)

# o3d.visualization.draw_geometries([temp_pcd], mesh_show_wireframe=True)

"새로운 포인트를 무작위로 생성해서 흩뿌리기(hole point, adjacent point갯수와 동일하게.. ,hole 경계 points들 내부에 생성한다) "
#####################STEP2############################
num_points = int(len(b_cood))
new_points = np.zeros((num_points,3))

points = np.asarray(mesh.vertices)
boundary_line = np.unique(boundary_line_i)
boundary_points = np.zeros([len(boundary_line),3])
for k in range(len(boundary_line)):
        boundary_points[k] = points[boundary_line[k],:]
"인접 점만 분리해서 추출"
adjacent_points = b_cood
for k in range(len(boundary_points)):
    a = np.unique(np.where(boundary_points[k] != adjacent_points)[0])
    if k != 0:
        adjacent_points = adjacent_points[a]
    else:
        adjacent_points = b_cood[a]


for i in range(num_points):
    new_points[i,0] = random.uniform(np.min(b_cood[:,0]), np.max(b_cood[:,0]))
    new_points[i,1] = random.uniform(np.min(b_cood[:,1]), np.max(b_cood[:,1]))
    new_points[i,2] = random.uniform(np.min(b_cood[:,2]), np.max(b_cood[:,2]))

"새로 생성된 포인트들과 기존에 있던 boundary points 합친 배열 임시 생성"
hole_points = np.concatenate((b_cood, new_points), axis=0)
tree1 = KDTree(boundary_points)
tree2 = KDTree(adjacent_points)

for j in range(len(new_points)):
    nearest_dist1, nearest_ind1 = tree1.query(new_points, k=4)
    nearest_dist2, nearest_ind2 = tree2.query(np.vstack((boundary_points[nearest_ind1[j][0]],boundary_points[nearest_ind1[j][0]])), k=4)
    new_points[j] = 2*(boundary_points[nearest_ind1[j][0]])-(adjacent_points[nearest_ind2[0][0]])

#####"조금 더 촘촘하게 하기 위한 code(아래 내용)#############
new_points = np.vstack([new_points,new_points])
for j in range(int(len(new_points)/2)):
    nearest_dist1, nearest_ind1 = tree1.query(new_points[0:int(len(new_points)/2),:], k=4)
    new_points[int(len(new_points)/2)+j] = ((boundary_points[nearest_ind1[j][0]]) + new_points[j])/2
hole_points = np.concatenate((b_cood, new_points), axis=0)
"반복"
new_points = np.vstack([new_points,new_points])
for j in range(int(len(new_points)/2)):
    nearest_dist1, nearest_ind1 = tree1.query(new_points[0:int(len(new_points)/2),:], k=4)
    new_points[int(len(new_points)/2)+j] = ((boundary_points[nearest_ind1[j][0]]) + new_points[j])/2
hole_points = np.concatenate((b_cood, new_points), axis=0)
"반복"
new_points = np.vstack([new_points,new_points])
for j in range(int(len(new_points)/2)):
    nearest_dist1, nearest_ind1 = tree1.query(new_points[0:int(len(new_points)/2),:], k=4)
    new_points[int(len(new_points)/2)+j] = ((boundary_points[nearest_ind1[j][1]]) + new_points[j])/2
hole_points = np.concatenate((b_cood, new_points), axis=0)

#############################
hole_points = np.concatenate((b_cood, new_points), axis=0)


Laplace_points = hole_points
color = np.zeros([len(hole_points),3])

color[0:len(b_cood),0] = 0
color[0:len(b_cood),1] = 1
color[0:len(b_cood),2] = 0

color[len(b_cood):len(b_cood)+num_points,0] = 1
color[len(b_cood):len(b_cood)+num_points,1] = 0
color[len(b_cood):len(b_cood)+num_points,2] = 0

temp_pcd.points = o3d.utility.Vector3dVector(Laplace_points)
temp_pcd.colors = o3d.utility.Vector3dVector(color)
o3d.visualization.draw_geometries([temp_pcd], mesh_show_wireframe=True)

np.asarray(temp_pcd.colors)


###############################STEP4###################################
new_points = np.unique(new_points, axis=0)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(new_points)
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(5))
#o3d.visualization.draw_geometries([pcd], mesh_show_wireframe=True) # 출력w
new_normals = np.asarray(pcd.normals)


a = int(added_num / 3)
added_points = np.asarray(mesh.vertices)[len(np.asarray(mesh.vertices)) - a : len(np.asarray(mesh.vertices))]
pcd2 = o3d.geometry.PointCloud()
pcd2.points = o3d.utility.Vector3dVector(added_points)
pcd2.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamKNN(5))
#o3d.visualization.draw_geometries([pcd2], mesh_show_wireframe=True) # 출력w
added_normals = np.asarray(pcd2.normals)

###############################STEP5################################
weight_new_points = np.power(new_normals, -1) * new_points
weight_added_points = np.power(added_normals, -1) * added_points

for i in range(len(weight_added_points)):
    min = np.inf
    for j in range(len(weight_new_points)):
        now = np.linalg.norm(new_points[j] - added_points[i])
        if min > now:
            min = now
            minJ = j
    close_p = new_points[minJ]
    added_points[i] = close_p

p = np.asarray(mesh.vertices)
p[len(p)-a : len(p)] = added_points

mesh.vertices = o3d.utility.Vector3dVector(p)
o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=True) # 출력w
