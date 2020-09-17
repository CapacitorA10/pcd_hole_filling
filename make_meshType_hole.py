
import numpy as np
import open3d as o3d

mesh = o3d.io.read_triangle_mesh("D:/bunny_mesh10k.ply")
#o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=True)

np.asarray(mesh.vertices)
np.asarray(mesh.triangles)

mesh_tree = o3d.geometry.KDTreeFlann(mesh)
max_distance = 0.02
#rand_value = np.random.randint(len(mesh.vertices))  #0~데이터 최대값 사이를 랜덤 생성
rand_value = 1000
[k, idx, _] = mesh_tree.search_radius_vector_3d(mesh.vertices[rand_value], max_distance) #거리로 가져오기



for i in idx :                                                              # 인덱스에 해당하는 모든 값에 대해 반복
    delete_point = np.where(np.asarray(mesh.triangles) == i)                # 인덱스가 30이라면, 30의 값을 가지고 있는 행을 모두 가져옴
    temp = np.delete(np.asarray(mesh.triangles), delete_point[0], axis=0)
    mesh.triangles = o3d.utility.Vector3iVector(temp)



#alived_vertices = np.delete(np.asarray(mesh.vertices), idx, axis=0) #index에 해당하는 꼭짓점들을 eliminate
#mesh.vertices = o3d.utility.Vector3dVector(alived_vertices)

np.asarray(mesh.vertices)[idx]=0 #꼭짓점들을 함부로 지우면 인덱스 정보가 날라감... 해결 방법 강구 필요. 일단은 0.0.0으로 점을 싹다바꾸기로...
o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=True)

o3d.io.write_triangle_mesh("D:/bunnyHole.ply", mesh)
pcd = o3d.io.read_point_cloud("D:/bunnyHole.ply")
o3d.visualization.draw_geometries([pcd])
