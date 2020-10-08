import numpy as np
import open3d as o3d
from hole_detection import hole_detection

mesh = o3d.io.read_triangle_mesh("D:/pointcloud\\sphere_hole.ply")
_, boundary_line_i, boundary_vertex_i = hole_detection(mesh)

"""무게중심 계산하여 vertex 넣기"""
boundary_vertex = np.asarray(mesh.vertices)[boundary_vertex_i,:]    #index정보를 이용해 boundary vertex추출

new_vertex = np.mean(boundary_vertex, axis=0)   # 무게 중심을 이용해 새 vertex만들기
vertices = np.asarray(mesh.vertices)            # mesh파일의 모든 vertex가져온 후
vertices = np.vstack([vertices,new_vertex])     # 새로 만든 무게 중심 추가

mesh.vertices = o3d.utility.Vector3dVector(vertices) #포인트 대임
"""색칠"""
colorInfo = np.ones([len(mesh.vertices),3]) #미리 데이터의 컬러를 설정\
colorInfo[:,:] = 0.75
colorInfo[max(np.shape(colorInfo))-1] = [1,0,0]
mesh.vertex_colors = o3d.utility.Vector3dVector(colorInfo)

"""생성된 vertex로 Mesh만들기"""
new_triangles = np.zeros_like([boundary_vertex]).squeeze()      # 새로 생성할 행렬 선언
for i in range(len(boundary_line_i)):
    # 새 trianlge에는 기존 선분들+마지막으로 우리가 추가했던 점의 index를 추가
    new_triangles[i,:] = np.hstack([boundary_line_i[i], max(np.shape(colorInfo))-1])
new_triangles = new_triangles.astype(np.int64)  #다시 정수형으로 변환

triangles = np.asarray(mesh.triangles)
triangles = np.vstack([triangles,new_triangles])     # 새로 만든 면 추가
mesh.triangles = o3d.utility.Vector3iVector(triangles)  #면 대입


"""출력"""
o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=True) # 출력
o3d.io.write_triangle_mesh("D:/pointcloud\\small_hole_filling.ply", mesh)

pcd = o3d.io.read_point_cloud("D:/pointcloud\\small_hole_filling.ply")
o3d.visualization.draw_geometries([pcd]) # 출력
