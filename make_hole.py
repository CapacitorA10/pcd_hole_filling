import numpy as np
import open3d as o3d

"""base 데이터 로드/생성"""
raw = np.loadtxt("D:/bunny_has_normal.xyz")

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(raw[:,:3]) #포인트 대임
pcd.normals = o3d.utility.Vector3dVector(raw[:,3:])#노멀 대입

'''트리 구성하여 구멍뚫기'''
pcd_tree = o3d.geometry.KDTreeFlann(pcd)
max_distance = 0.02

rand_value = np.random.randint(len(raw))  #0~데이터 최대값 사이를 랜덤 생성
[k, idx, _] = pcd_tree.search_radius_vector_3d(pcd.points[rand_value], max_distance) #거리로 가져오기
alived_points = np.delete(np.asarray(pcd.points), idx, axis=0) #index에 해당하는 녀석들 eliminate
alived_normals = np.delete(np.asarray(pcd.normals), idx, axis=0) #index에 해당하는 녀석들 eliminate

pcd_alived = o3d.geometry.PointCloud()
pcd_alived.points = o3d.utility.Vector3dVector(alived_points[:,:3]) #포인트 대임
pcd_alived.normals = o3d.utility.Vector3dVector(alived_normals[:,:3])#노멀 대입

o3d.visualization.draw_geometries([pcd_alived],mesh_show_wireframe=False) #원본 출력
