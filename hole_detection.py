import numpy as np
import open3d as o3d

mesh = o3d.io.read_triangle_mesh("D:/bunnyHole.ply")
#o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=True)
colorInfo = np.ones([len(mesh.vertices),3]) #미리 데이터의 컬러를 설정\
colorInfo[:,1] = 0.75

k = 0
boundary_triangle = np.zeros([1000,3])
boundary_line = np.zeros([1000,2])
for i in range(len(mesh.triangles)) :

    a_triangular = np.asarray(mesh.triangles)[i] # 해당 순서의 면에 속하는 배열 추출
    '''1번 꼭짓점'''
    temp = np.where(np.asarray(mesh.triangles) == a_triangular[0])[0] # 첫 번째 꼭짓점에 맞닿은 index
    first = np.asarray(mesh.triangles)[temp]                          # index로 해당 행렬 추출

    '''1번 to 2번'''
    number_of_tri = np.shape(np.where(first == a_triangular[1])[0])  # 첫 번째 꼭짓점에 연결된 삼각형들을 조사하여, 두 번째 꼭짓점에 맞닿는 애들을 조사
                                                                     # 이 때, 2라면, 2개의 삼각형이므로 면, 1이라면 1개의 삼각형이므로 hole
    if np.squeeze(number_of_tri) == 1 :
        boundary_triangle[k, :] = a_triangular               #경계선이 되는 삼각형을 배열로 저장
        boundary_line[k, :]     = a_triangular[[0,1]]        # 삼각형의 1번과 2번 사이니까 그냥 1,2번 행렬을 저장
        k = k + 1

    """1번 to 3번"""
    number_of_tri = np.shape(np.where(first == a_triangular[2])[0])  # 첫 번째 꼭짓점에 연결된 삼각형들을 조사하여, 3 번째 꼭짓점에 맞닿는 애들을 조사
                                                                     # 이 때, 2라면, 2개의 삼각형이므로 면, 1이라면 1개의 삼각형이므로 hole
    if np.squeeze(number_of_tri) == 1 :
        boundary_triangle[k, :] = a_triangular  # 경계선이 되는 삼각형을 배열로 저장
        boundary_line[k, :]     = a_triangular[[0, 2]]  # 삼각형의 2번과 3번 사이니까 2,3번 행렬을 저장
        k = k + 1


    '''2번 꼭짓점'''
    temp = np.where(np.asarray(mesh.triangles) == a_triangular[1])[0]  # 두 번째 꼭짓점에 맞닿은 index
    second = np.asarray(mesh.triangles)[temp]

    """2번 to 3번"""
    number_of_tri = np.shape(np.where(second == a_triangular[2])[0])  # 두 번째 꼭짓점에 연결된 삼각형들을 조사하여, 두 번째 꼭짓점에 맞닿는 애들을 조사
    # 이 때, 2라면, 2개의 삼각형이므로 면, 1이라면 1개의 삼각형이므로 hole
    if np.squeeze(number_of_tri) == 1:
        boundary_triangle[k, :] = a_triangular  # 경계선이 되는 삼각형을 배열로 저장
        boundary_line[k, :]     = a_triangular[[1, 2]] # 삼각형의 2번과 3번 사이니까 2,3번 행렬을 저장
        k = k + 1

"""데이터 최적화 및 색칠 해주는 과정"""
boundary_triangle = np.squeeze(boundary_triangle.reshape([1,-1]))       #일렬로 쭉 핀 다음에, 0인 요소들 제거할 것
boundary_triangle = np.delete(boundary_triangle, np.where(boundary_triangle == 0))      #0인 요소들 찾아서 제거
boundary_triangle = boundary_triangle.reshape([-1,3])                   #reshape하며 마무리. 이후 중복되는 값을 제거
boundary_triangle = np.unique(boundary_triangle, axis=0).astype(np.int64)           #중복되는 값까지 제거.

boundary_line = np.squeeze(boundary_line.reshape([1,-1]))
boundary_line = np.delete(boundary_line, np.where(boundary_line == 0))
boundary_vertex = np.unique(boundary_line).astype(np.int64)             # 일렬로 늘린 김에 vertex정보를 추출
boundary_line = boundary_line.reshape([-1,2]).astype(np.int64)

#색칠하는 과정
for u in range(len(boundary_line)) :
    for v in range(2) :
        colorInfo[boundary_line[u,v]] = [0,0,1]

mesh.vertex_colors = o3d.utility.Vector3dVector(colorInfo)

o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=True)
