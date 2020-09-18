import numpy as np
import open3d as o3d

mesh = o3d.io.read_triangle_mesh("D:/bunnyHole.ply")
#o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=True)
colorInfo = np.ones([len(mesh.vertices),3]) #미리 데이터의 컬러를 설정\
colorInfo[:,1] = 0.75


for i in range(len(mesh.triangles)) :

    a_triangular = np.asarray(mesh.triangles)[i] # 해당 순서의 면에 속하는 배열 추출
    '''1번 꼭짓점'''
    temp = np.where(np.asarray(mesh.triangles) == a_triangular[0])[0] # 첫 번째 꼭짓점에 맞닿은 index
    first = np.asarray(mesh.triangles)[temp]                          # index로 해당 행렬 추출

    '''1번 to 2번'''
    number_of_tri = np.shape(np.where(first == a_triangular[1])[0])  # 첫 번째 꼭짓점에 연결된 삼각형들을 조사하여, 두 번째 꼭짓점에 맞닿는 애들을 조사
                                                                     # 이 때, 2라면, 2개의 삼각형이므로 면, 1이라면 1개의 삼각형이므로 hole
    if np.squeeze(number_of_tri) == 1 :
        index_of_first = np.where(first == a_triangular[1])[0]
        need_to_red = first[index_of_first]                     # 색이 칠해져야 할 triangle 찾음. 이의 vertex들을 색칠하면 됨
        for j in range(3) :
            colorInfo[need_to_red[0,j]] = [0,1,0]

    """1번 to 3번"""
    number_of_tri = np.shape(np.where(first == a_triangular[2])[0])  # 첫 번째 꼭짓점에 연결된 삼각형들을 조사하여, 3 번째 꼭짓점에 맞닿는 애들을 조사
                                                                     # 이 때, 2라면, 2개의 삼각형이므로 면, 1이라면 1개의 삼각형이므로 hole
    if np.squeeze(number_of_tri) == 1 :
        index_of_first = np.where(first == a_triangular[2])[0]
        need_to_red = first[index_of_first]                     # 색이 칠해져야 할 triangle 찾음. 이의 vertex들을 색칠하면 됨
        for j in range(3) :
            colorInfo[need_to_red[0,j]] = [0,1,0]


    '''2번 꼭짓점'''
    temp = np.where(np.asarray(mesh.triangles) == a_triangular[1])[0]  # 두 번째 꼭짓점에 맞닿은 index
    second = np.asarray(mesh.triangles)[temp]

    """2번 to 3번"""
    number_of_tri = np.shape(np.where(second == a_triangular[2])[0])  # 두 번째 꼭짓점에 연결된 삼각형들을 조사하여, 두 번째 꼭짓점에 맞닿는 애들을 조사
    # 이 때, 2라면, 2개의 삼각형이므로 면, 1이라면 1개의 삼각형이므로 hole
    if np.squeeze(number_of_tri) == 1:
        index_of_first = np.where(second == a_triangular[2])[0]
        need_to_red = second[index_of_first]  # 색이 칠해져야 할 triangle 찾음. 이의 vertex들을 색칠하면 됨
        for j in range(3):
            colorInfo[need_to_red[0, j]] = [0, 1, 0]


mesh.vertex_colors = o3d.utility.Vector3dVector(colorInfo)

o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=True)
