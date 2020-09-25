import numpy as np
import open3d as o3d
from hole_detection import hole_detection

mesh = o3d.io.read_triangle_mesh("D:/pointcloud\\sphere_hole2.ply")
mesh_old = o3d.io.read_triangle_mesh("D:/pointcloud\\sphere_hole2.ply")
_, boundary_line_i, boundary_vertex_i = hole_detection(mesh)
#o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=True) # 출력


boundary_vertex = np.asarray(mesh.vertices)[boundary_vertex_i,:]    #index정보를 이용해 boundary vertex추출

#모든 좌표에 면적 구하기를 반복
s_sum = np.zeros(len(boundary_vertex)) + np.inf # 최소면적을 찾을 것이므로 일단 값이 무한대인 행렬 생성
for i in range(len(boundary_vertex)):
    min_s = min(s_sum)     #최대 면적인 값 저장하기 위함

    present_point_i  = boundary_vertex_i[i]
    overlap_lines_ii = np.where(boundary_line_i == present_point_i)[0] #현재 점을 공유하는 line index의 index를 찾기
    overlap_lines_i  = boundary_line_i[overlap_lines_ii]     #현재 점을 공유하는 line을 찾음

    other_lines_i    = boundary_line_i
    #겹치지 않는 line을 뽑아내는 과정
    for j in range(2):
        idx = np.where((other_lines_i == overlap_lines_i[j]).all(axis=1))
        other_lines_i = np.delete(other_lines_i, idx, axis=0)

    #위에서 뽑아낸 line들로 triangle을 만드는 과정
    temp_triangles = np.zeros([len(other_lines_i),3])
    for j in range(len(other_lines_i)):
        temp_triangles[j, :] = np.hstack([other_lines_i[j], present_point_i])
    temp_triangles = temp_triangles.astype(np.int64)

    # 각 triangles의 index정보를 다시 좌표정보로 변환
    coordinate_tri = np.zeros([len(other_lines_i),3,3]) #1개의 index에 3개의 좌표가 있으므로 3차원 배열 생성
    for j in range(len(temp_triangles)):
        tri = temp_triangles[j]
        for k in range(3):
            p = tri[k]
            coordinate_tri[j,k,:] = np.asarray(mesh.vertices)[p]

    #좌표정보를 바탕으로 면적 구하기 https://darkpgmr.tistory.com/86
    s = 0
    for j in range(len(coordinate_tri)):
        a1 = coordinate_tri[j,1,0] - coordinate_tri[j,0,0]
        a2 = coordinate_tri[j,1,1] - coordinate_tri[j,0,1]
        a3 = coordinate_tri[j,1,2] - coordinate_tri[j,0,2]
        b1 = coordinate_tri[j,2,0] - coordinate_tri[j,0,0]
        b2 = coordinate_tri[j,2,1] - coordinate_tri[j,0,1]
        b3 = coordinate_tri[j,2,2] - coordinate_tri[j,0,2]
        eq1 = ((a2 * b3) - (b2 * a3)) ** 2
        eq2 = ((b1 * a3) - (a1 * b3)) ** 2
        eq3 = ((a1 * b2) - (b1 * a2)) ** 2
        s = s + np.sqrt(eq1 + eq2 + eq3)/2
    s_sum[i] = s

    # 현재 찾은 값이 가장 작다면 현재 상태 저장
    if min_s > s_sum[i]:
        min_triangles = temp_triangles
        min_coordinate = coordinate_tri

"""지금까지 looping을 통해 면적이 최소가 되는 삼각형을 찾았고"""
'''이제 이들의 무게중심을 구하고 triangle을 씌우고 색칠까지ㄱ'''

colorInfo = np.ones([len(mesh.vertices) + len(min_triangles), 3])  # 미리 데이터의 컬러를 설정\
colorInfo = colorInfo * 0.75
#colorInfo[:,1] = 0.75

for i in range(len(min_coordinate)):
    center = np.mean(min_coordinate[i], axis=0)
    vertices = np.asarray(mesh.vertices)  # mesh파일의 모든 vertex가져온 후
    vertices = np.vstack([vertices, center])  # 새로 만든 무게 중심 추가
    mesh.vertices = o3d.utility.Vector3dVector(vertices)  # 포인트 대임

    new_triangles = np.zeros([3,3])      # 새로 생성할 triangle면 행렬 선언
    # 변 찾기
    a = min_triangles[i][[0, 1]]
    b = min_triangles[i][[0, 2]]
    c = min_triangles[i][[1, 2]]
    pnt = len(vertices) - 1 # pnt = 지금 추가하려는 triangle이 참조해야 하는 무게중심의 index
    new_triangles[0] = np.hstack([a, pnt])
    new_triangles[1] = np.hstack([b, pnt])
    new_triangles[2] = np.hstack([c, pnt])
    new_triangles = new_triangles.astype(np.int64)      #새로 생성된 3개의 삼각형들

    triangles = np.asarray(mesh.triangles)
    triangles = np.vstack([triangles, new_triangles])  # 새로 만든 면 추가
    mesh.triangles = o3d.utility.Vector3iVector(triangles)  # 면 대입

    # 색칠
    colorInfo[max(np.shape(vertices)) - 1] = [0.9, 0, 0]
    mesh.vertex_colors = o3d.utility.Vector3dVector(colorInfo)

o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=True) # 출력

"""Edge Swapping"""
added_num = 3 * len(min_triangles)
added_triangles = np.asarray(mesh.triangles)[len(triangles) - added_num : len(triangles)]   # 추가된 삼각형들을 추출

added_tri_coop  = np.zeros([len(added_triangles),3,3])


record = np.zeros(2 * len(added_triangles) + 1)

for i in range(len(added_triangles)):

    #if record에 존재하는 값 == 현재 찾으려는 값: ->하는 이유: 이미 수정 된 값을 다시 수정하려 하면 버그가 생기기 때문.
    #    continue
    rec = np.delete(record, np.where(record == 0) )
    added_rec = rec - len(np.asarray(mesh_old.triangles))
    if sum(added_rec == i) :
        continue

    '''swapping 시작'''
    # 모든 삼각형을 순회하면서 현재 면과 겹치는 삼각형들을 조사
    tri = added_triangles[i]
    '''0-1번 겹치는 삼각형 찾기'''
    idx1 = np.where(tri[0] == added_triangles)[0]   #added tri에서의 첫 번재 원소와 겹치는 행 조사
    idx1 = np.delete(idx1, np.where(idx1 == i))     #현재 순회중인 삼각형의 index는 미리 제거
    for j in idx1:
        idx2 = np.where(tri[1] == added_triangles[j])   # 두 번째 원소와 겹치는 행 조사
        if idx2[0] == 1:    # 이 if문을 통과하면, 두개의 삼각형 tri와 added_trianlges[j]가 맞닿은걸로 볼 수 있음
            tri2 = added_triangles[j]
            line = tri[np.where(tri == tri2)]   #line은 이제 이 두 삼각형의 중앙 선을 나타냄
            line_length = np.linalg.norm(np.asarray(mesh.vertices)[line[0]] - np.asarray(mesh.vertices)[line[1]])   #이 두 점 거리를 계산
            # 새로운 line을 찾아서 계산해보기
            spot1 = tri[np.where(tri != tri2)]
            spot2 = tri2[np.where(tri2 != tri)] # 원래 선이 아닌 다른 두 점을 추출
            line_length_new = np.linalg.norm(np.asarray(mesh.vertices)[spot1] - np.asarray(mesh.vertices)[spot2])

            if line_length_new < line_length: #새로 나온게 더 작다면 이걸로 대체함
                swapped_tri1 = np.zeros([3])
                swapped_tri2 = np.zeros([3]) #선언 먼저 하고, 하나씩 대입해야 오류가 안났음
                swapped_tri1[0] = tri[0]
                swapped_tri1[1] = spot1
                swapped_tri1[2] = spot2
                swapped_tri1 = swapped_tri1.astype(int)
                swapped_tri2[0] = tri[1]
                swapped_tri2[1] = spot1
                swapped_tri2[2] = spot2
                swapped_tri2 = swapped_tri2.astype(int)

            old = np.asarray(mesh.triangles)    #기존 트라이앵글 불러옴
            new = np.where((old == tri).all(axis=1))    #새 트라이앵글 넣을 주소
            new2 = np.where((old == tri2).all(axis=1))
            old[new] = swapped_tri1 #새 트라잉앵글 대입
            old[new2] = swapped_tri2
            #바뀐 점들의 좌표를 저장해놔서 다음 루프때 이 점은 지나지 않도록 주의하도록 하는 코드
            record[2*i] = np.asarray(new)
            record[2*i+1] = np.asarray(new2)
            record = record.astype(int)
            mesh.triangles = o3d.utility.Vector3iVector(old)

o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=True) # 출력
o3d.io.write_triangle_mesh("D:/pointcloud\\middle_hole_filling.ply", mesh)
