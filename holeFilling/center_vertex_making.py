import numpy as np
import open3d as o3d
from hole_detection import hole_detection

def center_vertex_making(mesh):
    boundary_triangle, boundary_line_i, boundary_vertex_i = hole_detection(mesh)
    # index정보를 이용해 boundary vertex추출
    boundary_vertex = np.asarray(mesh.vertices)[boundary_vertex_i, :]

    # 모든 좌표에 면적 구하기를 반복
    s_sum = np.zeros(len(boundary_vertex)) + np.inf  # 최소면적을 찾을 것이므로 일단 값이 무한대인 행렬 생성
    for i in range(len(boundary_vertex)):
        min_s = min(s_sum)  # 최대 면적인 값 저장하기 위함

        present_point_i = boundary_vertex_i[i]
        overlap_lines_ii = np.where(boundary_line_i == present_point_i)[0]  # 현재 점을 공유하는 line index의 index를 찾기
        overlap_lines_i = boundary_line_i[overlap_lines_ii]  # 현재 점을 공유하는 line을 찾음

        other_lines_i = boundary_line_i
        # 겹치지 않는 line을 뽑아내는 과정
        the_other_lines = other_lines_i
        for j in range(2):
            idx = np.where((other_lines_i == overlap_lines_i[j]).all(axis=1))
            other_lines_i = np.delete(other_lines_i, idx, axis=0)
            for k in range(2):
                idx2 = np.where((the_other_lines == overlap_lines_i[k][0]))[0]
                the_other_lines = np.delete(the_other_lines, idx2, axis=0)
                idx2 = np.where((the_other_lines == overlap_lines_i[k][1]))[0]
                the_other_lines = np.delete(the_other_lines, idx2, axis=0)

        # 위에서 뽑아낸 line들로 triangle을 만드는 과정
        temp_triangles = np.zeros([len(other_lines_i), 3])
        for j in range(len(other_lines_i)):
            temp_triangles[j, :] = np.hstack([other_lines_i[j], present_point_i])
        temp_triangles = temp_triangles.astype(np.int64)

        # 각 triangles의 index정보를 다시 좌표정보로 변환
        coordinate_tri2 = np.zeros([len(other_lines_i), 3, 3])  # 1개의 index에 3개의 좌표가 있으므로 3차원 배열 생성
        for j in range(len(temp_triangles)):
            tri = temp_triangles[j]
            for k in range(3):
                p = tri[k]
                coordinate_tri2[j, k, :] = np.asarray(mesh.vertices)[p]

        # 면적을 구할 triangle만 뽑아내는 과정
        temp_triangles_for_s = np.zeros([len(the_other_lines), 3])
        for j in range(len(the_other_lines)):
            temp_triangles_for_s[j, :] = np.hstack([the_other_lines[j], present_point_i])
        temp_triangles_for_s = temp_triangles_for_s.astype(np.int64)

        # 각 면적 전용 triangle만들기
        coordinate_tri = np.zeros([len(the_other_lines), 3, 3])  # 1개의 index에 3개의 좌표가 있으므로 3차원 배열 생성
        for j in range(len(temp_triangles_for_s)):
            tri = temp_triangles_for_s[j]
            for k in range(3):
                p = tri[k]
                coordinate_tri[j, k, :] = np.asarray(mesh.vertices)[p]

        # 좌표정보를 바탕으로 면적 구하기 https://darkpgmr.tistory.com/86
        s = 0
        for j in range(len(coordinate_tri)):
            a1 = coordinate_tri[j, 1, 0] - coordinate_tri[j, 0, 0]
            a2 = coordinate_tri[j, 1, 1] - coordinate_tri[j, 0, 1]
            a3 = coordinate_tri[j, 1, 2] - coordinate_tri[j, 0, 2]
            b1 = coordinate_tri[j, 2, 0] - coordinate_tri[j, 0, 0]
            b2 = coordinate_tri[j, 2, 1] - coordinate_tri[j, 0, 1]
            b3 = coordinate_tri[j, 2, 2] - coordinate_tri[j, 0, 2]
            eq1 = ((a2 * b3) - (b2 * a3)) ** 2
            eq2 = ((b1 * a3) - (a1 * b3)) ** 2
            eq3 = ((a1 * b2) - (b1 * a2)) ** 2
            s = s + np.sqrt(eq1 + eq2 + eq3) / 2
        s_sum[i] = s

        # 현재 찾은 값이 가장 작다면 현재 상태 저장
        if min_s > s_sum[i]:
            min_triangles = temp_triangles
            min_coordinate = coordinate_tri2

    """지금까지 looping을 통해 면적이 최소가 되는 삼각형을 찾았고"""
    '''이제 이들의 무게중심을 구하고 triangle을 씌우고 색칠까지ㄱ'''

    colorInfo = np.ones([len(mesh.vertices) + len(min_triangles), 3])  # 미리 데이터의 컬러를 설정\
    colorInfo = colorInfo * 0.75
    # colorInfo[:,1] = 0.75

    for i in range(len(min_coordinate)):
        center = np.mean(min_coordinate[i], axis=0)
        vertices = np.asarray(mesh.vertices)  # mesh파일의 모든 vertex가져온 후
        vertices = np.vstack([vertices, center])  # 새로 만든 무게 중심 추가
        mesh.vertices = o3d.utility.Vector3dVector(vertices)  # 포인트 대임

        new_triangles = np.zeros([3, 3])  # 새로 생성할 triangle면 행렬 선언
        # 변 찾기
        a = min_triangles[i][[0, 1]]
        b = min_triangles[i][[0, 2]]
        c = min_triangles[i][[1, 2]]
        pnt = len(vertices) - 1  # pnt = 지금 추가하려는 triangle이 참조해야 하는 무게중심의 index
        new_triangles[0] = np.hstack([a, pnt])
        new_triangles[1] = np.hstack([b, pnt])
        new_triangles[2] = np.hstack([c, pnt])
        new_triangles = new_triangles.astype(np.int64)  # 새로 생성된 3개의 삼각형들

        triangles = np.asarray(mesh.triangles)
        triangles = np.vstack([triangles, new_triangles])  # 새로 만든 면 추가
        mesh.triangles = o3d.utility.Vector3iVector(triangles)  # 면 대입

        # 색칠
        colorInfo[max(np.shape(vertices)) - 1] = [0.9, 0, 0]
        mesh.vertex_colors = o3d.utility.Vector3dVector(colorInfo)

    return mesh, (3 * len(min_triangles))
