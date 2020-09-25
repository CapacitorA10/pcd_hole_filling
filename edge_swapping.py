import numpy as np
import open3d as o3d

def edge_swapping(mesh, mesh_old, added_num, num):   # filling한 mesh, filling전 mesh, 추가된 갯수, 몇번째 꼭짓점으로 시행할지(1=01, 2=02, 3=12)

    if num == 1:
        num1 = 0
        num2 = 1
        num3 = 2
    elif num == 2:
        num1 = 0
        num2 = 2
        num3 = 1
    elif num == 3:
        num1 = 1
        num2 = 2
        num3 = 0
    else:
        print("Wrong parameter'num'")
        exit(3)

    len_of_mesh = len(np.asarray(mesh.triangles))
    added_triangles = np.asarray(mesh.triangles)[len_of_mesh - added_num : len_of_mesh]   # 추가된 삼각형들을 추출

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
        idx1 = np.where(tri[num1] == added_triangles)[0]   #added tri에서의 첫 번재 원소와 겹치는 행 조사
        idx1 = np.delete(idx1, np.where(idx1 == i))     #현재 순회중인 삼각형의 index는 미리 제거
        for j in idx1:
            idx2 = np.where(tri[num2] == added_triangles[j])   # 두 번째 원소와 겹치는 행 조사
            if idx2[0] > 0:    # 이 if문을 통과하면, 두개의 삼각형 tri와 added_trianlges[j]가 맞닿은걸로 볼 수 있음
                tri2 = added_triangles[j]
                line = tri[num1 : num2+1]   #line은 이제 이 두 삼각형의 중앙 선을 나타냄
                line_length = np.linalg.norm(np.asarray(mesh.vertices)[line[0]] - np.asarray(mesh.vertices)[line[1]])   #이 두 점 거리를 계산
                # 새로운 line을 찾아서 계산해보기
                spot1 = tri[num3]
                pts = np.sum(np.hstack([np.where(tri[num1] == tri2), np.where(tri[num2] == tri2)]))
                if pts == 1:
                    num4 = 2
                elif pts == 2:
                    num4 = 1
                elif pts == 3:
                    num4 = 0
                else:
                    exit(888)

                spot2 = tri2[num4] # 원래 선이 아닌 다른 두 점을 추출
                line_length_new = np.linalg.norm(np.asarray(mesh.vertices)[spot1] - np.asarray(mesh.vertices)[spot2])

                if line_length_new < line_length: #새로 나온게 더 작다면 이걸로 대체함
                    swapped_tri1 = np.zeros([3])
                    swapped_tri2 = np.zeros([3]) #선언 먼저 하고, 하나씩 대입해야 오류가 안났음
                    swapped_tri1[0] = tri[num1]
                    swapped_tri1[1] = spot1
                    swapped_tri1[2] = spot2
                    swapped_tri1 = swapped_tri1.astype(int)
                    swapped_tri2[0] = tri[num2]
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

    return mesh
