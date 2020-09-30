import numpy as np
import open3d as o3d
from hole_detection import hole_detection
from edge_swapping import edge_swapping
from center_vertex_making import center_vertex_making

#mesh load
mesh = o3d.io.read_triangle_mesh("D:/pointcloud\\sphere_hole2.ply")
mesh_old = o3d.io.read_triangle_mesh("D:/pointcloud\\sphere_hole2.ply")

#hole detect
boundary_triangle, boundary_line_i, boundary_vertex_i = hole_detection(mesh)

#hole filling
mesh, added_num = center_vertex_making(mesh)
o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=True) # 출력


# Edge swapping
mesh = edge_swapping(mesh, mesh_old, added_num,num=1)
mesh = edge_swapping(mesh, mesh_old, added_num,num=2)

o3d.visualization.draw_geometries([mesh], mesh_show_wireframe=True) # 출력
o3d.io.write_triangle_mesh("D:/pointcloud\\middle_hole_filling.ply", mesh)
