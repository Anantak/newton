import newton
import newton.geometry.utils

mesh_filename = "/home/ubuntu/Downloads/MAIN FRAME.obj"

# import openmesh
# m = openmesh.read_trimesh(mesh_filename)
# mesh_points = np.array(m.points())
# mesh_indices = np.array(m.face_vertex_indices(), dtype=np.int32).flatten()
# mesh = newton.Mesh(mesh_points, mesh_indices)
# print(mesh_points.shape)
# print(mesh_indices.shape)


mesh_1 = newton.geometry.utils.load_mesh(mesh_filename, "openmesh")
newton.geometry.utils.visualize_meshes([mesh_1])
