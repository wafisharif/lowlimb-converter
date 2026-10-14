# Modified by lowlimb-converter (2026-10-12), from MyoConverter @ cadf380: mesh extensions are matched in any case
# (".STL"), and OpenSim's placeholder for no mesh ("Unassigned") is skipped (Station 6, change 6.1).
# Modified by lowlimb-converter (2026-10-13): ASCII STL files are rewritten as binary STL (MuJoCo reads binary STL
# only) and OBJ files are copied (MuJoCo reads OBJ) (Station 6, change 6.7).
# Modified by lowlimb-converter (2026-10-13): a mesh file name that matches a file only up to case finds that file
# (Station 6, change 6.11).
""" This module contains a collection of utility functions useful for parsing and converting the OpenSim `BodySet`

@author: Aleksi Ikkala
"""

import numpy as np
import os
import trimesh
import pyvista
from shutil import copyfile

from loguru import logger

from myoconverter.xml.utils import create_symmetric_matrix


def valid_inertia(mass, inertia_vec):
  """ Check if given inertia vector is valid and can be used in the MuJoCo model.

  :param mass: Mass of a body/geom
  :param inertia_vec: Inertia vector
  :return: Boolean indicating whether the given vector is valid
  """
  values, vectors = np.linalg.eig(create_symmetric_matrix(inertia_vec))
  if mass > 0 and np.all(values > 0):
    return True
  else:
    return False

PLACEHOLDER_MESHES = ("", "unassigned")   # lowlimb-converter (change 6.1)


def is_placeholder_mesh(mesh_file):
  """ lowlimb-converter (change 6.1): True for OpenSim's "no file" placeholder (empty or "Unassigned"). """
  return mesh_file is None or mesh_file.strip().lower() in PLACEHOLDER_MESHES


def is_ascii_stl(path):
  """ lowlimb-converter (change 6.7): True if the STL file is in ASCII format. A binary STL is 84 + 50 n bytes, n the
  facet count stored at bytes 80-84 (its 80-byte header may itself start with "solid", so that alone doesn't
  decide). """
  size = os.path.getsize(path)
  with open(path, "rb") as f:
    head = f.read(84)
  if len(head) == 84 and 84 + 50 * int(np.frombuffer(head[80:84], dtype="<u4")[0]) == size:
    return False
  return head.lstrip().lower().startswith(b"solid")


def ascii_stl_to_binary(path, out_path):
  """ lowlimb-converter (change 6.7): rewrite an ASCII STL file as binary STL (same triangles, same vertex order;
  normals recomputed from the vertices, as MuJoCo does). :return: number of facets """
  vertices = []
  with open(path, "r", errors="replace") as f:
    for line in f:
      parts = line.split()
      if len(parts) == 4 and parts[0].lower() == "vertex":
        vertices.append([float(v) for v in parts[1:]])
  v = np.asarray(vertices, dtype=np.float64)
  if len(v) == 0 or len(v) % 3:
    raise ValueError(f"{path}: not a valid ASCII STL file ({len(v)} vertices)")
  tri = v.reshape(-1, 3, 3)
  normal = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
  length = np.linalg.norm(normal, axis=1, keepdims=True)
  normal = np.divide(normal, length, out=np.zeros_like(normal), where=length > 0)
  record = np.zeros(len(tri), dtype=[("normal", "<f4", 3), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
  record["normal"] = normal
  record["vertices"] = tri
  with open(out_path, "wb") as f:
    f.write(b"binary STL written by lowlimb-converter from an ASCII STL".ljust(80, b" "))
    f.write(np.uint32(len(tri)).tobytes())
    f.write(record.tobytes())
  return len(tri)


def find_mesh_file(geometry_folder, mesh_file):
  """ lowlimb-converter (change 6.11): the name of the file in geometry_folder for mesh_file: itself if it exists,
  otherwise the one file whose name differs from it only in case; mesh_file again if there is no such (unique) file
  (reading it then raises FileNotFoundError, as before). """
  if os.path.isfile(os.path.join(geometry_folder, mesh_file)) or not os.path.isdir(geometry_folder):
    return mesh_file
  matches = [f for f in os.listdir(geometry_folder) if f.lower() == mesh_file.lower()]
  return matches[0] if len(matches) == 1 else mesh_file


def copy_mesh_file(mesh_file, geometry_folder, output_geometry_folder):
  """ This function copies the original mesh file,converts it to stl and fixes unconnected facets (if necessary)

  :param mesh_file: Path to mesh file
  :param geometry_folder: Path to the folder where the mesh file is
  :param output_geometry_folder: Path to folder into which the mesh file will be copied to
  :return: Name of mesh, filename of converted stl mesh file; (None, None) for OpenSim's placeholder (change 6.1)
  :raises NotImplementedError: if given mesh file is not vtp, stl or obj (obj: change 6.7)
  """
  # lowlimb-converter (change 6.1): OpenSim's placeholder for a mesh with no file; OpenSim shows nothing for it
  if is_placeholder_mesh(mesh_file):
    return None, None
  # lowlimb-converter (change 6.11): a file that differs only in case (Leg39's fibula_R.vtp is fibula_r.vtp) is the
  # one meant; OpenSim finds it on case-insensitive file systems (macOS, Windows)
  source_file = find_mesh_file(geometry_folder, mesh_file)
  if source_file != mesh_file:
    logger.info(f"Mesh file {mesh_file} not found; using {source_file} (same name but for case)")

  mesh_name = mesh_file[:-4]
  stl_filename = mesh_name + ".stl"
  stl_filepath = os.path.join(output_geometry_folder, stl_filename)
  extension = mesh_file[-3:].lower()   # lowlimb-converter (change 6.1): ".STL" too

  # Transform a vtk file into an stl file and repair the mesh
  if extension == "vtp":

    # Read the vtp file with pyvista
    pymesh = pyvista.read(os.path.join(geometry_folder, source_file))

    # Convert into trimesh (see https://github.com/pyvista/pyvista/discussions/2268)
    pymesh = pymesh.extract_surface().triangulate()
    faces_as_array = pymesh.faces.reshape((pymesh.n_faces, 4))[:, 1:]
    tmesh = trimesh.Trimesh(pymesh.points, faces_as_array)

    # Repair the mesh
    trimesh.repair.fix_inversion(tmesh, True)
    trimesh.repair.fix_normals(tmesh, True)
    trimesh.repair.fill_holes(tmesh)

    # Save as binary stl file
    tmesh.export(stl_filepath)

  # Just copy the stl file (lowlimb-converter, change 6.7: an ASCII STL is rewritten as binary STL)
  elif extension == "stl":
    source = os.path.join(geometry_folder, source_file)
    if is_ascii_stl(source):
      n = ascii_stl_to_binary(source, stl_filepath)
      logger.info(f"Mesh file {mesh_file} is ASCII STL: rewritten as binary STL ({n} facets)")
    else:
      copyfile(source, stl_filepath)

  # lowlimb-converter (change 6.7): MuJoCo reads OBJ files itself
  elif extension == "obj":
    obj_filename = mesh_name + ".obj"
    copyfile(os.path.join(geometry_folder, source_file), os.path.join(output_geometry_folder, obj_filename))
    return mesh_name, obj_filename

  else:
    logger.critical("Geom file is not vtp, stl or obj!")
    raise NotImplementedError

  return mesh_name, stl_filename

def get_rgba(mesh):
  """ Return rgba string for given mesh

  :param mesh: XML element
  :return: Rgba string in format "r g b a"
  """
  color = mesh.find("Appearance/color")
  opacity = mesh.find("Appearance/opacity")
  visible = mesh.find("Appearance/visible")

  # Check if mesh is visible, or transparent
  if visible is not None:
    if visible.text=="false":
      alpha = "0"
    else:
      alpha = "1"
  elif opacity is None:
    alpha = "1"
  else:
    alpha = opacity.text

  if alpha == "0":
    logger.warning(f"Setting mesh {mesh.attrib['name']} to invisible")

  return ("1 1 1" if color is None else color.text) + " " + alpha
