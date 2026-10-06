"""Dump mesh vertices in their body frames (geom_pos + R(geom_quat) * mesh_vert), sorted, to .npz.
Usage: python mesh_vertices.py <model.xml> <ABSOLUTE geometry_dir> <out.npz>
Equal arrays across two models => meshes sit identically on their bodies (visual and convex-hull collision shape)."""
# Mesh vertices expressed in their body frame: geom_pos + R(geom_quat) * mesh_vert. Equal => same placement on the body.
import sys, os, numpy as np, mujoco
def body_verts(xml, geomdir):
    s=open(xml).read().replace('file="Geometry/', f'file="{geomdir}/')
    os.chdir(os.path.dirname(os.path.abspath(xml))); m=mujoco.MjModel.from_xml_string(s); out={}
    for g in range(m.ngeom):
        if m.geom_type[g]!=mujoco.mjtGeom.mjGEOM_MESH: continue
        mid=m.geom_dataid[g]; v=m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid]+m.mesh_vertnum[mid]]
        R=np.zeros(9); mujoco.mju_quat2Mat(R,m.geom_quat[g]); out[mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,g)]=np.sort(v@R.reshape(3,3).T+m.geom_pos[g],axis=0)
    return out
np.savez(sys.argv[3], **body_verts(sys.argv[1], sys.argv[2]))
