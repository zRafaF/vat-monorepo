"""
VAT — URDF robot model (optional mesh avatar)
=============================================
Loads the Unitree URDF + meshes and produces a single merged (vertices, faces) mesh
for the robot at a given joint configuration + base pose, so the viewer can draw the
real 3D model instead of the wireframe skeleton.

Auto-detected from ``client/<robot>_description`` (prefers go2w; no env var needed;
``GO2_URDF`` overrides). Fully guarded: needs ``yourdfpy`` + ``trimesh`` (+ ``pycollada`` for the
.dae meshes). If anything is missing/unloadable, ``available`` is False and the
viewer stays on the skeleton (reason logged once).

Performance: the per-link LOCAL meshes and the concatenated faces are built ONCE at
load; each frame only re-reads the (cheap) link transforms after ``update_cfg`` and
transforms the vertices — so it doesn't re-dump the whole scene every frame
(~75 ms → a few ms).

FK convention: joint angles come from the on-robot ``LowState`` (LEG_ORDER ×
[hip,thigh,calf]); the base frame is REP-103 (x fwd, y left, z up), matching the
viewer's robot pose. If the model looks mirrored/rotated or limbs bend the wrong
way, tweak the sign mapping in ``cfg_from_q`` or the ``_base_to_z_up`` fixup — both
are isolated here so they can never crash the render loop.
"""

from __future__ import annotations

import logging
import os

import numpy as np

log = logging.getLogger("urdf-robot")

LEG_ORDER = ["FR", "FL", "RR", "RL"]   # matches kinematics.LEG_ORDER / q ordering

# Decimate each visual mesh at LOAD to ~this fraction of its faces (the robot is
# viewed from afar, so full detail is wasted and slows the per-frame transform/upload).
# 1.0 / <=0 disables. Precise via fast-simplification if installed; otherwise a
# dependency-free vertex-clustering fallback. Tune with URDF_KEEP.
KEEP_FRACTION = float(os.environ.get("URDF_KEEP", "0.06"))
# Uniform visual scale for the avatar. The mesh is true-metric; raise this only if
# the world/cloud is mis-scaled (low floor confidence) and you want the robot to
# match it visually. The proper fix is calibrating CAMERA_HEIGHT / the floor anchor.
ROBOT_SCALE = float(os.environ.get("URDF_SCALE", "1.0"))


class URDFRobot:
    def __init__(self, urdf_path: str):
        self.available = False
        self.n_vertices = 0
        self._urdf = None
        self._scene = None
        self._nodes = []                 # [(graph_node_name, local_verts (M,3) f64)]
        self._faces = np.zeros((0, 3), np.int32)
        self._joint_names = []
        self._name_for = {}              # (leg, kind) -> urdf joint name
        self._base_to_z_up = np.eye(4)   # optional fixup rotation (identity by default)
        self._posed_local = None         # verts in base_link frame, cached per joint cfg
        self._last_q = None
        if not urdf_path or not os.path.isfile(urdf_path):
            log.info(f"[URDF] no URDF at '{urdf_path}'; mesh avatar disabled (skeleton only).")
            return
        try:
            import yourdfpy  # noqa: F401
            import trimesh   # noqa: F401
        except Exception as e:
            log.info(f"[URDF] yourdfpy/trimesh not installed ({e}); mesh avatar disabled. "
                     f"Run: cd client && uv sync")
            return
        try:
            self._load(urdf_path)
            self.available = self.n_vertices > 0
            if self.available:
                raw = getattr(self, "_raw_vertices", self.n_vertices)
                log.info(f"[URDF] loaded '{os.path.basename(urdf_path)}': "
                         f"{len(self._joint_names)} joints, {raw}→{self.n_vertices} verts "
                         f"(decimated to {100.0*self.n_vertices/max(raw,1):.0f}%) — mesh ready.")
            else:
                log.warning("[URDF] loaded but no mesh geometry found; skeleton only.")
        except Exception as e:
            log.warning(f"[URDF] load failed ({e}); mesh avatar disabled (skeleton only).")

    def _load(self, urdf_path):
        import yourdfpy
        urdf_path = os.path.abspath(urdf_path)
        mesh_dir = os.path.dirname(urdf_path)
        # Package root = nearest ancestor with a package.xml (so package://<pkg>/meshes/…
        # resolves even when the URDF lives in a urdf/ subdir, as b2w_description does).
        pkg_root = mesh_dir
        for _ in range(5):
            if os.path.exists(os.path.join(pkg_root, "package.xml")):
                break
            parent = os.path.dirname(pkg_root)
            if parent == pkg_root:
                break
            pkg_root = parent

        def _resolve(fname: str) -> str:
            rel = fname
            if "://" in rel:                       # strip package://<pkg>/
                rel = rel.split("://", 1)[1]
                parts = rel.split("/", 1)
                rel = parts[1] if len(parts) == 2 else parts[0]
            if os.path.isabs(rel) and os.path.exists(rel):
                return rel
            for root in (pkg_root, mesh_dir, os.path.dirname(mesh_dir)):
                cand = os.path.join(root, rel)
                if os.path.exists(cand):
                    return cand
            return os.path.join(pkg_root, rel)     # let trimesh raise if truly missing

        self._urdf = yourdfpy.URDF.load(
            urdf_path, build_scene_graph=True, load_meshes=True,
            filename_handler=_resolve)
        self._joint_names = list(getattr(self._urdf, "actuated_joint_names", []))
        for leg in LEG_ORDER:
            for kind, kw in (("hip", "hip"), ("thigh", "thigh"), ("calf", "calf")):
                for jn in self._joint_names:
                    low = jn.lower()
                    if low.startswith(leg.lower()) and kw in low:
                        self._name_for[(leg, kind)] = jn
                        break

        # Precompute per-node LOCAL vertices + ONE static faces array (with offsets).
        self._scene = self._urdf.scene
        graph = self._scene.graph
        faces, nverts, raw = [], 0, 0
        for node in graph.nodes_geometry:
            _T, gname = graph.get(node)
            geom = self._scene.geometry.get(gname)
            if geom is None or len(geom.vertices) == 0:
                continue
            v = np.asarray(geom.vertices, dtype=np.float64)
            f = np.asarray(geom.faces, dtype=np.int64)
            raw += len(v)
            v, f = self._decimate(v, f)              # low-poly for the far view
            if len(f) == 0:
                continue
            self._nodes.append((node, v))
            faces.append(f + nverts)
            nverts += len(v)
        self._faces = (np.concatenate(faces).astype(np.int32)
                       if faces else np.zeros((0, 3), np.int32))
        self.n_vertices = nverts
        self._raw_vertices = raw

    def _decimate(self, v, f):
        """Reduce a mesh to ~KEEP_FRACTION of its faces. Precise quadric decimation
        via fast-simplification if available; otherwise dependency-free vertex
        clustering. Returns (v, f) unchanged if disabled or already tiny."""
        keep = KEEP_FRACTION
        if keep <= 0.0 or keep >= 1.0 or len(f) < 64:
            return v, f
        try:
            import fast_simplification as fs
            vv, ff = fs.simplify(v.astype(np.float32), f.astype(np.int32),
                                 target_reduction=float(1.0 - keep))
            vv = np.asarray(vv, np.float64)
            ff = np.asarray(ff, np.int64)
            if len(ff) >= 4:
                return vv, ff
        except Exception:
            pass
        return self._vertex_cluster(v, f, keep)

    @staticmethod
    def _vertex_cluster(v, f, keep):
        """Dependency-free decimation: snap vertices to a grid sized from the bbox to
        land near ``keep``, merge per cell (mean), remap faces, drop degenerates."""
        lo, hi = v.min(axis=0), v.max(axis=0)
        diag = float(np.linalg.norm(hi - lo)) or 1.0
        target_cells = max(int(round((keep * len(v)) ** (1.0 / 3.0))), 2)
        cell = diag / target_cells
        keys = np.floor((v - lo) / cell).astype(np.int64)
        uniq, inv = np.unique(keys, axis=0, return_inverse=True)
        new_v = np.zeros((len(uniq), 3), np.float64)
        np.add.at(new_v, inv, v)
        new_v /= np.bincount(inv, minlength=len(uniq))[:, None]
        nf = inv[f]
        good = (nf[:, 0] != nf[:, 1]) & (nf[:, 1] != nf[:, 2]) & (nf[:, 0] != nf[:, 2])
        return new_v, nf[good]

    def cfg_from_q(self, q12) -> dict:
        """yourdfpy joint config from the 12 leg angles (LEG_ORDER × [hip,thigh,calf]).
        Unmapped joints (wheels) keep their default."""
        cfg = {}
        for i, leg in enumerate(LEG_ORDER):
            for k, kind in enumerate(("hip", "thigh", "calf")):
                jn = self._name_for.get((leg, kind))
                if jn is not None and 3 * i + k < len(q12):
                    cfg[jn] = float(q12[3 * i + k])
        return cfg

    def world_geometry(self, base_R, base_t, q12):
        """→ (vertices (N,3) f32 in world, faces (M,3) int32) for the posed robot, or
        None on any failure. ``base_R`` (3x3) + ``base_t`` (3,) place base_link in the
        world (same pose the skeleton uses). Faces are constant; only verts move."""
        if not self.available:
            return None
        try:
            q = np.asarray(q12, dtype=np.float64).reshape(-1)
            # Joint FK (update_cfg + per-node transform) is the costly step (~25 ms),
            # but joints change slowly — cache the base-frame posed verts and only
            # recompute when q actually moves. The base placement below is ~2 ms and
            # runs every frame, so the avatar still tracks the pose at full rate.
            if (self._posed_local is None or self._last_q is None
                    or np.abs(q - self._last_q).max() > 1e-3):
                self._urdf.update_cfg(self.cfg_from_q(q12))
                graph = self._scene.graph
                local = np.empty((self.n_vertices, 3), dtype=np.float64)
                off = 0
                for node, v in self._nodes:
                    T, _g = graph.get(node)          # node → base_link, 4x4
                    n = v.shape[0]
                    local[off:off + n] = v @ T[:3, :3].T + T[:3, 3]
                    off += n
                self._posed_local = local
                self._last_q = q
            R = np.asarray(base_R, dtype=np.float64).reshape(3, 3) @ self._base_to_z_up[:3, :3]
            t = np.asarray(base_t, dtype=np.float64).reshape(3)
            return ((self._posed_local * ROBOT_SCALE) @ R.T + t).astype(np.float32), self._faces
        except Exception as e:
            if not getattr(self, "_geom_warned", False):
                log.warning(f"[URDF] geometry build failed, falling back to skeleton: {e}")
                self._geom_warned = True
            return None
