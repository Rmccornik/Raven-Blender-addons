# ============================================================
#  Trajectory Stabilizer — Blender 5.1 Addon
#  Uruchomienie: Text Editor → Run Script
# ============================================================
bl_info = {
    "name":        "Trajectory Stabilizer",
    "author":      "Perplexity AI",
    "version":     (1, 3, 0),
    "blender":     (5, 1, 0),
    "location":    "3D Viewport > N-panel > Traj. Stab.",
    "description": "Stabilizacja ścieżki trajektorii LiDAR/IMU",
    "category":    "Import-Export",
}

import bpy
import os
import numpy as np
from bpy.props import (StringProperty, FloatProperty,
                       IntProperty, BoolProperty, PointerProperty)
from bpy.types import Operator, Panel, PropertyGroup
from bpy_extras.io_utils import ImportHelper


# ─── ALGORYTM ─────────────────────────────────────────────────────────────────

def _load(path):
    ts, pos, quat = [], [], []
    with open(path, "r") as fh:
        for line in fh:
            line = line.strip()
            if line.startswith("#") or not line:
                continue
            parts = line.split()
            if len(parts) != 8:
                continue
            v = list(map(float, parts))
            ts.append(v[0])
            pos.append(v[1:4])
            quat.append(v[4:8])
    return np.array(ts), np.array(pos), np.array(quat)


def _save(path, ts, pos, quat):
    with open(path, "w") as fh:
        fh.write("# timestamp tx ty tz qx qy qz qw
")
        for i in range(len(ts)):
            fh.write(
                f"{ts[i]:.6f} "
                f"{pos[i,0]:.6f} {pos[i,1]:.6f} {pos[i,2]:.6f} "
                f"{quat[i,0]:.6f} {quat[i,1]:.6f} "
                f"{quat[i,2]:.6f} {quat[i,3]:.6f}
"
            )


def _detect_jumps(pos, ts, sigma):
    diffs = np.diff(pos, axis=0)
    dt    = np.diff(ts)
    vel   = np.linalg.norm(diffs, axis=1) / dt
    med   = np.median(vel)
    std   = np.std(vel)
    thr   = med + sigma * std
    return np.where(vel > thr)[0] + 1, thr


def _interpolate_jumps(pos, ts, idx):
    out = pos.copy()
    n   = len(out)
    if len(idx) == 0:
        return out
    segs, seg = [], [idx[0]]
    for k in idx[1:]:
        if k == seg[-1] + 1:
            seg.append(k)
        else:
            segs.append(seg)
            seg = [k]
    segs.append(seg)
    for seg in segs:
        s = max(0, seg[0] - 1)
        e = min(n - 1, seg[-1] + 1)
        span = ts[e] - ts[s]
        for i in seg:
            a = (ts[i] - ts[s]) / span if span > 0 else 0.5
            out[i] = (1 - a) * out[s] + a * out[e]
    return out


def _smooth_pos(pos, ts, window, use_spline):
    from scipy.ndimage import uniform_filter1d
    sm = np.zeros_like(pos)
    for ax in range(3):
        sm[:, ax] = uniform_filter1d(pos[:, ax], size=window, mode="nearest")
    if use_spline:
        from scipy.interpolate import CubicSpline
        n    = len(ts)
        step = max(4, window)
        kidx = np.arange(0, n, step)
        if kidx[-1] != n - 1:
            kidx = np.append(kidx, n - 1)
        out = np.zeros_like(pos)
        for ax in range(3):
            cs = CubicSpline(ts[kidx], sm[kidx, ax], bc_type="not-a-knot")
            out[:, ax] = cs(ts)
        return out
    return sm


def _smooth_quat(quat, window):
    n    = len(quat)
    q    = quat / np.linalg.norm(quat, axis=1, keepdims=True)
    for i in range(1, n):
        if np.dot(q[i], q[i - 1]) < 0:
            q[i] = -q[i]
    out  = np.zeros_like(q)
    half = window // 2
    for i in range(n):
        chunk  = q[max(0, i - half):min(n, i + half + 1)]
        mq     = chunk[0].copy()
        for _ in range(5):
            dots   = chunk @ mq
            algn   = np.where(dots[:, None] >= 0, chunk, -chunk)
            mq     = algn.mean(axis=0)
            mq    /= np.linalg.norm(mq)
        out[i] = mq
    return out


def run_stabilization(props):
    path = bpy.path.abspath(props.filepath)
    if not os.path.isfile(path):
        raise FileNotFoundError(f"Nie znaleziono: {path}")
    ts, pos, quat = _load(path)
    if len(ts) < 10:
        raise ValueError(f"Za mało ramek: {len(ts)}")

    jump_idx, thr = _detect_jumps(pos, ts, props.jump_sigma)
    pos2  = _interpolate_jumps(pos, ts, jump_idx)
    pos3  = _smooth_pos(pos2, ts, props.smooth_pos_window, props.use_spline)
    quat2 = _smooth_quat(quat, props.smooth_quat_window)

    base, ext = os.path.splitext(path)
    out = base + props.output_suffix + ext
    _save(out, ts, pos3, quat2)

    def mj(p): return np.max(np.linalg.norm(np.diff(p, axis=0), axis=1)) * 100
    def md(q):
        a = [2*np.arccos(np.clip(abs(np.dot(q[i], q[i+1])), 0, 1))*180/np.pi
             for i in range(len(q)-1)]
        return np.max(a)

    rep = (
        f"Ramki:        {len(ts)}
"
        f"Czas:         {ts[-1]-ts[0]:.2f} s
"
        f"Uskoki:       {len(jump_idx)}
"
        f"Prog [cm/s]:  {thr*100:.2f}
"
        f"Max XYZ:      {mj(pos):.3f} cm -> {mj(pos3):.3f} cm
"
        f"Max kat:      {md(quat):.3f}  -> {md(quat2):.3f} deg
"
        f"Zapisano:     {os.path.basename(out)}"
    )
    return out, rep


# ─── PROPERTY GROUP ───────────────────────────────────────────────────────────

class TRJ_PG_Props(PropertyGroup):
    filepath: StringProperty(
        name="",
        description="Plik TXT z trajektoria",
        default="",
        subtype="FILE_PATH",
    )
    jump_sigma: FloatProperty(
        name="Jump sigma",
        description="Prog detekcji uskoków (median + N*sigma). Mniej = wiecej korekcji",
        default=4.0, min=1.0, max=10.0, step=10, precision=1,
    )
    smooth_pos_window: IntProperty(
        name="Okno XYZ",
        description="Szerokosc okna wygładzania translacji (ramki, nieparzyste)",
        default=7, min=3, max=51, step=2,
    )
    smooth_quat_window: IntProperty(
        name="Okno Quat",
        description="Szerokosc okna wygładzania orientacji kwaternionowej",
        default=5, min=3, max=51, step=2,
    )
    use_spline: BoolProperty(
        name="Splajn szescenny",
        description="Dopasuj splajn szescienny po filtrze usredniajacym",
        default=True,
    )
    output_suffix: StringProperty(
        name="Sufiks",
        description="Sufiks pliku wyjsciowego",
        default="_stabilized",
    )
    report_text: StringProperty(default="")


# ─── OPERATOR: BROWSE ─────────────────────────────────────────────────────────

class TRJ_OT_Browse(Operator, ImportHelper):
    bl_idname      = "trj.browse"
    bl_label       = "Wybierz plik"
    bl_description = "Otworz okno wyboru pliku TXT"
    filter_glob: StringProperty(default="*.txt;*.csv", options={"HIDDEN"})

    def execute(self, context):
        context.scene.trj_props.filepath = self.filepath
        return {"FINISHED"}


# ─── OPERATOR: RUN ────────────────────────────────────────────────────────────

class TRJ_OT_Run(Operator):
    bl_idname      = "trj.run"
    bl_label       = "Stabilizuj trajektorie"
    bl_description = "Uruchom korekcje uskoków i wygładzanie"

    def execute(self, context):
        props = context.scene.trj_props
        try:
            out, rep = run_stabilization(props)
            props.report_text = rep
            self.report({"INFO"}, f"OK -> {out}")
        except Exception as exc:
            props.report_text = f"BLAD:
{exc}"
            self.report({"ERROR"}, str(exc))
        return {"FINISHED"}


# ─── PANEL ────────────────────────────────────────────────────────────────────

class TRJ_PT_Panel(Panel):
    bl_label       = "Trajectory Stabilizer"
    bl_idname      = "TRJ_PT_Panel"
    bl_space_type  = "VIEW_3D"
    bl_region_type = "UI"
    bl_category    = "Traj. Stab."
    bl_options     = set()          # BRAK "DEFAULT_CLOSED" — panel zawsze otwarty

    def draw(self, context):
        layout = context.layout
        props  = context.scene.trj_props

        # --- PLIK ---
        box = layout.box()
        box.label(text="Plik trajektorii", icon="FILE_TEXT")
        row = box.row(align=True)
        row.prop(props, "filepath")
        row.operator("trj.browse", text="", icon="FILEBROWSER")

        # --- PARAMETRY ---
        box = layout.box()
        box.label(text="Parametry", icon="SETTINGS")
        col = box.column(align=True)
        col.prop(props, "jump_sigma",         slider=True)
        col.prop(props, "smooth_pos_window",  slider=True)
        col.prop(props, "smooth_quat_window", slider=True)
        col.separator()
        col.prop(props, "use_spline")

        # --- WYJSCIE ---
        box = layout.box()
        box.label(text="Plik wyjsciowy", icon="EXPORT")
        box.prop(props, "output_suffix")

        # --- PRZYCISK ---
        layout.separator()
        row = layout.row()
        row.scale_y = 1.8
        row.operator("trj.run", icon="PLAY")

        # --- RAPORT ---
        if props.report_text:
            box = layout.box()
            box.label(text="Raport", icon="INFO")
            for line in props.report_text.split("
"):
                row = box.row()
                row.label(text=line)


# ─── REJESTRACJA ──────────────────────────────────────────────────────────────

CLASSES = [
    TRJ_PG_Props,
    TRJ_OT_Browse,
    TRJ_OT_Run,
    TRJ_PT_Panel,
]


def register():
    for cls in CLASSES:
        bpy.utils.register_class(cls)
    bpy.types.Scene.trj_props = PointerProperty(type=TRJ_PG_Props)
    print("[TRJ] Trajectory Stabilizer zarejestrowany — N-panel > Traj. Stab.")


def unregister():
    for cls in reversed(CLASSES):
        bpy.utils.unregister_class(cls)
    del bpy.types.Scene.trj_props
    print("[TRJ] Trajectory Stabilizer wyrejestrowany.")


if __name__ == "__main__":
    # Bezpieczne ponowne uruchomienie z Text Editora:
    # wyrejestruj stara wersje jesli istnieje
    try:
        unregister()
    except Exception:
        pass
    register()
