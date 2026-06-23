"""
Raster Image  ->  Machine-Ready DXF Converter
Pathan Steel | Sheet Metal Laser Cutting

Pipeline:
  1. OpenCV pre-processing (grayscale + blur + threshold/Otsu)
  2. Potrace high-accuracy vectorization (turdsize noise filter, alphamax corners)
  3. Real-world mm scaling engine (bounding-box -> exact mm, $INSUNITS=4)
  4. Enforced closed-loop LWPOLYLINE generation (ezdxf, color 1 / red, R2000)

NOTE ON TRACING ENGINE:
  This uses `import potrace`, which is provided by EITHER:
    - potracer   (pure Python, installs on Windows with no compiler)  <-- default
    - pypotrace  (C-binding, faster, needs Visual C++ Build Tools)
  The code is API-compatible with both. To switch to pypotrace, just install
  it instead of potracer (uninstall potracer first, they share the module name).
"""

import os
import tempfile

import cv2
import numpy as np
import streamlit as st

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import ezdxf

# ---- Tracing engine import (works with potracer OR pypotrace) ----
try:
    import potrace
    POTRACE_OK = True
    POTRACE_ERR = ""
except Exception as exc:  # pragma: no cover
    POTRACE_OK = False
    POTRACE_ERR = str(exc)


# ============================================================
#  GEOMETRY / TRACING HELPERS
# ============================================================

def resolve_turnpolicy():
    """Constant name differs between potracer and pypotrace; resolve safely."""
    for name in ("POTRACE_TURNPOLICY_MINORITY", "TURNPOLICY_MINORITY"):
        val = getattr(potrace, name, None)
        if val is not None:
            return val
    return 4  # POTRACE_TURNPOLICY_MINORITY value in the reference C library


def to_xy(point):
    """Normalize a potrace point (numpy array / tuple / object) to (x, y)."""
    if hasattr(point, "x") and hasattr(point, "y"):
        return float(point.x), float(point.y)
    return float(point[0]), float(point[1])


def cubic_bezier(p0, p1, p2, p3, steps):
    """Flatten a cubic Bezier into `steps` straight points (excludes start)."""
    pts = []
    for i in range(1, steps + 1):
        t = i / steps
        mt = 1.0 - t
        a = mt * mt * mt
        b = 3 * mt * mt * t
        c = 3 * mt * t * t
        d = t * t * t
        x = a * p0[0] + b * p1[0] + c * p2[0] + d * p3[0]
        y = a * p0[1] + b * p1[1] + c * p2[1] + d * p3[1]
        pts.append((x, y))
    return pts


def _segments(curve):
    return curve.segments if hasattr(curve, "segments") else list(curve)


def _curves(path):
    return path.curves if hasattr(path, "curves") else list(path)


def curve_to_polygon(curve, steps):
    """Convert one closed potrace curve into a flat list of (x, y) vertices."""
    start = to_xy(curve.start_point)
    poly = [start]
    current = start
    for seg in _segments(curve):
        end = to_xy(seg.end_point)
        if seg.is_corner:
            poly.append(to_xy(seg.c))
            poly.append(end)
        else:
            poly.extend(cubic_bezier(current, to_xy(seg.c1), to_xy(seg.c2), end, steps))
        current = end
    # potrace loops are closed (last point == first); drop duplicate for clean LWPOLYLINE
    if len(poly) > 1 and abs(poly[-1][0] - poly[0][0]) < 1e-9 and abs(poly[-1][1] - poly[0][1]) < 1e-9:
        poly.pop()
    return poly


def trace_bitmap(binary, turdsize, alphamax, steps):
    """Run potrace on a binary image and return a list of polygon loops."""
    data = np.ascontiguousarray(binary > 0)
    bmp = potrace.Bitmap(data)
    path = bmp.trace(int(turdsize), resolve_turnpolicy(), float(alphamax), 1, 0.2)
    polys = []
    for curve in _curves(path):
        poly = curve_to_polygon(curve, steps)
        if len(poly) >= 3:
            polys.append(poly)
    return polys


# ============================================================
#  IMAGE PRE-PROCESSING (Stage 1)
# ============================================================

def preprocess(img_bgr, blur_amount, use_otsu, threshold, light_shape):
    """Grayscale -> blur -> binary threshold. Returns (binary, used_threshold)."""
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    if blur_amount > 0:
        k = 2 * int(blur_amount) + 1  # force odd kernel
        gray = cv2.GaussianBlur(gray, (k, k), 0)

    # By default the design is the DARK region -> make it foreground (white) via INV.
    # If the shape is light on a dark background, the operator ticks "Invert".
    mode = cv2.THRESH_BINARY if light_shape else cv2.THRESH_BINARY_INV

    if use_otsu:
        used, binary = cv2.threshold(gray, 0, 255, mode + cv2.THRESH_OTSU)
    else:
        used, binary = cv2.threshold(gray, int(threshold), 255, mode)
    return binary, used


# ============================================================
#  mm SCALING ENGINE (Stage 3)  +  Y-AXIS INVERSION
# ============================================================

def transform_polys(polys, target_w, target_h, lock_aspect):
    """Map pixel-space loops into mm, with Y inverted for CAD orientation."""
    all_pts = [pt for poly in polys for pt in poly]
    xs = [p[0] for p in all_pts]
    ys = [p[1] for p in all_pts]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    w_px = (max_x - min_x) or 1.0
    h_px = (max_y - min_y) or 1.0

    if lock_aspect:
        scale = float(target_w) / w_px   # width drives, aspect preserved
        sx = sy = scale
    else:
        sx = float(target_w) / w_px
        sy = float(target_h) / h_px

    out = []
    for poly in polys:
        out.append([((px - min_x) * sx, (max_y - py) * sy) for (px, py) in poly])
    return out, w_px * sx, h_px * sy


# ============================================================
#  DXF GENERATION (Stage 4)
# ============================================================

def build_dxf_bytes(polys):
    """Write closed red LWPOLYLINEs to an R2000 DXF (mm) and return raw bytes."""
    doc = ezdxf.new("R2000")
    doc.header["$INSUNITS"] = 4   # 4 = millimeters (CypCut / RDWorks read this natively)

    try:
        doc.layers.add("CUT", color=1)            # newer ezdxf
    except Exception:
        doc.layers.new("CUT", dxfattribs={"color": 1})  # older ezdxf

    msp = doc.modelspace()
    for poly in polys:
        msp.add_lwpolyline(
            poly,
            close=True,  # mathematically closed -> part drops out of the sheet
            dxfattribs={"layer": "CUT", "color": 1},  # color 1 = Red cutting profile
        )

    fd, tmp_path = tempfile.mkstemp(suffix=".dxf")
    os.close(fd)
    try:
        doc.saveas(tmp_path)
        with open(tmp_path, "rb") as f:
            data = f.read()
    finally:
        try:
            os.unlink(tmp_path)
        except OSError:
            pass
    return data


# ============================================================
#  PREVIEW
# ============================================================

def render_preview(polys, out_w, out_h):
    fig, ax = plt.subplots(figsize=(6, 6))
    for poly in polys:
        xs = [p[0] for p in poly] + [poly[0][0]]
        ys = [p[1] for p in poly] + [poly[0][1]]
        ax.plot(xs, ys, color="#d62728", linewidth=1.1)
    ax.set_aspect("equal", adjustable="box")
    ax.set_title(f"Cutting Path  |  {out_w:.1f} x {out_h:.1f} mm")
    ax.set_xlabel("mm")
    ax.set_ylabel("mm")
    ax.grid(True, linestyle=":", alpha=0.4)
    fig.tight_layout()
    return fig


# ============================================================
#  STREAMLIT UI
# ============================================================

st.set_page_config(page_title="Raster -> DXF Laser Converter", layout="wide")
st.title("Raster Image  ->  Machine-Ready DXF Converter")
st.caption("Pathan Steel  |  Customer image se seedha CypCut / RDWorks / AutoCAD ready DXF")

if not POTRACE_OK:
    st.error(
        "Potrace engine load nahi hua. Terminal me chalao:  pip install potracer\n\n"
        f"Detail: {POTRACE_ERR}"
    )
    st.stop()

with st.sidebar:
    st.header("Controls")

    st.subheader("Real-world Size")
    target_w = st.number_input("Target Width (mm)", min_value=1.0, value=100.0, step=1.0)
    target_h = st.number_input("Target Height (mm)", min_value=1.0, value=100.0, step=1.0)
    lock_aspect = st.checkbox("Lock aspect ratio (recommended)", value=True,
                              help="ON rakho to shape distort nahi hogi; Width drive karega.")

    st.divider()
    st.subheader("Stage 1: Threshold")
    use_otsu = st.checkbox("Auto Threshold (Otsu)", value=False)
    threshold = st.slider("Threshold Value", 0, 255, 127, disabled=use_otsu)
    light_shape = st.checkbox("Invert (white shape on dark background)", value=False)
    blur_amount = st.slider("Smoothing / Blur", 0, 5, 1,
                            help="Pixelation aur micro-noise kam karta hai.")

    st.divider()
    st.subheader("Stage 2: Vectorize")
    turdsize = st.slider("Noise Filter (Turdsize)", 0, 100, 10,
                         help="Is pixel-area se chhote speck/island discard ho jaayenge.")
    alphamax = st.slider("Corner Smoothing (alphamax)", 0.0, 1.34, 1.0, 0.01)
    steps = st.slider("Curve Resolution", 4, 30, 12,
                      help="Zyada = smoother curves, zyada points.")

    st.divider()
    st.subheader("Performance")
    limit_res = st.checkbox("Limit resolution for speed", value=True,
                            help="Final mm size par koi asar nahi, sirf processing fast hoti hai.")
    max_side = st.slider("Max processing side (px)", 400, 3000, 1500, disabled=not limit_res)

uploaded = st.file_uploader(
    "Customer image upload karo (JPG / PNG / WhatsApp photo / screenshot)",
    type=["jpg", "jpeg", "png", "bmp", "webp"],
)

if uploaded is None:
    st.info("Upload karo ek image taaki processing shuru ho.")
    st.stop()

# Decode upload
file_bytes = np.frombuffer(uploaded.read(), np.uint8)
img_bgr = cv2.imdecode(file_bytes, cv2.IMREAD_COLOR)
if img_bgr is None:
    st.error("Image read nahi hui. Koi dusri file try karo.")
    st.stop()

# Optional downscale for tracing speed (does NOT affect final mm size)
if limit_res:
    h, w = img_bgr.shape[:2]
    longest = max(h, w)
    if longest > max_side:
        scale = max_side / float(longest)
        img_bgr = cv2.resize(img_bgr, (int(w * scale), int(h * scale)),
                             interpolation=cv2.INTER_AREA)

# Stage 1
binary, used_thresh = preprocess(img_bgr, blur_amount, use_otsu, threshold, light_shape)

# Stage 2
try:
    polys = trace_bitmap(binary, turdsize, alphamax, steps)
except Exception as exc:
    st.error(f"Tracing error: {exc}")
    st.stop()

# Two-column layout
col1, col2 = st.columns(2)

with col1:
    st.subheader("Original")
    st.image(cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB), use_container_width=True)
    with st.expander("Processed binary (yahi trace hua)"):
        st.image(binary, use_container_width=True, clamp=True)

with col2:
    st.subheader("Cutting Path Preview")
    if not polys:
        st.warning(
            "Koi closed shape detect nahi hua.\n"
            "- Threshold slider adjust karo\n"
            "- 'Invert' option toggle karo (agar shape light hai)\n"
            "- Turdsize kam karo"
        )
    else:
        # Stage 3 + Y inversion
        tpolys, out_w, out_h = transform_polys(polys, target_w, target_h, lock_aspect)

        fig = render_preview(tpolys, out_w, out_h)
        st.pyplot(fig, use_container_width=True)
        plt.close(fig)

        # Stage 4
        dxf_bytes = build_dxf_bytes(tpolys)

        m1, m2, m3 = st.columns(3)
        m1.metric("Cut Loops", len(tpolys))
        m2.metric("Width (mm)", f"{out_w:.1f}")
        m3.metric("Height (mm)", f"{out_h:.1f}")
        if use_otsu:
            st.caption(f"Otsu auto threshold = {used_thresh:.0f}")
        if lock_aspect and abs(out_h - target_h) > 0.1:
            st.caption("Aspect locked: height ko width ke hisaab se auto-set kiya gaya hai.")

        out_name = os.path.splitext(uploaded.name)[0] + "_laser.dxf"
        st.download_button(
            "Download Machine-Ready DXF",
            data=dxf_bytes,
            file_name=out_name,
            mime="application/dxf",
            type="primary",
            use_container_width=True,
        )

st.divider()
st.caption(
    "Tip: Kerf compensation / lead-in CAM (CypCut) me lagana — yahan se clean closed "
    "vector profile milta hai. DXF units = mm, layer = CUT, color = Red (1)."
)