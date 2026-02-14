"""
Phasing Coil Former Generator
- Features independent Coil/Rib diameters and straight wire tunnels.
- Implements a 2mm internal Z-buffer to ensure clean circular exit holes.
- Automatically creates and uses an 'outputs' directory for generated files.
"""

import cadquery as cq
import math
import os

# ==========================================
# CONFIGURATION / VARIABLES SECTION
# ==========================================
FILENAME      = "phasing_coil_former" # Extension .step is added automatically
ENABLE_RIBS   = True                  # Toggle friction ribs on/off
CHAMFER_SIZE  = 1.0                   # Size of chamfer on top/bottom ends
WIRE_LEN      = 668.0                 # Target wire length in mm
WIRE_DIAM     = 3.2                   # Core wire diameter
PVC_ID        = 22.3                  # Inside diameter of PVC pipe
COIL_DIAMETER = 15.0                  # Desired coil diameter (capped for safety)
PITCH         = 8.9                   # Vertical distance per turn
END_BUFFER    = 12.0                  # Space for ribs and wire transitions
TUNNEL_TOL    = 0.2                   # Diameter clearance for wire tunnels
OUTPUT_DIR    = "outputs"             # Directory for generated files
# ==========================================

def build_coil_former():
    # ── Safety Validation ────────────────────────────────────
    max_safe_diam = PVC_ID - WIRE_DIAM
    actual_coil_diam = min(COIL_DIAMETER, max_safe_diam)
    
    if actual_coil_diam < COIL_DIAMETER:
        print(f"⚠️ Warning: COIL_DIAMETER capped at {actual_coil_diam:.2f}mm for PVC clearance.")

    # ── Derived Dimensions ───────────────────────────────────
    cylinder_r    = actual_coil_diam / 2.0
    rib_diam      = PVC_ID
    center_bore_r = (WIRE_DIAM + 0.2) / 2.0
    r_wire        = WIRE_DIAM / 2.0
    r_tunnel      = r_wire + (TUNNEL_TOL / 2.0)

    circumference = math.pi * actual_coil_diam
    base_turns    = math.sqrt(WIRE_LEN ** 2 / (circumference ** 2 + PITCH ** 2))
    
    extension_turns = (WIRE_DIAM) / circumference
    total_turns     = base_turns + extension_turns
    
    wh            = total_turns * PITCH
    total_height  = wh + (2 * END_BUFFER)

    start_z   = END_BUFFER
    end_z     = start_z + wh
    v_depth   = r_wire * math.sqrt(2)

    info = dict(turns=total_turns, height=total_height, final_diam=actual_coil_diam)

    # ── Main Body ────────────────────────────────────────────
    result = cq.Workplane("XY").circle(cylinder_r).extrude(total_height)

    # ── Chamfer Ends ─────────────────────────────────────────
    if CHAMFER_SIZE > 0:
        result = result.edges(">Z or <Z").chamfer(CHAMFER_SIZE)

    # ── Friction Ribs (Conditional) ──────────────────────────
    if ENABLE_RIBS:
        def _add_rib(z_pos):
            rib_height_diff = (rib_diam - actual_coil_diam) / 2.0
            return (
                cq.Workplane("XZ")
                .moveTo(cylinder_r, z_pos)
                .lineTo(rib_diam / 2, z_pos + rib_height_diff)
                .lineTo(rib_diam / 2, z_pos + rib_height_diff + 2)
                .lineTo(cylinder_r, z_pos + 2 * rib_height_diff + 2)
                .close()
                .revolve(360, (0, 0), (0, 1))
            )
        result = result.union(_add_rib(2.0))
        result = result.union(_add_rib(total_height - 6.0 - ((rib_diam - actual_coil_diam) / 2.0)))

    # ── V-Groove Helix ───────────────────────────────────────
    helix = cq.Wire.makeHelix(PITCH, wh, cylinder_r, lefthand=True)
    
    tan_y, tan_z = -2.0 * math.pi * cylinder_r, PITCH
    tan_len = math.hypot(tan_y, tan_z)
    profile_plane = cq.Plane(
        origin=cq.Vector(cylinder_r, 0, 0),
        xDir=cq.Vector(1, 0, 0),
        normal=cq.Vector(0, tan_y / tan_len, tan_z / tan_len),
    )

    oc = 1.0
    groove = (
        cq.Workplane(profile_plane)
        .moveTo(-v_depth, 0)
        .lineTo(oc, v_depth + oc)
        .lineTo(oc, -(v_depth + oc))
        .close()
        .sweep(helix, isFrenet=True)
        .translate(cq.Vector(0, 0, start_z))
    )
    result = result.cut(groove)

    # ── Straight Entry / Exit Tunnels ────────────────────────
    def _straight_tunnel(z_level, angle, is_top=False):
        # 2mm buffer ensures the tunnel meets the bore internally
        z_internal_buffer = 2.0
        z_exit = (total_height - z_internal_buffer) if is_top else z_internal_buffer
        
        p1 = cq.Vector(cylinder_r * math.cos(angle), cylinder_r * math.sin(angle), z_level)
        p2 = cq.Vector(0, 0, z_exit)
        
        path = cq.Wire.assembleEdges([cq.Edge.makeLine(p1, p2)])
        tunnel_plane = cq.Plane(origin=p1, normal=p1.sub(p2))
        
        return (
            cq.Workplane(tunnel_plane)
            .circle(r_tunnel)
            .sweep(cq.Workplane().newObject([path]))
        )

    angle_offset = (extension_turns / 2.0) * 2.0 * math.pi
    
    result = result.cut(_straight_tunnel(start_z + (WIRE_DIAM/2), -angle_offset, False))
    result = result.cut(_straight_tunnel(end_z - (WIRE_DIAM/2), (-2.0 * math.pi * total_turns) + angle_offset, True))

    # ── Center Bore ──────────────────────────────────────────
    bore = cq.Workplane("XY").circle(center_bore_r).extrude(total_height)
    result = result.cut(bore)

    return result, info

# ── Execution and Export ─────────────────────────────────────
if __name__ == "__main__":
    result, info = build_coil_former()
    
    # Path handling: Create output directory if it doesn't exist
    script_dir = os.path.dirname(__file__)
    abs_output_dir = os.path.join(script_dir, OUTPUT_DIR)
    
    if not os.path.exists(abs_output_dir):
        os.makedirs(abs_output_dir)
        print(f"📁 Created directory: {abs_output_dir}")

    export_name = f"{FILENAME}.step"
    output_path = os.path.join(abs_output_dir, export_name)
    
    cq.exporters.export(result, output_path)
    
    print("=" * 45)
    print(f"✅ Success! File saved to: {os.path.join(OUTPUT_DIR, export_name)}")
    print(f"   Final Coil Diam : {info['final_diam']:.2f} mm")
    print(f"   Total Height    : {info['height']:.2f} mm")
    print("=" * 45)