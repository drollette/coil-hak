"""
Phasing Coil Former Generator – CadQuery Port
Original OpenSCAD design by W7HAK

Generates a cylindrical coil former with a precision V-groove helix
for guiding wire placement.  Sized to friction-fit inside standard PVC pipe.
"""

import cadquery as cq
import math

# ==========================================================
#  USER INPUTS  (edit these to match your wire and pipe)
# ==========================================================
wire_len       = 668.0    # target wire length in mm
wire_diam      = 3.2      # wire diameter in mm (3.2 = RG58 core, 1.6 = bare)
pvc_inner_diam = 22.3     # inside diameter of your PVC pipe in mm
pitch          = 8.9      # vertical spacing between wraps in mm
rib_clearance  = 8.0      # solid space at top & bottom for friction ribs

# ==========================================================
#  DERIVED DIMENSIONS  (automatic – do not edit)
# ==========================================================
rib_diam       = pvc_inner_diam
cylinder_diam  = rib_diam - wire_diam
cylinder_r     = cylinder_diam / 2.0
center_bore_r  = (wire_diam + 0.2) / 2.0
r_wire         = wire_diam / 2.0

circumference  = math.pi * cylinder_diam
calc_turns     = math.sqrt(wire_len ** 2 /
                           (circumference ** 2 + pitch ** 2))
wh             = calc_turns * pitch           # winding height
total_height   = wh + 2 * rib_clearance

start_z    = rib_clearance
end_z      = start_z + wh
chamfer_h  = (rib_diam - cylinder_diam) / 2.0
v_depth    = r_wire * math.sqrt(2)            # 90° V cradles 50 % of wire

print("=" * 40)
print(f"  Wire length : {wire_len} mm")
print(f"  Wire diam   : {wire_diam} mm")
print(f"  PVC inner   : {pvc_inner_diam} mm")
print(f"  Pitch       : {pitch} mm")
print(f"  Turns       : {calc_turns:.2f}")
print(f"  Coil height : {wh:.2f} mm")
print(f"  Total height: {total_height:.2f} mm")
print("=" * 40)

# ==========================================================
#  MAIN BODY
# ==========================================================
body = cq.Workplane("XY").circle(cylinder_r).extrude(total_height)

# -- Bottom friction rib (starts at z = 2 mm) ----------------
bot_z = 2.0
bottom_rib = (
    cq.Workplane("XZ")
    .moveTo(cylinder_r, bot_z)
    .lineTo(rib_diam / 2, bot_z + chamfer_h)
    .lineTo(rib_diam / 2, bot_z + chamfer_h + 2)
    .lineTo(cylinder_r, bot_z + 2 * chamfer_h + 2)
    .close()
    .revolve(360, (0, 0), (0, 1))
)

# -- Top friction rib (starts at z = total_height − 8 mm) ----
top_z = total_height - 8.0
top_rib = (
    cq.Workplane("XZ")
    .moveTo(cylinder_r, top_z)
    .lineTo(rib_diam / 2, top_z + chamfer_h)
    .lineTo(rib_diam / 2, top_z + chamfer_h + 2)
    .lineTo(cylinder_r, top_z + 2 * chamfer_h + 2)
    .close()
    .revolve(360, (0, 0), (0, 1))
)

result = body.union(bottom_rib).union(top_rib)

# -- Centre bore -----------------------------------------------
bore = cq.Workplane("XY").circle(center_bore_r).extrude(total_height)
result = result.cut(bore)

# ==========================================================
#  V-GROOVE HELIX
# ==========================================================
# Build the helix at the origin, sweep the V-profile, then shift up.
helix = cq.Wire.makeHelix(pitch, wh, cylinder_r, lefthand=True)

# Tangent at the helix start point (cylinder_r, 0, 0).
# Left-hand helix moves in −Y and +Z at t = 0.
tan_y   = -2.0 * math.pi * cylinder_r
tan_z   = pitch
tan_len = math.hypot(tan_y, tan_z)
tangent = cq.Vector(0, tan_y / tan_len, tan_z / tan_len)

# Workplane at helix start, perpendicular to the tangent.
# xDir = radial-outward (1, 0, 0) which is already ⊥ tangent.
profile_plane = cq.Plane(
    origin=cq.Vector(cylinder_r, 0, 0),
    xDir=cq.Vector(1, 0, 0),
    normal=tangent,
)

# 90° V-groove triangle – tip inward, opening past the surface.
oc = 1.0                                    # overshoot for clean boolean
groove = (
    cq.Workplane(profile_plane)
    .moveTo(-v_depth, 0)                    # V tip inside the wall
    .lineTo(oc, v_depth + oc)               # upper-outer corner
    .lineTo(oc, -(v_depth + oc))            # lower-outer corner
    .close()
    .sweep(helix, isFrenet=True)
    .translate(cq.Vector(0, 0, start_z))    # shift to winding zone
)
result = result.cut(groove)

# ==========================================================
#  ENTRY / EXIT TUNNELS
# ==========================================================
def _make_tunnel(p_from, p_to):
    """Cylindrical tunnel of wire-radius between two 3-D points."""
    direction = p_to - p_from
    solid = cq.Solid.makeCylinder(
        r_wire, direction.Length, pnt=p_from, dir=direction
    )
    return cq.Workplane("XY").newObject([solid])

# Entry: from below the winding zone on the axis → helix start on the surface
result = result.cut(
    _make_tunnel(cq.Vector(0, 0, start_z - 4),
                 cq.Vector(cylinder_r, 0, start_z))
)

# Exit: from the helix end on the surface → above the winding zone on the axis
end_angle = -2.0 * math.pi * calc_turns
result = result.cut(
    _make_tunnel(
        cq.Vector(cylinder_r * math.cos(end_angle),
                  cylinder_r * math.sin(end_angle), end_z),
        cq.Vector(0, 0, end_z + 4),
    )
)

# ==========================================================
#  DISPLAY / EXPORT
# ==========================================================
# In CQ-editor the name `result` is picked up automatically.
# For STL export:  result.val().exportStl("phasing_coil.stl")
