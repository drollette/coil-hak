//! Core coil-former geometry engine.
//!
//! Ported to match the Python CadQuery implementation:
//! - Uses straight wire tunnels to prevent hole deformation.
//! - Implements a 2mm internal Z-buffer for clean exit holes.
//! - Includes safety diameter validation for PVC clearance.

use std::f64::consts::PI;

/// User-facing parameters that fully define a coil former.
#[derive(Debug, Clone, Copy)]
pub struct CoilParams {
    /// Target wire length in mm (electrical phase length).
    pub wire_len: f64,
    /// Wire diameter in mm.
    pub wire_diam: f64,
    /// Inside diameter of the PVC pipe in mm.
    pub pvc_inner_diam: f64,
    /// Vertical spacing between wraps in mm.
    pub pitch: f64,
    /// Solid space at top & bottom for friction ribs in mm.
    pub rib_clearance: f64,
    /// Centre bore diameter in mm. Pass 0 for auto-sizing.
    pub center_bore_diam: f64,
}

impl Default for CoilParams {
    fn default() -> Self {
        Self {
            wire_len: 90.83,
            wire_diam: 3.5,
            pvc_inner_diam: 23.5,
            pitch: 10.0,
            rib_clearance: 10.0,
            center_bore_diam: 0.0,
        }
    }
}

/// Computed dimensions derived from CoilParams.
#[derive(Debug, Clone, Copy)]
pub struct CoilDerived {
    pub rib_diam: f64,
    pub cylinder_diam: f64,
    pub cylinder_r: f64,
    pub center_bore_r: f64,
    pub r_wire: f64,
    pub calc_turns: f64,
    pub winding_height: f64,
    pub total_height: f64,
    pub start_z: f64,
    pub end_z: f64,
    pub chamfer_h: f64,
    pub v_depth: f64,
    pub pitch: f64,
    pub entry_angle: f64,
    pub exit_angle: f64,
    pub channel_extension: f64,
}

impl CoilDerived {
    pub fn from_params(p: &CoilParams) -> Self {
        let rib_diam = p.pvc_inner_diam;

        // Safety Validation: Ensure coil_diameter <= PVC_ID - WIRE_DIAM
        let max_safe_diam = p.pvc_inner_diam - p.wire_diam;
        let cylinder_diam = max_safe_diam; // Defaulting to max safe diameter

        let cylinder_r = cylinder_diam / 2.0;
        let center_bore_diam = if p.center_bore_diam > 0.0 {
            p.center_bore_diam
        } else {
            p.wire_diam + 0.2
        };
        let center_bore_r = center_bore_diam / 2.0;
        let r_wire = p.wire_diam / 2.0;

        let circumference = PI * cylinder_diam;
        let base_turns = (p.wire_len.powi(2) / (circumference.powi(2) + p.pitch.powi(2))).sqrt();

        // Extension for overlap (0.5 wire diameter at each end)
        let extension_turns = p.wire_diam / circumference;
        let total_turns = base_turns + extension_turns;

        let winding_height = total_turns * p.pitch;
        let total_height = winding_height + 2.0 * p.rib_clearance;

        let start_z = p.rib_clearance;
        let end_z = start_z + winding_height;
        let chamfer_h = (rib_diam - cylinder_diam) / 2.0;
        let v_depth = r_wire * 2.0_f64.sqrt();

        Self {
            rib_diam,
            cylinder_diam,
            cylinder_r,
            center_bore_r,
            r_wire,
            calc_turns: total_turns,
            winding_height,
            total_height,
            start_z,
            end_z,
            chamfer_h,
            v_depth,
            pitch: p.pitch,
            entry_angle: 0.0,
            exit_angle: -2.0 * PI * total_turns,
            channel_extension: 2.0, // 2mm internal buffer for clean exit holes
        }
    }
}

// ─── Mesh primitives ───────────────────────────────────────────────

#[derive(Debug, Clone)]
pub struct TriMesh {
    pub positions: Vec<f32>,
    pub indices: Vec<u32>,
}

impl TriMesh {
    pub fn new() -> Self {
        Self {
            positions: Vec::new(),
            indices: Vec::new(),
        }
    }
}

// ─── Transform helpers ─────────────────────────────────────────────

fn normalize(v: [f64; 3]) -> [f64; 3] {
    let len = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if len < 1e-12 {
        [0.0, 0.0, 1.0]
    } else {
        [v[0] / len, v[1] / len, v[2] / len]
    }
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

// ─── Continuous wire path geometry ────────────────────────────────

#[derive(Debug, Clone, Copy)]
struct PathPoint {
    position: [f64; 3],
    tangent: [f64; 3],
    #[allow(dead_code)]
    distance: f64,
}

fn helix_tangent(d: &CoilDerived, t: f64) -> [f64; 3] {
    let theta = d.entry_angle - 2.0 * PI * d.calc_turns * t;
    let omega = -2.0 * PI * d.calc_turns;
    [
        -d.cylinder_r * theta.sin() * omega,
        d.cylinder_r * theta.cos() * omega,
        d.winding_height,
    ]
}

fn helix_pos(d: &CoilDerived, t: f64) -> [f64; 3] {
    let theta = d.entry_angle - 2.0 * PI * d.calc_turns * t;
    [
        d.cylinder_r * theta.cos(),
        d.cylinder_r * theta.sin(),
        d.start_z + t * d.winding_height,
    ]
}

/// Generates a straight tunnel path from the helix start to the center bore
fn generate_straight_tunnel(p_start: [f64; 3], p_end: [f64; 3]) -> Vec<PathPoint> {
    let mut points = Vec::new();
    let segments = 10;
    let tangent = normalize([
        p_end[0] - p_start[0],
        p_end[1] - p_start[1],
        p_end[2] - p_start[2],
    ]);
    for i in 0..=segments {
        let t = i as f64 / segments as f64;
        points.push(PathPoint {
            position: [
                p_start[0] + t * (p_end[0] - p_start[0]),
                p_start[1] + t * (p_end[1] - p_start[1]),
                p_start[2] + t * (p_end[2] - p_start[2]),
            ],
            tangent,
            distance: t,
        });
    }
    points
}

fn generate_wire_path(d: &CoilDerived) -> Vec<PathPoint> {
    let mut full_path = Vec::new();

    // 1. Entry Tunnel: Helix start (surface) to internal 2mm buffer
    let entry_start = [d.cylinder_r, 0.0, d.start_z + d.r_wire];
    let entry_end = [0.0, 0.0, d.channel_extension];
    full_path.extend(generate_straight_tunnel(entry_start, entry_end));

    // 2. Helical Winding
    let n_helix = 400;
    for i in 0..=n_helix {
        let t = i as f64 / n_helix as f64;
        full_path.push(PathPoint {
            position: helix_pos(d, t),
            tangent: normalize(helix_tangent(d, t)),
            distance: t,
        });
    }

    // 3. Exit Tunnel: Helix end (surface) to H-2mm internal buffer
    let exit_angle = -2.0 * PI * d.calc_turns;
    let exit_start = [
        d.cylinder_r * exit_angle.cos(),
        d.cylinder_r * exit_angle.sin(),
        d.end_z - d.r_wire,
    ];
    let exit_end = [0.0, 0.0, d.total_height - d.channel_extension];
    full_path.extend(generate_straight_tunnel(exit_start, exit_end));

    full_path
}

// ─── SDF and Radius Logic ──────────────────────────────────────────

fn distance_to_wire_path(d: &CoilDerived, path: &[PathPoint], point: [f64; 3]) -> f64 {
    let wire_r = d.r_wire * 1.05; // Overlap for clean cut
    let mut min_dist = f64::INFINITY;

    for window in path.windows(2) {
        let p0 = window[0].position;
        let p1 = window[1].position;
        let edge = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
        let edge_len_sq = edge[0] * edge[0] + edge[1] * edge[1] + edge[2] * edge[2];
        if edge_len_sq < 1e-12 {
            continue;
        }
        let to_point = [point[0] - p0[0], point[1] - p0[1], point[2] - p0[2]];
        let t = (dot(to_point, edge) / edge_len_sq).clamp(0.0, 1.0);
        let closest = [
            p0[0] + t * edge[0],
            p0[1] + t * edge[1],
            p0[2] + t * edge[2],
        ];
        let dx = point[0] - closest[0];
        let dy = point[1] - closest[1];
        let dz = point[2] - closest[2];
        min_dist = min_dist.min((dx * dx + dy * dy + dz * dz).sqrt());
    }
    min_dist - wire_r
}

fn rib_radius_at(d: &CoilDerived, z: f64) -> f64 {
    let cyl_r = d.cylinder_r;
    let rib_r = d.rib_diam / 2.0;

    // Bottom rib
    let bs = 2.0;
    let bc = bs + d.chamfer_h;
    let bf = bc + 2.0;
    let be = bf + d.chamfer_h;
    if z >= bs && z <= be {
        return if z <= bc {
            lerp(cyl_r, rib_r, (z - bs) / d.chamfer_h)
        } else if z <= bf {
            rib_r
        } else {
            lerp(rib_r, cyl_r, (z - bf) / d.chamfer_h)
        };
    }
    // Top rib
    let ts = d.total_height - 8.0;
    let tc = ts + d.chamfer_h;
    let tf = tc + 2.0;
    let te = tf + d.chamfer_h;
    if z >= ts && z <= te {
        return if z <= tc {
            lerp(cyl_r, rib_r, (z - ts) / d.chamfer_h)
        } else if z <= tf {
            rib_r
        } else {
            lerp(rib_r, cyl_r, (z - tf) / d.chamfer_h)
        };
    }
    cyl_r
}

/// Combined radius at `(z, theta)` including rib profile and continuous
/// wire path subtraction (groove + transitions).
fn radius_at(d: &CoilDerived, z: f64, theta: f64, wire_path: &[PathPoint]) -> f64 {
    let base_r = rib_radius_at(d, z);
    
    // First, check if we are inside the center bore. 
    // If we are, return the bore radius immediately to prevent "ghost" geometry.
    let test_x = base_r * theta.cos();
    let test_y = base_r * theta.sin();
    
    // Calculate distance to the wire path.
    let dist = distance_to_wire_path(d, wire_path, [test_x, test_y, z]);

    if dist < 0.0 {
        // We are inside the wire tunnel/groove.
        // Binary search for the surface where the wire path ends.
        let mut r_min = d.center_bore_r;
        let mut r_max = base_r;

        for _ in 0..12 { // Increased precision to 12 iterations
            let r_mid = (r_min + r_max) / 2.0;
            let dist_mid = distance_to_wire_path(d, wire_path, [r_mid * theta.cos(), r_mid * theta.sin(), z]);

            if dist_mid < 0.0 {
                r_max = r_mid; // Still inside wire volume
            } else {
                r_min = r_mid; // Outside wire volume
            }
        }
        // Force the radius to never be smaller than the center bore.
        r_min.max(d.center_bore_r)
    } else {
        base_r
    }
}

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + t.clamp(0.0, 1.0) * (b - a)
}

// ─── Final Mesh Assembly ───────────────────────────────────────────

const SEGMENTS: u32 = 128;

/// Build the complete coil former mesh from parameters.
/// This version includes the friction rib profile, straight tunnels, 
/// and precision Z-level sampling.
pub fn build_coil_former(params: &CoilParams) -> (TriMesh, CoilDerived) {
    let d = CoilDerived::from_params(params);
    let n_around = SEGMENTS as usize;
    
    // Increase sampling resolution to ensure smooth surfaces and clean boolean cuts.
    let n_base_z = ((d.total_height * 6.0).ceil() as usize).max(300);

    let mut z_levels: Vec<f64> = (0..=n_base_z)
        .map(|i| d.total_height * (i as f64) / (n_base_z as f64))
        .collect();

    // --- Key transition points for crisp rib and tunnel edges ---
    let bs = 2.0;
    let bc = bs + d.chamfer_h;
    let bf = bc + 2.0;
    let be = bf + d.chamfer_h;

    let ts = d.total_height - 8.0;
    let tc = ts + d.chamfer_h;
    let tf = tc + 2.0;
    let te = tf + d.chamfer_h;

    for z in [
        bs, bc, bf, be,        // Bottom rib transitions
        ts, tc, tf, te,        // Top rib transitions
        d.start_z, d.end_z,    // Helix start/end
        2.0,                   // Bottom internal tunnel exit (buffer)
        d.total_height - 2.0,  // Top internal tunnel exit (buffer)
    ] {
        if z >= 0.0 && z <= d.total_height {
            z_levels.push(z);
        }
    }

    z_levels.sort_by(|a, b| a.partial_cmp(b).unwrap());
    z_levels.dedup_by(|a, b| (*a - *b).abs() < 0.001);

    let wire_path = generate_wire_path(&d);
    let mut mesh = TriMesh::new();

    // --- 1. Outer surface vertices (Ribs + Grooves) ---
    for &z in &z_levels {
        for i in 0..n_around {
            let theta = 2.0 * PI * (i as f64) / (n_around as f64);
            let r = radius_at(&d, z, theta, &wire_path);
            mesh.positions.push((r * theta.cos()) as f32);
            mesh.positions.push((r * theta.sin()) as f32);
            mesh.positions.push(z as f32);
        }
    }

    // --- 2. Inner surface vertices (Center Bore) ---
    let inner_offset = z_levels.len() * n_around;
    for &z in &z_levels {
        for i in 0..n_around {
            let theta = 2.0 * PI * (i as f64) / (n_around as f64);
            mesh.positions.push((d.center_bore_r * theta.cos()) as f32);
            mesh.positions.push((d.center_bore_r * theta.sin()) as f32);
            mesh.positions.push(z as f32);
        }
    }

    // --- 3. Face Indexing (Walls) ---
    for j in 0..(z_levels.len() - 1) {
        for i in 0..n_around {
            let ni = (i + 1) % n_around;
            
            // Outer Wall
            let v00 = (j * n_around + i) as u32;
            let v01 = (j * n_around + ni) as u32;
            let v10 = ((j + 1) * n_around + i) as u32;
            let v11 = ((j + 1) * n_around + ni) as u32;
            mesh.indices.extend_from_slice(&[v00, v01, v10, v01, v11, v10]);

            // Inner Wall
            let i00 = (inner_offset + j * n_around + i) as u32;
            let i01 = (inner_offset + j * n_around + ni) as u32;
            let i10 = (inner_offset + (j + 1) * n_around + i) as u32;
            let i11 = (inner_offset + (j + 1) * n_around + ni) as u32;
            mesh.indices.extend_from_slice(&[i00, i10, i01, i01, i10, i11]);
        }
    }

    // --- 4. End Caps (Annular Caps) ---
    // Bottom Cap
    for i in 0..n_around {
        let ni = (i + 1) % n_around;
        let o0 = i as u32;
        let o1 = ni as u32;
        let i0 = (inner_offset + i) as u32;
        let i1 = (inner_offset + ni) as u32;
        mesh.indices.extend_from_slice(&[o0, i0, o1, o1, i0, i1]);
    }

    // Top Cap
    let o_top = (z_levels.len() - 1) * n_around;
    let i_top = inner_offset + o_top;
    for i in 0..n_around {
        let ni = (i + 1) % n_around;
        let o0 = (o_top + i) as u32;
        let o1 = (o_top + ni) as u32;
        let i0 = (i_top + i) as u32;
        let i1 = (i_top + ni) as u32;
        mesh.indices.extend_from_slice(&[o0, o1, i0, o1, i1, i0]);
    }

    (mesh, d)
}

pub fn to_binary_stl(mesh: &TriMesh) -> Vec<u8> {
    let tri_count = mesh.indices.len() / 3;
    let mut buf = Vec::with_capacity(84 + tri_count * 50);
    buf.extend_from_slice(&[0u8; 80]);
    buf.extend_from_slice(&(tri_count as u32).to_le_bytes());
    for t in 0..tri_count {
        buf.extend_from_slice(&[0u8; 12]); // Normal
        for i in 0..3 {
            let idx = mesh.indices[t * 3 + i] as usize * 3;
            buf.extend_from_slice(&mesh.positions[idx].to_le_bytes());
            buf.extend_from_slice(&mesh.positions[idx + 1].to_le_bytes());
            buf.extend_from_slice(&mesh.positions[idx + 2].to_le_bytes());
        }
        buf.extend_from_slice(&[0u8; 2]);
    }
    buf
}
