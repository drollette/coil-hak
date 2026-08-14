# Phasing Coil Former Generator

A parametric generator for 3D-printable phasing coil formers, designed for HAM radio antenna phasing lines. Features a web UI with real-time 3D preview and exports to both STL and STEP formats.

## What It Does

Generates a cylindrical coil former with a precision V-groove helix that guides wire placement. The former is sized to friction-fit inside standard PVC pipe, making it easy to create weatherproof phasing coils for antenna arrays.

## Features

- **Web Interface**: Real-time 3D preview with adjustable parameters
- **Dual Export**: Download STL for 3D printing or STEP for CAD editing
- **V-Groove Helix**: Precision groove sized to cradle wire securely
- **Straight Wire Tunnels**: Clean entry/exit paths from groove to center bore
- **Friction Ribs**: Optional ribs for secure fit inside PVC pipe
- **Safety Capping**: Coil diameter automatically limited for PVC clearance

## Quick Start (Docker)

```bash
# Pull from GHCR
docker pull ghcr.io/drollette/coil-hak:latest
docker run -p 8000:8000 ghcr.io/drollette/coil-hak:latest

# Or build locally
docker build -t coil-hak .
docker run -p 8000:8000 coil-hak

# Or use docker-compose (exposes on port 8001)
docker-compose up --build
```

Then open http://localhost:8000 (or http://localhost:8001 when using docker-compose)

## Production Deployment (Cloudflare Containers)

In production this app is served at **w7hak.com/coil/** — not a subdomain — via
path-based routing: a Cloudflare Worker owns the `/coil/*` route on the
`w7hak.com` zone and forwards those requests to a Cloudflare Container running
this same Dockerfile unmodified. Everything else on the domain (the main
w7hak.com site, on Cloudflare Pages) is untouched.

```bash
npm install
npx wrangler deploy
```

Requirements:
- Docker running locally (Wrangler builds and pushes the image for you)
- A Cloudflare account on the Workers Paid plan ($5/mo — Containers has no free tier)
- The account must own the `w7hak.com` zone, so the route in `wrangler.jsonc` can attach

Notes:
- `wrangler.jsonc` sets `instance_type: "standard-1"` (½ vCPU / 4 GiB) since
  CadQuery/OpenCASCADE needs more headroom than the default "lite" instance.
  Adjust in the Cloudflare dashboard if generation is slow/OOMing, or scale
  down if it's comfortably idle.
- `max_instances` is pinned to `1`. Generated STL/STEP files live on the
  container's local disk (`OUTPUTS_DIR`), not shared storage, so a `/generate`
  call and the follow-up download must land on the same instance — don't
  raise this without moving job storage somewhere shared first.
- The container scales to zero after `sleepAfter` (10 minutes) of inactivity,
  so the first request after a quiet period will be slower (cold start).
- The app is prefix-aware via the `ROUTE_PREFIX` env var (set to `/coil` by
  `worker/index.js`), which controls both the API routes/static mount in
  `backend/main.py` and the download URLs it returns. Local Docker runs leave
  it unset and serve from the root as before.

## Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `wire_len` | Target wire length in mm | 90.83 |
| `wire_diam` | Wire diameter in mm | 3.5 |
| `pvc_id` | PVC pipe inner diameter in mm | 23.5 |
| `coil_diameter` | Former diameter (auto-capped for clearance) | 15.0 |
| `pitch` | Vertical spacing between wraps in mm | 10.0 |
| `end_buffer` | Space for ribs/transitions at ends in mm | 10.0 |
| `chamfer_size` | Edge chamfer size in mm | 0.5 |
| `enable_ribs` | Add friction ribs for PVC grip | true |

## Local Python Usage

For command-line generation without the web UI:

```bash
# Set up environment
mamba env create -f environment.yml
mamba activate coil-former

# Edit parameters in phasing_coil.py, then run
python phasing_coil.py
```

Output files are saved to the `outputs/` directory.

## Project Structure

```
├── backend/
│   ├── main.py          # FastAPI application
│   ├── geometry.py      # CadQuery geometry engine
│   └── schemas.py       # Pydantic models
├── wasm-coil-former/
│   └── static/
│       └── index.html   # Web frontend
├── worker/
│   └── index.js         # Cloudflare Worker + Container class (production deploy)
├── phasing_coil.py      # Standalone CLI script
├── Dockerfile
├── docker-compose.yml
├── wrangler.jsonc        # Cloudflare Containers/route config
├── package.json          # Wrangler/worker tooling (not the Python app)
├── environment.yml      # Conda dependencies
└── requirements.txt     # Python dependencies
```

## API

**POST /generate**

```json
{
  "wire_len": 90.83,
  "wire_diam": 3.5,
  "pvc_id": 23.5,
  "coil_diameter": 15.0,
  "pitch": 10.0,
  "end_buffer": 10.0,
  "enable_ribs": true,
  "chamfer_size": 0.5
}
```

Returns URLs to download generated STL and STEP files.

## License

MIT License - See [LICENSE](LICENSE) file

## Author

W7HAK - https://w7hak.com
