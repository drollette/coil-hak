# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Parametric generator for 3D-printable phasing coil formers (HAM radio antenna phasing lines). Python/CadQuery backend with a static HTML/JS frontend, containerized with Docker.

## Build & Run

```bash
# Docker (recommended)
docker-compose up --build
# Access: http://localhost:8000 (UI) | http://localhost:8000/docs (API docs)

# Local development
mamba env create -f environment.yml
mamba activate coil-former
pip install -r requirements.txt
uvicorn backend.main:app --reload --host 0.0.0.0 --port 8000

# Standalone CLI (generates STEP file to outputs/)
python phasing_coil.py

# Production deploy: Cloudflare Container, routed at w7hak.com/coil/* (not a
# subdomain). Requires Docker running locally and a Cloudflare account on the
# Workers Paid plan that owns the w7hak.com zone.
npm install
npx wrangler deploy
```

## Testing

No automated test suite exists. Test manually via the web UI or curl:
```bash
curl -X POST http://localhost:8000/generate \
  -H "Content-Type: application/json" \
  -d '{"wire_len": 90.83, "wire_diam": 3.5, "pvc_id": 23.5}'
```

## Architecture

**Backend (Python/FastAPI):**
- `backend/main.py` — FastAPI app. `POST /generate` creates a UUID job, calls geometry engine, exports STL+STEP, schedules 1-hour cleanup. Serves frontend as static files from `wasm-coil-former/static/`. All routes and the static mount live under `ROUTE_PREFIX` (env var, default `""`) — empty for local/docker dev (served at root), set to `/coil` in production so the app works when reached at `w7hak.com/coil/*`. The `/generate` response's `stl_url`/`step_url` are built with this same prefix.
- `backend/geometry.py` — CadQuery geometry engine. `build_coil_former()` creates the parametric 3D model (cylinder, V-groove helix, tunnels, optional friction ribs, chamfers). Returns a `CoilInfo` dataclass with computed values.
- `backend/schemas.py` — Pydantic models (`CoilRequest`, `CoilResponse`, `ComputedValues`) with validation ranges for all parameters.

**Frontend:**
- `wasm-coil-former/static/index.html` — Single-file SPA (vanilla HTML/CSS/JS, no framework). Styled to match w7hak.com's minimalist grayscale/IBM Plex Mono theme (no rounded corners, borders instead of shadows); self-hosted fonts are loaded cross-app from `/fonts/...` since both are served from the same origin in production. `API_URL` in the inline script is derived from `window.location.pathname` (not hardcoded) so it works whether the app is served at the root (local/docker dev) or under `/coil` (production) — it must stay in sync with the backend's `ROUTE_PREFIX` semantics, not duplicate a hardcoded prefix string. Calls `POST /generate` on the backend.

**Standalone CLI:**
- `phasing_coil.py` — Self-contained script with hardcoded parameters at top of file. Generates STEP output directly. Predates the backend but still functional.

**Infrastructure:**
- `Dockerfile` — Based on `condaforge/mambaforge`, installs CadQuery via conda + FastAPI via pip. Also the image deployed to production via Cloudflare Containers, unmodified.
- `docker-compose.yml` — Mounts `./outputs` and `./wasm-coil-former/static` (read-only, enables frontend hot reload).
- `environment.yml` — Conda env: Python 3.10 + CadQuery.
- `requirements.txt` — Pip: FastAPI, uvicorn, python-multipart.
- `wrangler.jsonc` / `worker/index.js` / `package.json` — Cloudflare Worker + Container deploy config. The Worker owns the `/coil/*` route on the `w7hak.com` zone (path-based routing, no subdomain) and proxies everything to a single Container instance (`getContainer(env.COIL_CONTAINER, "singleton")`) running this Dockerfile. Kept at `max_instances: 1` deliberately: generated files live on the container's local disk, so a `/generate` call and its follow-up download must land on the same instance. This is entirely separate from the w7hak.com repo/Pages deploy — no changes there are needed for this route to work, since Cloudflare Workers Routes can own subpaths of a zone that's otherwise served by Pages.

## Key Design Details

- Coil diameter is auto-capped to `pvc_id - wire_diam` for PVC clearance safety.
- Turns calculated as: `sqrt(wire_len² / (circumference² + pitch²))`.
- CadQuery uses boolean `.cut()` operations to subtract grooves, tunnels, and bores from the base cylinder.
- The `wasm-coil-former/` directory name is historical — it previously held a Rust/WASM implementation. The frontend HTML is still served from there.

## CI/CD

- `ci.yml` — Builds Docker image on push/PR to main/master, runs a smoke test against `POST /generate`.
- `docker-ghcr.yml` — Builds and pushes Docker image to `ghcr.io` on push to main/master, release publish, or manual dispatch.
- `release.yml` — Manual workflow dispatch to bump semver, create a git tag, and publish a GitHub Release (which triggers the GHCR push).
