"""
FastAPI application for the Coil Former generator.
Serves both the API and static frontend.
"""

import asyncio
import os
import shutil
import uuid
from pathlib import Path

from fastapi import APIRouter, FastAPI, BackgroundTasks, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from .coax_data import COAX_TYPES
from .electrical import estimate_electrical_length, estimate_inductance_uh
from .schemas import CoaxType, CoilRequest, CoilResponse, ComputedValues
from .geometry import build_coil_former, export_step, export_stl

# Paths
BASE_DIR = Path(__file__).parent.parent
OUTPUTS_DIR = BASE_DIR / "outputs"
STATIC_DIR = BASE_DIR / "wasm-coil-former" / "static"

# Ensure outputs directory exists
OUTPUTS_DIR.mkdir(exist_ok=True)

# Params for jobs whose STEP file hasn't been generated yet (see /generate
# and the lazy-export branch in download_file below), keyed by job_id.
# Cleared alongside the job's output directory in cleanup_job.
_pending_step_params: dict[str, CoilRequest] = {}

# When deployed behind the w7hak.com Worker route (path-based routing, no
# subdomain), requests arrive with the full "/coil" prefix still attached
# rather than stripped by a reverse proxy, so the app must serve routes and
# static files under that same prefix. Left empty for local/docker dev,
# where the app is accessed at the domain root.
ROUTE_PREFIX = os.environ.get("ROUTE_PREFIX", "").rstrip("/")

app = FastAPI(
    title="Coil Former API",
    description="Parametric coil former generator with CadQuery",
    version="2.0.0",
)

# CORS for development (allows localhost origins)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


async def cleanup_job(job_id: str, delay: int = 3600) -> None:
    """Delete job directory after specified delay (default 1 hour)."""
    await asyncio.sleep(delay)
    job_dir = OUTPUTS_DIR / job_id
    shutil.rmtree(job_dir, ignore_errors=True)
    _pending_step_params.pop(job_id, None)


router = APIRouter(prefix=ROUTE_PREFIX)


@router.post("/generate", response_model=CoilResponse)
def generate_coil(
    request: CoilRequest,
    background_tasks: BackgroundTasks
) -> CoilResponse:
    """
    Generate a coil former with the specified parameters.
    Returns URLs to download STL and STEP files.

    Deliberately a sync (not async) def: this does no awaiting, it's a
    single CPU-bound CadQuery/OpenCASCADE computation. FastAPI runs sync
    route handlers in a threadpool automatically, so this doesn't block the
    event loop (and therefore every other concurrent request — health
    checks, static assets, other jobs) for the several-second duration of a
    geometry rebuild the way an `async def` doing the same blocking work
    in-line would.
    """
    if request.wire_type == "coax" and (
        request.velocity_factor is None or request.frequency_mhz is None
    ):
        raise HTTPException(
            status_code=400,
            detail="velocity_factor and frequency_mhz are required when wire_type is 'coax'",
        )

    # Generate unique job ID
    job_id = str(uuid.uuid4())
    job_dir = OUTPUTS_DIR / job_id
    job_dir.mkdir(parents=True, exist_ok=True)

    # Build geometry
    result, info = build_coil_former(
        wire_len=request.wire_len,
        wire_diam=request.wire_diam,
        pvc_id=request.pvc_id,
        coil_diameter=request.coil_diameter,
        pitch=request.pitch,
        end_buffer=request.end_buffer,
        enable_ribs=request.enable_ribs,
        chamfer_size=request.chamfer_size,
        tunnel_tol=request.tunnel_tol,
        center_bore_diam=request.center_bore_diam,
    )

    # Only export STL here. STEP export is extra OpenCASCADE/export work
    # that's wasted on every interactive parameter tweak when the user just
    # wants the 3D preview — it's built lazily in download_file() the first
    # time (if ever) someone actually clicks "Download STEP" for this job.
    stl_path = job_dir / "coil.stl"
    export_stl(result, stl_path)
    _pending_step_params[job_id] = request

    # Schedule cleanup
    background_tasks.add_task(cleanup_job, job_id)

    inductance_uh = None
    electrical_degrees = None
    wavelength_fraction = None
    if request.wire_type == "coax":
        electrical_degrees, wavelength_fraction = estimate_electrical_length(
            wire_len_mm=request.wire_len,
            velocity_factor=request.velocity_factor,
            frequency_mhz=request.frequency_mhz,
        )
    else:
        inductance_uh = estimate_inductance_uh(
            turns=info.turns,
            coil_diameter_mm=info.coil_diameter,
            winding_height_mm=info.winding_height,
        )

    return CoilResponse(
        uuid=job_id,
        computed=ComputedValues(
            turns=round(info.turns, 2),
            total_height=round(info.total_height, 2),
            coil_diameter=round(info.coil_diameter, 2),
            winding_height=round(info.winding_height, 2),
            inductance_uh=round(inductance_uh, 3) if inductance_uh is not None else None,
            electrical_degrees=round(electrical_degrees, 2) if electrical_degrees is not None else None,
            wavelength_fraction=round(wavelength_fraction, 4) if wavelength_fraction is not None else None,
        ),
        stl_url=f"{ROUTE_PREFIX}/outputs/{job_id}/coil.stl",
        step_url=f"{ROUTE_PREFIX}/outputs/{job_id}/coil.step",
    )


@router.get("/outputs/{job_id}/{filename}")
def download_file(job_id: str, filename: str) -> FileResponse:
    """Serve generated output files (sync def — see generate_coil)."""
    file_path = OUTPUTS_DIR / job_id / filename

    if not file_path.exists() and filename == "coil.step":
        # Not exported at /generate time (see there) — build it now, on the
        # one request that actually needs it.
        request = _pending_step_params.get(job_id)
        if request is not None:
            result, _ = build_coil_former(
                wire_len=request.wire_len,
                wire_diam=request.wire_diam,
                pvc_id=request.pvc_id,
                coil_diameter=request.coil_diameter,
                pitch=request.pitch,
                end_buffer=request.end_buffer,
                enable_ribs=request.enable_ribs,
                chamfer_size=request.chamfer_size,
                tunnel_tol=request.tunnel_tol,
                center_bore_diam=request.center_bore_diam,
            )
            export_step(result, file_path)

    if not file_path.exists():
        raise HTTPException(status_code=404, detail="File not found")

    # Determine media type
    media_type = "application/octet-stream"
    if filename.endswith(".stl"):
        media_type = "model/stl"
    elif filename.endswith(".step"):
        media_type = "application/step"

    return FileResponse(
        path=file_path,
        media_type=media_type,
        filename=filename,
    )


@router.get("/coax-types", response_model=list[CoaxType])
async def list_coax_types() -> list[dict]:
    """Reference data for common coax cable types. See backend/coax_data.py
    for the values and their sources."""
    return COAX_TYPES


@router.get("/health")
async def health_check() -> dict:
    """Health check endpoint."""
    return {"status": "healthy", "version": "2.0.0"}


app.include_router(router)

# Serve static frontend files
# This must be mounted last to avoid catching API routes
app.mount(
    ROUTE_PREFIX or "/",
    StaticFiles(directory=str(STATIC_DIR), html=True),
    name="static",
)
