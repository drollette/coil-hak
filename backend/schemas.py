"""Pydantic request/response models for the Coil Former API."""

import math
from typing import Literal

from pydantic import BaseModel, Field, model_validator

# Geometric clearance constants, mirrored in the frontend's live slider
# constraints (wasm-coil-former/static/index.html) so the UI can't even
# reach a combination this validator would reject. Keep both in sync.
#
# GROOVE_DEPTH_FACTOR matches backend/geometry.py's `v_depth = r_wire *
# sqrt(2)`: the V-groove is cut to that depth below the coil's outer
# surface, so the coil radius must exceed it or the groove cuts through
# the center axis. CLEARANCE_MARGIN adds a safety buffer above the exact
# boundary so users land comfortably inside valid geometry, not right at
# the edge of it.
GROOVE_DEPTH_FACTOR = math.sqrt(2)
CLEARANCE_MARGIN = 1.1
# Physical constraint independent of the above: pitch must exceed wire
# diameter or adjacent turns overlap. The margin leaves visible separation.
MIN_PITCH_TO_WIRE_RATIO = 1.1


class CoilRequest(BaseModel):
    """Request parameters for generating a coil former."""

    wire_len: float = Field(
        default=668,
        ge=5.0,
        le=10000.0,
        description="Target wire length in mm. Raised to accommodate large-diameter coils, which need proportionally more wire per turn"
    )
    wire_diam: float = Field(
        default=2,
        ge=0.5,
        le=15.0,
        description=(
            "Wire (or, in coax mode, cable outer) diameter in mm. Range "
            "raised to fit common coax ODs — e.g. RG-213/LMR-400 are "
            "~10.3mm — on top of bare/insulated wire use."
        )
    )
    wire_type: Literal["wire", "coax"] = Field(
        default="wire",
        description=(
            "'wire' (bare/insulated single conductor, wound as a lumped "
            "inductor — see computed inductance_uh) or 'coax' (coaxial "
            "cable wound as a phasing/delay stub — see computed "
            "electrical_degrees/wavelength_fraction, which need "
            "velocity_factor and frequency_mhz)."
        )
    )
    coax_type: str | None = Field(
        default=None,
        description=(
            "Selected coax type id from GET /coax-types, for display/"
            "reference only — velocity_factor is what's actually used in "
            "the calculation, so this doesn't have to match it."
        )
    )
    velocity_factor: float | None = Field(
        default=None,
        ge=0.1,
        le=1.0,
        description="Coax velocity factor (0-1). Required when wire_type='coax'"
    )
    frequency_mhz: float | None = Field(
        default=None,
        ge=0.1,
        le=6000.0,
        description="Operating frequency in MHz, for coax electrical-length calculations. Required when wire_type='coax'"
    )
    pvc_id: float = Field(
        default=23.5,
        ge=10.0,
        le=250.0,
        description=(
            "PVC pipe inner diameter in mm. Sets the friction ribs' outer "
            "diameter (shown in the UI as 'Friction Rib Outer Diameter') "
            "and caps coil_diameter for clearance, regardless of whether "
            "enable_ribs is set."
        )
    )
    coil_diameter: float = Field(
        default=25,
        ge=5.0,
        le=200.0,
        description="Desired coil diameter in mm (capped for PVC clearance — see pvc_id)"
    )
    pitch: float = Field(
        default=8.9,
        ge=2.0,
        le=50.0,
        description="Vertical distance per turn in mm"
    )
    end_buffer: float = Field(
        default=9,
        ge=2.0,
        le=30.0,
        description="Space for ribs and transitions at top/bottom in mm"
    )
    enable_ribs: bool = Field(
        default=True,
        description="Enable friction ribs for PVC grip"
    )
    chamfer_size: float = Field(
        default=1,
        ge=0.0,
        le=3.0,
        description="Chamfer size on top/bottom edges in mm"
    )
    tunnel_tol: float = Field(
        default=0.2,
        ge=0.0,
        le=1.0,
        description="Extra diameter clearance for wire tunnels in mm"
    )
    center_bore_diam: float | None = Field(
        default=None,
        ge=0.5,
        le=200.0,
        description="Center bore diameter in mm (None = wire_diam + tunnel_tol)"
    )

    @model_validator(mode="after")
    def validate_geometry_clearances(self) -> "CoilRequest":
        """Reject parameter combinations that can't produce valid geometry.

        Belt-and-suspenders: the frontend keeps its sliders from reaching
        these combinations live (see index.html), but this validator is
        the actual source of truth and also covers direct API callers.
        """
        actual_coil_diam = min(self.coil_diameter, self.pvc_id - self.wire_diam)

        min_coil_diam = self.wire_diam * GROOVE_DEPTH_FACTOR * CLEARANCE_MARGIN
        if actual_coil_diam < min_coil_diam:
            raise ValueError(
                f"Coil diameter ({actual_coil_diam:.1f}mm after friction-rib "
                f"clearance capping) is too small for a {self.wire_diam}mm "
                f"wire/cable — the winding groove would cut through the "
                f"center. Increase coil diameter or friction rib outer "
                f"diameter, or reduce wire diameter. Minimum coil diameter "
                f"for this wire: {min_coil_diam:.1f}mm."
            )

        min_pitch = self.wire_diam * MIN_PITCH_TO_WIRE_RATIO
        if self.pitch < min_pitch:
            raise ValueError(
                f"Pitch ({self.pitch}mm) is too small for a {self.wire_diam}mm "
                f"wire/cable — adjacent turns would overlap. Minimum pitch "
                f"for this wire: {min_pitch:.1f}mm."
            )

        if self.center_bore_diam is not None:
            max_bore = actual_coil_diam - min_coil_diam
            if self.center_bore_diam > max(max_bore, 0):
                raise ValueError(
                    f"Center bore diameter ({self.center_bore_diam}mm) is too "
                    f"large for a {actual_coil_diam:.1f}mm coil with "
                    f"{self.wire_diam}mm wire/cable — there'd be no wall left "
                    f"for the winding groove. Maximum center bore diameter: "
                    f"{max(max_bore, 0):.1f}mm."
                )

        return self


class ComputedValues(BaseModel):
    """Computed geometry and electrical values returned by the API."""

    turns: float = Field(description="Number of coil turns")
    total_height: float = Field(description="Total former height in mm")
    coil_diameter: float = Field(description="Actual coil diameter (may be capped)")
    winding_height: float = Field(description="Height of the winding section in mm")
    inductance_uh: float | None = Field(
        default=None,
        description="Estimated self-inductance in microhenries, via Wheeler's formula (wire_type='wire' only)"
    )
    electrical_degrees: float | None = Field(
        default=None,
        description="Electrical length in degrees at frequency_mhz (wire_type='coax' only)"
    )
    wavelength_fraction: float | None = Field(
        default=None,
        description="Electrical length as a fraction of one wavelength (wire_type='coax' only)"
    )


class CoilResponse(BaseModel):
    """Response containing generated file paths and computed values."""

    uuid: str = Field(description="Unique job identifier")
    computed: ComputedValues
    stl_url: str = Field(description="URL to download STL file")
    step_url: str = Field(description="URL to download STEP file")


class CoaxType(BaseModel):
    """Reference data for one coax cable type. See backend/coax_data.py."""

    id: str
    name: str
    impedance_ohms: float | None = None
    velocity_factor: float | None = None
    outer_diameter_mm: float | None = None
    min_bend_radius_mm: float | None = None
    bend_radius_is_guideline: bool = Field(
        description="True if min_bend_radius_mm is a general 10x-OD industry guideline rather than a manufacturer-specified figure"
    )
    source_name: str | None = None
    source_url: str | None = None
