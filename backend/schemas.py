"""Pydantic request/response models for the Coil Former API."""

from typing import Literal

from pydantic import BaseModel, Field


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
