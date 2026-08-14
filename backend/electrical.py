"""Electrical estimates for the wound coil: self-inductance and, for coax,
electrical length. Pure formulas over the same parameters already used to
build the 3D geometry — independent of CadQuery.
"""

SPEED_OF_LIGHT_M_S = 299_792_458.0


def estimate_inductance_uh(
    turns: float, coil_diameter_mm: float, winding_height_mm: float
) -> float:
    """Wheeler's approximation for a single-layer air-core solenoid.

    L (uH) = d^2 * n^2 / (18d + 40l), with d (diameter) and l (winding
    length) in inches. Accurate to within a few percent for l > 0.4 * d;
    used here as an estimate, not a substitute for measurement.
    """
    d_in = coil_diameter_mm / 25.4
    l_in = winding_height_mm / 25.4
    denominator = 18 * d_in + 40 * l_in
    if denominator <= 0:
        return 0.0
    return (d_in**2) * (turns**2) / denominator


def estimate_electrical_length(
    wire_len_mm: float, velocity_factor: float, frequency_mhz: float
) -> tuple[float, float]:
    """Electrical length of a coax run of the given physical length.

    A wave in the cable travels at velocity_factor * c, so one wavelength
    in the cable spans (velocity_factor * c / frequency) of physical cable
    length. Returns (electrical_degrees, wavelength_fraction).
    """
    if frequency_mhz <= 0 or velocity_factor <= 0:
        return 0.0, 0.0
    freespace_wavelength_mm = (SPEED_OF_LIGHT_M_S / (frequency_mhz * 1_000_000)) * 1000
    cable_wavelength_mm = velocity_factor * freespace_wavelength_mm
    wavelength_fraction = wire_len_mm / cable_wavelength_mm
    electrical_degrees = 360.0 * wavelength_fraction
    return electrical_degrees, wavelength_fraction
