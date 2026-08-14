"""Reference data for common coaxial cable types.

Velocity factor and outer diameter come from manufacturer datasheets (linked
below). Minimum bend radius is manufacturer-sourced where a datasheet gave a
clear figure; where it didn't, we fall back to the standard cable-industry
rule of thumb of 10x the cable's outer diameter for a single/installation
bend (flagged via `bend_radius_is_guideline`) rather than presenting an
unverified number as if it were a spec.

These are representative values for a common/generic version of each cable
type — actual specs vary by manufacturer and specific part number. Always
check your own cable's datasheet for a real build.
"""

COAX_TYPES = [
    {
        "id": "rg58",
        "name": "RG-58",
        "impedance_ohms": 50,
        "velocity_factor": 0.66,
        "outer_diameter_mm": 4.95,
        "min_bend_radius_mm": 49.5,
        "bend_radius_is_guideline": True,
        "source_name": "Belden 8240 Technical Data Sheet",
        "source_url": "https://catalog.belden.com/techdata/EN/8240_techdata.pdf",
    },
    {
        "id": "rg8x",
        "name": "RG-8X",
        "impedance_ohms": 50,
        "velocity_factor": 0.78,
        "outer_diameter_mm": 6.15,
        "min_bend_radius_mm": 61.5,
        "bend_radius_is_guideline": True,
        "source_name": "Belden 9258 Technical Data Sheet",
        "source_url": "https://catalog.belden.com/techdata/EN/9258_techdata.pdf",
    },
    {
        "id": "rg213",
        "name": "RG-213",
        "impedance_ohms": 50,
        "velocity_factor": 0.66,
        "outer_diameter_mm": 10.29,
        "min_bend_radius_mm": 127.0,
        "bend_radius_is_guideline": False,
        "source_name": "Belden 8267 Technical Data Sheet",
        "source_url": "https://catalog.belden.com/techdata/EN/8267_techdata.pdf",
    },
    {
        "id": "rg6",
        "name": "RG-6",
        "impedance_ohms": 75,
        "velocity_factor": 0.82,
        "outer_diameter_mm": 6.99,
        "min_bend_radius_mm": 69.85,
        "bend_radius_is_guideline": False,
        "source_name": "Belden 1694A Technical Data Sheet",
        "source_url": "https://catalog.belden.com/techdata/EN/1694A_techdata.pdf",
    },
    {
        "id": "lmr400",
        "name": "LMR-400",
        "impedance_ohms": 50,
        "velocity_factor": 0.85,
        "outer_diameter_mm": 10.29,
        "min_bend_radius_mm": 25.4,
        "bend_radius_is_guideline": False,
        "source_name": "Times Microwave LMR-400 Datasheet",
        "source_url": "https://timesmicrowave.com/wp-content/uploads/2022/06/lmr-400-datasheet-1.pdf",
    },
    {
        "id": "rg316",
        "name": "RG-316",
        "impedance_ohms": 50,
        "velocity_factor": 0.70,
        "outer_diameter_mm": 2.5,
        "min_bend_radius_mm": 25.0,
        "bend_radius_is_guideline": False,
        "source_name": "Belden 83284 Technical Data Sheet",
        "source_url": "https://catalog.belden.com/techdata/EN/83284_techdata.pdf",
    },
    {
        "id": "rg174",
        "name": "RG-174",
        "impedance_ohms": 50,
        "velocity_factor": 0.66,
        "outer_diameter_mm": 2.79,
        "min_bend_radius_mm": 28.0,
        "bend_radius_is_guideline": False,
        "source_name": "Belden 8216 Technical Data Sheet",
        "source_url": "https://catalog.belden.com/techdata/EN/8216_techdata.pdf",
    },
    {
        "id": "rg142",
        "name": "RG-142",
        "impedance_ohms": 50,
        "velocity_factor": 0.70,
        "outer_diameter_mm": 4.95,
        "min_bend_radius_mm": 51.0,
        "bend_radius_is_guideline": False,
        "source_name": "Belden 84142 Technical Data Sheet",
        "source_url": "https://catalog.belden.com/techdata/EN/84142_techdata.pdf",
    },
    {
        "id": "custom",
        "name": "Custom",
        "impedance_ohms": None,
        "velocity_factor": None,
        "outer_diameter_mm": None,
        "min_bend_radius_mm": None,
        "bend_radius_is_guideline": False,
        "source_name": None,
        "source_url": None,
    },
]
