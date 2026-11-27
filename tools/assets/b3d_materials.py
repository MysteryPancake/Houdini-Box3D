"""The material presets of Box3D Configure: Type, Material and State fill an entry's Density, Bounce and Friction.

The figures are RBD Configure's own presets (read from its Houdini 22.0 Python Module once, now typed in here): the
same materials, densities in kg/m3, the two friction values and the bounce by type, so a preset gives the same numbers
as RBD Configure's. Nothing is read from Houdini's nodes when the asset is built or used. Edit here to change a preset:
make_configure_sop.py puts this table into the asset's Python Module.

Each material is (name, density, friction on dry and clean surfaces, friction on lubricated and greasy ones).
"""

# The Type menu's order (after None).
TYPES = ("Concrete", "Glass", "Wood", "Metal")

# Bounce (restitution) by type.
BOUNCES = {"Concrete": 0.25, "Glass": 0.2, "Wood": 0.5, "Metal": 0.5}

# The State menu.
STATES = ("Dry and Clean Surfaces", "Lubricated and Greasy Surfaces")

# Every type starts with Default.
DEFAULT = ("Default", 1000.0, 1.0, 0.35)

MATERIALS = {
    "Concrete": [
        ("Asbestos", 2600.0, 1.0, 0.35),
        ("Brick", 2000.0, 1.7, 0.97),
        ("Concrete", 2400.0, 1.77, 1.0),
        ("Concrete - Foamed", 600.0, 1.77, 1.0),
        ("Concrete - Lightweight", 1200.0, 1.77, 1.0),
        ("Earth - Loose", 1220.0, 1.0, 0.35),
        ("Earth - Compacted", 1600.0, 1.0, 0.35),
        ("Granite", 2691.0, 1.0, 0.35),
        ("Limestone", 2560.0, 1.0, 0.35),
        ("Marble", 2711.0, 1.0, 0.35),
        ("Plaster", 1120.0, 1.0, 0.35),
        ("Sandstone", 2250.0, 1.0, 0.35),
    ],
    "Glass": [
        ("Flint", 4400.0, 0.95, 0.35),
        ("Glass", 2600.0, 0.95, 0.35),
        ("Porcelaine", 2400.0, 0.8, 0.3),
        ("Pyrex", 2210.0, 0.8, 0.8),
    ],
    "Wood": [
        ("Balsa Wood", 130.0, 0.375, 0.2),
        ("Beech", 800.0, 0.375, 0.2),
        ("Birch", 670.0, 0.375, 0.2),
        ("Cedar", 480.0, 0.375, 0.2),
        ("Mahogany", 850.0, 0.375, 0.2),
        ("Maple", 675.0, 0.375, 0.2),
        ("Oak", 740.0, 0.62, 0.1),
        ("Pine", 510.0, 0.375, 0.2),
        ("Plywood", 575.0, 0.375, 0.2),
    ],
    "Metal": [
        ("Aluminium", 2712.0, 1.2, 0.3),
        ("Brass", 8480.0, 0.78, 0.42),
        ("Bronze", 8830.0, 0.45, 0.15),
        ("Brushed Metal", 7860.0, 1.2, 0.3),
        ("Copper", 8960.0, 1.0, 0.08),
        ("Gold", 19290.0, 1.4, 0.15),
        ("Iron", 7850.0, 1.0, 0.13),
        ("Lead", 11340.0, 1.05, 0.35),
        ("Nickel", 8900.0, 0.9, 0.28),
        ("Platinum", 21500.0, 1.2, 0.25),
        ("Silver", 10500.0, 1.4, 0.55),
        ("Steel", 8050.0, 0.45, 0.15),
        ("Tin", 7280.0, 0.32, 0.1),
        ("Titanium", 4500.0, 1.4, 0.55),
    ],
}


def table():
    """{index of the type in TYPES: [[density, name, dry friction, lubricated friction], ...]}, Default first."""
    return {i: [[d, n, f0, f1] for n, d, f0, f1 in (DEFAULT,) + tuple(MATERIALS[t])] for i, t in enumerate(TYPES)}


def bounces():
    """{index of the type in TYPES: bounce}."""
    return {i: BOUNCES[t] for i, t in enumerate(TYPES)}
