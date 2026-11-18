"""Where the nodes go in the Box3D Solver SOP and Box3D Configure (b3d_hda.layout): labelled boxes, flow down the page.

Rows, top to bottom: the inputs; each input prepared (geometry, constraints, collision, ground); the
simulation; each output built from it (simulation points, geometry, proxy, constraints) and the extras
(impacts, guide). Every node the builders create is named in a box, or `layout` prints it as unplaced."""

INPUTS = (0.35, 0.45, 0.65)
PREPARE = (0.4, 0.4, 0.5)
GROUND = (0.4, 0.55, 0.35)
SIMULATION = (0.6, 0.4, 0.3)
OUTPUT = (0.35, 0.5, 0.45)
EXTRA = (0.45, 0.45, 0.45)

SOLVER_LAYOUT = [
    [("Inputs", ["#0", "#1", "#2", "#3", "IN_geometry", "IN_constraints", "IN_proxy", "IN_collision"],
      {"color": INPUTS})],
    [("Geometry", ["world_scale_geometry", "world_scale_proxy", "sim_source", "sim_source_unscaled",
                   "prepare_pieces", "collision_names_pieces", "scale_attributes", "switch_scale_attributes"], {"color": PREPARE}),
     ("Constraints", ["world_scale_constraints", "scale_constraint_attribs", "switch_scale_constraint_attribs"],
      {"color": PREPARE}),
     ("Collision", ["world_scale_collision", "prepare_collision", "collision_names_collision", "collision_at_start", "collision_rest",
                    "collision_deforming", "collision_unpack", "COLLISION_STATIC"], {"color": PREPARE}),
     ("Ground", ["ground_heightfield_in", "ground_heightfield_scale", "switch_ground_scale", "GROUND_HEIGHTFIELD"],
      {"color": GROUND})],
    [("Simulation", ["dopnet"], {"color": SIMULATION})],
    [("Simulation Points", ["import_points", "points_only", "sim_points", "unscale_points",
                            "switch_unscale_points", "OUT_points"], {"color": OUTPUT}),
     ("Geometry Output", ["names_geometry", "remove_deleted_geometry", "transform_geometry", "unscale_geometry",
                          "switchif_beforestart", "OUT_geometry"], {"color": OUTPUT}),
     ("Proxy Output", ["names_proxy", "remove_deleted_proxy", "transform_proxy", "unscale_proxy",
                       "switchif_beforestart_proxy", "OUT_proxy"], {"color": OUTPUT}),
     ("Constraints Output", ["import_constraints", "blast_broken", "switchif_keepbroken", "transform_constraints",
                             "reset_world_anchors", "unscale_constraint_attribs",
                             "switch_unscale_constraint_attribs", "unscale_constraints", "constraints_result",
                             "switchif_beforestart_constraints", "OUT_constraints"],
      {"color": OUTPUT})],
    [("Impacts", ["import_impacts", "unscale_impacts", "switch_unscale_impacts", "IMPACTS"], {"color": EXTRA}),
     ("Guide", ["import_shapes", "tint_shapes", "unscale_shapes", "no_shapes", "switch_show_shapes",
                "switchif_beforestart_shapes", "import_collision_shapes", "tint_collision_shapes",
                "unscale_collision_shapes", "switch_show_collision_shapes", "switchif_beforestart_collision_shapes",
                "constraint_guide_radius", "false_color_constraints", "constraint_guide_wire", "switch_show_constraints", "switchif_beforestart_constraint_guide", "color_impacts",
                "switch_show_impacts", "switchif_beforestart_impact_guide", "import_collision_geo", "dim_collision_geo", "unscale_collision_geo",
                "switch_show_collision_geo", "switchif_beforestart_collision_geo", "import_ground_plane",
                "ground_plane_lines", "switch_ground_shaded", "unscale_ground_plane", "import_ground_field",
                "unscale_ground_field", "switch_unscale_ground_field", "switch_ground_guide",
                "switchif_beforestart_ground", "merge_guide", "guide_wireframe", "GUIDE_GEO"], {"color": EXTRA})],
]

# The DOP network inside it. The relationships are a grid: fourteen nodes, each wired only to its Enable Switch.
RELATIONSHIP_NAMES = ["weld", "spherical", "revolute", "prismatic", "distance", "motor", "parallel", "wheel",
                      "filter", "glue", "hard", "spring", "soft", "conetwist"]
DOP_LAYOUT = [
    [("Objects", ["pieces", "collision", "collision_static", "collision_static_data", "collision_source"],
      {"color": PREPARE}),
     ("Relationships", RELATIONSHIP_NAMES, {"grid": 5, "color": EXTRA})],
    [("Constraints", ["no_relationship"] + [f"switch_use_{n}" for n in RELATIONSHIP_NAMES]
                     + ["relationships", "constraintnetwork", "use_constraints", "pieces_and_collision",
                        "use_collision"], {"color": PREPARE}),
     ("Ground", ["ground_none", "groundplane", "ground_heightfield", "switch_ground", "ground_data"],
      {"color": GROUND}),
     ("Forces", ["FORCES", "scale_forces"], {"color": EXTRA})],
    [("Solve", ["objects_and_ground", "box3d_solver", "constraint_breaks", "enable_constraint_breaks", "multisolver",
                 "output"], {"color": SIMULATION})],
]

# Box3D Configure.
CONFIGURE_LAYOUT = [
    [("Inputs", ["#0", "#1", "#2", "#3", "IN_geometry", "IN_constraints", "IN_proxy", "IN_bounds"],
      {"color": INPUTS})],
    [("Pack", ["pack_geometry", "pack_proxy", "target"], {"color": PREPARE}),
     ("Bounds", ["bounds_box", "use_bounds_geometry", "bounds_object", "bounds_volume", "bounding_type",
                 "any_bounds"], {"color": GROUND})],
    [("Attributes", ["set_attributes", "add_pin_constraints", "strip_pin_marker", "strip_bounds_group"],
      {"color": SIMULATION})],
    [("Visualize", ["vis_range", "vis_color", "explode_pieces", "use_explode",
                    "OUT_visualize", "hist_grid", "hist_color", "hist_xform", "HISTOGRAM",
                    "DISPLAY"], {"color": GROUND})],
    [("Outputs", ["geometry_result", "proxy_result", "OUT_geometry", "OUT_constraints", "OUT_proxy"],
      {"color": OUTPUT})],
]

# Box3D Constraint Properties.
CONSTRAINT_PROPERTIES_LAYOUT = [
    [("Inputs", ["#0", "#1", "#2", "IN_geometry", "IN_constraints", "IN_proxy"], {"color": INPUTS})],
    [("Select", ["index_lines", "warn_if_no_constraints", "group_constraints", "pick_constraints"],
      {"color": PREPARE})],
    [("Attributes", ["set_properties", "run_vexpressions", "switch_vexpressions", "stash", "apply_edits", "strip_group",
                     "strip_index"],
      {"color": SIMULATION})],
    [("Viewer", ["edit_lines", "selected_lines", "CONSTRAINT", "get_xform", "HANDLE", "build_guides", "GUIDES",
                 "glow_lines", "GLOW", "number_points", "widgets"], {"color": GROUND})],
    [("Outputs", ["OUT_geometry", "OUT_constraints", "OUT_proxy"], {"color": OUTPUT})],
]
