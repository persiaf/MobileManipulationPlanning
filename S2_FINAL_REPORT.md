# Round S2 RC2 four-table result

Status: **PARTIAL**. The real Stage 1/RC1/RC2 run generated four tables and eight tasks. RC1 evaluated all four regions and produced 1572 candidates. Existing Bottom-Up RC2 selected four docks and covered tasks 1-4; tasks 5-8 had no feasible docking candidate under the unchanged reachability, footprint, and safety settings. Stage 3 was therefore not run and no path is claimed.

Table centers: (2.2,2.2), (5.6,2.2), (2.2,5.6), (5.6,5.6) m. Tasks use local offsets (-0.2,0), (0.2,0) and IDs 1-8 in table order.

The external result directory contains the MAT file, region coverage CSV, task CSV, summary CSV, run log, and four diagnostic figures. No core planner or safety threshold was changed.

Presentation figures were subsequently regenerated from that same S2 MAT file into the external `20261020_183401_round_s2_visual_style_unification/results/final_figures/` directory. Figure 4 explicitly reports that RC2 was blocked and no global path exists.

The final presentation replacement using the supplied single-table RC2 reference style is in `20261020_191406_round_s2_monochrome_reference_style/results/final_figures/`.
