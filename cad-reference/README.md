# CAD reference

3D/2D reference models for the LILYGO T-Beam Supreme, extracted from the
vendored `../Lora Source/dimensions/` archives (LilyGO's own repo, kept
elsewhere in this project only as reference — not built or modified there).
Pulled out here because they were buried in compressed archives inside a
directory that's otherwise just firmware examples, and are directly useful
for the mechanical backlog items (MECH-01/02/04): enclosure material and
gasket selection, CAD'ing a manufacturable build, and finalizing carabiner
mounts and cutouts.

## T-Beam-Supreme/

- **T-Beam-Supreme-Board.stp** — Full 3D STEP model of the board itself.
  Import this for real component clearances (ports, connectors, antenna,
  battery connector placement) rather than guessing from photos.
- **T-Beam-Supreme-Dimensions.dxf** — 2D board outline and mounting-hole
  dimension drawing.
- **T-Beam-Supreme-Bracket.stp** — A separate mounting bracket LilyGO
  provides for this board.
- **shell/backshell.stl, shell/topside.stl, shell/brackets.stl** — LilyGO's
  own existing enclosure shell design for this board (back half, top half,
  and mounting brackets). A real starting point for MECH-02 rather than a
  from-scratch design — still needs our own button/port cutouts, wall
  thickness/gasket work for the IP55 target (MECH-01), and carabiner mount
  changes (MECH-04), since this shell wasn't designed for those.

## Source

Pulled from `Lora Source/dimensions/` on 2026-09-07:
`T-Beam-Supreme.zip`, `T-Beam-Supreme-Shell.7z`, `T-Beam-Supreme-Bracket.zip`,
`T-BEAM-SUPREME-V3.0.DXF`. See that folder's own README.md for the full
LilyGO dimension-file index across all their boards, if a different board
revision's files are ever needed.
