# Idefix chassis, CAD source

Parametric OpenSCAD source for the three-deck Idefix chassis. Every dimension is a named parameter in `common.scad`. Change one parameter, re-render all three decks and their auxiliary parts, and every downstream STL updates consistently.

## What is in this directory

| File | Purpose | Renders |
|---|---|---|
| `common.scad` | Every dimension in the design, plus shared utility modules. | No (parameters only) |
| `deck_1.scad` | Bottom deck: motors, caster, battery, PCB. Full 250 mm circular plate. | Prints as one STL |
| `deck_2.scad` | Middle deck: Raspberry Pi 5, camera support plate. Full 250 mm circular plate. | Prints as one STL |
| `deck_3.scad` | Top deck: RPLIDAR C1 only. Rounded triangular plate, ~62% material reduction versus a full disc. | Prints as one STL |
| `standoff.scad` | Visualization of the M3 standoff columns between decks. Not a printable part. | No STL export |
| `Idefix.scad` | Assembly-level file that stacks all three decks with standoffs at their in-service Z positions, plus semi-transparent envelopes for the major components. Preview only. | No STL export |

## Prerequisites

- OpenSCAD, any recent version. Tested against 2021.01 and later. Download from `https://openscad.org/downloads.html`. The Python API used in some tutorials is not needed; the plain script interface is sufficient.

## Quick start

To just look at the assembly:

1. Open `Idefix.scad`.
2. Press F5 (preview). You should see three coloured deck plates stacked vertically with three standoff columns visible between decks 1 and 2, and three shorter columns between decks 2 and 3. Semi-transparent grey blocks mark the LiPo battery, PCB, Pi 5, camera, and LiDAR positions.

To export a printable STL:

1. Open one of `deck_1.scad`, `deck_2.scad`, or `deck_3.scad`.
2. Press F6 (full render). This takes a few seconds to a few minutes depending on `$fn`.
3. `File > Export > Export as STL`.
4. Slice as usual in your preferred slicer.

## Recommended print settings

Settings used for the reference build:

- Nozzle: 0.4 mm
- Layer height: 0.2 mm
- Material: PLA (PETG is a reasonable alternative if you find deck 3 deflects under the LiDAR)
- Infill: 40% gyroid on the three decks, 20% gyroid on caster pedestal and camera support
- Perimeters: 4 on the three decks, 3 on smaller parts
- Supports: none required if the plates are printed flat-side-down
- Orientation: flat face down, no rotation

## Modifying the design

**To change a dimension**: open `common.scad`, edit the parameter, save. Re-open the deck file you want to check and press F5.

**To move a component**: find its `_x` and `_y` parameters in `common.scad`. Coordinates are in millimetres, origin at the deck's geometric centre, +X to the right and +Y forward when the robot is viewed from above with the camera-facing edge at the top.

**To change a hole size**: find the `_hole_d` parameter for that component. If you want M3 instead of M2.5, replace `m2_5_clear` with `m3_clear` in the assignment.

**To swap in different components**: most component footprints are captured as either individual parameters (for holes on regular patterns like the Pi 5's 58 × 49 mm rectangle) or lists of positions (for irregular patterns like `pcb_hole_positions` and `motor_bracket_hole_positions`). Edit the list to match the new component's mounting hole layout.

**To increase the render smoothness before final STL export**: at the top of `common.scad`, `$fn = 120` (default) is fine for most cases. Raise to 200 if you want smoother circles in the final STL at the cost of slightly slower rendering.

## URDF coupling

Three parameters in `common.scad` are URDF-locked and MUST match the URDF's Xacro constants exactly:

- `wheel_radius`
- `track_width`
- `deck_radius` (URDF `base_radius`)

The `wheel_diameter` and `deck_diameter` derived quantities are automatically consistent. If you change any URDF-locked value, update the URDF at the same time and re-verify the simulation before printing new hardware.

## Assembly notes

1. Print all six STLs. Total print time on a Bambu A1 is approximately 6 hours.
2. Install M3 heat-set inserts (or just use nuts on the underside) at the PCB and Pi 5 boss positions.
3. Bolt the caster pedestal to the underside of deck 1 at the caster mounting hole positions, then bolt the ball caster to the pedestal from below.
4. Mount the motors to deck 1 using motor brackets (not included in this CAD; see `hardware/motor_bracket/` for the Waveshare N20 bracket used in the reference build).
5. Screw the M3 threaded standoffs into deck 1 from below through the standoff holes, using a nut on top to lock them.
6. Fit deck 2 over the standoff tops. Bolt down. Mount the Pi 5 to its bosses; bolt the camera support plate to its two front-edge holes.
7. Screw the shorter M3 standoffs into deck 2. Fit deck 3 over their tops. Bolt down.
8. Mount the RPLIDAR C1 to deck 3 with four M2.5 screws. Route the JST cable through the rear cutout down to the LiDAR-to-USB adapter on deck 2.

## Known limitations of the current version

- **Motor bracket not included in this CAD**. The bolt hole positions in deck 1 are correct for the reference bracket, but the bracket itself is designed separately.
- **No ESP32-S3 cradle**. The ESP32 dev board has no mounting holes, so the reference build uses a printed cradle (see `hardware/esp32_cradle/`) plus the USB-C access cutout in deck 2. The cutout parameters are in `common.scad` under the ESP32 block.
- **No cable management channels**. Cables run loose between decks in the reference build.
- **Caster pedestal is a compromise**. The 2 mm printed pedestal compensates for a 24 mm caster total drop against a 22 mm wheel radius; the alternative is a taller caster or a shorter one plus different geometry, either of which will change the URDF's `base_link` height.

## When something is wrong

If a hole does not line up on the physical build:

1. Open the relevant deck SCAD file.
2. Add a `translate([x, y, 0]) drill(m3_clear);` line inside the top-level `difference()` block at the required position. Note this in a comment.
3. Save. Reprint the deck.

If a preview render shows something wildly wrong before printing, the error is almost always in `common.scad`, not in a deck file. Check the parameter you last edited.

## Licence

Design files are released under CC BY 4.0. Attribution to the Idefix project, please cite the HardwareX paper when the DOI is live.
