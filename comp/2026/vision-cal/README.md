# Limelight field calibration

A laptop tool for checking and tuning the Limelights at the Doosan practice field or during an
event's field calibration time. It runs on the laptop (tethered to the robot network) and is
never deployed to the robot.

**What it does**
- Checks each camera's configured mount (from CAD / team measurements in
  `VisionConstants.cameraConstants`) against where the robot really is. It flags cameras that look
  wrong, and names common setup mistakes (flipped yaw/side/pitch sign, inches typed as meters,
  forward/side swapped). It **never computes new offsets**; fix offsets from CAD.
- Compares cameras against each other.
- Measures error vs distance for each camera.
- Finds misplaced tags.
- Sweeps **exposure and gain** per camera at every spot you choose, and recommends **one** setting
  per camera that works across all of those spots.

**What it never touches:** robot code, standard deviations, rejection logic, controller bindings.
On the cameras it changes only exposure/gain (temporarily during a sweep, restored after every
spot) and, only after you type `yes`, saved settings (`apply`) or field maps (`upload-fieldmap`).

## Running it

From `comp/2026` (WPILib VS Code terminal):

```
gradlew limelightCal --console=plain -Pcal="<command> [options]"
```

| Command | What it does |
|---|---|
| `check [--field f]` | Static mount checks + can we reach each camera |
| `backup` | Save each camera's pipeline settings (changes nothing) |
| `session --field f [--spots D1,D3] [--no-sweep] [--fast]` | Record at each spot, sweep exposure/gain |
| `report [--run dir]` | Write `report.md` (default: latest run) |
| `apply --camera limelight-shooter [--run dir]` | Save the recommended exposure/gain to that camera (asks first) |
| `fieldmap [--run dir] [--out path]` | Estimate tag positions -> WPILib layout `.json` + Limelight `.fmap` |
| `upload-fieldmap --file x.fmap [--cameras ...]` | Upload a field map (asks first) |

Common options:
- `--cameras a,b` limits which cameras are used.
- `--hosts limelight-shooter=10.1.77.16,...` if `name.local` doesn't resolve.
- `--team 177` or `--server <ip>` for NetworkTables.
- `--exposure-key` / `--gain-key` if your firmware names those pipeline settings differently.
  The defaults are `exposure` and `lcgain`, as in our saved `.vpr` files. The tool lists the
  likely names if the defaults aren't found.

Runs are saved in `vision-cal/runs/<timestamp>/`: backups, every frame, and `session.json`. Reports
can be re-run at home without the robot.

## Field files (`vision-cal/fields/*.json`)

Game-agnostic. Each file names the tag layout and the spots you might park at:

- `layout`: a WPILib `AprilTagFields` name, a WPILib layout JSON path, or a Limelight `.fmap`
  path. A `.fmap` also needs `fieldLengthM` and `fieldWidthM`. **WPILib 2026.1.1 has no 2026
  layout built in**, so use the official JSON or `.fmap`.
- Spot `type: tagRelative`: the robot is parked with one bumper face (`touching`: front, back,
  left or right) flush to the tag's face, plus `faceGapM` out from it and `lateralM` sideways
  (positive = to the left as seen from the field). The tool works out the robot's pose from the
  tag layout and the bumper size (PathPlanner `settings.json`, or `robotLengthM`/`robotWidthM`).
  No camera measuring is needed.
- Spot `type: absolute`: `x`, `y`, `headingDeg` measured from whatever reference you choose (for
  Doosan).
- `weight` (default 1): make important spots, like shooting positions, count more when choosing
  settings.
- `enabled: false` skips a spot unless you name it with `--spots`.

## Dashboard

During `session`, add these in Elastic:
- `/VisionCal/MarkSpot` as a **toggle button**: press it when parked.
- `/VisionCal/NextSpot` and `/VisionCal/Status` as text.

Enter in the terminal also marks a spot; `s` + Enter skips it, `q` + Enter stops.

## Field layouts

- `fields/2026-rebuilt-welded.json` and `fields/2026-rebuilt-andymark.json` are the official 2026
  tag layouts, converted from WPILib's field files (wpilibsuite/allwpilib `fields/.../frc/`).
  Ask each event which one its field uses.

### Making a custom layout for Doosan

The layout uses the official field frame, so set up Doosan in the same frame:
- **Origin (0, 0):** where the blue alliance wall meets the right side wall, as seen standing at the
  blue driver station. At Doosan, pick a real corner or line to stand in for this, and use it for
  everything.
- **+x** runs toward the red alliance wall. **+y** runs toward the left side wall. **z** is the
  height of the tag's center above the carpet.
- **Facing:** the direction the tag's printed face points. 0° = toward the red wall, 90° = toward
  the left side wall, 180° = toward the blue wall, 270° = toward the right side wall.

**Option A: let the tool measure the tags.** You only measure 2–3 robot spots.
1. Start from the official layout (`"layout": "2026-rebuilt-welded.json"` in `doosan.json`). Only
   tags you actually have will be seen.
2. Measure 2–3 floor spots (x, y, robot heading) in the frame above. Put them in `doosan.json` as
   `absolute` spots, and pick spots where several tags are visible.
3. `session --field vision-cal/fields/doosan.json --no-sweep`, then
   `fieldmap --out vision-cal/fields/doosan-layout`. This writes `doosan-layout.json` (WPILib) and
   `doosan-layout.fmap` (Limelight). Tags seen confidently from more than one spot or camera get
   their measured position; others keep the official one. Tag facing is kept from the official
   layout, so if a Doosan tag faces a different way than on the real field, fix its rotation by hand.
4. Point `doosan.json` at `doosan-layout.json` and run a `--no-sweep` session again. Tag offsets
   should now be near 0.

**Option B: tape-measure every tag.** Copy `2026-rebuilt-welded.json` and edit the `translation`
(x, y, z in meters) of each tag you have. Use the Limelight Map Builder
(https://tools.limelightvision.io/map-builder) to edit visually and export a `.fmap`. Option A is
a good check on Option B.

**Using it on the robot:** `upload-fieldmap --file vision-cal/fields/doosan-layout.fmap` replaces
the map on the cameras (asks first). Localization at Doosan then matches your field. **Before an
event, upload the official `.fmap` again.** Keep both files in the repo so swapping back is one
command.

## Direction questions

`VisionConstants.cameraConstants` is the source of truth for mounts. To catch a camera that's set
up backwards or on the wrong side, even if it never sees a tag, the tool asks you to look at the
robot and answer y/n. Enter skips; `--no-ask` turns the questions off.
- **`check` and the start of `session`:** each mount read back in words, e.g. "limelight-intake:
  mounted 0.43 m back and 0.25 m left of the robot's center, 0.48 m up; lens pointing backward;
  tilted 26° up. Correct?"
- **First spot of a `session`** (every spot with `--ask-every-spot`): "Robot at D1 should be facing
  the red alliance wall. limelight-intake should be pointing at the blue alliance wall. Is it?"

A "no" is recorded and shown at the top of that camera's section in the report.

`check` and the report also list where the pose saved inside a camera's pipeline (`rsf`, `rss`,
...) differs from `VisionConstants`. The robot code pushes `VisionConstants` at startup, so this
is housekeeping: re-save the camera's pose to match when convenient. For an offline comparison
against saved pipeline files: `check --offline --vpr-dir src/main/java/frc/robot/VisionConfigs/Doosan`.

## Before you start (both fields)

1. In each Limelight's web UI, check the camera in the **3D viewer**. The model should sit where
   the camera is on the robot (Limelight's own recommended check).
2. Robot code running and disabled, laptop tethered, Driver Station connected.
3. `check`: fix anything flagged before going further.

## Doosan practice field (~20 min)

1. `backup`
2. Pick 3–4 spots that see the tags you have. Measure each spot's x/y/heading from one fixed
   reference, and write them into a copy of `fields/doosan-template.json`.
3. `session --field fields/doosan.json --no-sweep`, then `report` and `fieldmap`.
   - Review the tag table. Small camera spread + big offset = the tag is misplaced. Big spread =
     look at the camera.
4. Optional: `upload-fieldmap --file <run>/fieldmap.fmap`, then re-run step 3 to confirm. Put the
   original map back before an event.
5. `session --field fields/doosan.json` (with sweep) at the spots you care about, then `report`.
6. `apply --camera <name>` for the cameras whose recommendation you agree with.

## Event field calibration (~12 min, fast mode)

1. `backup` (30 s)
2. `session --field fields/<event>.json --fast` with ~4 `tagRelative` spots (~15 s each plus
   driving).
3. `report` (~1 min). Read sections 2 and 3 first: any WRONG camera?
4. `apply` the cameras you agree with.
5. Confirm the Limelights have the event's field map (welded vs AndyMark).

## Limelight's own tuning guidance (from their AprilTag docs)

- Black level 0. The report flags it if not.
- The smallest exposure that still tracks reliably (less motion blur). The sweep prefers the
  lowest exposure within 5% of the best score.
- Moderate gain. Their quick start uses ~15; higher gain can hurt 3D stability. The report flags
  high gain.
- Highest resolution for 3D tracking.
- Mount the cameras above or below tag height, angled so tags look trapezoidal (not head-on).
