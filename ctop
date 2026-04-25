# `current_to_pressure` — Function Reference

Converts a measured 4–20 mA loop current into engineering pressure units (kg/cm²),
with built-in fault detection for broken or shorted sensor lines.

---

## The Code

```c
#include <stdbool.h>

/* Return type: carries value, original input, and validity status */
typedef enum {
    SENSE_OK,
    SENSE_FAULT_OPEN,    /* < 3.5 mA — broken wire, sensor unpowered */
    SENSE_FAULT_SHORT    /* > 20.5 mA — shorted loop, transmitter failed */
} SenseStatus;

typedef struct {
    float       value_kgcm2;   /* converted pressure (valid only if status == SENSE_OK) */
    float       raw_ma;        /* original current input (always set, useful for logs) */
    SenseStatus status;        /* validity of the reading */
} PressureReading;

/**
 * Convert a 4-20 mA loop current to pressure in kg/cm^2.
 *
 * @param current_ma     Measured loop current in milliamps.
 * @param sensor_fs_bar  Sensor full-scale rating (e.g. 10.0 for a 10-bar sensor).
 * @return PressureReading containing converted value, raw input, and status.
 */
PressureReading current_to_pressure(float current_ma, float sensor_fs_bar) {
    PressureReading r = { .raw_ma = current_ma };

    /* Fault detection — reject readings outside the valid loop range */
    if (current_ma < 3.5f)  { r.status = SENSE_FAULT_OPEN;  return r; }
    if (current_ma > 20.5f) { r.status = SENSE_FAULT_SHORT; return r; }

    /* Linear interpolation: map 4-20 mA to 0.0-1.0 */
    float fraction = (current_ma - 4.0f) / 16.0f;

    /* Clamp to handle small sensor offsets within the fault guard band */
    if (fraction < 0.0f) fraction = 0.0f;
    if (fraction > 1.0f) fraction = 1.0f;

    /* Scale the fraction to the sensor's engineering range */
    r.value_kgcm2 = fraction * sensor_fs_bar;
    r.status      = SENSE_OK;
    return r;
}
```

---

## Line-by-Line Explanation

### The function signature

```c
PressureReading current_to_pressure(float current_ma, float sensor_fs_bar)
```

Takes two inputs:

- `current_ma` — the measured loop current in milliamps.
- `sensor_fs_bar` — the sensor's full-scale rating in bar (10.0 for a 10-bar sensor,
  16.0 for a 16-bar sensor).

Returns a `PressureReading` struct, not a bare float, because we need to communicate
three things back: the converted pressure, the original current (for logging), and
whether the reading is even trustworthy.

### Initialization

```c
PressureReading r = { .raw_ma = current_ma };
```

Designated initializer — sets `r.raw_ma` to the input, and zero-initializes everything
else (`value_kgcm2 = 0.0`, `status = 0` which happens to be `SENSE_OK`). The status
field gets overwritten if anything goes wrong below.

### Fault detection — the most important part

```c
if (current_ma < 3.5f)  { r.status = SENSE_FAULT_OPEN;  return r; }
if (current_ma > 20.5f) { r.status = SENSE_FAULT_SHORT; return r; }
```

A 4–20 mA loop has a built-in diagnostic property: the *minimum* valid current is
4 mA, not 0 mA. That's the entire reason industry standardized on it instead of
0–20 mA.

| Observed current | What it means physically |
|---|---|
| ~0 mA | Wire is broken, or sensor has no power → **OPEN fault** |
| < 3.5 mA | Sensor malfunction, loose connector, or bad PSU |
| 4–20 mA | Normal operating range |
| > 20.5 mA | Wires shorted together, or transmitter failed → **SHORT fault** |

**Why thresholds of 3.5 and 20.5 instead of exactly 4 and 20?** Real transducers
have a small offset — a healthy sensor at 0 pressure might output 3.95 mA or 4.03 mA,
not exactly 4.00. Using exact bounds would flag healthy sensors as faulty. The 0.5 mA
guard band is conservative enough to ignore offset/drift but tight enough to catch
real faults.

When a fault is detected, we return early — `value_kgcm2` stays at 0.0 but the
status field tells the caller "don't trust the value." Validators downstream check
`if (status != SENSE_OK)` before doing any range comparison.

**Why this matters concretely:** the EB spec is `BP < 0.3 kg/cm²`. A broken BP wire
reads as 0 mA → if you blindly converted that to 0 kg/cm², it would pass the EB
check perfectly. You'd report a working emergency brake on a loco where the BP
sensor is *physically disconnected*. The fault check makes that impossible.

### Linear interpolation — the core conversion

```c
float fraction = (current_ma - 4.0f) / 16.0f;
```

The 4–20 mA standard is linear. The transducer outputs:

- 4 mA when pressure is 0
- 20 mA when pressure is at full scale
- proportional values in between

So we want a number from 0.0 to 1.0 representing "how far through the operating
range are we?"

```
fraction = (measured - minimum) / (maximum - minimum)
        = (current_ma - 4) / (20 - 4)
        = (current_ma - 4) / 16
```

**Worked examples** for `current_ma = 11.04`:

```
fraction = (11.04 - 4.0) / 16.0
        = 7.04 / 16.0
        = 0.44   (i.e. 44% of full scale)
```

Other reference points:

- `current_ma = 4.0` (zero pressure) → `fraction = 0.0`
- `current_ma = 20.0` (max pressure) → `fraction = 1.0`
- `current_ma = 12.0` (midpoint) → `fraction = 0.5`

### Clamping — defensive against borderline readings

```c
if (fraction < 0.0f) fraction = 0.0f;
if (fraction > 1.0f) fraction = 1.0f;
```

The fault check above passed currents in the range `3.5 ≤ current_ma ≤ 20.5`. That
means `fraction` could legitimately be:

- `(3.5 - 4) / 16 = -0.031` (slightly negative — sensor reading 3.6 mA at zero pressure)
- `(20.5 - 4) / 16 = 1.031` (slightly over 1.0 — sensor reading 20.4 mA at full scale)

These aren't faults; they're just sensor offset/drift within the acceptable band.
We clamp to `[0, 1]` because:

A negative pressure value would propagate into `range_contains_tol` and might pass
or fail in confusing ways. Clamping to 0 means "the sensor is reading very close
to zero, treat it as zero." Same logic on the high side — clamping to 1.0 means
"treat it as full scale."

This is the kind of edge case that, left unhandled, produces test reports with
"BP measured at -0.02 kg/cm²" which is physically nonsense and makes operators
distrust the whole system.

### Scaling fraction to engineering units

```c
r.value_kgcm2 = fraction * sensor_fs_bar;
```

Once we know the fraction (0.0 to 1.0), we multiply by the sensor's full-scale
value to get the actual pressure.

For `fraction = 0.44` and `sensor_fs_bar = 10.0`:

```
value_kgcm2 = 0.44 × 10.0 = 4.4 kg/cm²
```

For the same fraction but a 16-bar sensor:

```
value_kgcm2 = 0.44 × 16.0 = 7.04 kg/cm²
```

**This is why the function takes `sensor_fs_bar` as a parameter.** The same 11.04 mA
reading means *different* pressures on different sensors. The current is just a
percentage of the sensor's range — you need the sensor's range to translate it back.

A subtle naming point: "bar" and "kg/cm²" are used interchangeably here because they
differ by less than 2% (1 bar = 1.0197 kg/cm²), and Indian Railways specs treat them
as equivalent. If you're being strict, multiply the result by 1.0197 — but the
transducer's own error is bigger than that conversion factor, so it would be false
precision.

### Return

```c
r.status = SENSE_OK;
return r;
```

We only reach this line if all fault checks passed. The struct now contains:

- `value_kgcm2` — the pressure in engineering units
- `raw_ma` — the original current (preserved for logs)
- `status` — `SENSE_OK`

---

## The Full Mental Picture

```
current_ma = 11.04
       │
       ▼
   ┌────────────────┐
   │ Fault check    │  3.5 < 11.04 < 20.5  ✓ pass
   └───────┬────────┘
           ▼
   ┌────────────────┐
   │ fraction =     │  (11.04 - 4) / 16
   │ (mA - 4) / 16  │  = 0.44
   └───────┬────────┘
           ▼
   ┌────────────────┐
   │ Clamp to [0,1] │  0.44 is in range, no change
   └───────┬────────┘
           ▼
   ┌────────────────┐
   │ Scale to FS    │  0.44 × 10.0 = 4.4
   └───────┬────────┘
           ▼
   value_kgcm2 = 4.4 kg/cm²
   status      = SENSE_OK
```

---

## Design Notes

### Why this order: fault check → conversion → clamp → scale

You could write this in fewer lines by computing the fraction first and checking
faults afterward, but then you'd be doing math on potentially garbage data. By
rejecting faults early, the conversion math only runs on values you've already
decided are physically meaningful.

### What this function does *not* do

It doesn't check against a spec range. That's deliberate — `current_to_pressure`
only knows about the sensor, not about what the brake system *should* be doing
right now. Spec validation is a separate step (`validate_reading`) that compares
the converted pressure against `s->bp` or `s->bc` depending on the test phase.
Keeping these concerns separate means you can use this function for any pressure
reading anywhere in the system, regardless of which checkpoint it's part of.

### Reference table — common values

| `current_ma` | `fraction` | 10-bar sensor → kg/cm² | 16-bar sensor → kg/cm² | Status |
|---|---|---|---|---|
| 0.0   | —    | —    | —    | SENSE_FAULT_OPEN |
| 2.0   | —    | —    | —    | SENSE_FAULT_OPEN |
| 4.0   | 0.00 | 0.00 | 0.00 | SENSE_OK |
| 8.0   | 0.25 | 2.50 | 4.00 | SENSE_OK |
| 11.04 | 0.44 | 4.40 | 7.04 | SENSE_OK |
| 12.0  | 0.50 | 5.00 | 8.00 | SENSE_OK |
| 16.0  | 0.75 | 7.50 | 12.0 | SENSE_OK |
| 20.0  | 1.00 | 10.0 | 16.0 | SENSE_OK |
| 22.0  | —    | —    | —    | SENSE_FAULT_SHORT |
