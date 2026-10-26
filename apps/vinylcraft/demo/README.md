# VinylCraft demo samples

Small SVG files used to check the import → toolpath → DMPL pipeline. Every file here is
a **closed contour** test case: the point is that one drawn outline must emit as one
continuous toolpath, with a single pen-down at the start and a single pen-up at the end.

Keep these files. They are the regression set for contour chaining in `flatten_all`
(`apps/vinylcraft/src/main.rs`).

## Files

| File | What it is | Contours | U | D |
|---|---|---|---|---|
| `star.svg` | 10-point star, one closed path | 1 | 2 | 10 |
| `rounded.svg` | 200 × 100 rect with four 20 mm arcs | 1 | 2 | 100 |
| `cubic.svg` | open-ish path with a cubic Bézier corner | 1 | 2 | 51 |
| `two.svg` | two separate closed squares in one `<path>` | 2 | 4 | 8 |
| `test_square.svg` | square plus a circle (rect + circle elements) | 2 | 4 | 54 |

`U` = pen-up commands, `D` = pen-down commands in the emitted stream. A closed contour
should cost **one** `D` to enter and **one** `U` to leave; extra U/D pairs mean the
contour is being split into fragments and the blade will lift and re-drop at each vertex.

`cubic.svg` and `rounded.svg` have high `D` counts because arcs and Béziers are flattened
into many short line segments — that is expected. What matters is that the count of `U`
stays at 2: the blade drops once and stays down for the whole outline.

## Reproduce

From the repository root:

```bash
B=target/debug/vinylcraft.exe

$B --load apps/vinylcraft/demo/star.svg --cut --emit --out star.dmpl
$B --load apps/vinylcraft/demo/rounded.svg --cut --emit --out rounded.dmpl
$B --load apps/vinylcraft/demo/cubic.svg --cut --emit --out cubic.dmpl
$B --load apps/vinylcraft/demo/two.svg --cut --emit --out two.dmpl
$B --load apps/vinylcraft/demo/test_square.svg --cut --emit --out sq.dmpl

# count the pen commands in each stream
for f in star rounded cubic two sq; do
  printf '%-8s U=%s D=%s\n' "$f" \
    "$(tr ';' '\n' < $f.dmpl | grep -c '^U')" \
    "$(tr ';' '\n' < $f.dmpl | grep -c '^D')"
done
```

Expected output:

```
star     U=2 D=10
rounded  U=2 D=100
cubic    U=2 D=51
two      U=4 D=8
sq       U=4 D=54
```

## Failure signature to watch for

Before the chaining fix, `star.svg` reported **`paths 10`** and emitted ten `U` commands
with repeated coordinates, e.g. `U4940,4531;U4940,4531;`. If a future change makes any
single-contour file here report more than one path, the chaining in `flatten_all` has
regressed.

Multi-contour files (`two.svg`, `test_square.svg`) must report exactly 2 — one per real
outline. They must not merge into 1 (that would connect two separate shapes with a cut
across the gap) and must not explode into 4+.

## Note on units

The emitted stream is in device units from the active profile. The SC2 profile is
40 units/mm, so a 10 mm move is `U400`. Coordinates in the output therefore do not match
the SVG's millimetre viewBox values directly — that is correct, not a scaling bug.
