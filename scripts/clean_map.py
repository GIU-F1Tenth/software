"""
Clean a 3-level SLAM map (walls=0, outside=205, track=254) for map_converter.ipynb.

The SLAM PNG is already correctly quantized, but the walls form a DOTTED ring
(each dot is a tiny connected component), and there is speckle in every class.
Plain size-filtering drops the wall dots and erases the boundary, so we first
bridge the dotted walls with a morphological closing, then size-filter, then
fill holes so the outside region is a single solid enclosure around the track.

Output (single-channel 8-bit grayscale, map_converter cell-4 threshold = 210):
    walls   -> 0    (<= 210, wall)
    outside -> 205  (<= 210, wall)
    track   -> 254  (>  210, free — the white racing surface)
"""
import numpy as np
from PIL import Image
from scipy import ndimage

SRC = "/home/karim/Raceline-Optimization/maps/map.png"
DST = "/home/karim/Raceline-Optimization/maps/map1.png"

WALL = 0
OUTSIDE = 205
TRACK = 254

WALL_CLOSE_RADIUS = 2   # bridges dotted walls up to ~2*r pixels apart
MIN_WALL_SIZE = 50      # drop wall blobs smaller than this after closing
MIN_TRACK_SIZE = 500    # drop tiny white blobs — noise inside outside region
MIN_OUTSIDE_SIZE = 200  # drop tiny grey blobs — noise inside the track

arr = np.array(Image.open(SRC).convert("L"))
print(
    f"loaded {SRC}  shape={arr.shape}  values={sorted(np.unique(arr).tolist())}")

walls = arr == WALL
track = arr == TRACK
outside = arr == OUTSIDE


def disk(radius):
    y, x = np.ogrid[-radius:radius + 1, -radius:radius + 1]
    return (x * x + y * y) <= radius * radius


def drop_small(mask, min_size):
    lab, n = ndimage.label(mask)
    if n == 0:
        return mask
    sizes = ndimage.sum(mask, lab, range(n + 1))
    keep = sizes >= min_size
    keep[0] = False
    return keep[lab]


# Bridge the dotted walls into a continuous line before filtering speckle.
walls_closed = ndimage.binary_closing(walls, structure=disk(WALL_CLOSE_RADIUS))
walls_clean = drop_small(walls_closed, MIN_WALL_SIZE)

track_clean = drop_small(track, MIN_TRACK_SIZE)
outside_clean = drop_small(outside, MIN_OUTSIDE_SIZE)

# Fill pinholes inside the outside region so it is a single solid enclosure.
outside_filled = ndimage.binary_fill_holes(
    outside_clean | walls_clean) & ~walls_clean
outside_clean = outside_filled | outside_clean

print(f"walls:   {walls.sum()} -> {walls_clean.sum()}")
print(f"track:   {track.sum()} -> {track_clean.sum()}")
print(f"outside: {outside.sum()} -> {outside_clean.sum()}")

# Compose: start from track, stamp outside, then walls win everywhere.
out = np.full_like(arr, TRACK, dtype=np.uint8)
out[outside_clean] = OUTSIDE
out[walls_clean] = WALL

# Pixels in no surviving class adopt the value of their nearest kept pixel,
# which keeps the three-region boundaries sharp.
dropped = ~(walls_clean | track_clean | outside_clean)
if dropped.any():
    kept = ~dropped
    _, (iy, ix) = ndimage.distance_transform_edt(~kept, return_indices=True)
    out[dropped] = out[iy[dropped], ix[dropped]]

vals, counts = np.unique(out, return_counts=True)
Image.fromarray(out, mode="L").save(DST)
print(f"saved {DST}  final values={dict(zip(vals.tolist(), counts.tolist()))}")
