# Stanley Controller Derivation

## Reference
Stanley: The Robot That Won the DARPA Grand Challenge (Thrun et al., 2006)

## Setup
- Vehicle front axle position: p_fa = [px + L*cos(theta), py + L*sin(theta)]
- Closest point on trajectory to front axle: p_r = [x_r, y_r]
- Track heading at that point: theta_r

## Two Error Terms

### 1. Heading Error
psi_e = theta_r - theta
Wrapped to [-pi, pi].

### 2. Cross-Track Error (signed)
e_fa = signed distance from front axle to trajectory.
Positive = front axle is to the left of the track direction.

Computed as:
    track_vec = [cos(theta_r), sin(theta_r)]
    perp_vec  = [-sin(theta_r), cos(theta_r)]   (points left of track)
    e_fa = dot(p_fa - p_r, perp_vec)

## Control Law
    delta = psi_e - arctan(k * e_fa / (v + eps))

Note: the minus sign follows from the LEFT-normal convention for e_fa above.
e_fa > 0 means the vehicle is to the LEFT of track → needs a RIGHT correction → negative delta.
With the right-normal convention (e_fa positive = right of track), the formula would use a plus sign.

- k: cross-track gain (tunable). Larger k = more aggressive CTE correction.
- eps = 1e-3: prevents division by zero at standstill.
- arctan saturates the CTE term — natural anti-windup.

## Properties
- No solver, no matrix ops. Always < 0.1ms.
- At high speed: arctan term small → heading error dominates.
- At low speed: arctan term large → aggressive CTE correction.
- Output clipped to [-DELTA_MAX, DELTA_MAX].
