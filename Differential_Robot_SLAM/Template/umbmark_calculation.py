"""
UMBmark 10-run calculation template.
Input: 5 CW + 5 CCW endpoint errors.
Coordinate convention must match the UMBmark equations used here.
"""

import math
from statistics import mean

# ------------------------------------------------------------
# USER INPUT
# ------------------------------------------------------------
L = 4.0                 # square side [m]
b_nominal = 0.30       # nominal wheelbase [m]
D_avg = 0.10            # measured/calibrated average wheel diameter [m]

# Each tuple: (dX, dY), where dX = X_abs - X_odom
CW = [
    # (dX1, dY1),
    # (dX2, dY2),
    # (dX3, dY3),
    # (dX4, dY4),
    # (dX5, dY5),
]

CCW = [
    # (dX1, dY1),
    # (dX2, dY2),
    # (dX3, dY3),
    # (dX4, dY4),
    # (dX5, dY5),
]

if len(CW) != 5 or len(CCW) != 5:
    raise ValueError("UMBmark requires 5 CW runs and 5 CCW runs.")

x_cw = mean(x for x, y in CW)
y_cw = mean(y for x, y in CW)
x_ccw = mean(x for x, y in CCW)
y_ccw = mean(y for x, y in CCW)

r_cw = math.hypot(x_cw, y_cw)
r_ccw = math.hypot(x_ccw, y_ccw)
Emax = max(r_cw, r_ccw)

# Type A
alpha_x = -(x_cw + x_ccw) / (4 * L) * 180 / math.pi
alpha_y = -(y_cw - y_ccw) / (4 * L) * 180 / math.pi
alpha = (alpha_x + alpha_y) / 2

# Type B
beta_x = -(x_cw - x_ccw) / (4 * L) * 180 / math.pi
beta_y = -(y_cw + y_ccw) / (4 * L) * 180 / math.pi
beta = (beta_x + beta_y) / 2

# Wheelbase correction
Eb = 90 / (90 - alpha)
b_actual = Eb * b_nominal

# Unequal wheel diameter correction
if abs(beta) < 1e-12:
    R = math.inf
    Ed = 1.0
else:
    beta_rad = math.radians(beta)
    R = (L / 2) / math.sin(beta_rad / 2)
    Ed = (R + b_nominal / 2) / (R - b_nominal / 2)

cL = 2 / (Ed + 1)
cR = 2 / ((1 / Ed) + 1)

D_L = D_avg * cL
D_R = D_avg * cR

print("\n=== UMBmark Result ===")
print(f"CW centroid  : ({x_cw:.6f}, {y_cw:.6f}) m")
print(f"CCW centroid : ({x_ccw:.6f}, {y_ccw:.6f}) m")
print(f"CW radius    : {r_cw:.6f} m")
print(f"CCW radius   : {r_ccw:.6f} m")
print(f"Emax,syst    : {Emax:.6f} m ({Emax*1000:.2f} mm)")

print("\n--- Type A / Wheelbase ---")
print(f"alpha_x     : {alpha_x:.6f} deg")
print(f"alpha_y     : {alpha_y:.6f} deg")
print(f"alpha_UMB   : {alpha:.6f} deg")
print(f"Eb          : {Eb:.9f}")
print(f"b_actual    : {b_actual:.6f} m")

print("\n--- Type B / Wheel Diameter ---")
print(f"beta_x      : {beta_x:.6f} deg")
print(f"beta_y      : {beta_y:.6f} deg")
print(f"beta_UMB    : {beta:.6f} deg")
print(f"R           : {R:.6f} m")
print(f"Ed = DR/DL  : {Ed:.9f}")
print(f"cL          : {cL:.9f}")
print(f"cR          : {cR:.9f}")
print(f"D_L         : {D_L:.6f} m")
print(f"D_R         : {D_R:.6f} m")
