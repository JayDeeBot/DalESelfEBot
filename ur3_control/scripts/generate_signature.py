from svgpathtools import svg2paths
import numpy as np
import json
import matplotlib.pyplot as plt

# ==== Define target box corners ====
# Order: top-left, top-right, bottom-right, bottom-left
target_box = np.array([
    [0.0, 0.01],  # Top-left
    [0.06, 0.01],  # Top-right
    [0.06, 0.0],  # Bottom-right
    [0.0, 0.0],  # Bottom-left
])

z_height = 0.0
splines = []

# ==== Load and parse SVG ====
paths, _ = svg2paths('/home/jarred/git/DalESelfEBot/ur3_control/signature/signature.svg')

# ==== Get bounding box of signature ====
all_points = []

for path in paths:
    for segment in path:
        for t in np.linspace(0, 1, 20):
            point = segment.point(t)
            all_points.append([point.real, point.imag])

all_points = np.array(all_points)
min_xy = np.min(all_points, axis=0)
max_xy = np.max(all_points, axis=0)
signature_width = max_xy[0] - min_xy[0]
signature_height = max_xy[1] - min_xy[1]

# ==== Define transformation to map signature into the box ====
# We'll use top-left and bottom-right of target box to define scale and offset
box_top_left = target_box[0]
box_bottom_right = target_box[2]
box_width = box_bottom_right[0] - box_top_left[0]
box_height = box_top_left[1] - box_bottom_right[1]

def map_point(p):
    x_norm = (p[0] - min_xy[0]) / signature_width
    y_norm = 1.0 - (p[1] - min_xy[1]) / signature_height  # 🔄 flip Y-axis
    x_mapped = box_top_left[0] + x_norm * box_width
    y_mapped = box_bottom_right[1] + y_norm * box_height
    return [x_mapped, y_mapped, z_height]

# ==== Generate splines ====
for idx, path in enumerate(paths):
    waypoints = []
    for segment in path:
        for t in np.linspace(0, 1, 20):
            point = segment.point(t)
            mapped_point = map_point([point.real, point.imag])
            waypoints.append(mapped_point)

    if len(waypoints) > 1:
        splines.append({
            "id": idx + 1,
            "waypoints": waypoints
        })

# ==== Combine Splines ====
def merge_splines(spline_list, ids_to_merge):
    # Get and merge all waypoints from given IDs
    combined_waypoints = []
    for sid in ids_to_merge:
        spline = next((s for s in spline_list if s["id"] == sid), None)
        if spline:
            combined_waypoints.extend(spline["waypoints"])
    return combined_waypoints

# 1. Merge splines 2 to 10
combined_2_10 = merge_splines(splines, list(range(2, 11)))
# 2. Merge splines 12 and 13
combined_12_13 = merge_splines(splines, [12, 13])

# 3. Filter out original splines
all_original_ids = {s["id"] for s in splines}
merged_ids = set(range(2, 11)).union([12, 13])
keep_ids = all_original_ids - merged_ids
splines = [s for s in splines if s["id"] in keep_ids]

# 4. Append new merged splines
max_id = max(s["id"] for s in splines)
splines.append({
    "id": max_id + 1,
    "waypoints": combined_2_10
})
splines.append({
    "id": max_id + 2,
    "waypoints": combined_12_13
})

# ==== Save JSON ====
with open('/home/jarred/git/DalESelfEBot/ur3_control/signature/signature.json', 'w') as f:
    json.dump({"splines": splines}, f, indent=4)

print("✅ Saved signature.json with", len(splines), "splines.")

# ==== Visualise Result ====
plt.figure(figsize=(6, 6))
for spline in splines:
    wp = np.array(spline["waypoints"])
    plt.plot(wp[:, 0], wp[:, 1], linewidth=1)

# Draw target box for reference
target_box_closed = np.vstack([target_box, target_box[0]])  # close the loop
plt.plot(target_box_closed[:, 0], target_box_closed[:, 1], 'r--', label='Target Box')

plt.gca().invert_yaxis()
plt.title("Mapped Signature in Target Box")
plt.axis("equal")
plt.axis("on")
plt.legend()
plt.tight_layout()
plt.show()
