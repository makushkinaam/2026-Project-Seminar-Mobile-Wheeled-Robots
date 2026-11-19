import numpy as np
from PIL import Image
from pathlib import Path

resolution = 0.01
origin = [-3.03, -3.03, 0.0]

width = 606
height = 606

map_image = np.full((height, width), 254, dtype=np.uint8)


def draw_box(cx, cy, sx, sy):
    """Рисует прямоугольное препятствие в координатах Webots."""

    x_min = cx - sx / 2
    x_max = cx + sx / 2
    y_min = cy - sy / 2
    y_max = cy + sy / 2

    px_left = round((x_min - origin[0]) / resolution)
    px_right = round((x_max - origin[0]) / resolution)

    row_top = round((origin[1] + height * resolution - y_max) / resolution)
    row_bottom = round((origin[1] + height * resolution - y_min) / resolution)

    px_left = max(0, min(width, px_left))
    px_right = max(0, min(width, px_right))
    row_top = max(0, min(height, row_top))
    row_bottom = max(0, min(height, row_bottom))

    map_image[row_top:row_bottom, px_left:px_right] = 0


wall_thickness = 0.03

draw_box(0, -3, 6, wall_thickness)
draw_box(0, 3, 6, wall_thickness)
draw_box(-3, 0, wall_thickness, 6)
draw_box(3, 0, wall_thickness, 6)


num0_x, num0_y = -1.32, -0.85

num0_boxes = [
    (0, 0, 0.35, 1.3),
    (2.65, 0, 0.35, 1.3),
    (1.325, -0.475, 2.3, 0.35),
    (1.325, 0.475, 2.3, 0.35),
]

for x, y, sx, sy in num0_boxes:
    draw_box(num0_x + x, num0_y + y, sx, sy)


num8_x, num8_y = -1.32, 0.85

num8_boxes = [
    (0, 0, 0.35, 0.6),
    (2.65, 0, 0.35, 0.6),
    (0.6625, -0.475, 0.975, 0.35),
    (1.9875, 0.475, 0.975, 0.35),
    (0.6625, 0.475, 0.975, 0.35),
    (1.9875, -0.475, 0.975, 0.35),
    (1.325, 0, 0.35, 0.6),
]

for x, y, sx, sy in num8_boxes:
    draw_box(num8_x + x, num8_y + y, sx, sy)


output_path = Path(__file__).resolve().parent / "map.pgm"
output_path.parent.mkdir(parents=True, exist_ok=True)

#map_image = np.rot90(map_image, k=-1)

image = Image.fromarray(map_image, mode="L")

with open("map.pgm", "w") as f:
    f.write("P2\n")
    f.write(f"{width} {height}\n")
    f.write("255\n")

    for row in map_image:
        f.write(" ".join(map(str, row)) + "\n")

image.save("map.png")

print(f"Карта сохранена: {output_path}")
print(f"Размер карты: {map_image.shape}")
