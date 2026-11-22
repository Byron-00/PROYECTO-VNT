#!/usr/bin/env python3
"""
Interactive Sakhir obstacle placement.
F1TENTH Autonomous Racing - Part 2.
Author: Byron Ortega.
"""

from pathlib import Path
from PIL import Image, ImageDraw, ImageTk
from tkinter import Tk, Canvas, Label, messagebox
import re

home = Path.home()
repo = home / "PROYECTO-VNT"

original_dir = (
    home / "F1Tenth-Repository"
    / "src/f1tenth_gym_ros/maps"
)

source_png = original_dir / "Sakhir_map.png"
source_yaml = original_dir / "Sakhir_map.yaml"

out_dir = repo / "maps/part2"
media_dir = repo / "media/part2"

out_dir.mkdir(parents=True, exist_ok=True)
media_dir.mkdir(parents=True, exist_ok=True)

if not source_png.exists() or not source_yaml.exists():
    raise SystemExit("ERROR: Original Sakhir files not found.")

img = Image.open(source_png).convert("L")
yaml_text = source_yaml.read_text()

resolution_match = re.search(
    r"(?m)^resolution:\s*([0-9.]+)",
    yaml_text
)

if not resolution_match:
    raise SystemExit("ERROR: Map resolution not found.")

resolution = float(resolution_match.group(1))

# Physical obstacle radius: 20 centimeters.
radius_m = 0.20
radius_px = max(3, round(radius_m / resolution))

scale = min(1100 / img.width, 740 / img.height, 1.0)

preview = img.resize(
    (round(img.width * scale), round(img.height * scale)),
    Image.NEAREST
)

window = Tk()
window.title("SAKHIR - PLACE ONE TEST OBSTACLE")

Label(
    window,
    text=(
        "Click on a free section of the racing track.\n"
        "Avoid the starting position and track boundaries."
    ),
    font=("Arial", 12),
    pady=10
).pack()

photo = ImageTk.PhotoImage(preview)

canvas = Canvas(
    window,
    width=preview.width,
    height=preview.height
)

canvas.pack()
canvas.create_image(0, 0, anchor="nw", image=photo)

def place_obstacle(event):
    px = int(event.x / scale)
    py = int(event.y / scale)

    if not (0 <= px < img.width and 0 <= py < img.height):
        return

    margin = radius_px + 2

    if (
        px < margin or py < margin
        or px >= img.width - margin
        or py >= img.height - margin
    ):
        messagebox.showwarning(
            "Invalid position",
            "Choose another point inside the image."
        )
        return

    # Check the placement region is free.
    for yy in range(py - margin, py + margin + 1):
        for xx in range(px - margin, px + margin + 1):
            if (xx - px)**2 + (yy - py)**2 <= margin**2:
                if img.getpixel((xx, yy)) < 220:
                    messagebox.showwarning(
                        "Obstacle too close to a wall",
                        "Choose a clearer section of the track."
                    )
                    return

    # Create the physical map obstacle.
    modified = img.copy()
    draw = ImageDraw.Draw(modified)

    draw.ellipse(
        (
            px - radius_px,
            py - radius_px,
            px + radius_px,
            py + radius_px
        ),
        fill=0
    )

    png = out_dir / "Sakhir_test_one.png"
    modified.save(png)

    # Create matching map metadata.
    new_yaml = re.sub(
        r"(?m)^image:\s*.*$",
        "image: Sakhir_test_one.png",
        yaml_text
    )

    yaml_path = out_dir / "Sakhir_test_one.yaml"
    yaml_path.write_text(new_yaml)

    # Produce an annotated preview for documentation.
    annotated = img.convert("RGB")
    painter = ImageDraw.Draw(annotated)

    painter.ellipse(
        (
            px - radius_px - 7,
            py - radius_px - 7,
            px + radius_px + 7,
            py + radius_px + 7
        ),
        outline="red",
        width=3
    )

    evidence = media_dir / "02a-obstacle-placement-preview.png"
    annotated.save(evidence)

    print("\nTEST OBSTACLE CREATED")
    print("Pixel coordinates:", px, py)
    print("Radius:", radius_m, "meters")
    print("Map:", png)
    print("YAML:", yaml_path)
    print("Annotated preview:", evidence)
    print("Original Sakhir map: UNCHANGED")

    messagebox.showinfo(
        "Success",
        "Test obstacle created successfully.\n"
        "Original Sakhir map remains unchanged."
    )

    window.destroy()

canvas.bind("<Button-1>", place_obstacle)

window.mainloop()
