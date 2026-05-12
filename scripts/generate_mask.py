import argparse
import json
from pathlib import Path

import numpy as np
import yaml
from matplotlib import pyplot as plt
from matplotlib.widgets import Button, Slider
from PIL import Image


def load_map(yaml_path: Path):
    yaml_path = yaml_path.resolve()
    with open(yaml_path) as f:
        config = yaml.safe_load(f)
    image_path = (yaml_path.parent / config["image"]).resolve()
    image = np.array(Image.open(image_path).convert("L"))
    return image, config


class MaskEditor:
    OVERLAY_RGBA = (0.0, 1.0, 0.0, 0.45)

    def __init__(self, image: np.ndarray, config: dict, output_path: Path):
        self.image = image
        self.config = config
        self.output_path = output_path
        self.mask = np.zeros(image.shape, dtype=bool)
        self.brush_radius = 10
        self._paint_value: bool | None = None

        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.fig.subplots_adjust(bottom=0.18)
        self.ax.imshow(image, cmap="gray", vmin=0, vmax=255, interpolation="nearest")
        self.overlay = self.ax.imshow(self._overlay_rgba(), interpolation="nearest")
        self.ax.set_title(
            "LMB drag: paint   RMB drag: erase   Save writes JSON; closing also saves"
        )
        self.ax.set_xticks([])
        self.ax.set_yticks([])

        self.brush_slider = Slider(
            self.fig.add_axes([0.15, 0.08, 0.55, 0.03]),
            "Brush",
            1,
            max(100, min(image.shape) // 4),
            valinit=self.brush_radius,
            valstep=1,
        )
        self.brush_slider.on_changed(self._set_brush)

        self.clear_button = Button(self.fig.add_axes([0.76, 0.075, 0.08, 0.04]), "Clear")
        self.clear_button.on_clicked(self._clear)

        self.save_button = Button(self.fig.add_axes([0.86, 0.075, 0.08, 0.04]), "Save")
        self.save_button.on_clicked(lambda _: self.save())

        self.fig.canvas.mpl_connect("button_press_event", self._on_press)
        self.fig.canvas.mpl_connect("button_release_event", self._on_release)
        self.fig.canvas.mpl_connect("motion_notify_event", self._on_motion)
        self.fig.canvas.mpl_connect("close_event", lambda _: self.save())

    def _set_brush(self, value):
        self.brush_radius = int(value)

    def _overlay_rgba(self) -> np.ndarray:
        rgba = np.zeros((*self.mask.shape, 4))
        rgba[self.mask] = self.OVERLAY_RGBA
        return rgba

    def _refresh(self):
        self.overlay.set_data(self._overlay_rgba())
        self.fig.canvas.draw_idle()

    def _stamp(self, cx: int, cy: int, value: bool):
        h, w = self.mask.shape
        r = self.brush_radius
        y0, y1 = max(0, cy - r), min(h, cy + r + 1)
        x0, x1 = max(0, cx - r), min(w, cx + r + 1)
        if y0 >= y1 or x0 >= x1:
            return
        yy, xx = np.ogrid[y0 - cy : y1 - cy, x0 - cx : x1 - cx]
        disk = yy * yy + xx * xx <= r * r
        region = self.mask[y0:y1, x0:x1]
        region[disk] = value
        self.mask[y0:y1, x0:x1] = region

    def _cursor_cell(self, event):
        if event.inaxes != self.ax or event.xdata is None or event.ydata is None:
            return None
        return int(round(event.xdata)), int(round(event.ydata))

    def _on_press(self, event):
        cell = self._cursor_cell(event)
        if cell is None:
            return
        if event.button == 1:
            self._paint_value = True
        elif event.button == 3:
            self._paint_value = False
        else:
            return
        self._stamp(cell[0], cell[1], self._paint_value)
        self._refresh()

    def _on_release(self, _event):
        self._paint_value = None

    def _on_motion(self, event):
        if self._paint_value is None:
            return
        cell = self._cursor_cell(event)
        if cell is None:
            return
        self._stamp(cell[0], cell[1], self._paint_value)
        self._refresh()

    def _clear(self, _event):
        self.mask.fill(False)
        self._refresh()

    def save(self):
        payload = {
            "resolution": float(self.config["resolution"]),
            "origin": list(self.config.get("origin", [0.0, 0.0, 0.0])),
            "shape": list(self.mask.shape),
            "mask": self.mask.astype(np.uint8).tolist(),
        }
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.output_path, "w") as f:
            json.dump(payload, f)
        print(
            f"wrote {self.output_path}  "
            f"({int(self.mask.sum())}/{self.mask.size} cells true)"
        )

    def run(self):
        plt.show()


def main():
    parser = argparse.ArgumentParser(
        description="Highlight regions on a ROS2 map.yaml and export a binary mask."
    )
    parser.add_argument("map_yaml", type=Path)
    parser.add_argument("-o", "--output", type=Path, default=None)
    args = parser.parse_args()

    image, config = load_map(args.map_yaml)
    output = args.output or args.map_yaml.with_name(args.map_yaml.stem + ".mask.json")
    MaskEditor(image, config, output).run()


if __name__ == "__main__":
    main()
