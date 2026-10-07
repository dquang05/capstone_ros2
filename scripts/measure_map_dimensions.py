#!/usr/bin/env python3
"""
Interactive Restaurant Floor Plan Measuring & Annotation Tool
Project: Autonomous Mobile Robot (AMR) for Food Delivery
Description:
    Loads a floor plan image (e.g. docs/figures/restaurent_map.jpg),
    calibrates real-world scale (pixels per meter) using a scale bar,
    and allows interactive measuring of corridors, doorways, and table gaps.
    Saves publication-quality annotated figures and measurement tables for the thesis.

Usage:
    python3 scripts/measure_map_dimensions.py
    python3 scripts/measure_map_dimensions.py --image docs/figures/restaurent_map.jpg
"""

import os
import sys
import math
import argparse
import numpy as np
# pyrefly: ignore [missing-import]
import cv2
from PIL import Image, ImageDraw, ImageFont


class FloorPlanMeasurer:
    def __init__(self, image_path, output_path=None, pre_scale=None):
        # Resolve path relative to workspace root if needed
        script_dir = os.path.dirname(os.path.abspath(__file__))
        workspace_root = os.path.abspath(os.path.join(script_dir, ".."))

        if not os.path.exists(image_path):
            alt_path = os.path.join(workspace_root, image_path)
            if os.path.exists(alt_path):
                image_path = alt_path
            else:
                raise FileNotFoundError(f"Image file not found: {image_path} (also checked {alt_path})")

        self.image_path = os.path.abspath(image_path)

        if output_path:
            if not os.path.isabs(output_path):
                self.output_path = os.path.abspath(os.path.join(workspace_root, output_path))
            else:
                self.output_path = os.path.abspath(output_path)
        else:
            self.output_path = os.path.join(
                os.path.dirname(self.image_path), "restaurant_map_measured.png"
            )

        # Load original image
        self.orig_img = cv2.imread(self.image_path)
        if self.orig_img is None:
            raise ValueError(f"Failed to load image from {self.image_path}")

        self.h, self.w = self.orig_img.shape[:2]

        # Display image copy
        self.display_img = self.orig_img.copy()

        # Calibration state
        # Known default for restaurent_map.jpg: 5m = 225 px => 45.0 px/m
        self.scale_px_per_m = pre_scale
        self.is_calibrated = (pre_scale is not None and pre_scale > 0)

        # Dedicated calibration reference line (drawn on scale bar)
        self.calibration_line = None
        if self.is_calibrated and pre_scale == 45.0:
            self.calibration_line = {
                'p1': (939, 815),
                'p2': (1164, 815),
                'dist_m': 5.0,
                'color': (0, 0, 200)  # Red
            }

        # Calibration points for manual picking
        self.calib_pt1 = None
        self.calib_pt2 = None

        # Measurement data: list of dicts:
        # {'id': int, 'p1': (x,y), 'p2': (x,y), 'dist_m': float, 'label': str}
        self.measurements = []

        # Mouse interaction state
        self.mouse_down = False
        self.drag_start = None
        self.current_cursor = None
        self.click_pending = None  # For click-then-click mode

        # Window settings
        self.window_name = "AMR Floor Plan Measurer - [Press H for Help]"
        self.hud_visible = True

        # TrueType fonts for crisp Unicode Vietnamese text rendering
        font_paths = [
            "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
            "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
            "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
        ]
        self.font_path = None
        for fp in font_paths:
            if os.path.exists(fp):
                self.font_path = fp
                break

        try:
            if self.font_path:
                self.pil_font_badge = ImageFont.truetype(self.font_path, 13)
                self.pil_font_hud = ImageFont.truetype(self.font_path, 14)
                self.pil_font_sub = ImageFont.truetype(self.font_path, 11)
            else:
                self.pil_font_badge = ImageFont.load_default()
                self.pil_font_hud = ImageFont.load_default()
                self.pil_font_sub = ImageFont.load_default()
        except Exception:
            self.pil_font_badge = ImageFont.load_default()
            self.pil_font_hud = ImageFont.load_default()
            self.pil_font_sub = ImageFont.load_default()

        # Text render queues
        self.text_overlays = []

    def calculate_distance(self, p1, p2):
        """Calculates Euclidean pixel distance and real-world meter distance."""
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        px_dist = math.hypot(dx, dy)
        if self.is_calibrated and self.scale_px_per_m > 0:
            meter_dist = px_dist / self.scale_px_per_m
        else:
            meter_dist = None
        return px_dist, meter_dist

    def draw_dimension_line(self, img, p1, p2, dist_m=None, label=None,
                            color=(220, 20, 60), is_preview=False, is_scale_ref=False):
        """Draws clean CAD dimension line with perpendicular ticks and floating text (no box)."""
        p1 = (int(p1[0]), int(p1[1]))
        p2 = (int(p2[0]), int(p2[1]))

        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        length = math.hypot(dx, dy)
        if length < 1e-3:
            return

        # Unit vectors
        ux = dx / length
        uy = dy / length
        # Perpendicular normal vector
        nx = -uy
        ny = ux

        tick_len = 8 if not is_preview else 5

        # 1. Main measurement line
        thickness = 1 if is_preview else 2
        cv2.line(img, p1, p2, color, thickness, cv2.LINE_AA)

        # 2. End ticks and endpoint markers
        for pt in [p1, p2]:
            t1 = (int(pt[0] + nx * tick_len), int(pt[1] + ny * tick_len))
            t2 = (int(pt[0] - nx * tick_len), int(pt[1] - ny * tick_len))
            cv2.line(img, t1, t2, color, 2 if not is_preview else 1, cv2.LINE_AA)
            cv2.circle(img, pt, 3 if not is_preview else 2, color, -1, cv2.LINE_AA)

        # If it's the scale reference line, draw guide lines down to the scale bar ticks
        if is_scale_ref:
            cv2.line(img, (p1[0], p1[1]), (p1[0], p1[1] + 25), color, 1, cv2.LINE_AA)
            cv2.line(img, (p2[0], p2[1]), (p2[0], p2[1] + 25), color, 1, cv2.LINE_AA)

        # 3. Label text: Only length value, no "Đoạn 1 2 3", no box
        if is_scale_ref:
            text = f"{dist_m:.2f} m (Chuẩn tỉ lệ)"
        elif dist_m is not None:
            text = f"{dist_m:.2f} m"
        else:
            text = f"{int(length)} px"

        # Calculate bounding box using PIL font
        bbox = self.pil_font_badge.getbbox(text)
        tw = bbox[2] - bbox[0]
        th = bbox[3] - bbox[1]

        # Offset perpendicular to line so text sits cleanly above/beside line
        offset_dist = -13 if is_scale_ref else 12
        cx = int((p1[0] + p2[0]) / 2 + nx * offset_dist)
        cy = int((p1[1] + p2[1]) / 2 + ny * offset_dist)

        tx = cx - tw // 2
        ty = cy - th // 2

        # Keep text within image bounds
        tx = max(4, min(self.w - tw - 6, tx))
        ty = max(4, min(self.h - th - 6, ty))

        # Register text for unicode overlay pass with white stroke halo (no box)
        text_color = color if not is_preview else (80, 80, 80)
        self.text_overlays.append({
            'text': text,
            'pos': (tx, ty),
            'font': self.pil_font_badge,
            'color': text_color,
            'stroke': True
        })

    def draw_hud(self, img):
        """Draws top status bar and guide."""
        if not self.hud_visible:
            return

        overlay = img.copy()
        hud_h = 60
        cv2.rectangle(overlay, (0, 0), (self.w, hud_h), (25, 30, 42), -1)
        cv2.addWeighted(overlay, 0.88, img, 0.12, 0, img)
        cv2.line(img, (0, hud_h), (self.w, hud_h), (0, 160, 255), 2)

        # Row 1: Mode & Scale
        if self.is_calibrated:
            calib_str = f"TỈ LỆ: {self.scale_px_per_m:.2f} px/m (Đã cân bằng)"
            calib_col = (80, 240, 80)
            mode_str = f"CHẾ ĐỘ ĐO LỐI ĐI | Đã đo: {len(self.measurements)} đoạn"
        else:
            calib_str = "TỈ LỆ: CHƯA CÂN BẰNG (Click 2 điểm thước đo 5m hoặc nhấn 'D')"
            calib_col = (255, 180, 50)
            mode_str = "CHẾ ĐỘ CÂN CHỈNH TỈ LỆ (CALIBRATION)"

        keys_str = "[Kéo chuột / 2 Click]: Đo | [U]: Undo | [C]: Xóa hết | [S]: Lưu ảnh | [T]: Bảng số liệu | [D]: Tỉ lệ chuẩn | [Q]: Thoát"

        self.text_overlays.append({
            'text': mode_str,
            'pos': (15, 8),
            'font': self.pil_font_hud,
            'color': (255, 255, 255),
            'stroke': False
        })
        self.text_overlays.append({
            'text': calib_str,
            'pos': (self.w - 530, 8),
            'font': self.pil_font_hud,
            'color': calib_col,
            'stroke': False
        })
        self.text_overlays.append({
            'text': keys_str,
            'pos': (15, 34),
            'font': self.pil_font_sub,
            'color': (200, 215, 230),
            'stroke': False
        })

        # If calibration point 1 is picked, show marker
        if self.calib_pt1:
            cv2.circle(img, self.calib_pt1, 6, (0, 140, 255), -1, cv2.LINE_AA)
            self.text_overlays.append({
                'text': "Mốc 1",
                'pos': (self.calib_pt1[0] + 8, self.calib_pt1[1] - 18),
                'font': self.pil_font_sub,
                'color': (0, 140, 255),
                'stroke': True
            })

    def apply_text_overlays(self, img):
        """Flushes all queued text overlays in a single fast PIL pass with white halo outline."""
        if not self.text_overlays:
            return img

        pil_img = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        draw = ImageDraw.Draw(pil_img)

        for item in self.text_overlays:
            text = item['text']
            pos = item['pos']
            font = item['font']
            color = item['color']
            has_stroke = item.get('stroke', False)
            if has_stroke:
                draw.text(pos, text, font=font, fill=color, stroke_width=2, stroke_fill=(255, 255, 255))
            else:
                draw.text(pos, text, font=font, fill=color)

        self.text_overlays.clear()
        return cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)

    def render(self):
        """Redraws the canvas with all measurements and current preview."""
        img = self.orig_img.copy()

        # 1. Draw Calibration Reference Line if active
        if self.calibration_line:
            c = self.calibration_line
            self.draw_dimension_line(img, c['p1'], c['p2'], dist_m=c['dist_m'],
                                     color=c['color'], is_scale_ref=True)

        # 2. Draw user measurements
        colors = [
            (210, 30, 30),    # Crimson Red
            (30, 140, 210),   # Sky Blue
            (34, 139, 34),    # Forest Green
            (148, 0, 211),    # Dark Violet
            (218, 112, 214),  # Orchid
            (205, 133, 63),   # Peru Brown
            (0, 128, 128),    # Teal
        ]

        for idx, m in enumerate(self.measurements):
            col = colors[idx % len(colors)]
            self.draw_dimension_line(img, m['p1'], m['p2'], m['dist_m'], color=col)

        # 3. Render active preview line if mouse is dragging or click pending
        preview_p1 = None
        if self.mouse_down and self.drag_start and self.current_cursor:
            preview_p1 = self.drag_start
        elif self.click_pending and self.current_cursor:
            preview_p1 = self.click_pending

        if preview_p1 and self.current_cursor:
            px_d, m_d = self.calculate_distance(preview_p1, self.current_cursor)
            preview_col = (0, 140, 255) if not self.is_calibrated else (255, 120, 0)
            self.draw_dimension_line(img, preview_p1, self.current_cursor, m_d, is_preview=True, color=preview_col)

        # 4. Draw HUD
        self.draw_hud(img)

        # Single-pass Unicode text overlay
        self.display_img = self.apply_text_overlays(img)
        try:
            cv2.imshow(self.window_name, self.display_img)
        except Exception:
            pass

    def on_mouse(self, event, x, y, flags, param):
        """Mouse event callback supporting both drag-and-drop and click-then-click."""
        self.current_cursor = (x, y)

        if event == cv2.EVENT_LBUTTONDOWN:
            if not self.is_calibrated:
                # Calibration mode: pick 2 points
                if self.calib_pt1 is None:
                    self.calib_pt1 = (x, y)
                    print(f"\n[Calib] Điểm 1: ({x}, {y}). Hãy click Điểm 2 (ví dụ mốc 5m)...")
                else:
                    self.calib_pt2 = (x, y)
                    px_len = math.hypot(self.calib_pt2[0] - self.calib_pt1[0],
                                        self.calib_pt2[1] - self.calib_pt1[1])
                    print(f"[Calib] Điểm 2: ({x}, {y}) | Khoảng cách pixel: {px_len:.1f} px")

                    default_meters = 5.0
                    print(f">> Nhập khoảng cách thực tế giữa 2 điểm (m) [Mặc định: {default_meters}m]: ", end="", flush=True)
                    try:
                        user_in = input().strip()
                        real_m = float(user_in) if user_in else default_meters
                    except Exception:
                        real_m = default_meters

                    self.scale_px_per_m = px_len / real_m
                    self.is_calibrated = True
                    self.calibration_line = {
                        'p1': self.calib_pt1,
                        'p2': self.calib_pt2,
                        'dist_m': real_m,
                        'color': (0, 0, 200)
                    }
                    print(f"[THÀNH CÔNG] Đã hiệu chỉnh tỉ lệ: {self.scale_px_per_m:.2f} pixels/mét (1m = {self.scale_px_per_m:.2f} px)")
                    self.calib_pt1 = None
                    self.calib_pt2 = None
                self.render()
                return

            # Measurement mode
            if self.click_pending is None:
                self.mouse_down = True
                self.drag_start = (x, y)
            else:
                # Second click of click-then-click mode
                p1 = self.click_pending
                p2 = (x, y)
                self.click_pending = None
                self.add_measurement(p1, p2)
            self.render()

        elif event == cv2.EVENT_MOUSEMOVE:
            if self.mouse_down or self.click_pending:
                self.render()

        elif event == cv2.EVENT_LBUTTONUP:
            if self.mouse_down and self.drag_start:
                p1 = self.drag_start
                p2 = (x, y)
                self.mouse_down = False
                self.drag_start = None

                # Check if it was a tiny click or a real drag
                dist = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
                if dist < 4:
                    self.click_pending = p1
                else:
                    self.add_measurement(p1, p2)
                self.render()

        elif event == cv2.EVENT_RBUTTONDOWN:
            if self.click_pending:
                self.click_pending = None
                self.render()

    def add_measurement(self, p1, p2):
        """Adds a completed measurement line."""
        px_d, m_d = self.calculate_distance(p1, p2)
        if px_d < 3:
            return

        m_id = len(self.measurements) + 1
        m_item = {
            'id': m_id,
            'p1': p1,
            'p2': p2,
            'dist_px': px_d,
            'dist_m': m_d if m_d is not None else 0.0,
            'label': f"Đoạn #{m_id}"
        }
        self.measurements.append(m_item)
        print(f"[ĐO ĐẠC #{m_id}] ({p1[0]},{p1[1]}) -> ({p2[0]},{p2[1]}) = {m_d:.2f} m ({px_d:.1f} px)")

    def auto_calibrate_default(self):
        """Auto-calibrates based on the ground truth 5m scale bar (939 to 1164 px => 45.0 px/m)."""
        self.scale_px_per_m = 45.0
        self.is_calibrated = True
        self.calibration_line = {
            'p1': (939, 815),
            'p2': (1164, 815),
            'dist_m': 5.0,
            'color': (0, 0, 200)
        }
        print("\n[AUTO-CALIB] Đã thiết lập tỉ lệ chuẩn từ thước đo 5m ở góc phải:")
        print("             45.0 pixels/mét (5.0m = 225 pixels). Đã vẽ đoạn kích thước chuẩn 5.0m trên thước!")
        self.render()

    def undo(self):
        if self.measurements:
            removed = self.measurements.pop()
            print(f"[UNDO] Đã xóa đoạn #{removed['id']} ({removed['dist_m']:.2f} m)")
            self.render()
        else:
            print("[UNDO] Danh sách rỗng, không có đoạn nào để xóa.")

    def clear_all(self):
        if self.measurements:
            self.measurements.clear()
            print("[CLEAR] Đã xóa toàn bộ các đoạn đo đạc.")
            self.render()

    def print_table(self):
        """Prints a publication-ready formatted ASCII table."""
        if not self.measurements:
            print("\n[BẢNG KÍCH THƯỚC] Chưa có đoạn nào được đo.")
            return

        print("\n" + "=" * 76)
        print(f"{'BẢNG TỔNG HỢP KÍCH THƯỚC LỐI ĐI VÀ KHÔNG GIAN NHÀ HÀNG':^76}")
        print("=" * 76)
        print(f"{'STT':<6} | {'Tên / Vị trí':<22} | {'Tọa độ Pixel (P1 -> P2)':<24} | {'Kích thước (m)':<14}")
        print("-" * 76)
        for m in self.measurements:
            pts_str = f"({m['p1'][0]},{m['p1'][1]}) -> ({m['p2'][0]},{m['p2'][1]})"
            lbl = m['label']
            print(f"#{m['id']:<5} | {lbl:<22} | {pts_str:<24} | {m['dist_m']:>6.2f} m")
        print("=" * 76)
        print(f"Robot AMR Footprint: Dài = 0.76 m, Rộng = 0.60 m | Bán kính an toàn: 0.48 m")
        print(f"-> Tiêu chuẩn hành lang 1 chiều an toàn tối thiểu: >= 0.90 m - 1.10 m")
        print("=" * 76 + "\n")

    def save_image(self):
        """Saves high-res clean annotated image without the HUD overlay."""
        clean_img = self.orig_img.copy()

        # 1. Draw Calibration Reference Line if active
        if self.calibration_line:
            c = self.calibration_line
            self.draw_dimension_line(clean_img, c['p1'], c['p2'], dist_m=c['dist_m'],
                                     color=c['color'], is_scale_ref=True)

        # 2. Draw user measurements
        colors = [
            (210, 30, 30), (30, 140, 210), (34, 139, 34),
            (148, 0, 211), (218, 112, 214), (205, 133, 63), (0, 128, 128)
        ]
        self.text_overlays.clear()
        for idx, m in enumerate(self.measurements):
            col = colors[idx % len(colors)]
            self.draw_dimension_line(clean_img, m['p1'], m['p2'], m['dist_m'], color=col)

        # Footer info
        if self.is_calibrated:
            text_info = f"Tỉ lệ: 1m = {self.scale_px_per_m:.1f}px | Tổng số đoạn đo: {len(self.measurements)} | Khổ rộng robot: 0.60m"
            self.text_overlays.append({
                'text': text_info,
                'pos': (15, self.h - 22),
                'font': self.pil_font_sub,
                'color': (60, 60, 60),
                'stroke': False
            })

        final_img = self.apply_text_overlays(clean_img)
        os.makedirs(os.path.dirname(self.output_path), exist_ok=True)
        cv2.imwrite(self.output_path, final_img)
        print(f"\n[THÀNH CÔNG] Đã lưu bản vẽ đo kích thước tại: {self.output_path}")

    def run(self):
        """Main loop."""
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, min(1400, self.w), min(950, self.h))
        cv2.setMouseCallback(self.window_name, self.on_mouse)

        print("\n" + "=" * 70)
        print(f"   CÔNG CỤ ĐO ĐẠC MẶT BẰNG NHÀ HÀNG (RESTAURANT MAP MEASURER)")
        print("=" * 70)
        print(" * Bấm phím 'D': Tự động nạp tỉ lệ chuẩn từ thanh 5m (45.0 px/m)")
        print(" * Hoặc Click 2 điểm trên thanh tỉ lệ để tự Calibrate")
        print(" * Kéo chuột hoặc Click 2 điểm bất kỳ để đo chiều rộng lối đi")
        print(" * Phím 'U': Undo (xóa đoạn vừa vẽ)")
        print(" * Phím 'C': Xóa hết")
        print(" * Phím 'S': Lưu ảnh đã gắn kích thước (vào docs/figures/)")
        print(" * Phím 'T': In bảng số liệu ra terminal để chép vào thuyết minh")
        print(" * Phím 'R': Calibrate lại tỉ lệ")
        print(" * Phím 'Q' hoặc ESC: Thoát")
        print("=" * 70 + "\n")

        self.render()

        while True:
            key = cv2.waitKey(20) & 0xFF
            if key == ord('q') or key == 27:  # Q or ESC
                break
            elif key == ord('d') or key == ord('D'):
                self.auto_calibrate_default()
            elif key == ord('u') or key == ord('U'):
                self.undo()
            elif key == ord('c') or key == ord('C'):
                self.clear_all()
            elif key == ord('s') or key == ord('S'):
                self.save_image()
            elif key == ord('t') or key == ord('T'):
                self.print_table()
            elif key == ord('r') or key == ord('R'):
                self.is_calibrated = False
                self.calib_pt1 = None
                self.calib_pt2 = None
                self.calibration_line = None
                print("\n[CALIB] Đã đặt lại. Hãy click 2 điểm mốc để cân chỉnh tỉ lệ mới...")
                self.render()
            elif key == ord('h') or key == ord('H'):
                self.hud_visible = not self.hud_visible
                self.render()

        cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Floor Plan Measurement Tool for AMR Project")
    parser.add_argument("--image", default="docs/figures/restaurent_map.jpg",
                        help="Path to floor plan image (default: docs/figures/restaurent_map.jpg)")
    parser.add_argument("--scale", type=float, default=None,
                        help="Pre-calibrated pixels per meter (optional)")
    parser.add_argument("--output", default="docs/figures/restaurant_map_measured.png",
                        help="Path to save annotated output image")

    args = parser.parse_args()

    # Auto-detect DISPLAY if running on VM desktop
    if 'DISPLAY' not in os.environ:
        if os.path.exists('/tmp/.X11-unix/X1'):
            os.environ['DISPLAY'] = ':1'
        elif os.path.exists('/tmp/.X11-unix/X0'):
            os.environ['DISPLAY'] = ':0'

    measurer = FloorPlanMeasurer(args.image, output_path=args.output, pre_scale=args.scale)
    measurer.run()


if __name__ == "__main__":
    main()
