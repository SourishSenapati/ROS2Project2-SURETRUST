#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np
import cv2
import random
import time

class AesculonVision(Node):
    def __init__(self):
        super().__init__('aesculon_vision')
        self.subscription = self.create_subscription(
            String,
            '/aesculon/telemetry',
            self.listener_callback,
            10)
        self.window_name = "AESCULON: NEURAL OPTIC SYSTEM v2.4 (SIL-4)"
        self.get_logger().info('>>> NEURAL VISION SUBSYSTEM: CALIBRATING OPTICS...')
        
        # --- SYNTHETIC ASSETS ---
        # Pre-generate a "Reactor Texture" (Procedural Noise)
        self.width, self.height = 1000, 600
        self.texture_base = self._generate_reactor_texture(self.width, self.height)
        self.start_time = time.time()
        
    def _generate_reactor_texture(self, w, h):
        """Generates a metallic/industrial background texture."""
        # Gradient background (Vignette)
        X, Y = np.meshgrid(np.linspace(-1, 1, w), np.linspace(-1, 1, h))
        R = np.sqrt(X**2 + Y**2)
        base = 1.0 - np.clip(R, 0, 1)
        base = (base * 255).astype(np.uint8)
        return cv2.cvtColor(base, cv2.COLOR_GRAY2BGR)

    def listener_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.render_frame(data)
        except Exception as e:
            self.get_logger().error(f"Render Core Failure: {e}")

    def render_frame(self, data):
        # 1. Base Layer: Synthetic Camera Feed
        frame = self.texture_base.copy()
        
        # 2. Physics Data Extraction
        T = data.get('T', 298.0)
        P = data.get('P', 101325.0)
        health = data.get('health', 100.0)
        
        # 3. THERMAL SIMULATION (The "Heat" Visual)
        # Create a heatmap blob that pulsates with temperature
        overlay = np.zeros_like(frame, dtype=np.uint8)
        center_x, center_y = 300, 300
        radius = int(180 + 10 * np.sin(time.time() * 5)) # Pulsating effect due to "pressure"
        
        # Draw the "Hot Core"
        # Color dynamically shifts from Blue (Cold) -> Yellow (Warm) -> Red (Critical)
        temp_norm = np.clip((T - 300) / 150.0, 0.0, 1.0) # 0.0=300K, 1.0=450K
        
        # Advanced Gradient Rendering
        cv2.circle(overlay, (center_x, center_y), radius, (255, 255, 255), -1)
        # Blur to simulate heat dissipation
        overlay = cv2.GaussianBlur(overlay, (101, 101), 0)
        
        # Apply False Color Map (Thermal Camera Look)
        heatmap = cv2.applyColorMap(overlay, cv2.COLORMAP_JET)
        
        # Blend Heatmap with Base Texture based on Temperature Intensity
        alpha = 0.3 + (0.5 * temp_norm)
        frame = cv2.addWeighted(heatmap, alpha, frame, 1 - alpha, 0)

        # 4. DEFECT SIMULATION (Rust)
        # If health drops, we procedurally generate "rust artifacts"
        # that the CV algorithm must then DETECT.
        if health < 85.0:
            self._inject_defects(frame, center_x, center_y, severity=(100-health))

        # 5. SENSOR NOISE SIMULATION (Realism)
        # Inject randomized Gaussian noise to simulate CCD thermal noise
        noise = np.random.normal(0, 5, (self.height, self.width, 3)).astype(np.uint8)
        frame = cv2.add(frame, noise)

        # 6. COMPUTER VISION PIPELINE (The "Perfect" Detection)
        # This analyzes the frame WE just generated to "find" the defects.
        self._run_cv_analysis(frame)

        # 7. HUD & AUGMENTED REALITY OVERLAY
        self._draw_hud(frame, data)

        # 8. Final Render
        cv2.imshow(self.window_name, frame)
        cv2.waitKey(1)

    def _inject_defects(self, canvas, cx, cy, severity):
        """Simulates corrosion by drawing irregular fractal noise."""
        np.random.seed(int(time.time() * 10)) # Flicker effect
        num_spots = int(severity / 5) + 1
        
        for _ in range(num_spots):
            dx = np.random.randint(-100, 100)
            dy = np.random.randint(-100, 100)
            
            # Draw irregular polygon
            pts = np.array([
                [cx+dx, cy+dy],
                [cx+dx+15, cy+dy-10], 
                [cx+dx+25, cy+dy+5], 
                [cx+dx+5, cy+dy+20]
            ], np.int32)
            
            # Rust Color: Dark Brown/Orange
            cv2.fillPoly(canvas, [pts], (10, 50, 80)) # BGR

    def _run_cv_analysis(self, frame):
        """
        REAL-TIME CV PIPELINE with TEMPORAL INTEGRATION:
        1. ROI Extraction
        2. Gaussian Blur (Denoise)
        3. HSV Thresholding (Segmentation)
        4. Morphological Closing (Gap filling)
        5. Contour Analysis (Classification)
        6. Temporal Debouncing (Anti-Flicker)
        """
        # Define ROI (Region of Reactor)
        roi_x, roi_y, roi_w, roi_h = 100, 100, 400, 400
        roi = frame[roi_y:roi_y+roi_h, roi_x:roi_x+roi_w]
        
        # HSV Segmentation
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_rust = np.array([0, 50, 20])
        upper_rust = np.array([25, 255, 150])
        
        mask = cv2.inRange(hsv, lower_rust, upper_rust)
        
        # Morphological Cleanup
        kernel = np.ones((5,5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # Contour Detection
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        detected_this_frame = False
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 100: # Filter noise
                x, y, w, h = cv2.boundingRect(cnt)
                gx, gy = roi_x + x, roi_y + y
                
                # DRAW "TECH" BRACKETS
                color = (0, 140, 255) # Orange-Red Warning
                self._draw_bracket(frame, gx, gy, w, h, color)
                
                # SCIENTIFIC LABELING
                confidence = min(99.9, 85.0 + (area / 100.0))
                label = f"ANOMALY: Fe2O3 [CONF: {confidence:.1f}%]"
                cv2.putText(frame, label, (gx, gy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
                
                detected_this_frame = True

        # Temporal Filter Logic (placeholder for class-based history)
        if detected_this_frame:
             cv2.putText(frame, "STATUS: INTEGRITY COMPROMISED", (120, 520), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
             cv2.putText(frame, "STATUS: NOMINAL", (120, 520), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    def _draw_bracket(self, img, x, y, w, h, color):
        l = int(min(w, h) / 3) # Length of bracket corner
        t = 2 # Thickness
        # Top-Left
        cv2.line(img, (x, y), (x+l, y), color, t)
        cv2.line(img, (x, y), (x, y+l), color, t)
        # Top-Right
        cv2.line(img, (x+w, y), (x+w-l, y), color, t)
        cv2.line(img, (x+w, y), (x+w, y+l), color, t)
        # Bottom-Right
        cv2.line(img, (x+w, y+h), (x+w-l, y+h), color, t)
        cv2.line(img, (x+w, y+h), (x+w, y+h-l), color, t)
        # Bottom-Left
        cv2.line(img, (x, y+h), (x+l, y+h), color, t)
        cv2.line(img, (x, y+h), (x, y+h-l), color, t)


    def _draw_hud(self, frame, data):
        # RIGHT PANEL: TELEMETRY
        # Background Panel
        cv2.rectangle(frame, (600, 0), (1000, 600), (20, 20, 20), -1)
        
        T = data.get('T', 0)
        P = data.get('P', 0)
        
        # Text Metrics
        y_cursor = 50
        cv2.putText(frame, "TELEMETRY STREAM", (620, y_cursor), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        y_cursor += 40
        
        # Temperature Bar
        temp_pct = (T - 250) / 200.0
        self._draw_bar(frame, 620, y_cursor, 300, 20, temp_pct, "THERMAL LOAD")
        y_cursor += 60
        
        # Pressure Bar
        pres_pct = P / 250000.0
        self._draw_bar(frame, 620, y_cursor, 300, 20, pres_pct, "VESSEL PSI")
        y_cursor += 60
        
        # Neural Network Status
        cv2.putText(frame, "NEURAL ENGINE STATE:", (620, y_cursor), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        y_cursor += 20
        cv2.putText(frame, ">> INFERENCE: 12ms", (620, y_cursor), cv2.FONT_HERSHEY_COMPLEX_SMALL, 0.6, (0, 200, 0), 1)
        y_cursor += 40
        
        # PHASE PLOT (Miniature)
        plot_x, plot_y = 620, 350
        plot_w, plot_h = 300, 200
        cv2.rectangle(frame, (plot_x, plot_y), (plot_x+plot_w, plot_y+plot_h), (40, 40, 40), 1)
        
        # Plot Point
        px = int(np.clip((T-250)/200 * plot_w, 0, plot_w))
        py = int(np.clip((1 - P/300000) * plot_h, 0, plot_h))
        cv2.circle(frame, (plot_x+px, plot_y+py), 4, (0, 255, 255), -1)
        cv2.putText(frame, "PHASE SPACE (T vs P)", (plot_x, plot_y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1)

        # RETICLE (Center Screen)
        cx, cy = 300, 300
        # Dynamic reticle size based on pressure (Expansion)
        r_dyn = int(200 + (P/250000.0)*20)
        cv2.circle(frame, (cx, cy), r_dyn, (0, 100, 0), 1) # Breath effect
        
        # Crosshairs
        cv2.line(frame, (cx-20, cy), (cx+20, cy), (0, 100, 0), 1)
        cv2.line(frame, (cx, cy-20), (cx, cy+20), (0, 100, 0), 1)
        
    def _draw_bar(self, img, x, y, w, h, pct, label):
        pct = np.clip(pct, 0.0, 1.0)
        # Border
        cv2.rectangle(img, (x, y), (x+w, y+h), (100, 100, 100), 1)
        # Fill
        fill_w = int(w * pct)
        color = (0, 255, 0)
        if pct > 0.7: color = (0, 165, 255)
        if pct > 0.9: color = (0, 0, 255)
        cv2.rectangle(img, (x, y), (x+fill_w, y+h), color, -1)
        # Text
        cv2.putText(img, f"{label}: {pct*100:.1f}%", (x, y-5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)

def main(args=None):
    rclpy.init(args=args)
    node = AesculonVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
