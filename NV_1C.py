
import argparse
import numpy as np
import cv2

print("--- SCRIPT STARTED EXECUTION ---")
M = None
M_INV = None

def get_perspective_matrices(img_w=640, img_h=480):
    global M, M_INV
    if M is not None and M_INV is not None:
        return M, M_INV

    # Much wider and lower source trapezoid to capture full track width
    src = np.float32([
        [int(img_w * 0.40), int(img_h * 0.55)],  # Top-Left
        [int(img_w * 0.60), int(img_h * 0.55)],  # Top-Right
        [int(img_w * 0.95), int(img_h * 0.98)],  # Bottom-Right
        [int(img_w * 0.05), int(img_h * 0.98)]   # Bottom-Left
    ])

    dst = np.float32([
        [int(img_w * 0.2), 0],
        [int(img_w * 0.8), 0],
        [int(img_w * 0.8), img_h],
        [int(img_w * 0.2), img_h]
    ])

    M = cv2.getPerspectiveTransform(src, dst)
    M_INV = cv2.getPerspectiveTransform(dst, src)
    return M, M_INV


def get_combined_binary_mask(frame):
    """
    Isolates clean lane markings using balanced HSV color space and Canny edges.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # 1. Broad White Color Mask (captures bright dashed center lines and side lines)
    lower_white = np.array([0, 0, 200], dtype=np.uint8)
    upper_white = np.array([180, 50, 255], dtype=np.uint8)
    mask_white = cv2.inRange(hsv, lower_white, upper_white)

    # 2. Broad Yellow Color Mask (captures solid or yellow markers if present)
    lower_yellow = np.array([15, 80, 80], dtype=np.uint8)
    upper_yellow = np.array([35, 255, 255], dtype=np.uint8)
    mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # Combine color masks
    color_binary = cv2.bitwise_or(mask_white, mask_yellow)

    # 3. Grayscale Canny Edges to catch line boundaries cleanly without asphalt noise
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blur, 50, 150)

    # Combine color and edge maps
    combined = cv2.bitwise_or(color_binary, edges)

    # 4. Region of Interest (ROI) mask to ignore sky and car hood
    mask_roi = np.zeros_like(combined)
    h, w = frame.shape[:2]
    polygon = np.array([[(20, h), (200, 300), (440, 300), (620, h)]], dtype=np.int32)
    cv2.fillPoly(mask_roi, polygon, 255)

    return cv2.bitwise_and(combined, mask_roi)


def unwarp_point(x_warped, y_warped, m_inv):
    pt_warped = np.array([[[float(x_warped), float(y_warped)]]], dtype=np.float32)
    pt_orig = cv2.perspectiveTransform(pt_warped, m_inv)
    return int(round(pt_orig[0][0][0])), int(round(pt_orig[0][0][1]))


def fit_lane_polynomials(warped_binary):
    histogram = np.sum(warped_binary[warped_binary.shape[0] // 2:, :], axis=0)
    midpoint = int(histogram.shape[0] // 2)

    leftx_base = np.argmax(histogram[:midpoint])
    rightx_base = np.argmax(histogram[midpoint:]) + midpoint

    nwindows = 9
    window_height = int(warped_binary.shape[0] // nwindows)
    margin = 60
    minpix = 10

    nonzero = warped_binary.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    leftx_current = leftx_base
    rightx_current = rightx_base

    left_lane_inds = []
    right_lane_inds = []

    for window in range(nwindows):
        win_y_low = warped_binary.shape[0] - (window + 1) * window_height
        win_y_high = warped_binary.shape[0] - window * window_height

        win_xleft_low = leftx_current - margin
        win_xleft_high = leftx_current + margin
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin

        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                          (nonzerox >= win_xleft_low) & (nonzerox < win_xleft_high)).nonzero()[0]
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) &
                           (nonzerox >= win_xright_low) & (nonzerox < win_xright_high)).nonzero()[0]

        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)

        if len(good_left_inds) > minpix:
            leftx_current = int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = int(np.mean(nonzerox[good_right_inds]))

    left_lane_inds = np.concatenate(left_lane_inds) if len(left_lane_inds) > 0 else np.array([])
    right_lane_inds = np.concatenate(right_lane_inds) if len(right_lane_inds) > 0 else np.array([])

    left_fit = None
    right_fit = None

    # Lowered threshold requirements to easily catch dashed lines
    if len(left_lane_inds) > 10:
        leftx = nonzerox[left_lane_inds]
        lefty = nonzeroy[left_lane_inds]
        left_fit = np.polyfit(lefty, leftx, 2)

    if len(right_lane_inds) > 10:
        rightx = nonzerox[right_lane_inds]
        righty = nonzeroy[right_lane_inds]
        right_fit = np.polyfit(righty, rightx, 2)

    return left_fit, right_fit

def detect_lane(frame):
    h, w = frame.shape[:2]
    m_fwd, m_inv = get_perspective_matrices(img_w=w, img_h=h)

    binary_mask = get_combined_binary_mask(frame)
    warped_binary = cv2.warpPerspective(binary_mask, m_fwd, (w, h), flags=cv2.INTER_NEAREST)

    left_fit, right_fit = fit_lane_polynomials(warped_binary)

    if left_fit is None and right_fit is None:
        return {"center_x": -1, "lane": "unknown"}

    eval_y_warped = h - 40

    if left_fit is not None and right_fit is not None:
        left_x = left_fit[0] * (eval_y_warped**2) + left_fit[1] * eval_y_warped + left_fit[2]
        right_x = right_fit[0] * (eval_y_warped**2) + right_fit[1] * eval_y_warped + right_fit[2]
        center_x_warped = (left_x + right_x) / 2.0
    elif left_fit is not None:
        left_x = left_fit[0] * (eval_y_warped**2) + left_fit[1] * eval_y_warped + left_fit[2]
        center_x_warped = left_x + 160.0
    else:
        right_x = right_fit[0] * (eval_y_warped**2) + right_fit[1] * eval_y_warped + right_fit[2]
        center_x_warped = right_x - 160.0

    center_x_orig, _ = unwarp_point(center_x_warped, eval_y_warped, m_inv)

    if center_x_orig < 0 or center_x_orig >= w:
        return {"center_x": -1, "lane": "unknown"}

    car_center_x = w / 2.0  # 320 for 640x480

    # Lane classification logic relative to vehicle center
    if center_x_orig >= car_center_x:
        lane = "left"
    else:
        lane = "right"

    return {"center_x": int(center_x_orig), "lane": str(lane)}


def main():
    parser = argparse.ArgumentParser(description="Lane Detection Task 1C")
    parser.add_argument("video", type=str, help="Path to video file")
    parser.add_argument("--show", action="store_true", help="Show display window")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.video)

    if not cap.isOpened():
        print(f"[ERROR] Could not open video file at: {args.video}")
        return

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    delay = max(1, int(1000.0 / fps))

    frame_count = 0
    left_count = 0
    right_count = 0
    unknown_count = 0

    print("[INFO] Processing video frames...")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame_count += 1
        result = detect_lane(frame)

        lane = result.get("lane", "unknown")
        if lane == "left":
            left_count += 1
        elif lane == "right":
            right_count += 1
        else:
            unknown_count += 1

        if args.show:
            cx = result.get("center_x", -1)
            vis = frame.copy()
            if cx != -1:
                cv2.circle(vis, (cx, vis.shape[0] - 40), 8, (0, 255, 0), -1)
            
            cv2.putText(vis, f"Lane: {lane}", (30, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            
            cv2.imshow("Niti Vahan - Lane Detection", vis)
            if cv2.waitKey(delay) & 0xFF == 27:  # Press ESC to exit
                break

    cap.release()
    cv2.destroyAllWindows()

    print(f"\n--- FINAL RESULTS ---")
    print(f"Total Frames: {frame_count}")
    print(f"Left: {left_count} | Right: {right_count} | Unknown: {unknown_count}")

if __name__ == "__main__":
    main()


