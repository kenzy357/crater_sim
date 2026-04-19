import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

# Change this to a valid font on your system
font_path = "arial_black.ttf"

# -----------------------------
# Physical dimensions (cm)
# -----------------------------
BOX_HEIGHT = 32.0
BOX_WIDTH = 25.0
BOX_DEPTH = 25.0

A4_W = 21.0
A4_H = 29.7

ARUCO_SIZE = 15.0
ARUCO_OFFSET_BOTTOM = 3.0
TEXTBOX_W = 15.0
TEXTBOX_H = 7.0
TEXTBOX_OFFSET = 3.0


# -----------------------------
# Rendering resolution
# -----------------------------
DPI = 300
CM_TO_PX = DPI / 2.54

LIGHT_GRAY = (220, 220, 220)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

# -----------------------------
# Helper functions
# -----------------------------

def cm(val):
    return int(round(val * CM_TO_PX))

# -----------------------------
# 1. Generate A4 page
# -----------------------------

def generate_a4_page(landmark_nr):
    aruco_id = landmark_nr + 50

    w_px = cm(A4_W)
    h_px = cm(A4_H)
    page = np.full((h_px, w_px, 3), WHITE, np.uint8)

    # ArUCO marker
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_100)
    marker_px = cm(ARUCO_SIZE)
    marker = cv2.aruco.generateImageMarker(aruco_dict, aruco_id, marker_px)
    marker = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)

    x_m = (w_px - marker_px) // 2
    y_m = h_px - cm(ARUCO_OFFSET_BOTTOM) - marker_px
    page[y_m:y_m + marker_px, x_m:x_m + marker_px] = marker

    # Text box above marker
    box_w = cm(TEXTBOX_W)
    box_h = cm(TEXTBOX_H)
    x_b = (w_px - box_w) // 2
    y_b = y_m - cm(TEXTBOX_OFFSET) - box_h

    cv2.rectangle(page, (x_b, y_b), (x_b + box_w, y_b + box_h), BLACK, thickness=10)

    # Centered number
    font = cv2.FONT_HERSHEY_DUPLEX
    font_scale = 16
    thickness = 20
    text = f'{landmark_nr}'
    (tw, th), _ = cv2.getTextSize(text, font, font_scale, thickness)
    tx = x_b + box_w // 2
    ty = y_b + box_h // 2
    page = put_text_pil(page, (tx, ty), text)

    return page

def put_text_pil(image, loc, text):
    # Convert the image to RGB (OpenCV uses BGR)  
    cv2_im_rgb = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)  
    
    # Pass the image to PIL  
    pil_im = Image.fromarray(cv2_im_rgb)  
    
    draw = ImageDraw.Draw(pil_im)  
    # use a truetype font  
    font = ImageFont.truetype(font_path, 480)  
    
    # Draw the text  
    draw.text(loc, text, font=font, anchor='mm', fill=(0,0,0,255))  
    
    # Get back the image to OpenCV  
    cv2_im_processed = cv2.cvtColor(np.array(pil_im), cv2.COLOR_RGB2BGR)  
    return cv2_im_processed

# -----------------------------
# 2. Generate large face
# -----------------------------

def generate_large_face(a4_page):
    face_w = cm(BOX_WIDTH)
    face_h = cm(BOX_HEIGHT)
    face = np.full((face_h, face_w, 3), LIGHT_GRAY, np.uint8)

    ph, pw = a4_page.shape[:2]
    x = (face_w - pw) // 2
    y = (face_h - ph) // 2
    face[y:y + ph, x:x + pw] = a4_page

    return face

# -----------------------------
# 3. Generate small face
# -----------------------------

def generate_small_face(w_cm, h_cm):
    return np.full((cm(h_cm), cm(w_cm), 3), LIGHT_GRAY, np.uint8)

# -----------------------------
# 4. Assemble unrolled cube
# -----------------------------

def assemble_unrolled_cube(large_face, small_face):
    H = large_face.shape[0]
    W = large_face.shape[1]
    D = small_face.shape[1]

    canvas_h = H + 2 * D
    canvas_w = W * 4
    canvas = np.full((canvas_h, canvas_w, 3), 255, np.uint8)

    # Layout (classic cube net)
    canvas[D:H+D, W:2*W] = large_face       # front
    canvas[D:H+D, 2*W:3*W] = large_face     # right
    canvas[D:H+D, 3*W:4*W] = large_face     # back
    canvas[D:H+D, 0:W] = large_face         # left

    canvas[:D, W:2*W] = small_face         # top
    canvas[H+D:H+2*D, W:2*W] = small_face     # bottom

    return canvas

# -----------------------------
# Main
# -----------------------------

if __name__ == '__main__':
    # Argparse landmark_nr
    import argparse
    parser = argparse.ArgumentParser(description='Generate landmark texture.')
    parser.add_argument('--landmark_nr', type=int, help='Landmark number (integer).')
    args = parser.parse_args()
    landmark_nr = args.landmark_nr

    a4 = generate_a4_page(landmark_nr)
    large_face = generate_large_face(a4)
    small_face = generate_small_face(BOX_DEPTH, BOX_WIDTH)

    net = assemble_unrolled_cube(large_face, small_face)

    cv2.imwrite('a4_page.png', a4)
    cv2.imwrite('large_face.png', large_face)
    cv2.imwrite('box_net.png', net)
