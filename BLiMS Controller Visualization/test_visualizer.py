import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import math

# ====== USER INPUT ======
# Replace with your image and corner coordinates
image_path = r'C:\Users\trevo\OneDrive\Desktop\School\Files\Rocketry\Software\BLiMS Controller\Pictures\testsatelliteview.png'

top_left = (42.447644, -76.465630)       # (lat, lon)
bottom_right = (42.445305, -76.460735)

# Mock GPS test data
test_lat = 42.446305
test_lon = -76.463735
test_heading = 135.0  # degrees

# =========================

# Load image
img = mpimg.imread(image_path)
fig, ax = plt.subplots()
ax.imshow(img)

def gps_to_pixel(lat, lon, img_shape):
    lat_frac = (top_left[0] - lat) / (top_left[0] - bottom_right[0])
    lon_frac = (lon - top_left[1]) / (bottom_right[1] - top_left[1])
    y = lat_frac * img_shape[0]
    x = lon_frac * img_shape[1]
    return x, y

# Convert to pixel
x, y = gps_to_pixel(test_lat, test_lon, img.shape)

# Plot current point
ax.plot(x, y, 'bo', label='Test Location')

# Add heading arrow
arrow_len = 50  # pixels
angle_rad = math.radians(test_heading)
dx = arrow_len * math.cos(angle_rad)
dy = -arrow_len * math.sin(angle_rad)  # y-axis is inverted in image coords
ax.arrow(x, y, dx, dy, head_width=10, head_length=10, fc='blue', ec='blue')

# Labels
ax.set_title("Mock GPS Overlay Test")
ax.legend()
plt.show()
