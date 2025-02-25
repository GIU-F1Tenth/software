import cv2
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import sys

# Read the first system argument
input_image_path = sys.argv[1]

# Function to find contours and draw them on the image


def draw_contours(image_path):
    # Read input image and convert to grayscale
    orig_img = cv2.imread(image_path)
    gray = cv2.cvtColor(orig_img, cv2.COLOR_BGR2GRAY)

    # Convert grayscale to binary
    _, bw = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    # Find contours
    contours, _ = cv2.findContours(bw, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea)
    contours = [contours[1], contours[-2]]

    # Draw contours on the original image
    contour_img = np.zeros_like(orig_img)
    cv2.drawContours(contour_img, contours, -1, (0, 255, 0), 2)

    # Display the result
    plt.figure(figsize=(8, 6))
    plt.imshow(cv2.cvtColor(contour_img, cv2.COLOR_BGR2RGB))
    plt.title('Track Borders')
    plt.axis('off')
    plt.show()

    return contour_img, contours


# Read input image and convert to grayscale
orig_img, contours = draw_contours(input_image_path)
gray = cv2.cvtColor(orig_img, cv2.COLOR_BGR2GRAY)

# Convert grayscale to binary
_, bw = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

# Fill area outside lines (using flood fill)
bw2 = bw.copy()
h, w = bw.shape

plt.figure(figsize=(8, 6))
plt.imshow(bw, cmap='gray')
plt.title('Original Image')
plt.axis('off')
plt.show()

# Flood fill from point (0, 0)
cv2.floodFill(bw2, None, (0, 0), 255)

# Invert the image
bw2 = cv2.bitwise_not(bw2)


plt.figure(figsize=(8, 6))
plt.imshow(bw2, cmap='gray')
plt.title('Filled Area')
plt.axis('off')
plt.show()

# Compute distance transform
dist_transform = cv2.distanceTransform(bw2, cv2.DIST_L2, 5)

# # Mark outside pixels as zero
# dist_transform[bw2 == 255] = 0

# Display the distance transform
plt.figure(figsize=(8, 6))
plt.imshow(dist_transform, cmap='gray')
plt.title('Distance Transform')
plt.colorbar()
plt.show()

# Find start and end points
x0 = 0
y0 = np.argmax(dist_transform[:, x0])
x1 = w - 1
y1 = np.argmax(dist_transform[:, x1])

start_node = (y0, x0)
end_node = (y1, x1)

# Create cost matrix (use 100 - distance for shortest path calculation)
cost_mat = 100 - dist_transform

# Build graph from image
G = nx.grid_2d_graph(h, w)
for (x, y) in G.nodes:
    G.nodes[(x, y)]['cost'] = cost_mat[x, y]

# Add weights based on cost matrix
for (x, y) in G.nodes:
    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1),
                   (-1, -1), (-1, 1), (1, -1), (1, 1)]:  # 8-connectivity
        nx_x, nx_y = x + dx, y + dy
        if (0 <= nx_x < h) and (0 <= nx_y < w):
            dist = np.hypot(dx, dy)
            cost = (G.nodes[(x, y)]['cost'] +
                    G.nodes[(nx_x, nx_y)]['cost']) / 2
            G.add_edge((x, y), (nx_x, nx_y), weight=cost * dist)

plt.figure(figsize=(8, 6))
nx.draw(G, pos={(x, y): (y, x) for x, y in G.nodes}, node_size=10)
plt.title('Graph Representation')
plt.axis('off')
plt.show()

# Find shortest path using Dijkstra's algorithm
path = nx.shortest_path(G, source=start_node, target=end_node, weight='weight')

# Mark the path on the original image
path_img = orig_img.copy()
for (x, y) in path:
    path_img[x, y] = [255, 255, 255]  # Mark white pixels

# Display the result
plt.figure(figsize=(8, 6))
plt.imshow(cv2.cvtColor(path_img, cv2.COLOR_BGR2RGB))
plt.title('Shortest Path')
plt.axis('off')
plt.show()
