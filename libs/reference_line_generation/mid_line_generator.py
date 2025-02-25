import cv2
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
import heapq
import sys
import os


# Function to find contours and draw them on the image
def draw_contours(image_path):
    orig_img = cv2.imread(image_path)
    gray = cv2.cvtColor(orig_img, cv2.COLOR_BGR2GRAY)
    _, bw = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    contours, _ = cv2.findContours(bw, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea)
    contours = [contours[1], contours[-2]]

    contour_img = np.zeros_like(orig_img)
    cv2.drawContours(contour_img, contours, -1, (0, 255, 0), 2)

    plt.figure(figsize=(8, 6))
    plt.imshow(cv2.cvtColor(contour_img, cv2.COLOR_BGR2RGB))
    plt.title('Track Borders')
    plt.axis('off')
    plt.show()

    return contour_img, contours

# A* algorithm implementation


def a_star(graph, start, goal):
    open_set = []
    # (f_score, node)
    heapq.heappush(open_set, (0 + heuristic(start, goal), start))

    came_from = {}
    g_score = {start: 0}

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

        for neighbor in graph.neighbors(current):
            tentative_g_score = g_score[current] + \
                graph[current][neighbor]['weight']
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, neighbor))

    return None  # No path found


def heuristic(node, goal):
    # Use Manhattan or Euclidean distance as heuristic
    return abs(node[0] - goal[0]) + abs(node[1] - goal[1])

# Main part of the code


def main(input_image_path, output_image_path):
    # Step 1: Process the input image and find contours
    orig_img, contours = draw_contours(input_image_path)
    gray = cv2.cvtColor(orig_img, cv2.COLOR_BGR2GRAY)

    # Step 2: Create binary image
    _, bw = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)

    # Step 3: Perform flood fill to fill inside the track
    bw2 = bw.copy()
    cv2.floodFill(bw2, None, (0, 0), 255)
    bw2 = cv2.bitwise_not(bw2)

    # Step 4: Compute distance transform
    dist_transform = cv2.distanceTransform(bw2, cv2.DIST_L2, 5)

    # Step 5: Find the start and end points
    h, w = bw.shape
    x0 = 0
    y0 = np.argmax(dist_transform[:, x0])
    x1 = w - 1
    y1 = np.argmax(dist_transform[:, x1])

    start_node = (y0, x0)
    end_node = (y1, x1)

    # Step 6: Create a cost matrix for the graph
    cost_mat = 100 - dist_transform

    # Step 7: Build graph
    print("Building graph...")
    G = nx.grid_2d_graph(h, w)
    for (x, y) in G.nodes:
        G.nodes[(x, y)]['cost'] = cost_mat[x, y]

    # Step 8: Add edges with weight (cost matrix)
    print("Adding edges...")
    for (x, y) in G.nodes:
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            nx_x, nx_y = x + dx, y + dy
            if (0 <= nx_x < h) and (0 <= nx_y < w):
                dist = np.hypot(dx, dy)
                cost = (G.nodes[(x, y)]['cost'] +
                        G.nodes[(nx_x, nx_y)]['cost']) / 2
                G.add_edge((x, y), (nx_x, nx_y), weight=cost * dist)

    # Step 9: Find the shortest path using A* algorithm
    print("Finding shortest path...")
    path = a_star(G, start_node, end_node)

    # Step 10: Visualize the shortest path
    path_img = orig_img.copy()
    for (x, y) in path:
        path_img[x, y] = [255, 255, 255]  # Mark white pixels

    plt.figure(figsize=(8, 6))
    plt.imshow(cv2.cvtColor(path_img, cv2.COLOR_BGR2RGB))
    plt.title('Shortest Path')
    plt.axis('off')
    plt.show()

    # Step 11: Save the output image
    os.makedirs(os.path.dirname(output_image_path), exist_ok=True)
    cv2.imwrite(output_image_path, path_img)


if __name__ == '__main__':
    # Read the first system argument
    if len(sys.argv) != 3:
        print("Usage: python mid_line_generator.py <input_image_path> <output_image_path>")
        sys.exit(1)

    input_image_path = sys.argv[1]
    output_image_path = sys.argv[2]
    main(input_image_path, output_image_path)
