import numpy as np
# import pybullet as p
import heapq
from collections import defaultdict
import matplotlib.pyplot as plt

# Helper function to get obstacle coordinates as a NumPy array
def get_obstacle_coordinates():
    # Replace this function with your own code to get obstacle coordinates
    # For demonstration purposes, I'll assume you have a list of (x, y) obstacle positions
    obstacles = [(10, 10), (15, 12), (8, 18)]
    return np.array(obstacles)

def is_cell_passable(cell, grid):
    # Check if the cell is passable (not an obstacle)
    x, y = cell
    return grid[x, y] == 0

def create_graph(grid):
    graph = defaultdict(list)

    for x in range(grid.shape[0]):
        for y in range(grid.shape[1]):
            if is_cell_passable((x, y), grid):
                # Connect to neighboring passable cells (up, down, left, right)
                neighbors = [(x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)]
                for neighbor in neighbors:
                    if is_cell_passable(neighbor, grid):
                        graph[(x, y)].append(neighbor)

    return graph

def plot_grid(grid):
    plt.imshow(grid, cmap='gray', origin='lower')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Grid with Obstacles')
    plt.show()


def main():
    # Initialize PyBullet environment and get the race car object
    # p.connect(p.GUI)
    # car = p.loadURDF("path/to/race_car.urdf")
    
    # Get the obstacle coordinates
    obstacle_coords = get_obstacle_coordinates()

    # Define grid parameters
    grid_size = 40  # Adjust the size according to your environment
    grid = np.zeros((grid_size, grid_size))

    # Mark obstacle cells as impassable (you may need to convert obstacle coordinates to grid cells)
    for obstacle in obstacle_coords:
        x, y = obstacle
        grid_x = int((x + grid_size / 2) / grid_size * grid.shape[0])
        grid_y = int((y + grid_size / 2) / grid_size * grid.shape[1])
        grid[grid_x, grid_y] = 1

    plot_grid(grid)
    
    # Create the graph from the grid representation
    graph = create_graph(grid)

    # Now you have a graph representation of your environment, which you can use for pathfinding or other purposes.

if __name__ == "__main__":
    main()