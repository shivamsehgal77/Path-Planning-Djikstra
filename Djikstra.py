import pygame as pyg
import numpy as np
from queue import PriorityQueue
import time

class ObstacleMap:
    def __init__(self):
        # Initialize the obstacle map surface
        self.screen = pyg.Surface((600, 250))
        self.rect_color = (255, 255, 255)
        self.create_map()

    def create_map(self):
        # Fill the screen with red color (indicating obstacles)
        self.screen.fill((255, 0, 0))
        
        # Create the outer white boundary
        rectangle1 = pyg.Rect(5, 5, 590, 240)
        pyg.draw.rect(self.screen, self.rect_color, rectangle1)
        
        # Create additional rectangular obstacles
        pyg.draw.rect(self.screen, (255, 0, 0), pyg.Rect(95, 145, 60, 105))
        pyg.draw.rect(self.screen, (255, 0, 0), pyg.Rect(95, 0, 60, 105))
        
        # Create a hexagonal obstacle
        hexagon_dim = [(300, 44.22649731), (230.04809472, 84.61324865), (230.04809472, 165.38675135),
                       (300, 205.77350269), (369.95190528, 165.38675135), (369.95190528, 84.61324865)]
        pyg.draw.polygon(self.screen, (255, 0, 0), hexagon_dim)
        
        # Create a triangular obstacle
        triangle_dim = [(455, 246.18033989), (455.00, 3.81966011), (515.59016994, 125)]
        pyg.draw.polygon(self.screen, (255, 0, 0), triangle_dim)

    def is_valid_point(self, point):
        # Check if the point is within the screen bounds and not on an obstacle
        if (self.screen.get_at(point) != (255, 0, 0, 255) and
                0 <= point[0] < 600 and 0 <= point[1] < 250):
            return True
        return False

class Dijkstra:
    def __init__(self, start, goal, map_surface):
        # Initialize Dijkstra algorithm with start, goal, and map surface
        self.start = start
        self.goal = goal
        self.map_surface = map_surface
        self.start_node = (start[0], 250 - start[1])
        self.goal_node = (goal[0], 250 - goal[1])
        # Initializes the state of the start node [cost, index, parent_index, coordinate].
        self.node_state = [0, 0, -1, self.start_node]
        self.closed_list = {}
        self.global_dict = {self.start_node: self.node_state}
        self.open_list = PriorityQueue()
        self.open_list.put(self.node_state)
        self.goal_reached = False
        self.goal_index = None
        self.opendir = {self.start_node: None}
        self.closedir = {}

    def generate_new_node(self, node, direction, nc2c):
        # Define possible directions and their movement vectors
        directions = {
            'up': (0, -1),
            'down': (0, 1),
            'left': (-1, 0),
            'right': (1, 0),
            'up_left': (-1, -1),
            'up_right': (1, -1),
            'down_left': (-1, 1),
            'down_right': (1, 1)
        }
        # Define the cost for each movement direction
        cost_map = {
            'up': 1,
            'down': 1,
            'left': 1,
            'right': 1,
            'up_left': 1.4,
            'up_right': 1.4,
            'down_left': 1.4,
            'down_right': 1.4
        }
        # Calculate the new node position and cost
        direction_vector = directions[direction]
        new_node = (node[0] + direction_vector[0], node[1] + direction_vector[1])
        cost = nc2c + cost_map[direction]
        return new_node, cost

    def update_open_list(self, new_node, new_node_cost, parent_node):
        # Update the open list with new nodes
        if self.map_surface.get_at(new_node) != (255, 0, 0, 255):
            if new_node not in self.closedir:
                if new_node not in self.opendir:
                    node_index = len(self.global_dict)
                    self.global_dict[new_node] = [new_node_cost, node_index, parent_node[1], new_node]
                    self.open_list.put(self.global_dict[new_node])
                    self.opendir[new_node] = None
                else:
                    # Update the node if a lower cost path is found
                    if round(self.global_dict[new_node][0], 1) > round(new_node_cost, 1):
                        self.global_dict[new_node][2] = parent_node[1]
                        self.global_dict[new_node][0] = new_node_cost

    def find_path(self):
        # Main loop for finding the path using Dijkstra's algorithm
        while not self.open_list.empty():
            current_node = self.open_list.get()
            node_coord = current_node[-1]
            node_cost = current_node[0]
            self.closed_list[current_node[1]] = [node_cost, current_node[2], node_coord]
            self.closedir[node_coord] = None
            del self.opendir[node_coord]

            # Check if the goal is reached
            if node_coord == self.goal_node:
                self.goal_reached = True
                self.goal_index = current_node[1]
                break

            # Generate and evaluate new nodes in all possible directions
            directions = ['up', 'down', 'left', 'right', 'up_left', 'up_right', 'down_left', 'down_right']
            for direction in directions:
                new_node, new_node_cost = self.generate_new_node(node_coord, direction, node_cost)
                if new_node == self.goal_node:
                    node_index = len(self.global_dict)
                    self.closed_list[node_index] = [new_node_cost, current_node[1], new_node]
                    self.goal_reached = True
                    self.goal_index = node_index
                    break
                self.update_open_list(new_node, new_node_cost, current_node)
            if self.goal_reached:
                break

    def backtrack_path(self):
        # Backtrack to find the path from goal to start
        path = []
        if self.goal_reached:
            gni = self.goal_index
            while gni != -1:
                path.append(self.closed_list[gni][-1])
                gni = self.closed_list[gni][1]
            path.reverse()
        return path

def main():
    # Initialize Pygame
    pyg.init()
    obstacle_map = ObstacleMap()
    screen = pyg.display.set_mode((600, 250))
    pyg.display.set_caption('Dijkstra')

    # Get user input for start and goal coordinates
    while True:
        start_x = int(input("Please enter the x coordinate of start:"))
        start_y = int(input("Please enter the y coordinate of start:"))
        goal_x = int(input("Please enter the x coordinate of goal:"))
        goal_y = int(input("Please enter the y coordinate of goal:"))

        # Validate coordinates
        if (obstacle_map.is_valid_point((start_x, 250 - start_y)) and 
            obstacle_map.is_valid_point((goal_x, 250 - goal_y))):
            print("Valid coordinates received")
            break
        else:
            print("Invalid coordinates try again")

    # Create Dijkstra object and run the algorithm
    start = (start_x, start_y)
    goal = (goal_x, goal_y)
    dijkstra = Dijkstra(start, goal, obstacle_map.screen)

    start_time = time.time()
    dijkstra.find_path()
    path = dijkstra.backtrack_path()
    end_time = time.time()
    total_time = end_time - start_time

    print(f"Total runtime: {total_time:.2f} seconds")
    screen.blit(obstacle_map.screen, (0, 0))
    pyg.display.update()

    # Visualize the explored nodes
    for valu in dijkstra.closed_list.values():
        screen.set_at(valu[-1], (255, 0, 255))
        pyg.display.update()

    # Visualize the path
    if dijkstra.goal_reached:
        for co in path:
            screen.set_at(co, (0, 0, 0))
            pyg.time.wait(50)
            pyg.display.update()

    # Wait before closing the window
    pyg.time.wait(1000)
    running = True
    while running:
        for event in pyg.event.get():
            if event.type == pyg.QUIT:
                running = False

    pyg.quit()

if __name__ == "__main__":
    main()
