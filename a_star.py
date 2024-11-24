import pygame
from queue import PriorityQueue
import random
# PYGAME VARIABLE
WIDTH = 800
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("A* Path Finding Algorithm")

# COLOR RBG CONSTANT
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 255, 0)
YELLOW = (255, 255, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
PURPLE = (128, 0, 128)
ORANGE = (255, 165 ,0)
GREY = (128, 128, 128)
TURQUOISE = (64, 224, 208)

class Node:
	"""
A class representing each cell in the grid for pathfinding. It's like a node in a tree.
In pygame, we represent it as a square.
	"""
	def __init__(self, row, col, width, total_rows):
		"""
    Attributes:
        row (int): Row position of the node in the grid.
        col (int): Column position of the node in the grid.
        x (int): X position of the node in pixels.
        y (int): Y position of the node in pixels.
        color (str): Represents the status of the node (e.g., WHITE means unvisited).
        neighbors (list): List to store neighboring nodes.
        width (int): Width of the cell in pixels.
        total_rows (int): Total number of rows in the grid.
		"""
		self.row = row
		self.col = col
		self.x = row * width
		self.y = col * width 
		self.color = WHITE
		self.neighbors = []
		self.width = width
		self.total_rows = total_rows

	def get_pos(self):
		""" Returns the position of the node as (row, col)"""
		return self.row, self.col

	def is_closed(self):
		""" Checks if the node is marked as closed (Red = visited)"""
		return self.color == RED

	def is_open(self):
		"""	Checks if the node is marked as open (Green = in the open set) """
		return self.color == GREEN

	def is_barrier(self):
		""" Checks if the node is a barrier (Black = an obstacle)"""
		return self.color == BLACK

	def is_start(self):
		"""	Checks if the node is the start node (Orange = start node)"""
		return self.color == ORANGE

	def is_end(self):
		"""	Checks if the node is the end node (Turquoise = end node)"""
		return self.color == TURQUOISE

	def reset(self):
		"""Resets the node to its default state (white color)"""
		self.color = WHITE

	def make_start(self):
		"""Marks the node as the start node (orange color)"""
		self.color = ORANGE

	def make_closed(self):
		"""Marks the node as closed (red color)"""
		self.color = RED

	def make_open(self):
		"""Marks the node as open (green color)"""
		self.color = GREEN

	def make_barrier(self):
		"""Marks the node as a barrier (black color)"""
		self.color = BLACK

	def make_end(self):
		"""Marks the node as the end node (turquoise color)"""
		self.color = TURQUOISE

	def make_path(self):
		"""Marks the node as part of the final path (purple color)"""
		self.color = PURPLE

	def draw(self, win):
		"""Draws the node as a rectangle on the given window 'win'"""
		pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

	def update_neighbors(self, grid):
		"""Update neighbors with legal moves"""
		self.neighbors = []
		if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier(): # DOWN
			self.neighbors.append(grid[self.row + 1][self.col])

		if self.row > 0 and not grid[self.row - 1][self.col].is_barrier(): # UP
			self.neighbors.append(grid[self.row - 1][self.col])

		if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier(): # RIGHT
			self.neighbors.append(grid[self.row][self.col + 1])

		if self.col > 0 and not grid[self.row][self.col - 1].is_barrier(): # LEFT
			self.neighbors.append(grid[self.row][self.col - 1])

	def __lt__(self, other):
		return False

# h is the heuristic function. 
# h(x) estimates the cost to reach goal from node x
def h(p1, p2):
	"""
Manhattan heuristic function:
h(x) = |x1 - x2| + |y1 - y2|
	"""
	x1, y1 = p1
	x2, y2 = p2
	return abs(x1 - x2) + abs(y1 - y2)


def reconstruct_path(came_from, current, draw):
	"""
Reconstructs the path by following each node's predecessor until reaching the start node.
	"""
	while current in came_from:
		current = came_from[current]
		if not current.is_start():
			current.make_path()
		draw()


def algorithm(draw, grid, start, end):
	"""
A* Path Finding Algorithm
	"""
	# Initialize priority queue and tracking variables
	count = 0
	open_set = PriorityQueue()
	open_set.put((0, count, start)) # Add start node to the priority queue
	came_from = {} # Dictionary to keep track of path
	
	close_set = set()

	# For node x, gScore[x] is the currently known cost of the cheapest path from start to x.
	g_score = {node: float("inf") for row in grid for node in row}
	g_score[start] = 0

	# For node n, fScore[x] := gScore[x] + h(x). fScore[x] represents our current best guess as to
	# how cheap a path could be from start to finish if it goes through x.
	f_score = {node: float("inf") for row in grid for node in row}
	f_score[start] = h(start.get_pos(), end.get_pos())

	
	# List to track nodes in the open set (Do not have duplicates here)
	open_set_hash = {start}
	while not open_set.empty():
		for event in pygame.event.get(): # Allow user to close the window in the middle of algorithm
			if event.type == pygame.QUIT:
				pygame.quit()

		# Get the node with the lowest f_score from the priority queue
		current = open_set.get()[2]

		# Skip if already evaluated and in closed set
		if current in close_set:
			continue

		open_set_hash.remove(current)

		# If the current node is the end, reconstruct the path
		if current == end:
			reconstruct_path(came_from, end, draw)
			end.make_end()
			return True

		for neighbor in current.neighbors:
			# temp_g_score is the distance from start to the neighbor through current
			if neighbor in close_set:
				continue
			temp_g_score = g_score[current] + 1

			if temp_g_score < g_score[neighbor]:
				# This path to neighbor is better than any previous one. Record it!	
				came_from[neighbor] = current
				g_score[neighbor] = temp_g_score
				f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
				count += 1
				open_set.put((f_score[neighbor], count, neighbor))
				# Add neighbor to the open set if it's not there already
				if neighbor not in open_set_hash:
					open_set_hash.add(neighbor)
					neighbor.make_open()
		# Draw the current step
		draw()
		# Mark current as closed if it's not the start
		if current != start:
			current.make_closed()
		close_set.add(current)
	# If the loop exits, it means there is no path to the end node
	return False


def make_grid(rows, width):
	grid = []
	gap = width // rows
	for i in range(rows):
		grid.append([])
		for j in range(rows):
			node = Node(i, j, gap, rows)
			grid[i].append(node)

	return grid

def randomize_grid(grid, barrier_probability=0.3):
    """Function use to generate random start state."""
    start_node = grid[0][0]
    end_node = grid[-1][-1]
    start_node.reset()
    start_node.make_start()
    end_node.reset()
    end_node.make_end()
    for row in grid:
        for node in row:
            if node != grid[0][0] and node != grid[-1][-1]:  # Don't set barriers on start and end
                if random.random() < barrier_probability:  # Randomly decide to place a barrier
                    node.make_barrier()
                else:
                    node.reset()

def draw_grid(win, rows, width):
	gap = width // rows
	for i in range(rows):
		pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))
		for j in range(rows):
			pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width))


def draw(win, grid, rows, width):
	win.fill(WHITE)

	for row in grid:
		for node in row:
			node.draw(win)

	draw_grid(win, rows, width)
	pygame.display.update()

# Get clicked position
def get_clicked_pos(pos, rows, width):
	gap = width // rows
	y, x = pos

	row = y // gap
	col = x // gap

	return row, col

# Main function
def main(win, width):
	ROWS = 50
	grid = make_grid(ROWS, width)

	start = None
	end = None
	random_mode = False

	run = True
	while run:
		draw(win, grid, ROWS, width)
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False

			# Left click for status change: start->end->block 
			if pygame.mouse.get_pressed()[0]: # LEFT
				pos = pygame.mouse.get_pos()
				row, col = get_clicked_pos(pos, ROWS, width)
				node = grid[row][col]
				if not node.is_barrier():
					if not start and node != end:
						start = node
						start.make_start()

					elif not end and node != start:
						end = node
						end.make_end()

					elif node != end and node != start:
						node.make_barrier()

			# Right clicked for block reset
			elif pygame.mouse.get_pressed()[2]: # RIGHT
				pos = pygame.mouse.get_pos()
				row, col = get_clicked_pos(pos, ROWS, width)
				node = grid[row][col]
				node.reset()
				if node == start:
					start = None
				elif node == end:
					end = None

			if event.type == pygame.KEYDOWN:
				# Space for Path Finding
				if event.key == pygame.K_SPACE and start and end:
					for row in grid:
						for node in row:
							node.update_neighbors(grid)

					algorithm(lambda: draw(win, grid, ROWS, width), grid, start, end)
					
				# Press Q for Random Start State.
				if event.key == pygame.K_q:
					random_mode = not random_mode  # Toggle random mode
					if random_mode:
						randomize_grid(grid)  # Generate a random grid with barriers
						start = grid[0][0]  # Reset start and end
						end = grid[-1][-1]
						random_mode = False
				# Press C for Custom State.
				if event.key == pygame.K_c:
					start = None
					end = None
					grid = make_grid(ROWS, width)
			
            

	pygame.quit()

if __name__ == "__main__":
	main(WIN, WIDTH)
