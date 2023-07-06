import pygame
import math
from queue import PriorityQueue
import os, inspect
import pybullet as p
import pybullet_data
import math
import time

WIDTH = 550
WIN = pygame.display.set_mode((WIDTH, WIDTH))
pygame.display.set_caption("A* Path Finding Algorithm")

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

PATH = []
BARRIERpos = []
GOAL = []

class Spot:
	def __init__(self, row, col, width, total_rows):
		self.row = row
		self.col = col
		self.x = row * width
		self.y = col * width
		self.color = WHITE
		self.neighbors = []
		self.width = width
		self.total_rows = total_rows

	def get_pos(self):
		return self.row, self.col

	def is_closed(self):
		return self.color == RED

	def is_open(self):
		return self.color == GREEN

	def is_barrier(self):
		return self.color == BLACK

	def is_start(self):
		return self.color == ORANGE

	def is_end(self):
		return self.color == TURQUOISE

	def reset(self):
		self.color = WHITE

	def make_start(self):
		self.color = ORANGE

	def make_closed(self):
		self.color = RED

	def make_open(self):
		self.color = GREEN

	def make_barrier(self):
		self.color = BLACK

	def make_end(self):
		self.color = TURQUOISE

	def make_path(self):
		self.color = PURPLE

	def draw(self, win):
		pygame.draw.rect(win, self.color, (self.x, self.y, self.width, self.width))

	def update_neighbors(self, grid):
		self.neighbors = []
		if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier(): # DOWN
			self.neighbors.append(grid[self.row + 1][self.col])

		if self.row > 0 and not grid[self.row - 1][self.col].is_barrier(): # UP
			self.neighbors.append(grid[self.row - 1][self.col])

		if self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier(): # RIGHT
			self.neighbors.append(grid[self.row][self.col + 1])

		if self.col > 0 and not grid[self.row][self.col - 1].is_barrier(): # LEFT
			self.neighbors.append(grid[self.row][self.col - 1])
		
		# diagonals but weird
		# if (self.row < self.total_rows - 1) and (not grid[self.row + 1][self.col].is_barrier()) and (self.col > 0) and (not grid[self.row][self.col - 1].is_barrier()): # DOWN LEFT
		# 	self.neighbors.append(grid[self.row + 1][self.col - 1])

		# if self.row < self.total_rows - 1 and not grid[self.row + 1][self.col].is_barrier() and self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier(): # DOWN RIGHT
		# 	self.neighbors.append(grid[self.row + 1][self.col + 1])

		# if self.row > 0 and not grid[self.row - 1][self.col].is_barrier() and self.col > 0 and not grid[self.row][self.col - 1].is_barrier(): # UP LEFT
		# 	self.neighbors.append(grid[self.row - 1][self.col - 1])

		# if self.row > 0 and not grid[self.row - 1][self.col].is_barrier() and self.col < self.total_rows - 1 and not grid[self.row][self.col + 1].is_barrier(): # UP RIGHT
		# 	self.neighbors.append(grid[self.row - 1][self.col + 1])

	def __lt__(self, other):
		return False


def h(p1, p2): #manhattan distance: heuristic function
	x1, y1 = p1
	x2, y2 = p2
	return abs(x1 - x2) + abs(y1 - y2)
	# return math.sqrt((x1-x2)**2 + (y1-y2)**2)


def reconstruct_path(came_from, current, draw): # draw path IMPORTANT BECAUSE ROBOT NEED TO FOLLOW THIS
	while current in came_from:
		current = came_from[current]
		current.make_path()
		PATH.append([current.x / 50, -(current.y / 50) + 10])
		draw()


def algorithm(draw, grid, start, end):
	count = 0
	open_set = PriorityQueue()
	open_set.put((0, count, start)) #add to priority queue
	came_from = {}
	g_score = {spot: float("inf") for row in grid for spot in row}
	g_score[start] = 0 #distance from start
	f_score = {spot: float("inf") for row in grid for spot in row}
	f_score[start] = h(start.get_pos(), end.get_pos()) #distance to end

	open_set_hash = {start}

	while not open_set.empty():
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				pygame.quit() #way to exit the loop

		current = open_set.get()[2] #starting at the start node
		open_set_hash.remove(current)

		if current == end:
			reconstruct_path(came_from, end, draw)
			end.make_end()
			return True

		for neighbor in current.neighbors:
			temp_g_score = g_score[current] + 1

			if temp_g_score < g_score[neighbor]:
				came_from[neighbor] = current
				g_score[neighbor] = temp_g_score
				f_score[neighbor] = temp_g_score + h(neighbor.get_pos(), end.get_pos())
				if neighbor not in open_set_hash:
					count += 1
					open_set.put((f_score[neighbor], count, neighbor))
					open_set_hash.add(neighbor)
					neighbor.make_open()

		draw()

		if current != start:
			current.make_closed()

	return False


def make_grid(rows, width): #makes grid
	grid = []
	gap = width // rows
	for i in range(rows):
		grid.append([])
		for j in range(rows):
			spot = Spot(i, j, gap, rows)
			grid[i].append(spot)

	return grid #2 dimensional list


def draw_grid(win, rows, width): #draws gridlines
	gap = width // rows
	for i in range(rows):
		pygame.draw.line(win, GREY, (0, i * gap), (width, i * gap))
		for j in range(rows):
			pygame.draw.line(win, GREY, (j * gap, 0), (j * gap, width))


def draw(win, grid, rows, width):
	win.fill(WHITE)

	for row in grid:
		for spot in row:
			spot.draw(win) #draws each box color

	draw_grid(win, rows, width) #draws gridlines
	pygame.display.update() #update display


def get_clicked_pos(pos, rows, width): #gets mouse click position
	gap = width // rows
	y, x = pos

	row = y // gap
	col = x // gap

	return row, col


def main(win, width):
	ROWS = 11
	grid = make_grid(ROWS, width)

	start = None
	end = None

	run = True
	while run:
		draw(win, grid, ROWS, width)
		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				run = False

			if pygame.mouse.get_pressed()[0]: # left click
				pos = pygame.mouse.get_pos()
				row, col = get_clicked_pos(pos, ROWS, width)
				spot = grid[row][col]
				if not start and spot != end:
					start = spot
					start.make_start()

				elif not end and spot != start:
					end = spot
					end.make_end()
					GOAL.append(spot.x / 50)
					GOAL.append(-(spot.y / 50) + 10)

				elif spot != end and spot != start:
					spot.make_barrier()
					BARRIERpos.append([spot.x / 50, -(spot.y / 50) + 10])

			elif pygame.mouse.get_pressed()[2]: # RIGHT
				pos = pygame.mouse.get_pos()
				row, col = get_clicked_pos(pos, ROWS, width)
				spot = grid[row][col]
				spot.reset()
				if spot == start:
					start = None
				elif spot == end:
					end = None

			if event.type == pygame.KEYDOWN:
				if event.key == pygame.K_SPACE and start and end:
					for row in grid:
						for spot in row:
							spot.update_neighbors(grid)

					algorithm(lambda: draw(win, grid, ROWS, width), grid, start, end)

				if event.key == pygame.K_c:
					start = None
					end = None
					grid = make_grid(ROWS, width)
            

	pygame.quit()

main(WIN, WIDTH)
BARRIER = []
[BARRIER.append(x) for x in BARRIERpos if x not in BARRIER] # gets rid of repetitve terms

# start of pybullet stuff
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
print("current_dir=" + currentdir)
parentdir = os.path.join(currentdir, "../gym")

os.sys.path.insert(0, parentdir)



cid = p.connect(p.SHARED_MEMORY)
if (cid < 0):
  p.connect(p.GUI)

p.resetSimulation()
p.setGravity(0, 0, -10)

useRealTimeSim = 1

#for video recording (works best on Mac and Linux, not well on Windows)
#p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "racecar.mp4")
p.setRealTimeSimulation(useRealTimeSim)  # either this
#p.loadURDF("plane.urdf")
p.loadSDF(os.path.join(pybullet_data.getDataPath(), "stadium.sdf"))

car = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "racecar/racecar.urdf"))
for i in range(p.getNumJoints(car)):
  print(p.getJointInfo(car, i))

inactive_wheels = [5, 7]
wheels = [2, 3]

for wheel in inactive_wheels:
  p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=0, force=0)

steering = [4, 6]

#targetVelocitySlider = p.addUserDebugParameter("wheelVelocity", -10, 10, 0) #-10 to 10, start at 0
#maxForceSlider = p.addUserDebugParameter("maxForce", 0, 20, 20) #0 to 10, start at 10
#steeringSlider = p.addUserDebugParameter("steering", -0.5, 0.5, 0) #-0.5 to 0.5, start at 0


#BOX IMPLEMENTATION 
box_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.4, 0.4, 0.1])

# Create the box body
for coordinate in BARRIER:
	box_body = p.createMultiBody(
        baseMass=1.0,  # Mass of the box
        baseCollisionShapeIndex=box_shape,  # Corrected argument name
        basePosition=[coordinate[0], coordinate[1], 1],  # Initial position of the box
    )


def moveTo(targetX, targetY):
  pos, hquat = p.getBasePositionAndOrientation(car)
  h = p.getEulerFromQuaternion(hquat)
  x = pos[0]
  y = pos[1]
  distance = math.sqrt((targetX - x)**2 + (targetY - y)**2)
  theta = math.atan2((targetY - y), (targetX - x))
  while distance > 1:
    pos, hquat = p.getBasePositionAndOrientation(car)
    h = p.getEulerFromQuaternion(hquat)
    x = pos[0]
    y = pos[1]
    distance = math.sqrt((targetX - x)**2 + (targetY - y)**2)
    theta = math.atan2((targetY - y), (targetX - x))
    
    maxForce = 20
    targetVelocity = 10*distance
    steeringAngle = theta - h[2]
    if steeringAngle > (math.pi / 2) or steeringAngle < -(math.pi / 2):
       steeringAngle = h[2] - theta
    else:
       steeringAngle = theta - h[2]
       
    for wheel in wheels:
        p.setJointMotorControl2(car,
                            wheel,
                            p.VELOCITY_CONTROL,
                            targetVelocity=targetVelocity,
                            force=maxForce)

    for steer in steering:
        p.setJointMotorControl2(car, steer, p.POSITION_CONTROL, targetPosition=steeringAngle)

    steering
    if (useRealTimeSim == 0):
        p.stepSimulation()
    time.sleep(0.01)


PATH.reverse()
for coordinate in PATH:
   moveTo(coordinate[0], coordinate[1])


while(True): 
    pos, hquat = p.getBasePositionAndOrientation(car)
    h = p.getEulerFromQuaternion(hquat)
    x = pos[0]
    y = pos[1]
    distance = math.sqrt((GOAL[0] - x)**2 + (GOAL[1] - y)**2)
    theta = math.atan2((GOAL[1] - y), (GOAL[0] - x))


    maxForce = 20
    targetVelocity = 10*distance
    steeringAngle = theta - h[2]
    if steeringAngle > (math.pi / 2) or steeringAngle < -(math.pi / 2):
       steeringAngle = h[2] - theta
    else:
       steeringAngle = theta - h[2]

    for wheel in wheels:
        p.setJointMotorControl2(car,
                                wheel,
                                p.VELOCITY_CONTROL,
                                targetVelocity=targetVelocity,
                                force=maxForce)

    for steer in steering:
        p.setJointMotorControl2(car, steer, p.POSITION_CONTROL, targetPosition=steeringAngle)

    steering
    if (useRealTimeSim == 0):
        p.stepSimulation()
    time.sleep(0.01)
