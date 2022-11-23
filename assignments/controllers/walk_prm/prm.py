
from graph import Graph, Vertex
import random
from PIL import Image, ImageDraw

PATH_FILE = "prm_path.txt"
GRAPH_FILE = "prm_graph.jpg"

# bounds for the world/map
x_lower = 0
x_upper = 500
y_lower = 0
y_upper = 500

# dimensions of world/map
width = x_upper - x_lower
height = y_upper - y_lower

# color constants for PIL
white = (255, 255, 255)
black = (0, 0, 0)
blue = (0, 0, 255)
red = (255, 0, 0)
green = (0,128,0)
purple = (153, 119, 187)

# start and goal 2d locations
start = (x_lower+20, y_lower+20)
goal = (x_upper-20, y_upper-20)

# create empty PIL image to draw on in memory, image saved at end
image = Image.new("RGB", (width, height), black)
draw = ImageDraw.Draw(image)

def create_2d_world():
    world = [ [0] * x_upper for _ in range(y_upper)]
    world[goal[0]][goal[1]] = 2
    return world

def PRM(world):

    VERTICES = 1000          # Number of vertices
    NEIGHBORS = 5           # Number of neighbors per vertex

    # create graph, start and goal vertices
    roadmap = Graph()
    start_vertex = Vertex(start)
    goal_vertex = Vertex(goal)

    # add start and goal vertices to the graph
    roadmap.add_vertex(start_vertex)
    roadmap.add_vertex(goal_vertex)

    # draw start and goal vertices
    draw.point([start, goal], red)

    # add N vertices to the world/map 
    while len(roadmap.vertices) != VERTICES:

        # generate random points for vertices
        x = random.randint(x_lower + 1, x_upper - 1)
        y = random.randint(y_lower + 1, y_upper - 1)

        if world[x][y] != -1 and [x,y] not in roadmap.vertices:
            roadmap.add_vertex(Vertex((x,y)))
            draw.point([x,y], green)

    print("\nCreating graph vertices and edges:")
    for vertex in roadmap.vertices:
        print(".", end="")
        neighbors = roadmap.get_neighbors(vertex, NEIGHBORS)
        for neighbor in neighbors:
            if roadmap.has_edge(vertex, neighbor) == False:
                roadmap.add_edge(vertex, neighbor)
                draw.line([vertex.coords, neighbor.coords], white)   

    print("\n\nFinding shortest path from start to goal....")
    return roadmap.dijkstra(start_vertex, goal_vertex)

path = PRM(create_2d_world())
prev = None

with open(PATH_FILE, 'w') as file:
    while path:
        vertex = path.pop()
        if prev != None:
            draw.line([prev, vertex.coords], purple, 2)
        file.write('%d,%d\n' % (vertex.coords))
        prev = vertex.coords

image.save(GRAPH_FILE)

print("\nPath written to: %s" % PATH_FILE)
print("Graph image written to: %s" % GRAPH_FILE)
