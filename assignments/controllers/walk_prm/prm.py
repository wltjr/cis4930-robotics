
from graph import Graph, Vertex
import random
from PIL import Image, ImageDraw

GRAPH_FILE = "prm_graph.jpg"
PATH_FILE_PATTERN = "prm_path_%d.%s"

# bounds for the world/map
x_lower = 0
x_upper = 1024
y_lower = 0
y_upper = 1024

# dimensions of world/map
width = x_upper - x_lower
height = y_upper - y_lower

# color constants for PIL
gray = (100, 100, 100)
black = (0, 0, 0)
red = (255, 0, 0)
green = (0,128,0)
purple = (153, 119, 187)

# start and goal 2d locations
start_goals = [
        [(x_lower+20, y_lower+20), (x_upper-20, y_upper-20)],
        [(x_upper-20, y_lower+20), (x_lower+20, y_upper-20)],
        [(x_upper/2, y_lower+20), (x_upper/2, y_upper-20)]
    ]

# create empty PIL image to draw on in memory, image saved at end
image = Image.new("RGB", (width, height), black)
draw = ImageDraw.Draw(image)

class Roadmap:

    def __init__(self, vertices, neighbors):
        """
        :param vertices         the number of vertices
        :param neighbors        the number of neighbors per vertex
        """
        self.vertices = vertices
        self.neighbors = neighbors
        self.roadmap = Graph()

    def add_start_goal(self, start, goal):
        self.start_vertex = Vertex(start)
        self.goal_vertex = Vertex(goal)

        # add start and goal vertices to the graph
        self.add_vertex(self.start_vertex, red)
        self.add_vertex(self.goal_vertex, green)

        self.add_edges([self.start_vertex,self.goal_vertex])

    def add_vertex(self, vertex, color):
        self.roadmap.add_vertex(vertex)
        draw.point(vertex.coords, color)

    def add_edges(self, vertices):
        print("Creating graph vertices and edges: ", end="")
        for vertex in vertices:
            print(".", end="")
            neighbors = self.roadmap.get_neighbors(vertex, self.neighbors)
            for neighbor in neighbors:
                if self.roadmap.has_edge(vertex, neighbor) == False:
                    self.roadmap.add_edge(vertex, neighbor)
                    draw.line([vertex.coords, neighbor.coords], gray)   

    def create_map(self):
        # add N vertices to the world/map 
        while len(self.roadmap.vertices) != self.vertices:

            # generate random points for vertices
            x = random.randint(x_lower + 1, x_upper - 1)
            y = random.randint(y_lower + 1, y_upper - 1)

            if [x,y] not in roadmap.roadmap.vertices:
                self.add_vertex(Vertex((x,y)), green)
        
        self.add_edges(self.roadmap.vertices)

roadmap = Roadmap(100, 5)
roadmap.create_map()
image.save(GRAPH_FILE)
print("\nGraph image written to: %s" % GRAPH_FILE)
i = 1

graph = image.copy()

for i, (start, goal) in enumerate(start_goals):
    print("\nFinding shortest path from start %s to goal %s" % (start, goal))
    roadmap.add_start_goal(start, goal)
    path = roadmap.roadmap.dijkstra(roadmap.start_vertex, roadmap.goal_vertex)
    prev = None

    text_file = PATH_FILE_PATTERN % (i+1, "txt")
    image_file = PATH_FILE_PATTERN % (i+1, "jpg")

    with open(text_file, 'w') as file:
        while path:
            vertex = path.pop()
            if prev != None:
                draw.line([prev, vertex.coords], purple, 2)
            file.write('%d,%d\n' % (vertex.coords))
            prev = vertex.coords

    image.save(image_file)

    print("Path written to: %s" % text_file)
    print("Graph path image written to: %s" % image_file)

    image = graph.copy()
    draw = ImageDraw.Draw(image)
