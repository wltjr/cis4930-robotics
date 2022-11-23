# Graph Implementation

import math

"""
Vertex class represents a vertex in a graph
"""
class Vertex:

    def __init__(self, coords):
        """
        Vertex class constructor to create new instances of Vertex class/object

        :param coords       the 2D integer coordinates of the vertex
        :param edges        list of neigbor vertices with a connection/edge
        :param parent       parent vertex as part of a path
        """
        self.coords = coords
        self.edges = []
        self.parent = None

    def __eq__(self, coords):
        """
        Check equality of vertex based on coordinates, override default comparison

        :param coords       the 2D integer coordinates of a vertex to compare

        :return boolean     true if the coordinates are the same, false otherwise
        """
        return self.coords == coords

    def __hash__(self):
        return hash((self.coords[0], self.coords[1]))


"""
Graph class represents a graph with vertices and edges
"""
class Graph:

    def __init__(self):
        """
        Graph class constructor to create new instances of Graph class/object
        """
        self.vertices = []

    def add_edge(self, vertexA, vertexB):
        """
        Add an edge between two vertices

        :param vertexA        a vertex to add an edge
        :param vertexB        the other vertex to add an edge
        """
        vertexA.edges.append(vertexB)
        vertexB.edges.append(vertexA)

    def add_vertex(self, vertex):
        """
        Add a vertex to the graph

        :param vertex     the vertex being added to the graph
        """
        self.vertices.append(vertex)

    def get_neighbors(self, vertex, number):
        """
        Find and return all neighbors of a vertex within the Euclidean distance

        :param vertex       a vertex to get its neighbors
        :param number       the number of neighbors to find/get

        :return integer     the distance between the two vertices
        """

        neighbor_dists = []

        for v in self.vertices:
            if v != vertex:
                neighbor_dists.append([v,math.dist(vertex.coords, v.coords)])

        neighbor_dists.sort(key=lambda item:item[1])

        neighbors = []

        for neighbor,dist in neighbor_dists:
            neighbors.append(neighbor)
            if len(neighbors) >= number:
                break

        return neighbors


    def has_edge(self, vertexA, vertexB):
        """
        Check if an edge exists between two vertices

        :param vertexA        a vertex to compare
        :param vertexB        the other vertex to compare

        :return boolean     true if each vertex has an edge to the other vertex, false otherwise
        """
        return vertexA in vertexB.edges and vertexB in vertexA.edges

    def dijkstra(self, start, goal):
        dist = {}

        for vertex in self.vertices:
            dist[vertex] = 1000000
        dist[start] = 0

        while not dist == False:
            vertex = min(dist, key=dist.get)

            if vertex == goal:
                break

            for neighbor in vertex.edges:
                alt = dist[vertex] + math.dist(vertex.coords, neighbor.coords)
                if dist.get(neighbor) != None and alt < dist[neighbor]:
                    dist[neighbor] = alt
                    neighbor.parent = vertex

            del dist[vertex]

        path_stack = []

        while vertex != start and vertex != None:
            path_stack.append(vertex)
            vertex = vertex.parent

        path_stack.append(start)

        return path_stack
