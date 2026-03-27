from collections import deque, defaultdict

class Graph:
    """Graph implementation for BFS traversal"""
    
    def __init__(self):
        self.graph = defaultdict(list)
    
    def add_edge(self, u, v):
        """Add an edge from vertex u to vertex v"""
        self.graph[u].append(v)
    
    def bfs(self, start):
        """
        Breadth-First Search traversal starting from a given vertex
        
        Args:
            start: Starting vertex for BFS
        
        Returns:
            List of vertices in BFS order
        """
        visited = set()
        queue = deque([start])
        visited.add(start)
        result = []
        
        while queue:
            vertex = queue.popleft()
            result.append(vertex)
            
            # Visit all adjacent vertices
            for neighbor in self.graph[vertex]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append(neighbor)
        
        return result
    
    def bfs_shortest_path(self, start, end):
        """
        Find shortest path between two vertices using BFS
        
        Args:
            start: Starting vertex
            end: Ending vertex
        
        Returns:
            Shortest path as a list, or None if no path exists
        """
        visited = set([start])
        queue = deque([(start, [start])])
        
        while queue:
            vertex, path = queue.popleft()
            
            if vertex == end:
                return path
            
            for neighbor in self.graph[vertex]:
                if neighbor not in visited:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))
        
        return None


# Example usage
if __name__ == "__main__":
    # Create a graph
    g = Graph()
    
    # Add edges
    edges = [
        (0, 1), (0, 2),
        (1, 2), (1, 3),
        (2, 0), (2, 3),
        (3, 3)
    ]
    
    for u, v in edges:
        g.add_edge(u, v)
    
    # BFS traversal from vertex 0
    print("BFS Traversal starting from vertex 0:")
    print(g.bfs(0))
    
    # Find shortest path
    print("\nShortest path from 0 to 3:")
    print(g.bfs_shortest_path(0, 3))
