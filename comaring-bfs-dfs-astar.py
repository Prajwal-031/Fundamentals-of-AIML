from collections import deque
import heapq
import matplotlib.pyplot as plt
import numpy as np


grid = [
    [0, 1, 0, 0, 0],
    [0, 1, 0, 1, 0],
    [0, 0, 0, 1, 0],
    [1, 1, 0, 0, 0]
]

"""
grid = [
[0,0,0,0,0,0,1,0,0,0],
[1,1,1,1,1,0,1,0,1,0],
[0,0,0,0,1,0,1,0,1,0],
[0,1,1,0,1,0,1,0,1,0],
[0,1,0,0,0,0,1,0,1,0],
[0,1,0,1,1,1,1,0,1,0],
[0,1,0,0,0,0,0,0,1,0],
[0,1,1,1,1,1,1,0,1,0],
[0,0,0,0,0,0,0,0,1,0],
[1,1,1,1,1,1,1,0,0,0]
]

 
grid = [
[0,0,0,0,0,0],
[0,1,1,1,1,0],
[0,0,0,0,1,0],
[1,1,1,0,1,0],
[0,0,0,0,0,0],
[1,1,1,1,1,0]
]
"""




"""
# Define the grid once
grid = [
    [0, 0, 0, 0, 1],
    [1, 0, 0, 0, 0],
    [0, 1, 0, 0, 0],
    [0, 0, 0, 1, 0]
]


grid = [
    [0, 1, 0, 0, 0],
    [0, 1, 0, 1, 0],
    [0, 0, 0, 1, 0],
    [1, 1, 0, 0, 0]
]
"""
def bfs(grid):
    n = len(grid)
    m = len(grid[0])
    queue = deque()
    queue.append((0, 0, [(0, 0)]))
    visited = [[False] * m for _ in range(n)]
    visited[0][0] = True
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    while queue:
        x, y, path = queue.popleft()
        if (x, y) == (n - 1, m - 1):
            return path, len(path) - 1

        for dx, dy in directions:
            nx = x + dx
            ny = y + dy
            if 0 <= nx < n and 0 <= ny < m:
                if grid[nx][ny] == 0 and not visited[nx][ny]:
                    visited[nx][ny] = True
                    queue.append((nx, ny, path + [(nx, ny)]))
    return None, None

def dfs(grid, x, y, goal, visited, path, all_paths):
    n = len(grid)
    m = len(grid[0])
    if (x, y) == goal:
        all_paths.append(path.copy())
        return

    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    for dx, dy in directions:
        nx = x + dx
        ny = y + dy
        if 0 <= nx < n and 0 <= ny < m:
            if grid[nx][ny] == 0 and not visited[nx][ny]:
                visited[nx][ny] = True
                path.append((nx, ny))
                dfs(grid, nx, ny, goal, visited, path, all_paths)
                path.pop()
                visited[nx][ny] = False

def get_dfs_optimal(grid):
    n = len(grid)
    m = len(grid[0])
    visited = [[False] * m for _ in range(n)]
    visited[0][0] = True
    all_paths = []
    dfs(grid, 0, 0, (n - 1, m - 1), visited, [(0, 0)], all_paths)
    if all_paths:
        optimal_path = min(all_paths, key=len)
        return optimal_path, len(optimal_path) - 1
    return None, None

def heuristic(x, y, gx, gy):
    return abs(x - gx) + abs(y - gy)

def astar(grid):
    n = len(grid)
    m = len(grid[0])
    goal = (n - 1, m - 1)
    pq = []
    heapq.heappush(pq, (0, 0, 0, [(0, 0)]))
    visited = set()
    directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    while pq:
        f, x, y, path = heapq.heappop(pq)
        if (x, y) == goal:
            return path, len(path) - 1

        if (x, y) in visited:
            continue

        visited.add((x, y))

        for dx, dy in directions:
            nx = x + dx
            ny = y + dy
            if 0 <= nx < n and 0 <= ny < m and grid[nx][ny] == 0:
                g = len(path)
                h = heuristic(nx, ny, goal[0], goal[1])
                f = g + h
                heapq.heappush(pq, (f, nx, ny, path + [(nx, ny)]))
    return None, None

# Run all algorithms
print("BFS:")
bfs_path, bfs_length = bfs(grid)
if bfs_path:
    print("Path:", bfs_path)
    print("Length:", bfs_length)
else:
    print("No path found")

print("\nDFS (Optimal):")
dfs_path, dfs_length = get_dfs_optimal(grid)
if dfs_path:
    print("Path:", dfs_path)
    print("Length:", dfs_length)
else:
    print("No path found")

print("\nA*:")
astar_path, astar_length = astar(grid)
if astar_path:
    print("Path:", astar_path)
    print("Length:", astar_length)
else:
    print("No path found")

# Function to plot the grid and path
def plot_path(grid, path, title, color):
    n = len(grid)
    m = len(grid[0])
    grid_array = np.array(grid)
    
    plt.figure(figsize=(6, 6))
    plt.imshow(grid_array, cmap='Greys', origin='upper')
    
    if path:
        path_x = [p[1] for p in path]
        path_y = [p[0] for p in path]
        plt.plot(path_x, path_y, color=color, linewidth=3, marker='o', markersize=8)
    
    plt.title(title)
    plt.xticks(range(m))
    plt.yticks(range(n))
    plt.grid(True, which='both', color='black', linewidth=1)
    plt.show()

# Plot for each algorithm
plot_path(grid, bfs_path, 'BFS Path', 'blue')
plot_path(grid, dfs_path, 'DFS Optimal Path', 'green')
plot_path(grid, astar_path, 'A* Path', 'red')