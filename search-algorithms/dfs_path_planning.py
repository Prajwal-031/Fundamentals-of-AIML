def dfs(grid, x, y, goal, visited, path, all_paths):

    n = len(grid)
    m = len(grid[0])

    if (x, y) == goal:
        all_paths.append(path.copy())
        return

    directions = [(-1,0),(1,0),(0,-1),(0,1)]

    for dx, dy in directions:
        nx = x + dx
        ny = y + dy

        if 0 <= nx < n and 0 <= ny < m:
            if grid[nx][ny] == 0 and not visited[nx][ny]:

                visited[nx][ny] = True
                path.append((nx,ny))

                dfs(grid, nx, ny, goal, visited, path, all_paths)

                path.pop()
                visited[nx][ny] = False


grid = [
    [0, 1, 0, 0, 0],
    [0, 1, 0, 1, 0],
    [0, 0, 0, 1, 0],
    [1, 1, 0, 0, 0]
]

n = len(grid)
m = len(grid[0])

visited = [[False]*m for _ in range(n)]
visited[0][0] = True

all_paths = []

dfs(grid,0,0,(n-1,m-1),visited,[(0,0)],all_paths)

print("Total Paths:",len(all_paths))

for p in all_paths:
    print(p)

# Find the optimal (shortest) path
if all_paths:
    optimal_path = min(all_paths, key=len)
    print("Optimal Path:", optimal_path)
    print("Optimal Path Length:", len(optimal_path) - 1)  # subtract 1 because path includes start
else:
    print("No path found")