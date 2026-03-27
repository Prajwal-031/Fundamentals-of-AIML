from collections import deque

def bfs(grid):

    n = len(grid)
    m = len(grid[0])

    queue = deque()
    queue.append((0,0,[(0,0)]))

    visited = [[False]*m for _ in range(n)]
    visited[0][0] = True

    directions = [(-1,0),(1,0),(0,-1),(0,1)]

    while queue:

        x,y,path = queue.popleft()

        if (x,y) == (n-1,m-1):
            print("Shortest path length:",len(path)-1)
            print("Path:",path)
            return

        for dx,dy in directions:

            nx = x + dx
            ny = y + dy

            if 0<=nx<n and 0<=ny<m:

                if grid[nx][ny]==0 and not visited[nx][ny]:

                    visited[nx][ny]=True
                    queue.append((nx,ny,path+[(nx,ny)]))

grid = [
    [0, 1, 0, 0, 0],
    [0, 1, 0, 1, 0],
    [0, 0, 0, 1, 0],
    [1, 1, 0, 0, 0]
]

bfs(grid)