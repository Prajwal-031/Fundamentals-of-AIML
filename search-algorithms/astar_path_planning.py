"""
Implement  pathfinding with obstacle handling in grid navigation using A* algorithm. 
The grid is represented as a 2D list where 0 indicates a free cell and 1 indicates an obstacle. 
The algorithm should find the shortest path from the top-left corner (0,0) to the bottom-right corner (n-1,m-1) while avoiding obstacles. 
The heuristic function used is the Manhattan distance.
"""

import heapq

def heuristic(x,y,gx,gy):
    return abs(x-gx)+abs(y-gy)

def astar(grid):

    n=len(grid)
    m=len(grid[0])

    goal=(n-1,m-1)

    pq=[]
    heapq.heappush(pq,(0,0,0,[(0,0)]))

    visited=set()

    directions=[(-1,0),(1,0),(0,-1),(0,1)]

    while pq:

        f,x,y,path = heapq.heappop(pq)

        if (x,y)==goal:
            print("Final Path:",path)
            print("Total Cost:",len(path)-1)
            return

        if (x,y) in visited:
            continue

        visited.add((x,y))

        for dx,dy in directions:

            nx=x+dx
            ny=y+dy

            if 0<=nx<n and 0<=ny<m and grid[nx][ny]==0:

                g=len(path)
                h=heuristic(nx,ny,goal[0],goal[1])

                f=g+h

                heapq.heappush(pq,(f,nx,ny,path+[(nx,ny)]))

# Test the algorithm
grid = [
    [0, 1, 0, 0, 0],
    [0, 1, 0, 1, 0],
    [0, 0, 0, 1, 0],
    [1, 1, 0, 0, 0]
]

astar(grid)