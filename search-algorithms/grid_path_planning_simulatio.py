"""
feat: add grid-based path planning simulation using BFS, DFS, and A* algorithms

- Implemented BFS, DFS, and A* for shortest path discovery
- Supports obstacle handling in a 2D grid
- Added visualization and step-by-step exploration simulation using matplotlib
"""


import matplotlib.pyplot as plt
import time
from collections import deque
import heapq

##Grid: 0 = free, 1 = blocked

grid = [
    [0,0,0,1,0,0,0,0,0,0],
    [0,1,0,1,0,1,1,1,1,0],
    [0,1,0,0,0,0,0,0,1,0],
    [0,1,1,1,1,1,1,0,1,0],
    [0,0,0,0,0,0,1,0,1,0],
    [0,1,1,1,1,0,1,0,1,0],
    [0,0,0,0,1,0,0,0,1,0],
    [1,1,1,0,1,1,1,0,1,0],
    [0,0,0,0,0,0,0,0,1,0],
    [0,1,1,1,1,1,1,0,0,0],
]



rows = len(grid)
cols = len(grid[0])
start = (0, 0)
goal = (rows-1, cols-1)
dirs = [(1,0), (-1,0), (0,1), (0,-1)]

def draw_grid(path=None, explored=None, title=""):
    if path is None: path = []
    if explored is None: explored = []
    plt.figure(figsize=(6,6))
    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == 1:
                plt.scatter(c,r,color='black',s=180)
            elif (r,c) in path:
                plt.scatter(c,r,color='blue',s=120)
            elif (r,c) in explored:
                plt.scatter(c,r,color='lightblue',s=40)
            else:
                plt.scatter(c,r,color='white',edgecolors='gray',s=50)
    plt.scatter(start[1], start[0], color='green', s=180, label='Start')
    plt.scatter(goal[1], goal[0], color='red', s=180, label='Goal')
    plt.gca().invert_yaxis()
    plt.legend(); plt.title(title)
    plt.xticks(range(cols)); plt.yticks(range(rows))
    plt.grid(True, which='both', color='gray', linewidth=0.5)
    plt.show()


def draw(explored=None, path=None, robot=None, title=""):
    if explored is None: explored = []
    if path is None: path = []
    fig = plt.gcf()
    fig.set_size_inches(6,6)
    plt.clf()
    for r in range(rows):
        for c in range(cols):
            if grid[r][c] == 1:
                plt.scatter(c,r,color='black',s=400)
            elif (r,c) in path:
                plt.scatter(c,r,color='yellow',s=200)
            elif (r,c) in explored:
                plt.scatter(c,r,color='lightblue',s=200)
            else:
                plt.scatter(c,r,color='white',edgecolors='gray',s=200)
    if robot:
        plt.scatter(robot[1],robot[0],color='orange',s=400)
    plt.scatter(start[1],start[0],color='green',s=400)
    plt.scatter(goal[1],goal[0],color='red',s=400)
    plt.gca().invert_yaxis()
    if title:
        plt.title(title)
    plt.xticks(range(cols)); plt.yticks(range(rows))
    plt.grid(True, which='both', color='gray', linewidth=0.5)
    plt.draw()
    plt.pause(0.1)


def bfs():
    q = deque([start]); visited={start}; parent={start:None}; explored=[]
    while q:
        n=q.popleft(); explored.append(n)
        if n==goal: break
        for dx,dy in dirs:
            m=(n[0]+dx,n[1]+dy)
            if 0<=m[0]<rows and 0<=m[1]<cols and grid[m[0]][m[1]]==0 and m not in visited:
                visited.add(m); parent[m]=n; q.append(m)
    if goal not in parent: return None, explored
    path=[]; cur=goal
    while cur: path.append(cur); cur=parent[cur]
    return list(reversed(path)), explored

def dijkstra():
    pq = []
    heapq.heappush(pq, (0, start))
    
    dist = {start: 0}
    parent = {start: None}
    visited = set()
    explored = []

    while pq:
        cost, n = heapq.heappop(pq)

        if n in visited:
            continue

        visited.add(n)
        explored.append(n)

        if n == goal:
            break

        for dx, dy in dirs:
            m = (n[0] + dx, n[1] + dy)

            if 0 <= m[0] < rows and 0 <= m[1] < cols and grid[m[0]][m[1]] == 0:
                new_cost = cost + 1

                if m not in dist or new_cost < dist[m]:
                    dist[m] = new_cost
                    parent[m] = n
                    heapq.heappush(pq, (new_cost, m))

    if goal not in parent:
        return None, explored

    path = []
    cur = goal
    while cur:
        path.append(cur)
        cur = parent[cur]

    return list(reversed(path)), explored


def dfs():
    stack=[start]; visited={start}; parent={start:None}; explored=[]
    while stack:
        n=stack.pop(); explored.append(n)
        if n==goal: break
        for dx,dy in dirs:
            m=(n[0]+dx,n[1]+dy)
            if 0<=m[0]<rows and 0<=m[1]<cols and grid[m[0]][m[1]]==0 and m not in visited:
                visited.add(m); parent[m]=n; stack.append(m)
    if goal not in parent: return None, explored
    path=[]; cur=goal
    while cur: path.append(cur); cur=parent[cur]
    return list(reversed(path)), explored

def heuristic(a,b):
    return abs(a[0]-b[0])+abs(a[1]-b[1])

def astar():
    pq=[]; heapq.heappush(pq,(heuristic(start,goal),start))
    visited=set(); parent={start:None}; g={start:0}; explored=[]
    while pq:
        _, n = heapq.heappop(pq)
        if n in visited: continue
        visited.add(n); explored.append(n)
        if n==goal: break
        for dx,dy in dirs:
            m=(n[0]+dx,n[1]+dy)
            if 0<=m[0]<rows and 0<=m[1]<cols and grid[m[0]][m[1]]==0:
                ng = g[n]+1
                if m not in g or ng < g[m]:
                    g[m]=ng; parent[m]=n
                    heapq.heappush(pq,(ng+heuristic(m,goal),m))
    if goal not in parent: return None, explored
    path=[]; cur=goal
    while cur: path.append(cur); cur=parent[cur]
    return list(reversed(path)), explored

def bfs_sim():
    q = deque([start]); visited={start}; parent={start:None}; explored=[]
    while q:
        n=q.popleft(); explored.append(n)
        draw(explored=explored, robot=n, title='BFS Exploration')
        if n==goal: break
        for dx,dy in dirs:
            m=(n[0]+dx,n[1]+dy)
            if 0<=m[0]<rows and 0<=m[1]<cols and grid[m[0]][m[1]]==0 and m not in visited:
                visited.add(m); parent[m]=n; q.append(m)
    if goal not in parent: return None, explored
    path=[]; cur=goal
    while cur: path.append(cur); cur=parent[cur]
    path = list(reversed(path))
    draw(explored=explored, path=path, robot=goal, title='BFS Final Path')
    return path, explored

def dfs_sim():
    stack=[start]; visited={start}; parent={start:None}; explored=[]
    while stack:
        n=stack.pop(); explored.append(n)
        draw(explored=explored, robot=n, title='DFS Exploration')
        if n==goal: break
        for dx,dy in dirs:
            m=(n[0]+dx,n[1]+dy)
            if 0<=m[0]<rows and 0<=m[1]<cols and grid[m[0]][m[1]]==0 and m not in visited:
                visited.add(m); parent[m]=n; stack.append(m)
    if goal not in parent: return None, explored
    path=[]; cur=goal
    while cur: path.append(cur); cur=parent[cur]
    path = list(reversed(path))
    draw(explored=explored, path=path, robot=goal, title='DFS Final Path')
    return path, explored

def astar_sim():
    pq=[]; heapq.heappush(pq,(heuristic(start,goal),start))
    visited=set(); parent={start:None}; g={start:0}; explored=[]
    while pq:
        _, n = heapq.heappop(pq)
        if n in visited: continue
        visited.add(n); explored.append(n)
        draw(explored=explored, robot=n, title='A* Exploration')
        if n==goal: break
        for dx,dy in dirs:
            m=(n[0]+dx,n[1]+dy)
            if 0<=m[0]<rows and 0<=m[1]<cols and grid[m[0]][m[1]]==0:
                ng = g[n]+1
                if m not in g or ng < g[m]:
                    g[m]=ng; parent[m]=n
                    heapq.heappush(pq,(ng+heuristic(m,goal),m))
    if goal not in parent: return None, explored
    path=[]; cur=goal
    while cur: path.append(cur); cur=parent[cur]
    path = list(reversed(path))
    draw(explored=explored, path=path, robot=goal, title='A* Final Path')
    return path, explored

def dijkstra_sim():
    pq = []
    heapq.heappush(pq, (0, start))

    dist = {start: 0}
    parent = {start: None}
    visited = set()
    explored = []

    while pq:
        cost, n = heapq.heappop(pq)

        if n in visited:
            continue

        visited.add(n)
        explored.append(n)

        draw(explored=explored, robot=n, title='Dijkstra Exploration')

        if n == goal:
            break

        for dx, dy in dirs:
            m = (n[0] + dx, n[1] + dy)

            if 0 <= m[0] < rows and 0 <= m[1] < cols and grid[m[0]][m[1]] == 0:
                new_cost = cost + 1

                if m not in dist or new_cost < dist[m]:
                    dist[m] = new_cost
                    parent[m] = n
                    heapq.heappush(pq, (new_cost, m))

    if goal not in parent:
        return None, explored

    path = []
    cur = goal
    while cur:
        path.append(cur)
        cur = parent[cur]

    path = list(reversed(path))
    draw(explored=explored, path=path, robot=goal, title='Dijkstra Final Path')

    return path, explored

def compare_algorithms():
    results = {}

    start_time = time.time()
    p, e = bfs()
    results["BFS"] = (len(p)-1 if p else 0, len(e), time.time()-start_time)

    start_time = time.time()
    p, e = dfs()
    results["DFS"] = (len(p)-1 if p else 0, len(e), time.time()-start_time)

    start_time = time.time()
    p, e = dijkstra()
    results["Dijkstra"] = (len(p)-1 if p else 0, len(e), time.time()-start_time)

    start_time = time.time()
    p, e = astar()
    results["A*"] = (len(p)-1 if p else 0, len(e), time.time()-start_time)

    return results

def simulate_all():
    plt.ion()
    plt.figure(figsize=(6,6))
    print('Simulating BFS...')
    p_bfs, e_bfs = bfs_sim()
    time.sleep(1)
    print('Simulating DFS...')
    p_dfs, e_dfs = dfs_sim()
    time.sleep(1)
    print('Simulating A*...')
    p_astar, e_astar = astar_sim()
    time.sleep(1)
    plt.show()
    print('Simulating Dijkstra...')
    p_dij, e_dij = dijkstra_sim()
    plt.ioff() 
    return p_bfs, e_bfs, p_dfs, e_dfs, p_astar, e_astar, p_dij, e_dij

if __name__=="__main__":
    p_bfs, e_bfs = bfs()
    p_dfs, e_dfs = dfs()
    p_astar, e_astar = astar()
    p_dij, e_dij = dijkstra()
    results = compare_algorithms()
    print("BFS path length:", len(p_bfs)-1 if p_bfs else "none", "explored:", len(e_bfs), "nodes")
    print("BFS path:", p_bfs)
    print("DFS path length:", len(p_dfs)-1 if p_dfs else "none", "explored:", len(e_dfs), "nodes")
    print("DFS path:", p_dfs)
    print("A* path length:", len(p_astar)-1 if p_astar else "none", "explored:", len(e_astar), "nodes")
    print("A* path:", p_astar)
    print("Dijkstra path length:", len(p_dij)-1 if p_dij else "none","explored:", len(e_dij), "nodes")
    print("Dijkstra path:", p_dij)
    print("\nAlgorithm Comparison\n")
    print("Algorithm | Path Length | Nodes Explored | Time")
    print("-----------------------------------------------")
    for k,v in results.items():
        print(f"{k:9} | {v[0]:11} | {v[1]:14} | {v[2]:.6f}s")
    simulate_all()

    