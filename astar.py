graph = {
's': {'a':1,'b':4},
'a': {'d':12,'b':2,'c':5},
'b': {'c':2},
'c': {'d':3},
'd': {}
}

h = {'s':7,'a':6,'b':2,'c':1,'d':0}

def astar(start, goal):

    open = [start]
    g = {start:0}
    parent = {start:None}
 
    while open:

        n = open[0]
        for node in open:
            if g[node] + h[node] < g[n] + h[n]:
                n = node

        if n == goal:
            path = []
            while n:
                path.append(n)
                n = parent[n]

            print("Path:", path[::-1])
            print("Cost:", g[goal])
            return

        open.remove(n)

        for v in graph[n]:
            cost = g[n] + graph[n][v]

            if v not in g or cost < g[v]:
                g[v] = cost
                parent[v] = n
                open.append(v)

astar('s','d')