#With DFS

def dfs():
    stack = [(0,0)]
    visited = set()

    while stack:
        a, b = stack.pop()

        if (a,b) in visited:
            continue

        visited.add((a,b))
        print((a,b))

        if b == 2:
            print("Goal reached:", (a,b))
            return

        states = [
            (3,b),   
            (a,4),   
            (0,b),   
            (a,0),   
            (max(0,a-(4-b)), min(4,b+a)),  
            (min(3,a+b), max(0,b-(3-a)))   
        ]

        for s in states:
            if s not in visited:
                stack.append(s)

dfs()