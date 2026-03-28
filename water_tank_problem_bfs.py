# With BFS

from collections import deque


def bfs():
    queue = deque([(0, 0)])
    visited = set([(0, 0)])

    while queue:
        a, b = queue.popleft()
        print((a, b))

        if b == 2:
            print("Goal reached:", (a, b))
            return

        states = [
            (3, b),
            (a, 4),
            (0, b),
            (a, 0),
            (max(0, a - (4 - b)), min(4, b + a)),
            (min(3, a + b), max(0, b - (3 - a)))
        ]

        for s in states:
            if s not in visited:
                visited.add(s)
                queue.append(s)


bfs()
