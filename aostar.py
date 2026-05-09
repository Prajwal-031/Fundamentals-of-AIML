graph = {
    'A': [[('B', 1)], [('C', 2), ('D', 3)]],
    'B': [[('E', 6)], [('F', 8)]],
    'C': [[('G', 2)], [('H', 0), ('I', 0)]],
    'D': [[('J', 0)]],
    'E': [[]],
    'F': [[]],
    'G': [[]],
    'H': [[]],
    'I': [[]],
    'J': [[]]
}

# Heuristic values
h = {}

for node, groups in graph.items():
    if groups == [[]] or groups == []:
        h[node] = 0
    else:
        h[node] = min(sum(cost for (_, cost) in group) for group in groups)

mark = {}
memo = {}
solved = set()


def solve(node):
    """Compute best cost for a node and store best child group."""

    if graph[node] == [[]] or graph[node] == []:
        memo[node] = h[node]
        solved.add(node)
        return memo[node]

    if node in memo:
        return memo[node]

    best_cost = float('inf')
    best_group = None

    for group in graph[node]:
        total = 0
        for (child, edge_cost) in group:
            total += edge_cost + solve(child)

        if total < best_cost:
            best_cost = total
            best_group = group

    memo[node] = best_cost
    mark[node] = best_group
    solved.add(node)

    return memo[node]


def build_solution(node, visited=None):
    """Return the optimal solution path."""

    if visited is None:
        visited = set()

    if node in visited:
        return []

    visited.add(node)

    if node not in mark:
        return [node]

    result = [node]

    for (child, _) in mark[node]:
        result += build_solution(child, visited)

    return result


root = 'A'

cost = solve(root)
solution_path = build_solution(root)

print("AO* Optimal Solution Path:", solution_path)
print("Total Cost:", cost)