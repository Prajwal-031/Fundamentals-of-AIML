# Alpha-Beta Pruning Algorithm
# Tree from your notebook example

# Tree Structure:
#                MAX
#             /        \
#           MIN        MIN
#         /    \      /    \
#       MAX   MAX   MAX   MAX
#      / \    / \   / \   / \
#    10  9  14 18  5  4  50 3

import math

# Leaf nodes
tree = [
    [[10, 9], [14, 18]],
    [[5, 4], [50, 3]]
]

pruned = []   # store pruned nodes


def alpha_beta(node, depth, alpha, beta, maximizingPlayer):
    
    # Leaf node
    if depth == 3:
        return node

    # MAX player
    if maximizingPlayer:
        maxEval = -math.inf

        for child in node:
            eval = alpha_beta(child, depth + 1, alpha, beta, False)

            maxEval = max(maxEval, eval)
            alpha = max(alpha, eval)

            # Pruning condition
            if beta <= alpha:
                print("Pruned at MAX node")
                break

        return maxEval

    # MIN player
    else:
        minEval = math.inf

        for i, child in enumerate(node):
            eval = alpha_beta(child, depth + 1, alpha, beta, True)

            minEval = min(minEval, eval)
            beta = min(beta, eval)

            # Pruning condition
            if beta <= alpha:
                print("Pruned at MIN node")

                # Store remaining branches as pruned
                for remaining in node[i+1:]:
                    pruned.append(remaining)

                break

        return minEval


# Run algorithm
optimal_value = alpha_beta(tree, 0, -math.inf, math.inf, True)

print("\nOptimal Value:", optimal_value)
print("Pruned Branches:", pruned)