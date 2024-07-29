# Pacman-AI
## Overview of Search Algorithms and Their Implementation in the Pacman Project

In the Pacman project, several search algorithms have been implemented to solve various search problems and find optimal paths. These algorithms include Depth-First Search, Breadth-First Search, A* Search, and Uniform Cost Search. Below is an overview and initial explanations of these algorithms and their implementation.

### SearchProblem Class

This class is defined abstractly and specifies the general structure of the search problem. The methods of this class are:

- `getStartState(self)`: Returns the start state of the problem.
- `isGoalState(self, state)`: Checks if the given state is the goal state.
- `getActions(self, state)`: Returns a list of possible actions for the given state.
- `getNextState(self, state, action)`: Returns the next state based on the current state and action.
- `getActionCost(self, state, action, next_state)`: Returns the cost of transitioning from the current state to the next state given the specified action.
- `getCostOfActionSequence(self, actions)`: Calculates the total cost of the given sequence of actions.
- `expand(self, state)`: Returns the children of the current state along with the corresponding actions and costs.

### Depth-First Search (DFS)

The `depthFirstSearch` function implements the Depth-First Search algorithm:

- This algorithm uses a stack to keep track of nodes being explored.
- New nodes are added to the stack, and the search continues until the stack is empty or the goal node is found.
- The advantage of this algorithm is that it can easily explore deeper into the search space, but it may lead to excessive memory use or getting stuck in infinite loops due to not exploring shallower nodes.

### Breadth-First Search (BFS)

The `breadthFirstSearch` function implements the Breadth-First Search algorithm:

- This algorithm uses a queue to track nodes.
- New nodes are added to the end of the queue and removed from the front to reach the goal.
- This algorithm guarantees that if action costs are non-negative, it will find the optimal path. However, it may consume a lot of memory if the search space is large.

### Uniform Cost Search (UCS)

The `uniformCostSearch` function implements the Uniform Cost Search algorithm:

- It is similar to BFS but explores nodes based on the lowest total cost.
- It uses a priority queue where nodes are sorted and examined based on their total cost.
- This algorithm ensures finding the optimal path even if the costs are irregular.

### A* Search

The `aStarSearch` function implements the A* Search algorithm:

- This algorithm uses a priority queue where nodes are ordered by their total cost (the actual cost so far plus the heuristic estimate to the goal).
- The heuristic estimate to the goal is computed using a heuristic function like `manhattanHeuristic`.
- This algorithm finds the optimal path as long as the heuristic function is admissible and consistent.

### Heuristic Functions

Heuristic functions are used to estimate the remaining cost to reach the goal:

- `nullHeuristic`: Always returns zero and does not provide an estimate of the remaining cost.
- `manhattanHeuristic`: Computes the Manhattan distance (horizontal and vertical distance) between the current state and the goal.

### Search Agents in `searchAgents.py`

The `searchAgents.py` file includes classes that implement various search agents:

- `SearchAgent`: A general search agent that can select different search algorithms based on input parameters.
- `GoWestAgent`: A simple agent that always moves west.
- `StayEastSearchAgent` and `StayWestSearchAgent`: Agents used for searching in the eastern and western parts of the map, respectively.

### Example Commands to Run Algorithms in the Terminal

To execute the search algorithms in the Pacman game, you can use the following commands:

- **Depth-First Search:**
  ```bash
  python pacman.py -l mediumMaze -p SearchAgent
  ```

- **Breadth-First Search:**
  ```bash
  python pacman.py -l mediumMaze -p SearchAgent -a fn=bfs
  ```

- **Uniform Cost Search:**
  ```bash
  python pacman.py -l mediumMaze -p SearchAgent -a fn=ucs
  ```

- **A* Search with Manhattan Heuristic:**
  ```bash
  python pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic
  ```

- **A* Search with Null Heuristic:**
  ```bash
  python pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=astar,heuristic=nullHeuristic
  ```

These explanations should help you understand the search algorithms and their implementation in the Pacman project and enable you to apply them effectively in various search and pathfinding tasks.

---
