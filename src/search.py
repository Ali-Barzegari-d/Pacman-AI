import util

class SearchProblem:
    

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def expand(self, state):
        
        util.raiseNotDefined()

    def getActions(self, state):
        """
          state: Search state

        For a given state, this should return a list of possible actions.
        """
        util.raiseNotDefined()

    def getActionCost(self, state, action, next_state):
        
        util.raiseNotDefined()

    def getNextState(self, state, action):
        
        util.raiseNotDefined()

    def getCostOfActionSequence(self, actions):
        
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    
    explored = set() 
    frontier = util.Stack()  
    frontier.push((problem.getStartState(), []))  

    while not frontier.isEmpty():
        state, path = frontier.pop()

        if problem.isGoalState(state):
            return path  

        if state not in explored:
            explored.add(state)  

            
            for successor, action, stepCost in problem.expand(state):
                if successor not in explored:
                    frontier.push((successor, path + [action]))

    return []  
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    explored = set()  
    frontier = util.Queue() 
    frontier.push((problem.getStartState(), []))  

    while not frontier.isEmpty():
        state, path = frontier.pop()

        if problem.isGoalState(state):
            return path  

        if state not in explored:
            explored.add(state)  

            
            for successor, action ,stateCost  in problem.expand(state):
                if successor not in explored:
                    frontier.push((successor, path + [action]))

    return []  
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    
    return 0

def manhattanHeuristic(state, problem):
    ## state --> current_location  
    goal_location = problem.goalState  
    manhattan_distance = abs(state[0] - goal_location[0]) + abs(state[1] - goal_location[1])
    return manhattan_distance


def aStarSearch(problem, heuristic=manhattanHeuristic):
    
    explored = set() 
    frontier = util.PriorityQueue() 
    startState = problem.getStartState()
    frontier.push((startState, []), heuristic(startState, problem)) 

    while not frontier.isEmpty():
        state, path = frontier.pop()

        if problem.isGoalState(state):
            return path 

        if state not in explored:
            explored.add(state)  

           
            for successor, action,stepCost in problem.expand(state):
                if successor not in explored:
                    newPath = path + [action]
                    totalCost = problem.getCostOfActionSequence(newPath) + heuristic(successor, problem)
                    frontier.push((successor, newPath), totalCost)

    return [] 
    util.raiseNotDefined()


def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    explored = set() 
    frontier = util.PriorityQueue()  
    startState = problem.getStartState()
    frontier.push((startState, []), 0)  

    while not frontier.isEmpty():
        state, path = frontier.pop()

        if problem.isGoalState(state):
            return path  

        if state not in explored:
            explored.add(state)  

            
            for successor, action, stepCost in problem.expand(state):
                if successor not in explored:
                    newPath = path + [action]
                    totalCost = problem.getCostOfActionSequence(newPath)
                    frontier.push((successor, newPath), totalCost)

    return []  
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

##Ali Barzegari dahaj