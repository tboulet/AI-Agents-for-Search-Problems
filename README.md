# AI-Agents-for-Search-Problems
An implementation in python of some algorithms for search problems such as A*, that can be applied to any problem object that follow the SearchProblem interface.

The main feature is to define relatively quickly your concrete search problem as a subclass of the interface SearchProblem in order to solve it using already implemented algos. Some of those algos including BFS, DFS, Uniform Cost Search (Dijkstra ) and A* are already implemented. 

An example of how to use those algorithms once you have defined your SearchProblem can be found in example.py or here :

    from search_problems.FindCandy import FindCandyProblem
    from SearchAlgorithms import DFS, BFS, UCS, A_star
    from utils import manhattan_distance

    #Define problem
    problem = FindCandyProblem(side_lenght=10)
    print("Start state:")
    print(problem.get_start_state())

    #Define algorithm solving it, solve it
    print("Solving...")
    def h(state):
        return manhattan_distance(state.pos, problem.goal_pos)
    list_of_actions = A_star(heuristic = h).solve(problem)

    #Test the solution
    print("\nTesting solution :", list_of_actions)
    problem.apply_solution(list_of_actions)

The search problems have to be deterministic : one action in one state always leads to exactly one state.



## Define your search problem
You need to first define your state class that must inherit the State class. A state represent a state of the problem with every information you are giving to the agent. Please implements the following methods, as well as the __str__ method eventually :

    __hash__()  : a state should be hashable
    __eq__(state : State) : two state should be comparable
      
Then you must define your search problem class. Every search problem class must inherit the class SearchProblem and implements the following methods :

    get_start_state()                                  : return the initial state
    is_goal_state(state : State)                       : return whether the state is a goal state
    get_actions(state : State)                         : return the list of actions available in a given state
    get_transition(state : State, action : object)     : return a tuple (next_state, cost_of_action) that represents the transition
      
A path finding problem can be found in search_problem/FindCandy.py as an example.
      

## Define your search algorithm
Some algorithms are already implemented.

    from SearchAlgorithms import DFS, BFS, UCS, A_star
    
But you can also define other search algorithms by inheriting the SearchAlgorithm class and implements the abstract required methods.


## Solve your search problem
Once it is done, you can solve your problem by using method .solve() on an instance of your search problem class.
    
    problem = YourSearchProblem(*args)
    list_of_actions = DFS().solve(problem)
    problem.apply_solution(list_of_actions)

