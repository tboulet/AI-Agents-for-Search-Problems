from FindCandy import SensorlessFindCandyProblem
from SearchAlgorithms import DFS, BFS, UCS, A_star

#Define problem
problem = SensorlessFindCandyProblem()
print("Start state:")
print(problem.get_start_state())

#Define algorithm solving it, solve it
print("Solving...")
list_of_actions = UCS().solve(problem)

#Test the solution
print("\nTesting solution :", list_of_actions)
problem.apply_solution(list_of_actions)