import context
from SearchProblemsAI.search_problems.FindCandy import SensorlessFindCandyProblem
from SearchProblemsAI.SearchAlgorithms import DFS, BFS, UCS, A_star

#Define problem
print(SensorlessFindCandyProblem)
problem = SensorlessFindCandyProblem(side_lenght=4, wall_ratio=0.1)
print("Start state:")
print(problem.get_start_state())

#Define algorithm solving it, solve it
print("Solving...")
list_of_actions = UCS().solve(problem)

#Test the solution
print("\nTesting solution :", list_of_actions)
problem.apply_solution(list_of_actions)