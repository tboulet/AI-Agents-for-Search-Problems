import context
from SearchProblemsAI.search_problems.FindCandy import FindCandyProblem
from SearchProblemsAI.SearchAlgorithms import NonDeterministicSearchProblemAlgorithm
from SearchProblemsAI.SearchAlgorithms import DFS, BFS, UCS, A_star
from SearchProblemsAI.utils import manhattan_distance

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