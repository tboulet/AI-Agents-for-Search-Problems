import context
from SearchProblemsAI.search_problems.FindCandy import NonDeterministicFindCandyProblem
from SearchProblemsAI.SearchAlgorithms import NonDeterministicSearchProblemAlgorithm

#Define problem
problem = NonDeterministicFindCandyProblem(side_lenght=4, wall_ratio=0.1)
print("Start state:")
print(problem.get_start_state())

#Define algorithm solving it, solve it
print("Solving...")
algo = NonDeterministicSearchProblemAlgorithm()
plan = algo.solve(problem = problem)

#Test the solution
print("\nTesting solution :")
print(plan)
problem.apply_solution(plan)