from FindCandy import NonDeterministicFindCandyProblem
from SearchAlgorithms import NonDeterministicSearchProblemAlgorithm

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
problem.apply_solution(plan)