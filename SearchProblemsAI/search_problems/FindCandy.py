"""
Module for the FindCandy problem. The goal is for an agent to navigate inside a 10*10 grid, in order to reach a location with a candy.
10% of the map are walls, and the rest is empty. The map size and wall percentage are parameters. The initial location of the agent and of the goal are random, as well as the walls.

A non deterministic version is also implemented, where an action to a certain direction will lead to 1 or 2 steps (chosen randomly).
A sensorless version is also implemented, where the goal is now to find a sequence of actions that would lead every possible initial state to a goal state (and not only 1 starting state).
"""
import random
from SearchProblemsAI.SearchProblem import SearchProblem, State, NonDeterministicSearchProblem, SensorlessSearchProblem, SensorlessSearchProblem_v2

class FindCandyState(State):
    def __init__(self, map, pos, goal_pos):
        """
        The state object need to contain every information the agent can access from its perspective.        
        """
        self.map = map
        self.pos = pos
        self.goal_pos = goal_pos
    
    def __str__(self):
        res = "\n"
        for j, line in enumerate(self.map):
            to_print = ""
            for (i, is_free) in enumerate(line):
                if self.pos == (i, j):
                    to_print += "|A|"
                elif self.goal_pos == (i, j):
                    to_print += "|G|"
                elif is_free:
                    to_print += "| |"
                else:
                    to_print += "|#|"
            res += to_print + "\n"
        return res
    
    def __hash__(self):
        return hash(self.pos)
    
    def __eq__(self, other):
        """Inside a problem, the state is defined as unique by the position of the agent only."""
        return self.pos == other.pos

class FindCandyProblem(SearchProblem):
    """The FindCandyProblem class."""
    def __init__(self, side_lenght = 10, wall_ratio = 0.3):
        self.side_lenght = side_lenght
        super().__init__() 
        self.map = [[random.random() > wall_ratio for _ in range(side_lenght)] for _ in range(side_lenght)]
        pos = random.randint(0, side_lenght - 1), random.randint(0, side_lenght - 1)
        xg, yg = random.randint(0, side_lenght - 1), random.randint(0, side_lenght - 1)
        while (xg, yg) == pos or not self.map[yg][xg]:
            xg, yg = random.randint(0, side_lenght - 1), random.randint(0, side_lenght - 1)
        self.goal_pos = xg, yg
        self.start_state = FindCandyState(self.map, pos, self.goal_pos)
    
    def get_start_state(self) -> FindCandyState:
        return self.start_state
    
    def is_goal_state(self, state) -> bool:
        return state.pos == self.goal_pos
    
    def get_actions(self, state) -> list[object]:
        actions = []
        x, y = state.pos
        if x > 0 and self.map[y][x-1]:
            actions.append('left')
        if x < self.side_lenght - 1 and self.map[y][x+1]:
            actions.append('right')
        if y > 0 and self.map[y-1][x]:
            actions.append('up')
        if y < self.side_lenght - 1 and self.map[y+1][x]:
            actions.append('down')
        return actions        
    
    def get_transition(self, state, action) -> tuple[FindCandyState, float]:
        x, y = state.pos
        if action == 'left':
            x -= 1
        elif action == 'right':
            x += 1
        elif action == 'up':
            y -= 1
        elif action == 'down':
            y += 1
        child_state = FindCandyState(self.map, (x, y), self.goal_pos)
        return child_state, 1
    
    


class NonDeterministicFindCandyProblem(FindCandyProblem, NonDeterministicSearchProblem):
    """A Non deterministic version of the FindCandyProblem class. Moving to a direction will lead to randomly 1 or 2 steps."""

    def __init__(self, side_lenght = 10, wall_ratio = 0.4):
        FindCandyProblem.__init__(self, side_lenght, wall_ratio)
    
    def get_transition(self, state, action) -> list[FindCandyState]:
        x, y = state.pos
        childs = list()
        if action == 'left':
            x -= 1
            if x > 0 and self.map[y][x-1]: 
                child_state = FindCandyState(self.map, (x-1, y), self.goal_pos)
                childs.append(child_state)
        elif action == 'right':
            x += 1
            if x < self.side_lenght - 1 and self.map[y][x+1]: 
                child_state = FindCandyState(self.map, (x+1, y), self.goal_pos)
                childs.append(child_state)
        elif action == 'up':
            y -= 1
            if y > 0 and self.map[y-1][x]: 
                child_state = FindCandyState(self.map, (x, y-1), self.goal_pos)
                childs.append(child_state)
        elif action == 'down':
            y += 1
            if y < self.side_lenght - 1 and self.map[y+1][x]: 
                child_state = FindCandyState(self.map, (x, y+1), self.goal_pos)
                childs.append(child_state)
        child_state = FindCandyState(self.map, (x, y), self.goal_pos)
        return childs + [child_state]



class SensorlessFindCandyProblem(SensorlessSearchProblem_v2):
    """The sensorless version of the FindCandy problem. 
    Goal is to find a sequence of actions that would lead every possible initial state to a goal state (and not only 1 starting state)."""
    def __init__(self, side_lenght = 10, wall_ratio = 0.3) -> None:
        physical_problem = FindCandyProblem(side_lenght = side_lenght, wall_ratio = wall_ratio)
        initial_belief_state = set()
        for y in range(physical_problem.side_lenght):
            for x in range(physical_problem.side_lenght):
                if physical_problem.map[y][x]:
                    initial_state = FindCandyState(physical_problem.map, (x, y), physical_problem.goal_pos)
                    initial_belief_state.add(initial_state)
        super().__init__(physical_problem, initial_belief_state)