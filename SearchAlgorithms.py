"""Module implementing the SearchAlgorithm base class from which all search algorithms can be built.
Several algorithms are implemented in this module such as DFS, BFS, A_star.

Properties:
Completness : whether a solution is found (if one exists)
Optimality : whether an optimal solution is found (if a solution exists)
Space complexity : in terms of states kept in memory. b is the branching factor, m the depth of a solution, M is the maximum depth.
Time complexity
"""
from abc import ABC, abstractmethod
from typing import Union
from random import random
import queue
import heapq

from SearchProblem import Node, OrderedNode, State, SearchProblem, NonDeterministicSearchProblem


class SearchAlgorithm(ABC):
    """The mother class for every SEARCH algorithm. The method solve() is the minimal representation of how a SEARCH algorithm works.
    Every SEARCH algorithm can be built from this class by implementing methods called in solve().
    """
    
    def solve(self, problem : SearchProblem)  -> Union[list[object], None]:
        """Solve the problem using a search algorithm.
        Return a list of actions that lead to the goal state starting from the start state.
        """
        self.init_solver(problem)
        while self.should_keep_searching():
            node = self.extract_best_node_to_explore()
            if problem.is_goal_state(node.state):
                return self.reconstruct_path(node)
            else:
                actions = problem.get_actions(node.state)
                for action in actions:
                    child_state, cost = problem.get_transition(node.state, action)
                    self.deal_with_child_state(child_state, node, action, cost)

        print("No path found")
        return None
    
    @abstractmethod
    def init_solver(self, problem : SearchProblem) -> None:
        """
        Initializes objects such as stacks, queues, sets at the beginning of a solving operation.
        """
        
    @abstractmethod
    def should_keep_searching(self) -> bool:
        """
        Return whether the algorithm should keep searching.
        """
    
    @abstractmethod
    def extract_best_node_to_explore(self) -> Node:
        """
        Return the next node to explore from the frontier and remove it from the frontier.
        """
    
    @abstractmethod
    def deal_with_child_state(self, child_state : State, node : Node, action : object, cost : float) -> None:
        """Given a possibly new child state found, add the corresponding node (or not) to the frontier.
        child_state : a state
        node : the parent of child_state
        action : the action taken to reach this child_state 
        cost : the cost of the action taken to reach this child_state 
        """
    
    #Permanent methods
    def reconstruct_path(self, node : Node) -> list[object]:
        """Given a node, return a list of actions that lead to the node.
        """
        if node.parent == None:
            return []
        return self.reconstruct_path(node.parent) + [node.action]
    
    

class DFS_treeSearch(SearchAlgorithm):
    """Depth First Search algorithm for trees.
    Complete, suboptimal, space complexity: O(bm), time complexity: O(b^m)
    """
    def __init__(self):
        super().__init__()
    
    def init_solver(self, problem):
        initial_node = Node(problem.get_start_state(), None, None)
        self.frontier = [initial_node]
    
    def should_keep_searching(self):
        return len(self.frontier) > 0

    def extract_best_node_to_explore(self):
        return self.frontier.pop()
        
    def deal_with_child_state(self, child_state, node, action, cost):
        child_node = Node(state = child_state, parent = node, action = action)
        self.frontier.append(child_node)
        

class DFS(SearchAlgorithm):
    """Depth First Search algorithm.
    Complete, suboptimal, space complexity: O(bm), time complexity: O(b^M)"""
    def __init__(self):
        super().__init__()
    
    def init_solver(self, problem):
        initial_node = Node(problem.get_start_state(), None, None)
        self.frontier = [initial_node]
        self.explored = {initial_node.state : initial_node} 
    
    def should_keep_searching(self):
        return len(self.frontier) > 0

    def extract_best_node_to_explore(self):
        return self.frontier.pop()
        
    def deal_with_child_state(self, child_state, node, action, cost):
        if not child_state in self.explored:
            child_node = Node(state = child_state, parent = node, action = action)
            self.frontier.append(child_node)
            self.explored[child_state] = child_node


class BFS(SearchAlgorithm):
    """Breadth First Search algorithm.
    Complete, suboptimal, space complexity: O(b^m), time complexity: O(b^m)"""
    def __init__(self):
        super().__init__()
    
    def init_solver(self, problem):
        initial_node = Node(problem.get_start_state(), None, None)
        self.frontier = [initial_node]
        self.explored = {initial_node.state : initial_node} 
    
    def should_keep_searching(self):
        return len(self.frontier) > 0

    def extract_best_node_to_explore(self):
        return self.frontier.pop(0)
        
    def deal_with_child_state(self, child_state, node, action, cost):
        if not child_state in self.explored:
            child_node = Node(state = child_state, parent = node, action = action)
            self.frontier.append(child_node)
            self.explored[child_state] = child_node


class UCS(SearchAlgorithm):
    """Uniform Cost Search algorithm.
    Complete, optimal, space complexity: O(b^m), time complexity: O(b^m)"""
    def __init__(self):
        super().__init__()
        
    def init_solver(self, problem):
        initial_node = OrderedNode(problem.get_start_state(), None, None, 0)
        self.frontier = [initial_node]
        self.explored = {initial_node.state : initial_node}
        
    def should_keep_searching(self):
        return len(self.frontier) > 0
    
    def extract_best_node_to_explore(self):
        return heapq.heappop(self.frontier)
    
    def deal_with_child_state(self, child_state: State, node: OrderedNode, action: object, cost: float):        
        if (not child_state in self.explored) or (self.explored[child_state].cost > self.explored[node.state].cost + cost):
            new_cost = self.explored[node.state].cost + cost
            child_node = OrderedNode(state = child_state, parent = node, action = action, cost = new_cost)
            heapq.heappush(self.frontier, child_node)
            self.explored[child_state] = child_node
        

class A_star(SearchAlgorithm):
    """A* algorithm.
    Complete, optimal if admissible heuristic"""
    def __init__(self, heuristic = lambda state : 0):
        super().__init__()
        self.heuristic = heuristic
        
    def init_solver(self, problem : SearchProblem) -> None:
        initial_state = problem.get_start_state()
        initial_node = OrderedNode(initial_state, None, None, self.heuristic(initial_state))
        self.frontier = [initial_node]
        self.explored = {initial_node.state : initial_node}
        
    def should_keep_searching(self) -> bool:
        return len(self.frontier) > 0
    
    def extract_best_node_to_explore(self) -> OrderedNode:
        return heapq.heappop(self.frontier)
    
    def deal_with_child_state(self, child_state: State, node: OrderedNode, action: object, cost: float) -> None:    
        if child_state in self.explored:
            new_cost = self.explored[node.state].cost + cost + self.heuristic(child_state)
            if self.explored[child_state].cost > new_cost:
                child_node = OrderedNode(state = child_state, parent = node, action = action, cost = new_cost)
                heapq.heappush(self.frontier, child_node)
                self.explored[child_state] = child_node
            
        else:
            new_cost = self.explored[node.state].cost + cost + self.heuristic(child_state)
            child_node = OrderedNode(state = child_state, parent = node, action = action, cost = new_cost)
            heapq.heappush(self.frontier, child_node)
            self.explored[child_state] = child_node
            
            








class NonDeterministicSearchProblemAlgorithm:
    """A solving algorithms that deals with Search Problem that are non deterministic, 
    ie that returns a list of states instead of a tuple (state, cost) for the method .get_transition()
    
    Returns a conditional plan that leads the agent to a goal state in any situations he might fell in.
    The form of this plan is a tuple plan = (action, {state1 : plan1, state2 : plan2, ...}).
    """
    def solve(self, problem : NonDeterministicSearchProblem):
        return self.or_search(problem.get_start_state(), problem, set())
    
    def or_search(self, state : State, problem : NonDeterministicSearchProblem, path : set):
        """Return a plan = {"action" : action, "plans" : {state1 : plan1, ...}) that give the agent the action to take as well as the next plans he will have to consider.
        """
        if problem.is_goal_state(state):
            return ()
        if state in path:
            return 
        for action in problem.get_actions(state):
            plan_statesToPlans = self.and_search(problem.get_transition(state, action), problem, path | {state})
            if plan_statesToPlans is not None:
                return (action, plan_statesToPlans)
    
    def and_search(self, states : list[State], problem : NonDeterministicSearchProblem, path : set):
        """Return a dictionnary {state1 : plan1, ...} that describes what the agent should do in any situation the nature will lead to.
        """
        plan_statesToPlans = {}
        for state in states:
            plan_actionAndPlan = self.or_search(state, problem, path)
            if plan_actionAndPlan is None:
                return None
            plan_statesToPlans[state] = plan_actionAndPlan
        return plan_statesToPlans
    

class NonDeterministicSearchProblemAlgorithm_v2:    #WIP
    """A solving algorithms that deals with Search Problem that are non deterministic, 
    ie that returns a list of states instead of a tuple (state, cost) for the method .get_transition()
    
    Returns a conditional plan that leads the agent to a goal state in any situations he might fell in.
    The form of this plan is a tuple plan = (action, {state1 : plan1, state2 : plan2, ...}).
    """
    def solve(self, problem : NonDeterministicSearchProblem):
        return self.or_search(problem.get_start_state(), problem, set())
    
    def or_search(self, state : State, problem : NonDeterministicSearchProblem, path : set):
        """Return a plan = {"action" : action, "plans" : {state1 : plan1, ...}) that give the agent the action to take as well as the next plans he will have to consider.
        """
        if problem.is_goal_state(state):
            return ()
        if state in path:
            return 
        for action in problem.get_actions(state):
            plan_statesToPlans = self.and_search(problem.get_transition(state, action), problem, path | {state})
            if plan_statesToPlans is not None:
                return (action, plan_statesToPlans)
    
    def and_search(self, states : list[State], problem : NonDeterministicSearchProblem, path : set):
        """Return a dictionnary {state1 : plan1, ...} that describes what the agent should do in any situation the nature will lead to.
        """
        plan_statesToPlans = {}
        for state in states:
            plan_actionAndPlan = self.or_search(state, problem, path)
            if plan_actionAndPlan is None:
                return None
            plan_statesToPlans[state] = plan_actionAndPlan
        return plan_statesToPlans