from abc import ABC, abstractmethod
import random
from typing import Union

#Definition of objects : 
#   Node, which is a node in the search tree (or OrderedNode)
#   State, which is a state in the search space
#   Belief State, which is a set of state the agent believe it could be in
#   Percept, which is a percept of a state

class Action: pass
class Plan: pass

class Node:
    """The Node class for the SEARCH problem. A node is simply defined as a state, a parent node, and an action taken to reach this state."""
    def __init__(self, state : "State", parent : "Node", action : Action) -> None:
        self.state = state
        self.parent = parent
        self.action = action    

class OrderedNode(Node):
      """Some algorithms requiring to keep the cost in memory and to be able to compare node.
      Such node are 'lower' if their cost is 'lower'."""
      def __init__(self, state: "State", parent: "OrderedNode", action: Action, cost : float) -> None:
            super().__init__(state, parent, action)
            self.cost = cost
      def __lt__(self, other : "OrderedNode") -> bool:
            return self.cost < other.cost
          
class State(ABC):
      """The state of a problem. Every state should be hashable and comparable.
      """
    
      @abstractmethod
      def __hash__(self) -> int:
          pass
        
      @abstractmethod
      def __eq__(self, __o: "State") -> bool:
          pass
      
class BeliefState(State):
    """A belief state, composed of a set of possible states from the point of view of the agent."""
    
    def __init__(self, states : set[State]) -> None:
        super().__init__()
        self.states = states

    def __hash__(self) -> int:
        return hash(tuple(self.states))

    def __eq__(self, other : State) -> bool:
        return self.states == other.states
    
    def __str__(self) -> str:
        # a = ""
        # for s in self.states:
        #     a += '\n\n' + str(s)
        # return a
        for state in self.states:
            return str(state)
            

class Percept(State):
    """A percept is associated to one or several states, it is one of the percept of a state received for the agent."""
    
    @abstractmethod
    def __init__(self, state : State) -> None:
        """Initialize the percept with the state it is associated to.
        """
        

#Definition of SearchProblem in its most basic form as well as some of its variants.
class SearchProblem(ABC):
    """The class for defining a search problem.
    """

    @abstractmethod
    def get_start_state(self) -> State:
        """Returns the start state for the search problem.
        """

    @abstractmethod
    def is_goal_state(self, state) -> bool:
        """Returns True if and only if the state is a valid goal state.
          state: Search state
        """
        
    @abstractmethod
    def get_actions(self, state) -> list[Action]:
        """Returns a list of actions that can be executed in the given state.
          state: Search state
        """

    @abstractmethod
    def get_transition(self, state, action) -> tuple[State, float]:
        """This return a tuple composed of the next state and the cost for reaching this state from its parent.
          state: Search state
          action: Action to be executed
        """
        
    def apply_solution(self, list_of_actions : list[Action]) -> None:
        if list_of_actions is None:
            print("No path found.")
            return
        total_cost = 0
        state = self.get_start_state()
        for action in list_of_actions:
            if not action in self.get_actions(state):
                raise Exception("The action " + str(action) + " is not available in the state " + str(state))
            if self.is_goal_state(state) and list_of_actions != []:
                raise Exception("The goal state has been reached, but they were other actions to take for reaching a goal according to the input.\nTotal cost:", total_cost)
            state, cost = self.get_transition(state, action)
            total_cost += cost
        if self.is_goal_state(state):
            print("The goal state has been reached, and the total cost is:", total_cost)
        else:
            raise Exception("The goal state has not been reached, but the input list of actions is empty.")





class NonDeterministicSearchProblem(SearchProblem):
    """The class for defining a NON DETERMINISTIC search problem. 
    What changes is that the get_transition method returns a list of states and not a tuple (state, cost). This list represent the possible next states given such action was taken.
    """
    @abstractmethod
    def get_transition(self, state, action) -> list[State]:
        """This return a list of states than can be reached by using action in a state.
          state: Search state
          action: Action to be executed
        """

    def apply_solution(self, plan : tuple[Action, Plan]): # plan = (action, plans)        plans = {state : [action, plans]}
        actions_taken = []
        if plan is None:
            print("No path found.")
            return
        state = self.get_start_state()
        while (self.is_goal_state(state)) or plan != ():
            if self.is_goal_state(state):
                print("A goal state has been reached following this plan.")
                print("Action taken:", actions_taken)
                return
            
            action, plans = plan
            if action not in self.get_actions(state):
                raise Exception("The action " + str(action) + " is not available in the state " + str(state))
            
            states = self.get_transition(state, action)    
            state = random.choice(states)
            if state not in plans:
                raise Exception("The state is not in the plan.")
            plan = plans[state]
            actions_taken.append(action)
        raise Exception("Plan has been followed, but the goal state has not been reached.")





class SensorlessSearchProblem(SearchProblem):
    """Class for transforming a search problem into a sensorless search problem. Requires to specify the set of possible states at the start."""
    
    def __init__(self, problem : SearchProblem, initial_belief_state_set : set[State]) -> None:
        super().__init__()
        self.physical_problem = problem
        self.inital_belief_state = BeliefState(initial_belief_state_set)
    
    def get_start_state(self) -> BeliefState:
        return self.inital_belief_state
    
    def is_goal_state(self, belief_state : BeliefState) -> bool:
        for state in belief_state.states:
            if not self.physical_problem.is_goal_state(state):
                return False
        return True

    def get_actions(self, belief_state : BeliefState) -> list[Action]:
        return list(set().union(*[self.physical_problem.get_actions(state) for state in belief_state.states]))

    def get_transition(self, belief_state : BeliefState, action : Action) -> tuple[BeliefState, float]:
        new_states = set()
        pessimistic_cost = 0
        for state in belief_state.states:
            if action in self.physical_problem.get_actions(state):
                new_state, cost = self.physical_problem.get_transition(state, action)
                new_states.add(new_state)
                pessimistic_cost = max(pessimistic_cost, cost)
            else:
                new_states.add(state)
        new_belief_state = BeliefState(new_states)
        return new_belief_state, pessimistic_cost

class SensorlessSearchProblem_v2(SensorlessSearchProblem):
    """Improve the definition a a sensorless problem for solving. 
    This class consider that a belief state is a goal belief state if all of its states are state that were goal state previously.
    The v1 class only consider a belief state as a goal belief state if all of its states are goal states.
    """
    def __init__(self, problem: SearchProblem, initial_belief_state_set: set[State]) -> None:
        super().__init__(problem, initial_belief_state_set)
    
    def get_start_state(self) -> BeliefState:
        belief_state =  super().get_start_state()
        belief_state.was_goal_state = {state : self.physical_problem.is_goal_state(state) for state in belief_state.states}
        return belief_state
    
    def get_transition(self, belief_state : BeliefState, action : Action) -> tuple[BeliefState, float]:
        new_states = set() 
        new_was_goal_state = dict()
        pessimistic_cost = 0
        for state in belief_state.states:
            if action in self.physical_problem.get_actions(state):
                new_state, cost = self.physical_problem.get_transition(state, action)
                new_states.add(new_state)
                pessimistic_cost = max(pessimistic_cost, cost)
                new_was_goal_state[new_state] = belief_state.was_goal_state[state] or self.physical_problem.is_goal_state(new_state)
            else:
                new_states.add(state)
                new_was_goal_state[state] = belief_state.was_goal_state[state]
        new_belief_state = BeliefState(new_states)
        new_belief_state.was_goal_state = new_was_goal_state
        return new_belief_state, pessimistic_cost
        
    def is_goal_state(self, belief_state : BeliefState) -> bool:
        for state in belief_state.states:
            if not belief_state.was_goal_state[state]:
                return False
        return True




class PartiallyObservableSearchProblem(NonDeterministicSearchProblem):  #WIP
    
    def __init__(self, physical_problem : SearchProblem) -> None:
        self.physical_problem = physical_problem
        super().__init__()
        
    def predict(self, belief_state : BeliefState, action : Action) -> BeliefState:
        """Return the belief state the agent is in after taking an action."""
        if isinstance(self.physical_problem, NonDeterministicSearchProblem):
            states = set().union(*[self.physical_problem.get_transitions(state, action) for state in belief_state.states])
        elif isinstance(self.physical_problem, SearchProblem):
            states = {self.physical_problem.get_transition(state, action) for state in belief_state.states}
        else : raise Exception("The problem is not a deterministic or non deterministic problem.")
        return BeliefState(states = states)
    
    def possible_percepts(self, belief_state : BeliefState) -> set[Percept]:
        """Return the set of possible percepts the agent can perceive."""
        return set().union(*[self.physical_problem.get_actions(state) for state in belief_state.states])

