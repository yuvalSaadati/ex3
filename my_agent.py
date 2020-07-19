# name : yuval saadati
# id: 205956634
import pddlsim

from valid_actions import ValidActions
from valid_actions import PythonValidActions
from collections import defaultdict
# the keys are all the tiles,
# the value of each key is the connected tile to the key tile
maze_dictionary = {}
# list of balls that reached the goal
reached_balls = []
# list of peoples that reached the goal
reached_person = []

# Using dijsktra algorithm to find the shortest path in graph
# From benalexkeen.com
class Graph():
    def __init__(self):
        """
        self.edges is a dict of all possible next nodes
        e.g. {'X': ['A', 'B', 'C', 'E'], ...}
        self.weights has all the weights between two nodes,
        with the two nodes as a tuple as the key
        e.g. {('X', 'A'): 7, ('X', 'B'): 2, ...}
        """
        self.edges = defaultdict(list)
        self.weights = {}

    def add_edge(self, from_node, to_node, weight):
        # Note: assumes edges are bi-directional
        self.edges[from_node].append(to_node)
        self.edges[to_node].append(from_node)
        self.weights[(from_node, to_node)] = weight
        self.weights[(to_node, from_node)] = weight

class Executor(object):

    def __init__(self):
        super(Executor, self).__init__()

    def initialize(self, services):
        global goals
        global person_name_position_dictionary
        self.services = services
        self.valid_actions_options = ValidActions(self.services.parser, self.services.pddl, self.services.perception)
        self.python_valid_actions_options = PythonValidActions(self.services.parser, self.services.perception)

    # Using dijsktra algorithm to find the shortest path in graph
    # From benalexkeen.com
    def dijsktra(self, graph, initial, end):
        # shortest paths is a dict of nodes
        # whose value is a tuple of (previous node, weight)
        shortest_paths = {initial: (None, 0)}
        current_node = initial
        visited = set()

        while current_node != end:
            visited.add(current_node)
            destinations = graph.edges[current_node]
            weight_to_current_node = shortest_paths[current_node][1]

            for next_node in destinations:
                weight = graph.weights[(current_node, next_node)] + weight_to_current_node
                if next_node not in shortest_paths:
                    shortest_paths[next_node] = (current_node, weight)
                else:
                    current_shortest_weight = shortest_paths[next_node][1]
                    if current_shortest_weight > weight:
                        shortest_paths[next_node] = (current_node, weight)

            next_destinations = {node: shortest_paths[node] for node in shortest_paths if node not in visited}
            if not next_destinations:
                return "Route Not Possible"
            # next node is the destination with the lowest weight
            current_node = min(next_destinations, key=lambda k: next_destinations[k][1])

        # Work back through destinations in shortest path
        path = []
        while current_node is not None:
            path.append(current_node)
            next_node = shortest_paths[current_node][0]
            current_node = next_node
        # Reverse path
        path = path[::-1]
        return path

    # Get maze goal. Only one goal exists
    def get_maze_goal(self):
        dict_peoples_goals = {}
        for sub_goal in self.services.goal_tracking.uncompleted_goals[0].parts:
            if type(sub_goal) == pddlsim.parser_independent.Literal:
                dict_peoples_goals[sub_goal.args[0]] = sub_goal.args[1]
            else:
                for part in sub_goal.parts:
                    dict_peoples_goals[part.args[0]] = part.args[1]
        return dict_peoples_goals

    # Create dictionary for maze graph
    def create_maze_dictionary(self):
        global maze_dictionary
        # probability of each action in domain maze
        dict_prob = self.python_valid_actions_options.get_prob_list()
        # the current state of each predicate
        dict = self.services.perception.get_state()
        # creating dictionary for maze domain. the keys are all the tiles,
        # and the value of each key is the connected tile to the key tile
        for key in dict:
            for tuple_dedication in dict[key]:
                for action in self.services.parser.actions:
                    if key in action:
                        # first tile
                        a = tuple_dedication[0]
                        # second tile
                        b = tuple_dedication[1]
                        if a in maze_dictionary.keys():
                            if b not in maze_dictionary[a]:
                                # adding connected tile to the exists key tile
                                maze_dictionary[a].append((b, dict_prob[action][0], action))
                        else:
                            # create new key for not existing tile in dictionary
                            maze_dictionary[a] = [(b, dict_prob[action][0], action)]

    # Create graph for maze domain
    def create_maze_graph(self):
        global maze_dictionary
        self.create_maze_dictionary()
        graph = Graph()
        edges = []
        # convert the maze dictionary to graph
        for key in maze_dictionary:
            for tuple in maze_dictionary[key]:
                t_t_prob = (key, tuple[0], 1 - tuple[1])
                edges.append(t_t_prob)
        for edge in edges:
            graph.add_edge(*edge)
        return graph

    # Return dictionary of person name and person position in maze domain
    def get_peoples_name_position(self):
        # list of peoples that need to reach the goal
        people_name_list = []
        for sub_goal in self.services.goal_tracking.uncompleted_goals[0].parts:
            if type(sub_goal) == pddlsim.parser_independent.Literal:
                people_name_list.append(sub_goal.args[0])
            else:
                for part in sub_goal.parts:
                    people_name_list.append(part.args[0])

        # dictionary of person name and person position by current state
        person_name_position_dictionary = {}
        # the current state of each predicate
        dict = self.services.perception.get_state()
        for tuple_n_p in dict['at']:
            if tuple_n_p[0] in people_name_list:
                person_name_position_dictionary[tuple_n_p[0]] = tuple_n_p[1]
        return person_name_position_dictionary

    # Return all the goals in football domain
    def get_all_balls_goals(self):
        # create dictionary - keys are the balls names and value is the ball goal
        goals = {}
        for sub_goal in self.services.goal_tracking.uncompleted_goals[0].parts:
            if type(sub_goal) == pddlsim.parser_independent.Literal:
                goals[sub_goal.args[0]] = sub_goal.args[1]
            else:
                for part in sub_goal.parts:
                    goals[part.args[0]] = part.args[1]
        return goals

    # Create football pitch graph, the wights of the edges is the probability of the action 'kick'.
    # Using this graph to find path for kicking the ball to goal
    def football_pitch_kick(self):
        # probability of kick action
        dict_prob = self.python_valid_actions_options.get_prob_list()
        # the value of the probability
        probability = dict_prob['kick'][0]
        # create new graph object
        graph = Graph()
        # edges are connected tiles in football pitch
        edges = []
        # the current state of each predicate
        current_state = self.services.perception.get_state()
        for predicate in current_state:
            if predicate == "connected":
                for connected_tiles in current_state[predicate]:
                    t1 = connected_tiles[0]
                    t2 = connected_tiles[1]
                    # create edge for the connected tiles t1 and t2
                    edges.append((t1, t2, 1 - probability))
        # from benalexkeen.com for using dijsktra algorithm
        for edge in edges:
            graph.add_edge(*edge)
        return graph

    # Create football pitch graph, the wights of the edges is the probability of the action 'move'.
    # Using this graph to find path to move person from tile to tile
    def football_pitch_move(self):
        # probability of kick action
        dict_prob = self.python_valid_actions_options.get_prob_list()
        # the value of the probability
        probability = dict_prob['move'][0]
        # create new graph object
        graph = Graph()
        # edges are connected tiles in football pitch
        edges = []
        # the current state of each predicate
        current_state = self.services.perception.get_state()
        for predicate in current_state:
            if predicate == "connected":
                for connected_tiles in current_state[predicate]:
                    t1 = connected_tiles[0]
                    t2 = connected_tiles[1]
                    # create edge for the connected tiles t1 and t2
                    edges.append((t1, t2, 1 - probability))
        # from benalexkeen.com for using dijsktra algorithm
        for edge in edges:
            graph.add_edge(*edge)
        return graph

    # Return the position of the football player
    def football_player_at(self):
        # the current state of each predicate
        dict = self.services.perception.get_state()
        robby_at = ""
        for key in dict:
            for tuple_dedication in dict[key]:
                if key == 'at-robby':
                    robby_at = tuple_dedication[0]
        return robby_at

    # Return the closet ball to person
    def closest_ball_at(self):
        global reached_balls
        # the current state of each predicate
        dict = self.services.perception.get_state()
        # using the graph which represent moves in football pitch
        graph = self.football_pitch_move()
        # get player position
        person_at = self.football_player_at()
        save_closet_ball = ''
        balls_dict = self.get_all_balls_goals()
        balls_name = balls_dict.keys()
        balls_at_dict = {}
        for ball in dict['at-ball']:
            balls_at_dict[ball[0]] = ball[1]

        # find the last ball which has not yet reached its goal
        if len(dict['ball']) == len(reached_balls) + 1:
            for ball in balls_name:
                if ball not in reached_balls:
                    return ball, balls_at_dict[ball]

        # find the closet ball the person which has not yet reached its goal
        for ball in balls_name:
            min_path = len(graph.edges.values()) + 100
            if len(self.dijsktra(graph, person_at, balls_at_dict[ball])) < min_path and ball not in reached_balls:
                # find the shortest path from person to some ball
                min_path = len(self.dijsktra(graph, person_at, balls_at_dict[ball]))
                save_closet_ball = ball
        return save_closet_ball, balls_at_dict[save_closet_ball]

    # Behavior in which a person go to the closet ball
    def go_to_the_ball(self):
        person_at = self.football_player_at()
        ball_at = self.closest_ball_at()
        ball_at_position = ball_at[1]
        # football pitch graph, the wights of the edges is the probability of the action 'move'.
        graph = self.football_pitch_move()
        # find the shortest path from person to goal
        my_path = self.dijsktra(graph, person_at, ball_at_position)
        return "(move " + my_path[0] + " " + my_path[1] + ")"

    # Return the neighbor with the closet path to goal
    def closet_neighbor(self, ball_at_position, goal_ball_position):
        graph = self.football_pitch_kick()
        # list of ball_at_position neighbors tiles
        t_neighbors = list(set(graph.edges[ball_at_position]))
        min_path = len(graph.edges.values()) + 100
        save_min_path = t_neighbors[0]
        # find the shortest path from person to one of the neighbors
        for t in t_neighbors:
            if len(self.dijsktra(graph, t, goal_ball_position)) < min_path:
                min_path = len(self.dijsktra(graph, t, goal_ball_position))
                save_min_path = t
        return save_min_path

    # Behavior in which a person kick the ball towards the goal
    def kick_the_ball_towards_the_goal(self):
        # if the person in the same position like the person, he can kick the ball
        # football pitch graph, the wights of the edges is the probability of the action 'kick'.
        graph = self.football_pitch_kick()
        # the position of the football player
        person_at = self.football_player_at()
        ball_at = self.closest_ball_at()
        ball_at_name = ball_at[0]
        ball_at_position = ball_at[1]
        # keys are the balls names and value is the ball goal
        goals_dict = self.get_all_balls_goals()
        # find the goal of ball_at_name
        goal_ball_position = goals_dict[ball_at_name]
        # find the shortest path from person to goal, where the person can kick the ball
        my_path = self.dijsktra(graph, person_at, goal_ball_position)
        save_min_path = self.closet_neighbor(ball_at_position, goal_ball_position)
        return "(kick " + ball_at_name + " " + ball_at_position + " " + my_path[1] + " " + save_min_path + ")"

    # Behavior in which a person scores
    def score(self):
        # list of all balls that reached the goal
        global reached_balls
        ball_at = self.closest_ball_at()
        ball_at_name = ball_at[0]
        ball_at_position = ball_at[1]
        # keys are the balls names and value is the ball goal
        goals_dict = self.get_all_balls_goals()
        # find the goal of ball_at_name
        goal_ball_position = goals_dict[ball_at_name]
        reached_balls.append(ball_at_name)
        # get the closet neighbor to ball
        save_min_path = self.closet_neighbor(ball_at_position, goal_ball_position)
        return "(kick " + ball_at_name + " " + ball_at_position + " " + goal_ball_position + " " + save_min_path + ")"

    # Return the next action to apply
    def next_action(self):
        # the keys are all the tiles,
        # the value of each key is the connected tile to the key tile
        global maze_dictionary
        # list of all the people who reached the goal
        global reached_person
        if self.services.goal_tracking.reached_all_goals():
            return None
        # maze domain
        if self.services.parser.domain_name == 'maze':
            # create maze graph
            graph = self.create_maze_graph()
            # dictionary of person name and person position in maze domain
            person_name_position_dictionary = self.get_peoples_name_position()
            # the first person in dictionary
            person_name = person_name_position_dictionary.keys()[0]
            person_position = person_name_position_dictionary[person_name]
            goals_dict = self.get_maze_goal()
            # get the goal in maze domain
            goal = goals_dict[person_name]
            for name in person_name_position_dictionary.keys():
                if name not in reached_person:
                    # get the name of the person who has not yet reached the goal
                    person_name = name
                    # get the position of the person who has not yet reached the goal
                    person_position = person_name_position_dictionary[name]
            # find path from the person to goal
            my_path = self.dijsktra(graph, person_position, goal)
            if len(my_path) < 2:
                return None
            # find the direction that the person can move from his position to my_path[1]
            # my_path[1] is the next tile that the person need to according to my_path
            directions = [i for i in maze_dictionary[my_path[0]] if my_path[1] in i]
            if my_path[1] == goal:
                # reached_person is list of all the people that reached the goal
                reached_person.append(person_name)
            # return (direction + person_name + person position + next tile)
            return "(" + directions[0][2] + " " + person_name + " " + my_path[0] + " " + my_path[1] + ")"
        # football domain
        else:
            # the position of the football player
            person_at = self.football_player_at()
            # the closet ball to person is tuple (ball name, ball position)
            ball_at = self.closest_ball_at()
            ball_at_name = ball_at[0]
            ball_at_position = ball_at[1]
            # keys are the balls names and value is the ball goal
            goals_dict = self.get_all_balls_goals()
            # find the goal of ball_at_name
            goal_ball_position = goals_dict[ball_at_name]
            if person_at == ball_at_position:
                # the person is in the same position like the ball, he can kick the ball towards the goal
                # football pitch graph, the wights of the edges is the probability of the action 'kick'.
                graph = self.football_pitch_kick()
                # find the shortest path from person to goal, where the person can kick the ball
                my_path = self.dijsktra(graph, person_at, goal_ball_position)
                if my_path[1] == goal_ball_position:
                    # can run score behavior
                    return self.score()
                # kick the ball towards the goal
                return self.kick_the_ball_towards_the_goal()
            else:
            # the person is not in the same position like the ball,
            # he can only move to the ball
                return self.go_to_the_ball()