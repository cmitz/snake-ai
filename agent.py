from gameobjects import GameObject
from move import Move, Direction

import math

bias_g_weight = 5
bias_snake_body_adjacent = -20
bias_less_corners = -5
bias_wall_adjacent = -2


class Agent:

    def __init__(self):
        self.current_position = None
        self.food_positions = []
        self.current_goal = None
        self.current_route = []
        self.board_size = (0, 0)
        self.board = None

    def get_move(self, board, score, turns_alive, turns_to_starve, direction):
        if len(self.current_route) > 0:
            return self.current_route.pop(0)

        self.scan_board(board)
        self.current_route = self.calculate_route(self.current_position, direction)

        if len(self.current_route) > 0:
            return self.current_route.pop(0)
        else:
            for move in Move:
                pos = self.next_position(self.current_position, self.move_from_direction(direction, move))
                if not self.blocked_field(self.game_object_at(pos)):
                    return move
            return Move.STRAIGHT

    def on_die(self):
        self.current_position = None
        self.food_positions = []
        self.current_goal = None
        self.current_route = []
        self.board_size = (0, 0)
        self.board = None

    def scan_board(self, board):
        self.board_size = (len(board), len(board[0]))
        self.board = board
        self.food_positions = []

        for x in range(self.board_size[0]):
            for y in range(self.board_size[1]):
                game_obj = board[x][y]

                if game_obj == GameObject.SNAKE_HEAD:
                    self.current_position = (x, y)

                elif game_obj == GameObject.FOOD:
                    self.food_positions.append((x, y))

        return self.food_positions

    def calculate_route(self, current_position, start_direction):
        moves = []

        route_elements = []
        total_cost = None
        for food in self.food_positions:
            res = self.a_star(current_position, food, start_direction)

            if res is None:
                return []
            r, t = self.flatten_path(res)
            if total_cost is None or t < total_cost:
                total_cost = t
                route_elements = r

        temp_position = current_position
        last_direction = start_direction
        for next_position in list(reversed(route_elements)):
            if next_position != temp_position:
                move = self.move_from_steps(temp_position, next_position, last_direction)

                last_direction = Direction((last_direction.value + move.value) % 4)
                temp_position = next_position

                moves.append(move)

        return moves

    def flatten_path(self, route):
        total_cost = 0
        route_elements = [route.position]
        while route.parent is not None:
            route = route.parent

            route_elements.append(route.position)
            total_cost += route.cost()
        return route_elements, total_cost

    def a_star(self, current_position, goal, start_direction):
        open_list = []
        closed_list = []
        nodes_expanded = 0

        first_node = SearchNode(current_position, None, self.game_object_at(current_position), start_direction)
        open_list.append(first_node)

        while len(open_list) > 0:
            open_list = sorted(open_list, key=lambda node: node.cost())
            current_node = open_list.pop(0)

            nodes_expanded += 1

            for child_node in self.adjacent_nodes(current_node):
                if self.blocked_field(child_node.game_object):
                    break
                child_node = SearchNode(child_node.position, current_node, child_node
                                        .game_object, child_node.direction)

                if child_node.position == goal:
                    return child_node

                child_node.g = child_node.parent.g + 1
                child_node.heuristic = self.calculate_heuristic(child_node, goal)

                skip = False
                for existing in open_list:
                    if existing.position == child_node.position \
                            and existing.cost() <= child_node.cost():
                        skip = True
                for existing in closed_list:
                    if existing.position == child_node.position \
                            and existing.cost() <= child_node.cost():
                        skip = True
                if not skip:
                    open_list.append(child_node)

            closed_list.append(current_node)

    def calculate_heuristic(self, child_node, goal):
        bias = 0

        adjacent_nodes = self.adjacent_nodes(child_node)
        for node in adjacent_nodes:
            if node.game_object == GameObject.SNAKE_BODY:
                bias += bias_snake_body_adjacent
            elif node.game_object == GameObject.WALL:
                bias += bias_wall_adjacent

            if child_node.parent is not None and child_node.parent.direction == child_node.direction:
                bias += bias_less_corners

        return bias + Agent.calculate_distance(child_node.position, goal)

    def adjacent_nodes(self, search_node):
        p = search_node.position
        fields = []
        for i in range(3):
            fields.append(self.adjacent_node(p, Direction((search_node.direction.value - 1 + i) % 4), search_node))

        return filter(None, fields)

    def game_object_at(self, position):
        px = position[0]
        py = position[1]
        if 0 <= px < len(self.board) and 0 <= py < len(self.board[0]):
            return self.board[px][py]
        else:
            return GameObject.WALL

    def adjacent_node(self, current_position, direction, parent):
        position = Agent.next_position(current_position, direction)
        px = position[0]
        py = position[1]

        if 0 <= px < len(self.board) and 0 <= py < len(self.board[0]):
            return SearchNode(position, parent, self.board[px][py], direction)

    @staticmethod
    def move_from_steps(current_position, next_position, current_direction):
        if Agent.calculate_distance(current_position, next_position) != 1:
            print(f"Shit's fucked UP yo! c:{current_position} n:{next_position}")

        for move in Move:
            if Agent.next_position(current_position,
                                   Agent.move_from_direction(current_direction, move)) == next_position:
                return move

    @staticmethod
    def move_from_direction(direction, move):
        return Direction((direction.value + move.value) % 4)

    @staticmethod
    def calculate_distance(position1, position2):
        dx = abs(position1[0] - position2[0])
        dy = abs(position1[1] - position2[1])
        return math.sqrt(dx * dx + dy * dy)

    @staticmethod
    def next_position(current_position, direction):
        return tuple(map(lambda a, b: a + b, current_position, Direction.get_xy_manipulation(direction)))

    @staticmethod
    def blocked_field(game_object):
        if game_object == GameObject.SNAKE_HEAD \
                or game_object == GameObject.SNAKE_BODY \
                or game_object == GameObject.WALL:
            return True


class SearchNode:
    def __init__(self, position, parent, game_object, direction):
        self.position = position
        self.g = 0
        self.heuristic = 0
        self.parent = parent
        self.game_object = game_object
        self.direction = direction

    def cost(self):
        return math.floor((bias_g_weight * self.g) + self.heuristic)

    def __str__(self):
        return f"N{self.position}/{self.cost()} <- {self.parent}"
