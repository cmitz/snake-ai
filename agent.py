from gameobjects import GameObject
from move import Move, Direction

import math


class Agent:

    def __init__(self):
        self.current_position = None
        self.current_goal = None
        self.current_route = []
        self.board_size = (0, 0)
        self.board = None

    def get_move(self, board, score, turns_alive, turns_to_starve, direction):
        """This function behaves as the 'brain' of the snake. You only need to change the code in this function for
        the project. Every turn the agent needs to return a move. This move will be executed by the snake. If this
        functions fails to return a valid return (see return), the snake will die (as this confuses its tiny brain
        that much that it will explode). The starting direction of the snake will be North.

        :param board: A two dimensional array representing the current state of the board. The upper left most
        coordinate is equal to (0,0) and each coordinate (x,y) can be accessed by executing board[x][y]. At each
        coordinate a GameObject is present. This can be either GameObject.EMPTY (meaning there is nothing at the
        given coordinate), GameObject.FOOD (meaning there is food at the given coordinate), GameObject.WALL (meaning
        there is a wall at the given coordinate. TIP: do not run into them), GameObject.SNAKE_HEAD (meaning the head
        of the snake is located there) and GameObject.SNAKE_BODY (meaning there is a body part of the snake there.
        TIP: also, do not run into these). The snake will also die when it tries to escape the board (moving out of
        the boundaries of the array)

        :param score: The current score as an integer. Whenever the snake eats, the score will be increased by one.
        When the snake tragically dies (i.e. by running its head into a wall) the score will be reset. In ohter
        words, the score describes the score of the current (alive) worm.

        :param turns_alive: The number of turns (as integer) the current snake is alive.

        :param turns_to_starve: The number of turns left alive (as integer) if the snake does not eat. If this number
        reaches 1 and there is not eaten the next turn, the snake dies. If the value is equal to -1, then the option
        is not enabled and the snake can not starve.

        :param direction: The direction the snake is currently facing. This can be either Direction.NORTH,
        Direction.SOUTH, Direction.WEST, Direction.EAST. For instance, when the snake is facing east and a move
        straight is returned, the snake wil move one cell to the right.

        :return: The move of the snake. This can be either Move.LEFT (meaning going left), Move.STRAIGHT (meaning
        going straight ahead) and Move.RIGHT (meaning going right). The moves are made from the viewpoint of the
        snake. This means the snake keeps track of the direction it is facing (North, South, West and East).
        Move.LEFT and Move.RIGHT changes the direction of the snake. In example, if the snake is facing north and the
        move left is made, the snake will go one block to the left and change its direction to west.
        """
        if len(self.current_route) > 0:
            return self.current_route.pop(0)

        self.scan_board(board)
        self.current_route = self.calculate_route(self.current_position, self.current_goal, direction)

        return self.current_route.pop(0)

    def on_die(self):
        """This function will be called whenever the snake dies. After its dead the snake will be reincarnated into a
        new snake and its life will start over. This means that the next time the get_move function is called,
        it will be called for a fresh snake. Use this function to clean up variables specific to the life of a single
        snake or to host a funeral.
        """
        pass

    def scan_board(self, board):
        self.board_size = (len(board), len(board[0]))
        self.board = board

        food_positions = []

        for x in range(self.board_size[0]):
            for y in range(self.board_size[1]):
                game_obj = board[x][y]

                if game_obj == GameObject.SNAKE_HEAD:
                    self.current_position = (x, y)

                elif game_obj == GameObject.FOOD:
                    food_positions.append((x, y))

        food_positions = sorted(food_positions, key=lambda f: self.calculate_distance(f, self.current_position))
        self.current_goal = food_positions[0]

    def calculate_route(self, current_position, goal, start_direction):
        moves = []

        route = self.a_star(current_position, goal, start_direction)
        print(f"Route: {route}")

        route_elements = [route.position]
        while route.parent is not None:
            route_elements.append(route.parent.position)
            route = route.parent

        temp_position = current_position
        temp_direction = start_direction
        for next_position in list(reversed(route_elements)):
            move = self.move_from_steps(temp_position, next_position, temp_direction)

            temp_position = next_position
            temp_direction = Direction((temp_direction.value + move.value) % 4)

            moves.append(move)

        return moves

    def a_star(self, current_position, goal, start_direction):
        open_list = []
        closed_list = []
        nodes_expanded = 0

        pos = self.pos_adj(current_position, start_direction)
        open_list.append(SearchNode(pos, None, self.game_object_at(pos)))

        print(f"Stats: start={current_position}, goal={goal}, start_direction={start_direction}")

        while len(open_list) > 0:
            open_list = sorted(open_list, key=lambda node: node.cost())
            current_node = open_list.pop(0)

            nodes_expanded += 1

            if not self.blocked_field(current_node.game_object):
                for child_node in self.adjacent_nodes(current_node):
                    child_node = SearchNode(child_node.position, current_node, child_node
                                            .game_object)

                    if child_node.position == goal:
                        return child_node

                    child_node.g = child_node.parent.g + 1
                    child_node.heuristic = self.calculate_heuristic(child_node.position, goal)

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
        print(f"A* completed, no route found;")

    def calculate_heuristic(self, position, goal):
        bias = 0
        game_object = self.board[position[0]][position[1]]
        if game_object == GameObject.SNAKE_HEAD \
                or game_object == GameObject.SNAKE_BODY \
                or game_object == GameObject.WALL:
            bias += 99

        return bias + Agent.calculate_distance(position, goal)

    def game_object_at(self, position):
        px = position[0]
        py = position[1]
        if 0 <= px < len(self.board) and 0 <= py < len(self.board[0]):
            return self.board[px][py]
        else:
            return GameObject.WALL

    def adjacent_nodes(self, search_node):
        p = search_node.position
        fields = []
        for i in range(4):
            fields.append(self.field_adj(p, Direction(i % 4)))

        return filter(None, fields)

    def field_adj(self, current_position, direction):
        adj = Agent.pos_adj(current_position, direction)
        px = adj[0]
        py = adj[1]

        if 0 <= px < len(self.board) and 0 <= py < len(self.board[0]):
            return Field(adj, self.board[adj[0]][adj[1]])

    @staticmethod
    def move_from_steps(current_position, next_position, current_direction):
        if Agent.calculate_distance(current_position, next_position) != 1:
            print(f"Shit's fucked UP yo! c:{current_position} n:{next_position}")

        for move in Move:
            if Agent.pos_adj(current_position, Direction((current_direction.value + move.value) % 4)) == next_position:
                return move

    @staticmethod
    def blocked_field(game_object):
        if game_object == GameObject.SNAKE_HEAD \
                or game_object == GameObject.SNAKE_BODY \
                or game_object == GameObject.WALL:
            return True

    @staticmethod
    def calculate_distance(position1, position2):
        dx = abs(position1[0] - position2[0])
        dy = abs(position1[1] - position2[1])
        return math.sqrt(dx * dx + dy * dy)

    @staticmethod
    def pos_adj(current_position, direction):
        return tuple(map(lambda a, b: a + b, current_position, Direction.get_xy_manipulation(direction)))


class Field:
    def __init__(self, position, game_object):
        self.position = position
        self.game_object = game_object


class SearchNode:
    def __init__(self, position, parent, game_object):
        self.position = position
        self.g = 0
        self.heuristic = 0
        self.parent = parent
        self.game_object = game_object

    def cost(self):
        return self.g + self.heuristic

    def __str__(self):
        return f"N{self.position} <- {self.parent}"
