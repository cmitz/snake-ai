from gameobjects import GameObject
from move import Move, Direction

import math


class Agent:

    def __init__(self):
        self.food_nodes = []
        self.current_position = (-1, -1)

        self.current_goal_food = None
        self.directions_to_goal_position = []

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
        if len(self.directions_to_goal_position) == 0 or board[self.current_goal_food.x][
            self.current_goal_food.y] != GameObject.FOOD:
            self.calculate_route(board, direction)

        direction = self.directions_to_goal_position.pop(0)
        print(f"Direction: {direction}")
        return direction

    def on_die(self):
        """This function will be called whenever the snake dies. After its dead the snake will be reincarnated into a
        new snake and its life will start over. This means that the next time the get_move function is called,
        it will be called for a fresh snake. Use this function to clean up variables specific to the life of a single
        snake or to host a funeral.
        """
        pass

    def calculate_route(self, board, direction):
        self.directions_to_goal_position.clear()
        self.scan_board(board)

        self.current_goal_food = sorted(self.food_nodes,
                                        key=lambda f: self.heuristic_cost(board, self.current_position, (f.x, f.y)))[0]

        print(f"Start position: {self.current_position}")
        print(f"New goal: ({self.current_goal_food.x}, {self.current_goal_food.y})")

        pos_ahead = self.position_ahead_from_snake(direction, self.current_position)

        node = self.a_star(board, pos_ahead)
        route_reversed = []
        while node is not None:
            pos = (node.x, node.y)
            route_reversed.append(pos)

            node = node.parent

        self.directions_to_goal_position = self.route_to_directions(self.current_position, direction,
                                                                    list(reversed(route_reversed)))

    def route_to_directions(self, current_position, direction, route):
        directions = [Move.STRAIGHT]

        for step in route:
            if direction == Direction.NORTH:
                if step[1] == current_position[1] - 1:  # Straight ahead
                    directions.append(Move.STRAIGHT)
                elif step[0] == current_position[0] + 1:  # To Right
                    directions.append(Move.RIGHT)
                    direction = Direction.EAST
                elif step[0] == current_position[0] - 1:  # To LEFT
                    directions.append(Move.LEFT)
                    direction = Direction.WEST

            elif direction == Direction.EAST:
                if step[0] == current_position[0] + 1:  # Straight ahead
                    directions.append(Move.STRAIGHT)
                elif step[1] == current_position[1] - 1:  # To Right
                    directions.append(Move.RIGHT)
                    direction = Direction.SOUTH
                elif step[1] == current_position[1] + 1:  # To Left
                    directions.append(Move.LEFT)
                    direction = Direction.NORTH

            elif direction == Direction.SOUTH:
                if step[1] == current_position[1] + 1:  # Straight ahead
                    directions.append(Move.STRAIGHT)
                elif step[0] == current_position[0] - 1:  # To Right
                    directions.append(Move.RIGHT)
                    direction = Direction.WEST
                elif step[0] == current_position[0] + 1:  # To Left
                    directions.append(Move.LEFT)
                    direction = Direction.EAST

            elif direction == Direction.WEST:
                if step[0] == current_position[0] - 1:  # Straight ahead
                    directions.append(Move.STRAIGHT)
                elif step[1] == current_position[1] - 1:  # To Right
                    directions.append(Move.RIGHT)
                    direction = Direction.NORTH
                elif step[1] == current_position[1] + 1:  # To Left
                    directions.append(Move.LEFT)
                    direction = Direction.SOUTH

            current_position = step

        return directions

    def a_star(self, board, start_position):
        open_list = []
        closed_list = []

        open_list.append(SearchNode(start_position))

        while len(open_list) > 0:
            open_list = sorted(open_list, key=lambda node: node.f, reverse=True)
            search_node = open_list.pop()

            # print(f"Expand node: ({search_node.x}, {search_node.y}) f = {search_node.f}")
            for child in search_node.adjacent(board):
                if child.x == self.current_goal_food.x and child.y == self.current_goal_food.y:
                    return child

                child.g = search_node.g + 1
                child.h = self.heuristic_cost(board, (child.x, child.y),
                                              (self.current_goal_food.x, self.current_goal_food.y))
                child.f = child.g + child.h

                push_to_open = True
                for n in open_list:
                    if n.x == child.x and n.y == child.y and n.f < child.f:
                        push_to_open = False
                for n in closed_list:
                    if n.x == child.x and n.y == child.y and n.f < child.f:
                        push_to_open = False
                if push_to_open:
                    open_list.append(child)

        print("A* completed, no route found")

    def heuristic_cost(self, board, from_position, to_position):
        bias = 0

        game_object = board[from_position[0]][from_position[1]]
        if game_object == GameObject.WALL:
            print(f"Game object: {game_object}")
            bias = 99
        elif game_object == GameObject.FOOD:
            print(f"Game object: {game_object}")
            bias = -1

        distance_x = abs(from_position[0] - to_position[0])
        distance_y = abs(from_position[1] - to_position[1])
        return bias + math.sqrt((distance_x * distance_x) + (distance_y * distance_y))

    def position_ahead_from_snake(self, direction, current_position):
        if direction == Direction.NORTH:
            return current_position[0], current_position[1] - 1
        elif direction == Direction.EAST:
            return current_position[0] + 1, current_position[1]
        elif direction == Direction.SOUTH:
            return current_position[0], current_position[1] + 1
        elif direction == Direction.WEST:
            return current_position[0] - 1, current_position[1]

    def scan_board(self, board):
        self.food_nodes = []
        self.current_position = (-1, -1)

        for x in range(len(board)):
            for y in range(len(board[x])):
                game_object = board[x][y]

                if game_object == GameObject.SNAKE_HEAD:
                    self.current_position = (x, y)
                if game_object == GameObject.FOOD:
                    self.food_nodes.append(self.FoodNode(x, y))

        self.current_goal_food = self.food_nodes[0]

    class FoodNode:
        def __init__(self, x, y):
            self.x = x
            self.y = y


class SearchNode:
    def __init__(self, position, parent=None):
        self.parent = parent
        self.x = position[0]
        self.y = position[1]
        self.g = 0
        self.h = 0
        self.f = 0

    def adjacent(self, board):
        adjacent = []

        positions = [(self.x, self.y + 1),
                     (self.x + 1, self.y),
                     (self.x, self.y - 1),
                     (self.x - 1, self.y)]

        for position in positions:
            if position[0] < 0 or position[1] < 0 \
                    or position[0] >= self.board_size_x(board) \
                    or position[1] >= self.board_size_y(board):
                break

            go = board[position[0]][position[1]]
            if go != GameObject.WALL and go != GameObject.SNAKE_HEAD and go != GameObject.SNAKE_BODY:
                adjacent.append(SearchNode(position, self))

        return adjacent

    def board_size_x(self, board):
        return len(board)

    def board_size_y(self, board):
        return len(board[0])

    def __str__(self):
        return f"Searchnode, pos: ({self.x}, {self.y}), parent: <{self.parent}>"
