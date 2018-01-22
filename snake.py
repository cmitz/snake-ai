from random import randint

from agent import Agent
from gameobjects import GameObject
from move import Direction, Move


class Snake:

    def __init__(self, board_width, board_height, max_tics_to_starve):
        self.board_width = board_width
        self.board_height = board_height
        self.x = randint(0, board_width - 1)
        self.y = randint(0, board_height - 1)
        self.direction = Direction.NORTH
        self.body_parts = []
        self.score = 0
        self.tics_alive = 0
        self.tics_to_starve = max_tics_to_starve
        self.max_tics_to_starve = max_tics_to_starve
        self.agent = Agent()

    def update(self, board):
        if len(self.body_parts) > 0 and self.body_parts[0] != (self.x, self.y):
            self.body_parts = [(self.x, self.y)] + self.body_parts
            del self.body_parts[-1]

        # check starvation (if enabled)
        if self.tics_to_starve != -1 and self.tics_to_starve == 0:
            return True

        # retrieve move from the agent
        move = self.agent.get_move(board.get_copy(), self.score, self.tics_alive, self.tics_to_starve, self.direction)

        # check return value of get_move
        if not (move == Move.RIGHT or move == Move.LEFT or move == Move.STRAIGHT):
            return True

        self.direction = self.direction.get_new_direction(move)
        manipulation = self.direction.get_xy_manipulation()
        self.x += manipulation[0]
        self.y += manipulation[1]

        # check if died
        if self.died(board):
            return True

        # check on collision with food
        if board.board[self.x][self.y] == GameObject.FOOD:
            self.body_parts = [(self.x, self.y)] + self.body_parts
            self.score += 1
            board.eat_food(self.x, self.y)
            if self.max_tics_to_starve != -1:
                self.tics_to_starve = self.max_tics_to_starve + 1

        self.tics_alive += 1
        if self.max_tics_to_starve != -1:
            self.tics_to_starve -= 1

        return False

    def reset(self, board):
        print("Score achieved: {}. Turns it took: {}".format(self.score, self.tics_alive))
        self.agent.on_die()
        self.tics_alive = 0
        self.score = 0
        self.direction = Direction.NORTH
        self.tics_to_starve = self.max_tics_to_starve
        self.x, self.y = board.get_free_xy()
        self.body_parts = []

    def contains_body(self, x, y):
        return (x, y) in self.body_parts

    def contains_head(self, x, y):
        return self.x == x and self.y == y

    def died(self, board):
        if self.x < 0 or self.x >= board.width:
            return True
        if self.y < 0 or self.y >= board.height:
            return True
        if board.is_wall_at(self.x, self.y):
            return True
        if (self.x, self.y) in self.body_parts:
            return True
        return False
