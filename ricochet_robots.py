from abc import ABC

from search import Problem, Node, astar_search, greedy_search
import sys
from copy import deepcopy
import collections
import math

class RRState:
    state_id = 0

    def __init__(self, board):
        self.board = board
        self.id = RRState.state_id
        RRState.state_id += 1

    def __lt__(self, other):
        return self.id < other.id


class Board:
    def __init__(self, numPos, numBorders, robotPos, targetPos, walls):
        self.target = ()
        self.borders = {}
        self.robots = {}

        self.numPos = numPos
        self.numBorders = numBorders
        self.robotPos = robotPos
        self.targetPos = targetPos
        self.walls = walls


        for i in range(0, 4, 1):
            self.robots[(robotPos[i][1])] = robotPos[i][0]

        for i in range(0, numBorders, 1):
            coorx = walls[i][0][0]
            coory = walls[i][0][1]
            if (coorx, coory) not in self.borders:
                self.borders[(coorx, coory)] = []
            self.borders[(coorx, coory)].append(walls[i][1])

            if walls[i][1] == 'l':
                if (coorx, coory - 1) not in self.borders:
                    self.borders[(coorx, coory - 1)] = []
                if('r' not in self.borders[(coorx, coory - 1)]):
                    self.borders[(coorx, coory - 1)].append('r')

            elif walls[i][1] == 'r':
                if (coorx, coory + 1) not in self.borders:
                    self.borders[(coorx, coory + 1)] = [] 
                if('l' not in self.borders[(coorx, coory + 1)]):
                    self.borders[(coorx, coory + 1)].append('l')

            elif walls[i][1] == 'u':
                if (coorx - 1, coory) not in self.borders:
                    self.borders[(coorx - 1, coory)] = []
                if('d' not in self.borders[(coorx - 1, coory)]):
                    self.borders[(coorx - 1, coory)].append('d')

            elif walls[i][1] == 'd':
                if (coorx + 1, coory) not in self.borders:
                    self.borders[(coorx + 1, coory)] = []
                if('d' not in self.borders[(coorx + 1, coory)]):
                    self.borders[(coorx + 1, coory)].append('u')

        self.target = ((targetPos[1][0], targetPos[1][1]),targetPos[0])
        pass

    def robot_position(self, robot: str):
        for robotPos in self.robots:
            if(self.robots[robotPos] == robot):
                return robotPos
        pass


def parse_instance(filename: str) -> Board:
    f = open(filename, "r")

    lines = f.readlines()

    numPos = int(lines[0])

    robotPos = []

    walls = []

    for i in range(1, 5, 1):
        robotPosAux = lines[i].split()
        robotPos.append((robotPosAux [0], (int(robotPosAux [1]), int(robotPosAux[2]))))

    targetPosAux = lines[5].split()
    targetPos = (targetPosAux [0], (int(targetPosAux [1]), int(targetPosAux [2])))

    numBorders = int(lines[6])

    for i in range(7, 7 + numBorders, 1):
        bordersPosAux = lines[i].split()
        walls.append(((int(bordersPosAux[0]), int(bordersPosAux[1])), bordersPosAux[2]))

    f.close()
    board = Board(numPos, numBorders, robotPos, targetPos, walls)
    return board
    pass


class RicochetRobots(Problem):
    def __init__(self, board):
        self.initial = RRState(board)
        self.goal = board.target
        self.robotsAux = []

    def actions(self, state: RRState):
        actions = []
        counter = 0
        i = 0
        
        for robotPos in state.board.robots:
            if (robotPos[0], robotPos[1] + 1) not in state.board.robots and robotPos[1] < state.board.numPos:
                pos = (state.board.robots[robotPos], 'r')
                if robotPos in state.board.borders:
                    if 'r' not in state.board.borders[robotPos]:
                        actions.append(pos)
                else:
                    actions.append(pos)
              
            if (robotPos[0], robotPos[1] - 1) not in state.board.robots and robotPos[1] > 1:
                pos = (state.board.robots[robotPos], 'l')
                if robotPos in state.board.borders:
                    if 'l' not in state.board.borders[robotPos]:
                        actions.append(pos)
                else:
                    actions.append(pos)

            if (robotPos[0] - 1, robotPos[1]) not in state.board.robots and robotPos[0] > 1 :
                pos = (state.board.robots[robotPos], 'u')
                if robotPos in state.board.borders:
                    if 'u' not in state.board.borders[robotPos]:
                        actions.append(pos)
                else:
                    actions.append(pos)


            if (robotPos[0] + 1, robotPos[1]) not in state.board.robots and robotPos[0] < state.board.numPos:
                pos = (state.board.robots[robotPos], 'd')
                if robotPos in state.board.borders:
                    if 'd' not in state.board.borders[robotPos]:
                        actions.append(pos)
                else:
                    actions.append(pos)


        return actions
        
        pass

    def result(self, state: RRState, action):
        i = 0
        robot = action[0]
        direction = action[1]
        coor = ()

        newBoard = Board(state.board.numPos, state.board.numBorders, state.board.robotPos,
                         state.board.targetPos, state.board.walls)

        newBoard.robots = deepcopy(state.board.robots)
        newBoard.borders = deepcopy(state.board.borders)

        for robotPos in newBoard.robots:
            if newBoard.robots[robotPos] == robot:
                del newBoard.robots[robotPos]
                coor = robotPos
                coorx = robotPos[0]
                coory = robotPos[1]
                break

        if direction == 'r':
            while (coorx, coory + 1) not in newBoard.robots and coory < newBoard.numPos:
                if coor in newBoard.borders:
                    if 'r' in newBoard.borders[coor]:
                        break
                coory += 1
                coor = (coorx, coory)

            newBoard.robots[coor] = robot
            if(newBoard.robots in self.robotsAux):
                return state
            else:
                self.robotsAux.append(newBoard.robots)
                return RRState(newBoard)

        elif direction == 'l':
            while (coorx, coory - 1) not in newBoard.robots and coory > 1:
                if coor in newBoard.borders:
                    if 'l' in newBoard.borders[coor]:
                        break
                coory -= 1
                coor = (coorx, coory)

            newBoard.robots[coor] = robot
            if(newBoard.robots in self.robotsAux):
                return state
            else:
                self.robotsAux.append(newBoard.robots)
                return RRState(newBoard)

        elif direction == 'u':
            while (coorx - 1, coory) not in newBoard.robots and coorx > 1:
                if coor in newBoard.borders:
                    if 'u' in newBoard.borders[coor]:
                        break
                coorx -= 1
                coor = (coorx, coory)
            
            newBoard.robots[coor] = robot
            if(newBoard.robots in self.robotsAux):
                return state
            else:
                self.robotsAux.append(newBoard.robots)
                return RRState(newBoard)
            
        elif direction == 'd':
            while (coorx + 1, coory) not in newBoard.robots and coorx < newBoard.numPos:
                if coor in newBoard.borders:
                    if 'd' in newBoard.borders[coor]:
                        break
                coorx += 1
                coor = (coorx, coory)

            newBoard.robots[coor] = robot
            if(newBoard.robots in self.robotsAux):
                return state
            else:
                self.robotsAux.append(newBoard.robots)
                return RRState(newBoard)


    def goal_test(self, state: RRState):
        if state.board.target[0] in state.board.robots:
            return state.board.robots[state.board.target[0]] == state.board.target[1]
        else :
            return False
        pass


    def h(self, node: Node):
        """ Função heuristica utilizada para a procura A*. """
        targetPos = node.state.board.target[0]
        robot_pos = ()
        h = 0

        for robotPos in node.state.board.robots:
            if(node.state.board.robots[robotPos] == node.state.board.target[1]):
                break
        
        distX = abs(int(targetPos[0]) - int(robotPos[0]))
        distY = abs(int(targetPos[1]) - int(robotPos[1]))
        distance = distY + distX
        return distance
        pass

if __name__ == "__main__":

    filename = sys.argv[1]

    board = parse_instance(filename)

    problem = RicochetRobots(board)

    solution_node = astar_search(problem, problem.h)

    print(len(solution_node.solution()))
    for solution in solution_node.solution():
        print(solution[0] + " " + solution[1])
    pass