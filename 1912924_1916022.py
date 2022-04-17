import time
import queue
import copy
import sys
import itertools    
import threading
import os
from queue import Queue
import numpy as np
from munkres import Munkres


# =================== GENERAL ===========================
# cấu trúc của một state
class Node:
    def __init__ (self, map = 0, agent=0, box=0, moves = "", distance = 0): #khởi tạo state
        if agent == 0:
            self.Agent = self.getAgentPos(map)
        else:
            self.Agent = agent
        if box == 0:        
            self.Box = self.getBoxPos(map)
        else:
            self.Box = box
        #xu ly map
        self.Moves = moves
        self.Distance = distance  
    def getBoxPos(self, map):                       # lấy danh sách vị trí các hộp trên map
        posList = []
        for j,row in enumerate(map):
            for i,col in enumerate(row):
                if map[j][i] == "*" or map[j][i] == "$":
                    posList.append((i,j))
        posList.sort()
        return posList    
    def getAgentPos(self, map):                     # lấy vị trí của nhân vật
        for j,row in enumerate(map):
            for i,col in enumerate(row):
                if map[j][i] == "@" or map[j][i] == "+":
                    return (i,j)
        return
    # kiểm tra nước đi có hợp lệ hay không, nếu hợp lệ, kết quả trả về là state đích của nước đi đó
    def validMove(self, move, map, goalList = [], cost = (1,1,1)):
        copyNode = copy.deepcopy(self)
        aPos = copyNode.Agent
        x = aPos[0]
        y = aPos[1]
        bPos = copyNode.Box
        path = copyNode.Moves
        dis = copyNode.Distance
        NM = cost[1]
        PI = cost[0]
        PO = cost[2]
        #============
        if move == "L":
            nextpos = (x-1,y)
            nextnextpos = (x-2,y)
        elif move == "R":
            nextpos = (x+1,y)
            nextnextpos = (x+2,y)
        elif move == "U":
            nextpos = (x, y-1)
            nextnextpos = (x, y-2)
        else:
            nextpos = (x, y+1)
            nextnextpos = (x, y+2)

        if map[nextpos[1]][nextpos[0]] == "#": #vi tri ke tiep la tuong
            return False

        elif listContains(nextpos, bPos) != -1:
            #box o next move
            #case 1: next la tuong
            if map[nextnextpos[1]][nextnextpos[0]] == "#":
                return False
            #case 2: next la hop    
            elif map[nextnextpos[1]][nextnextpos[0]] == "X":#deadlock
                return False
            elif listContains(nextnextpos, bPos) != -1:     #nextnext la hop
                return False
            else:                                   #case 3: trong ->co the di chuyen duoc box -> di chuyen
                if listContains(nextpos, goalList) != -1:
                    #box cu nam trong goal list -> push out
                    dis += PO
                else:
                                #typeofmove = "PushIn"
                    dis+= PI
                i = listContains(nextpos, bPos)
                bPos[i] = nextnextpos
                bPos.sort()
                aPos = nextpos
                path += move
                return Node(0, aPos, bPos,path,dis)
        else:
            #next move trong -> move
            dis += NM
            aPos = nextpos
            path += move
            return Node(0, aPos, bPos, path,dis)

    # so sánh một state này với một state khác
    def isEqualTo(self, other):
        if self.Agent != other.Agent:
            return False
        if self.Box != other.Box:
            return False
        return True   

# return the order number of the item if it's in the list, else return -1
def listContains(item, list):
    for i in range(len(list)):  
        if list[i] == item:
            return i
    return -1

# tìm và đánh dấu tất cả deadlock bằng ký tự 'X'
def getDeadPos(map, goalList):
    EmptyMap(map)
    #   tìm các deadlock ở các góc 
    for j,row in enumerate(map):
        for i,col in enumerate(row):
            if map[j][i] == " " and goalList.__contains__((i,j)) == False:
                L = map[j][i-1]
                R = map[j][i+1]
                U = map[j-1][i]
                D = map[j+1][i]
                if L=="#"  and U=="#":
                    map[j][i] = "X"
                elif L=="#" and D=="#":
                    map[j][i] = "X"
                elif R=="#" and D=="#":
                    map[j][i] = "X"
                elif  R=="#" and U=="#":            
                    map[j][i] = "X"
    # deadlock: nằm ở sát tường                 
    flag = 1
    while flag == 1:
        flag = 0
        for j,row in enumerate(map):
            rowDL = []
            for i,col in enumerate(row):
                if map[j][i] == "X" :
                    rowDL.append(i)                 
            for i in range(len(rowDL) - 1):
                allDeadLockU = True
                allDeadLockD = True
                for n in range(rowDL[i],rowDL[i+1]):
                    if map[j][n] == " " and map[j-1][n] != "#":
                        allDeadLockU = False
                    if map[j][n] == " " and map[j+1][n] != "#":
                        allDeadLockD = False
                    if goalList.__contains__((n,j)):
                        allDeadLockU = False
                        allDeadLockD = False
                    if map[j][n] == "#":
                        allDeadLockU = False
                        allDeadLockD = False
                if allDeadLockD == True or allDeadLockU == True:
                    for n in range(rowDL[i],rowDL[i+1]):
                        if map[j][n] == " ":
                            map[j][n] = "X"
                            flag = 1    
        #tìm deadlock ở giữa map, khi bị chặn 2 đầu bởi deadlock và xung quanh là tường           
        for i in range(len(map[0])):
            colDL = []
            for j in range(len(map)):
                if map[j][i] == "X":
                    colDL.append(j)      
            for j in range(len(colDL) - 1):
                allDeadLockL = True
                allDeadLockR = True
                for n in range(colDL[j],colDL[j+1]):
                    if map[n][i] == " " and map[n][i-1] != "#":
                        allDeadLockL = False
                    if map[n][i] == " " and map[n][i+1] != "#":
                        allDeadLockR = False
                    if goalList.__contains__((i,n)):
                        allDeadLockL = False
                        allDeadLockR = False
                    if map[n][i] == "#":
                        allDeadLockL = False
                        allDeadLockR = False        
                if allDeadLockL == True or allDeadLockR == True:
                    for n in range(colDL[j],colDL[j+1]):
                        if map[n][i] == " ":
                            map[n][i] = "X"
                            flag = 1                           
    return

# lấy danh sách các đích đến từ map
def getGoal(map):
    posList = []
    for j,row in enumerate(map):
        for i,col in enumerate(row):
            if map[j][i] == "*" or map[j][i] == "+" or map[j][i] == ".":
                posList.append((i,j))
    posList.sort()
    return posList 

# kiểm tra xem visited có chứs successor hay chưa
def notContains(visited, successor, donotput = 0):
    for visitedNode in visited:
        #case 1: agent o vi tri da co san
        if visitedNode[0] == successor.Agent:
            #xet neu vi tri cac box da co trong list -> visitted
            for boxPos in visitedNode[1]:
                if boxPos == successor.Box:
                    return False
            #ko nam trong list -> not visitted:
            # -> add vao visitted:
            if donotput == 0:
                visitedNode[1].append(successor.Box)
                return True
        #case 2: vi tri agent khac: chua co trong visitted -> add vao visitted:
    if donotput == 0:
        bPosList = [successor.Box]
        visited.append((successor.Agent, bPosList))
    return True

# kiểm tra một trạng thái xem đó có phải trạng thái đích hay không
def isSolution(successor, goalList):
    if successor.Box == goalList:
        return True
    return False    

# dọn sạch các đối tượng người, hộp và đích trên map
def EmptyMap(map):
    for j,row in enumerate(map):
        for i,col in enumerate(row):
            if map[j][i] == "*" or map[j][i] == "+" or map[j][i] == ".":
                map[j][i] = " "
            elif map[j][i] == "$" or map[j][i] == "@":
                map[j][i] = " "     
    return

# lấy kích thước của danh sách visied
def getvisitedsize(visited):
    if len(visited) == 0:
        return 0
    else:
        size = 0
        for i in range(len(visited)):
            size += len(visited[i][1])

        return size    

# =================== FOR BFS ===========================

# giải thuật BFS giải sokoban
def BFSsolver2(map):
    goalList = getGoal(map)                 # lấy danh sách các vị trí đích
    rootNode = Node(map)                    # state khởi đầu lấy từ map
    bPosList = [rootNode.Box]               # lấy vị trí các hộp tại state khởi đầu
    visited = [(rootNode.Agent,bPosList)]   # thêm state đầu tiên ào visited
    nodeQueue = queue.Queue()               # tạo queue
    nodeQueue.put(rootNode)                 # và push trạng thái khởi đầu vào queue
    getDeadPos(map, goalList)               # tìm các vị trí deadlock
    
    while not nodeQueue.empty():                #lấy ra node đầu queue cho tới khi queue rỗng
        node = nodeQueue.get()
        for move in ["L", "R", "U", "D"]:       # thử các trường hợp nước đi là UP, DOWN, RIGHT hoặc LEFT
            successor = node.validMove(move, map)
            if successor != False:              #           nếu nước đi đó hợp lệ:
                if successor.Box == goalList:   #           kiểm tra xem nó có phải là lời giải cần tìm không 
                    return successor.Moves, getvisitedsize(visited) # nếu có trả về đường đi và số trạng thái đã khám phá
                if notContains(visited, successor) == True: # kiểm tra trạng thái đã ở được khám phá hay chưa
                    nodeQueue.put(successor)                # nếu chưa, thêm vào danh sách visited và thêm vào nodequeue
                                                            # ở đây, hàm notContains đã làm hành động thêm vào list visited 
                                                            # nên ta không cần thêm một lần nữa
    return False, 0                             # trả về nếu không tìm được đường đi

# =================== FOR A* ============================
#cấu trúc priorityQueue dùng cho A*
class PriorityQueue(object):
    def __init__(self):
        list = []
        self.pQueue = list
    def isEmpty(self):
        if len(self.pQueue) == 0:
            return True
        return False
    def len(self):
        return len(self.pQueue)
    
    def push(self, item, priority):
        #priority queue = list of tuple (prio, queue)
        if self.isEmpty() == True: #neu empty -> them tuple
            #print("empty->append")
            queue = Queue()
            queue.put(item)
            self.pQueue.append((priority,queue))
        else:                       #ko empty -> xet cac truong hop
            index = -1
            for i in range(len(self.pQueue)): #tim tuple voi priority == priority
                if self.pQueue[i][0] == priority:
                    index = i
            if index != -1: #da ton tai queue voi priority 
                self.pQueue[index][1].put(item)
            else:           #chua ton tai queue -> them tuple
                index = -1
                if priority < self.pQueue[0][0]:       #priority cao nhat: cao hon ca [0] -> them tuple
                    queue = Queue()
                    queue.put(item)
                    self.pQueue.insert(0, (priority,queue))
                elif priority > self.pQueue[len(self.pQueue)-1][0]: #priority thap nhat: thap hon ca [end] -> them tuple
                    queue = Queue()
                    queue.put(item)
                    self.pQueue.append((priority,queue))
                else:                                           #middle
                    for i in range(len(self.pQueue)):
                        if  priority < self.pQueue[i][0]:
                            index = i                           #tim index de chen vao -> them tuple       
                            break
                    if index!= -1:
                        queue = Queue()
                        queue.put(item)
                        self.pQueue.insert(index, (priority,queue))  
                    else:
                        print("ERROR")            
           
    def pop(self):
        #print("get into pop()")
        #print("info :", len(self.pQueue))
        if self.isEmpty() == False:
            #print("False")
            item = self.pQueue[0][1].get()
            #print("get() done")
            if self.pQueue[0][1].empty() == True:
                #print("qQueue.queue empty")
                del self.pQueue[0]
            #print("get item done")
            return item    
        else:
            #print("return nothing")
            return 
    # Kiểm tra xem state đã nằm trong priorityQueue chưa, nếu đã nằm trong pQ và có quãng đường từ gốc lớn hơn 
    # quãng đường từ gốc của state đang nằm trong pQ, ta cập nhật state đó với quãng đường dài hơn, nếu chưa nằm
    # trong pQ thì thêm vào pQ
    def checkIfContains(self,successor, h):
        if self.isEmpty() == True:
            return False
        else: 
            contains = False
            for i in range(len(self.pQueue)):
                for j in range(self.pQueue[i][1].qsize()):
                    node = self.pQueue[i][1].get()
                    if node.isEqualTo(successor) == True: #node === successor
                        #check distance
                        if node.Distance > successor.Distance: 
                            totalDistance = successor.Distance + h
                            self.push(successor, totalDistance)
                        if self.pQueue[i][1].empty() == True:
                            del self.pQueue[i]
                        return True 
                    self.pQueue[i][1].put(node)
                if contains == True: 
                    return True        
        return False
     
# giải thuật A* giải sokoban
def aStarSolver(map):
    rootNode = Node(map)                        #trạng thái khởi đầu
    bPosList = [rootNode.Box]                   # danh sách hộp tại trạng thái khởi đầu
    visited = [(rootNode.Agent,bPosList)]
    priorityQ = PriorityQueue()
    priorityQ.push(rootNode, 0)
    goalList = getGoal(map)
    map2 = copy.deepcopy(map)
    EmptyMap(map2)
    getDeadPos(map,goalList)
    alldistance = getAllDistance(goalList, map2)  

    # lấy ra phần tử đầu cho tới khi queue trống
    while not priorityQ.isEmpty():
        node = priorityQ.pop()                               #
        # thử các bước đi 
        for move in ["L", "R", "U", "D"]:
            successor = node.validMove(move, map, goalList, (1,1,1))
            if successor != False:                              #nếu bước đi là hợp lệ
                if notContains(visited, successor) == False:    #state đã qua -> closed
                    continue
                else:                                           #state chưa qua
                    h = Heuristic(successor, goalList, alldistance) #tính hàm lượng giá h 
                    totalDistance = successor.Distance + h          #tổng chi phí theo ước tính
                    if isSolution(successor, goalList):
                        return successor.Moves, getvisitedsize(visited)     #nếu successor là mục tiêu thì trả về đường đi và visited              
                    if priorityQ.checkIfContains(successor,h) == False:     # kiểm tra xem successor đã nằm trong priorityQueue chưa
                        priorityQ.push(successor, totalDistance)            # 1. nếu đã có: so sánh quãng đường đến đích của state mới và state 
                                                                            # có sẵn trong queue. nếu quãng đường state mới bé hơn thì pop state cũ
                                                                            # và thêm vào state mới ( đã thực hiện pop trong checkIfContains)
                                                                            # 2. nếu chưa: thêm vào priority
    #print("END")
    return False, 0

# tìm quãng đường từ vị trí agent đến vị trí hộp gần nhất) 
def getMinDistance(successor, goalList = []):
    mindis = 10000
    for i in successor.Box:
        if i not in goalList:
            dis = abs(successor.Agent[0] - i[0]) + abs(successor.Agent[1] - i[1])
            if  dis < mindis:
                mindis = dis
    return mindis

# hàm Heuristic, trả về lượng giá của state đó theo ước tính
def Heuristic(successor,goalList = [], allDistance= [], cost = 1):
    boxList = copy.deepcopy(successor.Box)
    length = len(goalList)
    matrix = np.zeros((length, length))
    for i in range(length):
        for j in range(length):
            matrix[i][j] = allDistance[i][boxList[j][0]][boxList[j][1]]
    m = Munkres()
    #print(matrix)
    matrix2 = copy.deepcopy(matrix)   
    indexes = m.compute(matrix)
    total = 0
    for row, column in indexes:
        value = matrix2[row][column]
        total += value
    mindis = getMinDistance(successor, goalList)
    total += mindis*cost   
    return total

# trả về các khoảng cách từ một vị trí bất kỳ đến vị trí đích đối với từng đích trong goalList
def getAllDistance(goalList = [], emptyMap = []):
    x = len(emptyMap[0])
    y = len(emptyMap)
    distanceMapList = []
    for n in range(len(goalList)):
        map = np.zeros((x,y))
        for i in range(x):
            for j in range(y):
                map[i][j] = 999
        map[goalList[n][0], goalList[n][1]] = 0
        pqueue = queue.Queue()
        pqueue.put(goalList[n])
        while not pqueue.empty():
            position = pqueue.get()
            for direction in [(-1,0),(1,0),(0,-1),(0,1)]:
                box = (position[0]+direction[0], position[1]+direction[1])
                agent = (position[0]+2*direction[0], position[1]+2*direction[1])
                if map[box[0]][box[1]] == 999:
                    if emptyMap[box[1]][box[0]] != "#" and emptyMap[agent[1]][agent[0]] != "#":
                        map[box[0]][box[1]] = map[position[0]][position[1]] + 1
                        pqueue.put((box[0], box[1]))
        distanceMapList.append(map)
    return distanceMapList    

#==================== VISUALIZE =================
# clear screen function
def screen_clear():
   # for mac and linux(here, os.name is 'posix')
   if os.name == 'posix':
      _ = os.system('clear')
   else:
      # for windows platfrom
      _ = os.system('cls')

# in đường di chuyển tới đích của một map
def printPath(map, path, x, y):
    screen_clear()
    printMap(map)
    time.sleep(1)
    screen_clear()
  
    oldPos = [" ", "."]
    for i,move in enumerate(path):
        #print(move)
        if map[y][x] == "@":
            n = 0
        elif map[y][x] == "+":
            n = 1  
        else:
            #print("loi tai move "+ move + map[y][x])
            return 

        #===  
        if move == "L":
            nextpos = (x-1,y)
            nextnextpos = (x-2,y)
        elif move == "R":
            nextpos = (x+1,y)
            nextnextpos = (x+2,y)
        elif move == "U":
            nextpos = (x, y-1)
            nextnextpos = (x, y-2)
        else:
            nextpos = (x, y+1)
            nextnextpos = (x, y+2)
        #-----------------------
        if map[nextpos[1]][nextpos[0]] == " ":
            map[nextpos[1]][nextpos[0]] = "@"
            map[y][x] = oldPos[n]
        elif map[nextpos[1]][nextpos[0]] == ".":
            map[nextpos[1]][nextpos[0]] = "+"
            map[y][x] = oldPos[n] 
        elif map[nextpos[1]][nextpos[0]] == "$":
            map[nextpos[1]][nextpos[0]] = "@" 
            map[y][x] = oldPos[n]
            if map[nextnextpos[1]][nextnextpos[0]] == " ":
                map[nextnextpos[1]][nextnextpos[0]] = "$"
            else:    
                map[nextnextpos[1]][nextnextpos[0]] = "*"
        elif map[nextpos[1]][nextpos[0]] == "*":            
            map[nextpos[1]][nextpos[0]] = "+"
            map[y][x] = oldPos[n] 
            if map[nextnextpos[1]][nextnextpos[0]] == " ":
                map[nextnextpos[1]][nextnextpos[0]] = "$"
            else:    
                map[nextnextpos[1]][nextnextpos[0]] = "*"
        (x,y) = nextpos                                               
        printMap(map)
        if i != len(path) - 1:
            time.sleep(0.3)
            screen_clear()

# in một map
def printMap(map):
    for j, row in enumerate(map):
        for i, col in enumerate(row):
            print(col + " ", end="")
        print()

#=================== MAP ==========================

def microcosmos40():
    map = []
    map.append(["#", "#", "#", "#", "#", "#", "#", "#", "#"])
    map.append(["#", "#", "#", " ", " ", " ", "#", "#", "#"])
    map.append(["#", " ", " ", "$", "#", " ", " ", "#", "#"])
    map.append(["#", " ", " ", ".", "@", ".", "$", "#", "#"])
    map.append(["#", "#", ".", "#", " ", "#", " ", " ", "#"])
    map.append(["#", "#", " ", "$", " ", " ", " ", " ", "#"])
    map.append(["#", "#", "#", " ", " ", "#", "#", "#", "#"])
    map.append(["#", "#", "#", "#", "#", "#", "#", "#", "#"])     
    return map    

def microcosmos07():
    map = []
    map.append(["#", "#", "#", "#", "#", "#", "#"])
    map.append(["#", "#", " ", " ", " ", "#", "#"])
    map.append(["#", "#", " ", "#", "$", "#", "#"])
    map.append(["#", "#", " ", "@", " ", " ", "#"])        
    map.append(["#", " ", ".", "#", "$", " ", "#"])
    map.append(["#", " ", ".", " ", " ", "#", "#"])
    map.append(["#", " ", ".", "#", "$", "#", "#"])
    map.append(["#", "#", " ", " ", " ", "#", "#"])
    map.append(["#", "#", "#", "#", "#", "#", "#"])
    return map

def microcosmos01():
    map = []
    map.append(["#", "#", "#", "#", "#", "#", "#", "#", "#"])
    map.append(["#", " ", " ", "#", "#", "#", " ", " ", "#"])
    map.append(["#", " ", "$", " ", "*", " ", "$", " ", "#"])
    map.append(["#", " ", " ", " ", "+", " ", " ", " ", "#"])
    map.append(["#", "#", "#", " ", ".", "$", "#", "#", "#"])
    map.append(["#", "#", "#", " ", ".", " ", "#", "#", "#"])
    map.append(["#", "#", "#", "#", "#", "#", "#", "#", "#"])     
    return map 

def microcosmos12():
	map = []
	map.append(["#", "#", "#", "#", "#", "#", "#"])
	map.append(["#", " ", " ", " ", " ", "#", "#"])
	map.append(["#", " ", " ", "*", " ", " ", "#"])
	map.append(["#", " ", "$", "*", "$", " ", "#"])
	map.append(["#", "#", " ", "*", " ", "#", "#"])
	map.append(["#", "#", " ", "*", " ", "#", "#"])
	map.append(["#", "#", " ", ".", " ", "#", "#"])
	map.append(["#", "#", "#", "+", "#", "#", "#"])
	map.append(["#", "#", "#", "#", "#", "#", "#"])
	return map

def microcosmos28():
	map = []
	map.append(["#", "#", "#", "#", "#", "#", "#", "#", "#"])
	map.append(["#", "#", "#", "#", "#", "#", " ", " ", "#"])	
	map.append(["#", "#", "#", "#", "#", " ", ".", " ", "#"])
	map.append(["#", "#", "#", "#", " ", "$", " ", " ", "#"])
	map.append(["#", " ", "$", " ", "$", ".", "@", "#", "#"])
	map.append(["#", " ", " ", ".", " ", " ", "#", "#", "#"])
	map.append(["#", "#", "#", "#", "#", "#", "#", "#", "#"])
	return map

def minicosmos01():
    map = []
    map.append(["#", "#", "#", "#", "#", "#", "#", "#"])
    map.append(["#", "#", "#", " ", " ", " ", "#", "#"])
    map.append(["#", " ", "$", " ", "#", " ", "#", "#"])
    map.append(["#", " ", "#", " ", " ", ".", " ", "#"])
    map.append(["#", " ", " ", " ", " ", "#", " ", "#"])
    map.append(["#", "#", " ", "#", " ", " ", " ", "#"])
    map.append(["#", "#", "@", " ", " ", "#", "#", "#"])
    map.append(["#", "#", "#", "#", "#", "#", "#", "#"])     
    return map   

def minicosmos02():
    map = []
    map.append(["#", "#", "#", "#", "#", "#", "#", "#"])
    map.append(["#", "#", "#", " ", " ", " ", "#", "#"])
    map.append(["#", " ", "$", " ", "#", " ", "#", "#"])
    map.append(["#", " ", "#", " ", " ", ".", " ", "#"])
    map.append(["#", " ", " ", " ", " ", "#", " ", "#"])
    map.append(["#", "#", "$", "#", ".", " ", " ", "#"])
    map.append(["#", "#", "@", " ", " ", "#", "#", "#"])
    map.append(["#", "#", "#", "#", "#", "#", "#", "#"])     
    return map   

def minicosmos03():
    map = []
    map.append(["#", "#", "#", "#", "#", "#", "#", "#"])
    map.append(["#", "#", "#", " ", " ", " ", "#", "#"])
    map.append(["#", " ", "$", " ", "#", " ", "#", "#"])
    map.append(["#", " ", "#", " ", " ", ".", " ", "#"])
    map.append(["#", " ", ".", " ", " ", "#", " ", "#"])
    map.append(["#", "#", "$", "#", ".", "$", " ", "#"])
    map.append(["#", "#", "@", " ", " ", "#", "#", "#"])
    map.append(["#", "#", "#", "#", "#", "#", "#", "#"])     
    return map   

def minicosmos34():
	map = []
	map.append(["#", "#", "#", "#", "#", "#", "#", "#", "#"])
	map.append(["#", "#", " ", " ", "#", " ", " ", "#", "#"])
	map.append(["#", " ", " ", "*", "*", "$", ".", "#", "#"])
	map.append(["#", " ", " ", " ", "#", " ", " ", " ", "#"])
	map.append(["#", "#", "#", " ", "@", " ", " ", " ", "#"])
	map.append(["#", "#", "#", "#", "#", "#", " ", " ", "#"])
	map.append(["#", "#", "#", "#", "#", "#", "#", "#", "#"])
	return map

def minicosmos35():
	map = []
	map.append(["#", "#", "#", "#", "#", "#", "#", "#"])
	map.append(["#", "#", "#", " ", " ", "#", "#", "#"])
	map.append(["#", " ", " ", " ", "*", "$", " ", "#"])
	map.append(["#", " ", "#", " ", " ", "#", "@", "#"])
	map.append(["#", " ", "#", " ", "*", ".", " ", "#"])
	map.append(["#", " ", " ", " ", "#", "#", "#", "#"])
	map.append(["#", "#", "#", "#", "#", "#", "#", "#"])
	return map

#=========================================== MAIN ===============================
def main():
    done = True
    maps = []
    maps.append(microcosmos01())
    maps.append(microcosmos07())
    maps.append(microcosmos12())
    maps.append(microcosmos28())
    maps.append(microcosmos40())
    maps.append(minicosmos01())
    maps.append(minicosmos02())
    maps.append(minicosmos03())
    maps.append(minicosmos34())
    maps.append(minicosmos35())
    while done:
        print("Please choose map to run:\n<<A* will take a little bit more time to solve microcosmos>>\nmicrocosmos01: 1\tminicosmos01: 6\nmicrocosmos07: 2\tminicosmos02: 7\nmicrocosmos12: 3\tminicosmos03: 8\nmicrocosmos28: 4\tminicosmos34: 9\nmicrocosmos40: 5\tminicosmos35: 10\ntype 0 to escape")
        input1 = int(input())
        while input1< 0 or input1>10:
            print("invalid, please enter the number between 0 and 10")
            input1 = int(input())
        if input1 == 0:
            break
        map = copy.deepcopy(maps[input1-1])
        screen_clear()
        
        print("Please choose solver:\nBFS: 1\nA*: 2\ntype 0 to escape")
        input2 = int(input())
        while input2!= 0 and input2!=1 and input2!=2:
            print("invalid, please enter the number between 0 and 2")
            input2 = int(input())
        
        if input2 == 0:
            break
        screen_clear()
        animating = False
        #here is the animation
        def animate():
            for c in itertools.cycle(['|', '/', '-', '\\']):
                if animating:
                    break
                sys.stdout.write('\rloading ' + c)
                sys.stdout.flush()
                time.sleep(0.1)
            sys.stdout.write('\rDone!     ')

        t = threading.Thread(target=animate)
        t.start()
        ##long process here
        map2 = copy.deepcopy(map)
        for j,row in enumerate(map):
            for i,col in enumerate(row):
                if map[j][i] == "@" or map[j][i] == "+":
                    x = i
                    y = j
        start_time = time.time()


        if input2 == 1:
            path, explore = BFSsolver2(map) #bfssolver(original map)
        elif input2 == 2:
            path, explore = aStarSolver(map)
        else: 
            print("fault here")
        end_time = time.time()
        animating = True
        time.sleep(2)
        if path != False:
            printPath(map2,path,x,y)    #printpath(original map)
            time.sleep(1)
            print("It takes %s seconds to get the solution" % (end_time - start_time))
            print("Solution found:",path)
            print("Solution costs",len(path),"moves")
            print("Explored states:", explore)
        else:
            print("Solution not found")
        print("Do you wish to continue?\n0: escape\n1: continue")
        input3 = input()
        if input3 == '0':
            break
        else:
            screen_clear()

main()