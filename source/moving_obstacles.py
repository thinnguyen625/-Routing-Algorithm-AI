import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.animation import ImageMagickWriter
from matplotlib.pyplot import figure
from shapely.geometry import Point, Polygon, LineString
from shapely.geometry.polygon import LinearRing
import numpy as np
from PriorityQueue import PriorityQueue
from Heuristic import *
import sys

filename = sys.argv[1]
d = sys.argv[2]

g_score = {}
f_score = {}
close_set = set()
open_set = PriorityQueue()
came_from = {}
fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
count = 0
check = tuple()

def readFromFile(filename):
    fin = open(filename, "r")
    # Dòng 1: Kích thước [w,h] của bản đồ
    size = fin.readline().strip("\n").split(",")
    width = int(size[0])
    height = int(size[1])
    # Dòng 2: 2 cặp toạ độ (sX,sY),(gX,gY)
    point = fin.readline().strip("\n").split(",")
    start = (int(point[0]), int(point[1]))
    goal = (int(point[2]), int(point[3]))
    # Dòng 3: Số lượng chướng ngại vật (đa giác) n
    nObj = int(fin.readline())
    verticeslist = []
    # n dòng tiếp theo: Dòng thứ i: Chứa thông tin đa giác thứ i: Cứ 2 cặp số là tọa độ của một đỉnh.
    for i in range(nObj):
        tmp = fin.readline().strip("\n").split(",")
        for j in range(len(tmp)):
            tmp[j] = float(tmp[j])
        #tmp = list(map(float, tmp))
        verticeslist.append(tmp)
    fin.close()
    return width, height, start, goal, verticeslist

def makeObjects(verticeslist):
    objects = []
    for i in verticeslist:
        vertices = []
        for j in range(0, len(i) - 1, 2):
            vertices.append((i[j], i[j+1]))
        objects.append(Polygon(vertices))
    return objects

def moveObjects(objs, dist):
    objects = []
    coors = []
    for k in range(len(objs)):
        vertices = []
        x,y = objs[k].exterior.xy
        coors.append((x,y))
        for m in range(len(coors[k][0])):
            coors[k][0][m] += dist
            vertices.append((coors[k][0][m], coors[k][1][m]))
        objects.append(Polygon(vertices))
    return objects
 
def overlapTriggered(point, objects):
    for i in objects:
        linearring = LinearRing(list(i.exterior.coords))
        if (i.contains(point) or
            i.touches(point) or
            linearring.contains(point)):
            return True
    return False

def displayObjects(objs):
    coors = []
    for k in range(len(objs)):
        x,y = objs[k].exterior.xy
        coors.append((x,y))
        plt.plot(coors[k][0], coors[k][1])

def displayRobot(newx, newy):
    plt.scatter(newx, newy, marker = "*", color = "pink", lw = 5)
    plt.annotate("Robot", (newx, newy - 1))

firstpathx = []
firstpathy = []

realpathx = []
realpathy = []

def displayPath(xs, ys):
    for i in range(len(firstpathx) - 1):
        plt.plot([firstpathx[i], firstpathx[i + 1]], [firstpathy[i], firstpathy[i + 1]], color = "black", linewidth = 1, linestyle = "--")
    plt.scatter(s[0], s[1], marker = "o", color = "green", lw = 5)
    plt.scatter(g[0], g[1], marker = "o", color = "red", lw = 5)
    for i in range(len(xs) - 1):
        plt.plot([xs[i], xs[i + 1]], [ys[i], ys[i + 1]], color = "orange", linewidth = 2, linestyle = "-")

def driving(w, h, g, xs, ys, objs):
    global check
    code = 0
    if xs == [] and ys == []:
        notfound(objs)
        plt.pause(2)
        return
    if check == g:
        plt.pause(1)
        return
    else:
        for i in range(len(xs)):
            plt.clf()
            plt.xlim(0, w)
            plt.ylim(0, h)
            dist = 0
            if (i % 2 == 0):
                dist = float(d)
            else:
                dist = -float(d)
            tobjs = moveObjects(objs, dist)
            txs = xs
            tys = ys
            if (i == len(xs) - 4):
                for j in range(i, len(xs)):
                    plt.clf()
                    plt.xlim(0, w)
                    plt.ylim(0, h)

                    displayObjects(tobjs)
                    displayRobot(txs[j], tys[j])
                    displayPath(txs, tys)
                    plt.pause(0.1)
                for i in range(len(realpathx) - 1):
                    plt.plot([realpathx[i], realpathx[i + 1]], [realpathy[i], realpathy[i + 1]], color = "orange", linewidth = 2, linestyle = "-")
                plt.scatter(g[0], g[1], marker = "*", color = "blue", lw = 5)
                plt.annotate("Initial Cost: " + str(len(firstpathx)) +"\nReal Cost: " + str(len(realpathx) + len(txs)), (float(w/2), float(h/2)), color = "red", fontsize = 20, horizontalalignment = "center")
                code = 1
                break
            displayObjects(tobjs)
            displayRobot(txs[i], tys[i])
            displayPath(txs, tys)
            check = (txs[i + 2], tys[i + 2])
            if (overlapTriggered(Point(txs[i + 4], tys[i + 4]), tobjs) or
                overlapTriggered(Point(txs[i + 3], tys[i + 3]), tobjs) or
                overlapTriggered(Point(txs[i + 2], tys[i + 2]), tobjs) or
                overlapTriggered(Point(txs[i + 1], tys[i + 1]), tobjs)):
                curr = (txs[i], tys[i])

                for j in range(0, i + 1):
                    realpathx.append(txs[j])
                    realpathy.append(tys[j])
                break
            plt.pause(0.1)
        if code == 1:
            return
        newxs, newys = recalculate(w, h, curr, g, tobjs)
        driving(w, h, g, newxs, newys, objs)

def trackingPath(curr, g, came_from):
    data = []
    tmp = tuple(g)
    while tmp in came_from:
        data.append(tmp)
        tmp = came_from[tmp]
    data.append(curr)
    data.reverse()
    #minStep = len(data)
    return data

def isValid(w, h, objs, neighbor):
    check = True
    for i in range(len(objs)):
        linearring = LinearRing(list(objs[i].exterior.coords))
        if (objs[i].contains(Point(neighbor[0], neighbor[1])) or
            objs[i].touches(Point(neighbor[0], neighbor[1])) or
            linearring.contains(Point(neighbor[0], neighbor[1])) or
            linearring.intersects(Point(neighbor[0], neighbor[1]))):
            check = False
            break
    return ((check and 0 < neighbor[0] < w) and (0 < neighbor[1] < h))

#def isValid(w, h, objs, neighbor):
#    return ((0 <= neighbor[0] < w) and (0 <= neighbor[1] < h))

def astar(w, h, curr, g, objs):
    open_set.put(curr, 0)
    g_score[curr] = 0
    f_score[curr] = g_score[curr] + heuristic(curr, g)
    neighbors = [(-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1), (1, 0), (1, -1), (0, -1)]

    while not open_set.empty():
        _, tmp = open_set.get()
        #print(tmp)
        if tmp == g:
            return trackingPath(curr, g, came_from)
        close_set.add(tmp)
        for direction in neighbors:
            neighbor = (tmp[0] + direction[0], tmp[1] + direction[1])
            if not isValid(w, h, objs, neighbor):
                continue
            if neighbor in close_set and g_score[tmp] + 1 >= g_score[neighbor]:
                continue
            if neighbor not in g_score or g_score[tmp] + 1 < g_score[neighbor]:
                came_from[neighbor] = tmp
                g_score[neighbor] = g_score[tmp] + 1
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, g)
                open_set.put(neighbor, f_score[neighbor])
    return []

def recalculate(w, h, curr, g, objs):
    g_score.clear()
    f_score.clear()
    close_set.clear()
    open_set.clear()
    came_from.clear()

    xs = []
    ys = []
    minPath = astar(w, h, curr, g, objs)   
    if minPath == []:
        return [], []
    for k in range(len(minPath)):
        xs.append(minPath[k][0])
        ys.append(minPath[k][1])
    return xs, ys

def notfound(objs):
    #plt.clf()
    plt.annotate("PATH NOT FOUND\nOR ROBOT STUCKED IN ONE OF THE OBJECTS!", (float(w/2), float(h/2)), color = "red", fontsize = 10, horizontalalignment = "center")
    plt.xlim(0, w)
    plt.ylim(0, h - 1)
    plt.scatter(s[0], s[1], marker = "o", color = "green", lw = 5)
    plt.scatter(g[0], g[1], marker = "o", color = "red", lw = 5)
    displayObjects(objs)
    plt.show()
    return

w, h, s, g, vl = readFromFile(filename)

def main():
    objs = makeObjects(vl)
    curr = s
    xs = []
    ys = []
    xs, ys = recalculate(w, h, curr, g, objs)
    if xs == [] and ys == []:
        notfound(objs)
    else:
        global firstpathx
        global firstpathy
        firstpathx = xs
        firstpathy = ys
        anim = FuncAnimation(fig, driving(w, h, g, xs, ys, objs), frames = 60, interval=20, repeat = False, blit = True)
        #anim.save("AStar_Moving_Obstacles.html", writer = "ImageMagickWriter", dpi = 96)
        plt.show()
        plt.close()
        return

if __name__ == "__main__":
    main()