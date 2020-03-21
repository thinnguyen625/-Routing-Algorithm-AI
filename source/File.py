import numpy as np
from bresenham import bresenham
from shapely.geometry import Point, Polygon

def readFromFile(file_name):
    fin = open(file_name, "r")
    # Dòng 1: Kích thước [w,h] của bản đồ
    size = fin.readline().strip("\n").split(",")
    width = int(size[0])
    height = int(size[1])

    # Dòng 2: 2 cặp toạ độ (sX,sY),(gX,gY)
    point = fin.readline().strip("\n").split(",")
    start = (height - int(point[1]) - 1, int(point[0]))
    goal = (height - int(point[3]) - 1, int(point[2]))
    # Dòng 3: Số lượng chướng ngại vật (đa giác) n
    nObj = int(fin.readline())
    objects = []
    # n dòng tiếp theo: Dòng thứ i: Chứa thông tin đa giác thứ i: Cứ 2 cặp số là tọa độ của một đỉnh.
    map = np.zeros((height, width))  # ma trận bản đồ

    for i in range(nObj):
        tmp = fin.readline().strip("\n").split(",")
        j = 0
        
        while(j != len(tmp)):
            if(j + 2 < len(tmp)):
                line = list(bresenham(int(tmp[j]), int(tmp[j + 1]), int(tmp[j + 2]), int(tmp[j + 3])))
            else:
                line = list(bresenham(int(tmp[j]), int(tmp[j + 1]), int(tmp[0]), int(tmp[1])))

            k = 0
            while(k != len(line)):
                map[height - line[k][1] - 1][line[k][0]] = 1
                k += 1
            j += 2
        
        vertices = []
        for m in range(0, len(tmp) - 1, 2):
            vertices.append((int(tmp[m]), int(tmp[m+1])))
        objects.append(Polygon(vertices))

    fin.close()
    return map, width, height, start, goal, objects


