from BFS import BFS
from DFS import DFS
from UCS import UCS
from Greedy import Greedy
from AStar import AStar
from AStar_NPoint import AStarNPoint
from GUI import *
from GUI_NPoint import *

if __name__ == "__main__":
    if sys.argv[1] == "BFS":
            input_name = sys.argv[2]

            findPath = BFS(input_name)
            map, map_width, map_height, start, goal, objects = findPath.getMapInformation()
            screen_width = ITEM_WIDTH * map_width + MARGIN * map_width
            screen_height = ITEM_HEIGHT * map_height + MARGIN * map_height

            gui = GUI(map, map_height, map_width, start, goal, screen_width, screen_height, "BFS")
            gui.drawMap()
            gui.ready()

            findPath.runBFS(gui, input_name)

            gui.wait()

    elif sys.argv[1] == "DFS":
            input_name = sys.argv[2]

            findPath = DFS(input_name)
            map, map_width, map_height, start, goal, objects = findPath.getMapInformation()
            screen_width = ITEM_WIDTH * map_width + MARGIN * map_width
            screen_height = ITEM_HEIGHT * map_height + MARGIN * map_height

            gui = GUI(map, map_height, map_width, start, goal, screen_width, screen_height, "DFS")
            gui.drawMap()
            gui.ready()

            findPath.runDFS(gui, input_name)

            gui.wait()

    elif sys.argv[1] == "UCS":
            input_name = sys.argv[2]

            findPath = UCS(input_name)
            map, map_width, map_height, start, goal, objects = findPath.getMapInformation()

            screen_width = ITEM_WIDTH * map_width + MARGIN * map_width
            screen_height = ITEM_HEIGHT * map_height + MARGIN * map_height

            gui = GUI(map, map_height, map_width, start, goal, screen_width, screen_height, "UCS")
            gui.drawMap()
            gui.ready()

            findPath.runUCS(gui, input_name)

            gui.wait()

    elif sys.argv[1] == "Greedy":
            input_name = sys.argv[2]

            findPath = Greedy(input_name)
            map, map_width, map_height, start, goal, objects = findPath.getMapInformation()

            screen_width = ITEM_WIDTH * map_width + MARGIN * map_width
            screen_height = ITEM_HEIGHT * map_height + MARGIN * map_height

            gui = GUI(map, map_height, map_width, start, goal, screen_width, screen_height, "Greedy Best First Search")
            gui.drawMap()
            gui.ready()

            findPath.runGreedy(gui, input_name)

            gui.wait()

    elif sys.argv[1] == "AStar":
            input_name = sys.argv[2]

            findPath = AStar(input_name)
            map, map_width, map_height, start, goal, objects = findPath.getMapInformation()

            screen_width = ITEM_WIDTH * map_width + MARGIN * map_width
            screen_height = ITEM_HEIGHT * map_height + MARGIN * map_height

            gui = GUI(map, map_height, map_width, start, goal, screen_width, screen_height, "A Star")
            gui.drawMap()
            gui.ready()

            findPath.runAStar(gui, input_name)

            gui.wait()

    elif sys.argv[1] == "AStarNPoint":
            input_name = sys.argv[2]

            findPath = AStarNPoint(input_name)
            map, map_width, map_height, start, pick_up, goal, objects = findPath.getMapInformation()
            
            screen_width = ITEM_WIDTH * map_width + MARGIN * map_width
            screen_height = ITEM_HEIGHT * map_height + MARGIN * map_height

            gui_n = GUI_NPoint(map, map_height, map_width, start, pick_up, goal, screen_width, screen_height, "A Star With N Pickup Point")
            gui_n.drawMap()
            gui_n.ready()

            findPath.runAStarNPoint(gui_n, input_name)

            gui_n.wait()
    else:
        Notification().error("Error", "Parameter is incorrect!")