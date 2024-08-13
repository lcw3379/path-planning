import pygame
import random
import math
import time


start = (10,10)
end = (830,600)
width = 900
height = 700
pygame.init()
grid_size = 1   
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("RRT")

path_surface = pygame.Surface(screen.get_size(), pygame.SRCALPHA)
path_surface = path_surface.convert_alpha()
path_surface.fill((0, 0, 0, 0))  #투명 배경

colors = {
    "start": (255, 0, 0),
    "end": (0, 255, 0),
    "path": (255, 0, 0),
    "wall": (40, 40, 40),
    "empty": (255, 255, 255),
    "trees": (0, 0, 255),
}

max_radius = 50
num_obstacles = 25
obstacle_radius = 30



class Obstacle():
    def __init__(self):
        self.radius = random.randint(30,max_radius)
        self.position = (random.randint(self.radius, width - self.radius), random.randint(self.radius, height - self.radius))

obstacles = [Obstacle() for _ in range(num_obstacles)]

class Node():
    def __init__(self, x, y, parent = None, g=None):
        self.parent = parent
        self.x = x
        self.y = y
        self.g = g


def check_collision(node, near_node):
    for obstacle in obstacles:
        if distance(node, Node(obstacle.position[0], obstacle.position[1])) <= obstacle.radius  or distance(Node((node.x+near_node.x)/2,(node.y+near_node.y)/2),Node(obstacle.position[0], obstacle.position[1]))<= obstacle.radius:
            return True
    return False



def distance(a,b):
    return math.sqrt((a.x-b.x)**2+(a.y-b.y)**2)

def find_near(node_list, rand_node):
    return min(node_list, key=lambda node:distance(node, rand_node))

def find_near_radius(node_list, new_node, search_radius):
    temp_node_list = []
    for node in node_list:
        if distance(node,new_node)<=search_radius:
            temp_node_list.append(node)
    return temp_node_list

def search_mincost(near_list, new_node):
    return min(near_list, key=lambda node:node.g )
    

def steer(rand_node, near_node, step_size):
    theta = math.atan2(rand_node.y-near_node.y, rand_node.x - near_node.x)
    return Node(int(near_node.x + step_size*math.cos(theta)), int(near_node.y + step_size * math.sin(theta)))


def update_tree(near_list, new_node, new_path):
    for node in near_list:
        if not node.parent == None:
            g = distance(new_node, node)
            if node.g > new_node.g + g:
                if not check_collision(node, new_node):
                    node.parent = new_node
                    node.g = node.parent.g + g
                    
                
def draw_path(new_path):    
    if new_path :
        for node in new_path: # path 그리기
            if node.parent is not None:
                pygame.draw.line(path_surface, colors["path"], (node.x, node.y), (node.parent.x, node.parent.y),2)
            pygame.display.update()

def search_goal(new_node, end_node, step_size):
    if distance(new_node, end_node) <= step_size:
        #if collision_check(end_node, new_node):
        return True
    return False
        
# def calculate_new(step_size, rand_node, near_node):
#     return Node(near_node.x+int(step_size*abs(rand_node.x-near_node.x)/math.sqrt((rand_node.x-near_node.x)**2+(rand_node.y-near_node.y)**2)), 
#                 near_node.y+int(step_size*abs(rand_node.y-near_node.y)/math.sqrt((rand_node.x-near_node.x)**2+(rand_node.y-near_node.y)**2)))

def visualize(node_list, start_node, end_node):
    screen.fill((255,255,255))
    for obstacle in obstacles:
        pygame.draw.circle(screen, colors["wall"], (obstacle.position[0], obstacle.position[1]), obstacle.radius)
    for node in node_list:
        if node.parent:
            pygame.draw.line(screen, colors["trees"], (node.x, node.y), (node.parent.x, node.parent.y))
    
    pygame.draw.circle(screen, colors["start"],(start_node.x, start_node.y),8)
    pygame.draw.circle(screen, colors["end"],(end_node.x, end_node.y),8)
    screen.blit(path_surface, (0, 0))
    pygame.display.update()

def RRT(start, end):   
    start_node = Node(start[0], start[1], g=0)
    end_node = Node(end[0], end[1])
    node_list = []
    node_list.append(start_node)
    step_size = 55
    search_radius = 65
    min_g = 100000
    path = []
    new_path = []
    path.append(end_node)
    
    for i in range(600000):
    
        rand_node = Node(random.randint(0,width),random.randint(0,height))
        
        near_node = find_near(node_list, rand_node)
        
        # if rand_node.x==near_node.x and rand_node.y == near_node.y:
        #     continue
        #new_node = calculate_new(step_size,rand_node, near_node)
        new_node = steer(rand_node, near_node, step_size)

        #print(rand_node.x, rand_node.y, near_node.x, near_node.y, new_node.x, new_node.y)
        if not check_collision(new_node, near_node): # 새 노드가 장애물 속에 있는지 아닌지 체크
            near_list = find_near_radius(node_list, new_node, search_radius)
            min_cost_node = search_mincost(near_list, new_node)
            if not check_collision(min_cost_node, new_node): #노드와 이어진 경로가 장애물에 걸리는지 체크
                new_node.parent = min_cost_node
                new_node.g = new_node.parent.g + distance(new_node, new_node.parent)
                update_tree(near_list, new_node, new_path)
                
                
                node_list.append(new_node)

        visualize(node_list,start_node, end_node)
        
        if search_goal(new_node, end_node, step_size/2.5):
            print(new_node.g)
            if min_g < new_node.g:
                continue
            end_node.parent = new_node
            min_g = new_node.g
            while new_node:
                path.append(new_node)
                new_node = new_node.parent
            print("path searched")
            path_surface.fill((0, 0, 0, 0))
            new_path = path
            path = []
            draw_path(new_path)
        pygame.display.update()
    # for node in path: # path 그리기
    #     if node.parent is not None:
    #         pygame.draw.line(screen, colors["path"], (node.x, node.y), (node.parent.x, node.parent.y),2)
    #         pygame.display.update()


    # 0. 초기화     
    # 1. 랜덤한 위치 설정
    # 2. 가장 가까운 노드 찾기
    # 3. 가까운 노드에서 랜덤노드 방향으로 step_size만큼 거리에 새 노드 생성
    # 3. 충돌 검사 후 node_list에 append
    # 4. 다음 랜덤 위치 설정


RRT(start, end)
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            break

pygame.quit()