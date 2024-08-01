import math
import pygame
import heapq
import time


start_pos = (0,7)
end_pos = (13,7)
maze = [
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],

]

    

class Node:
    def __init__(self, position, parent = None):
        self.position = position
        self.parent = parent
        self.f = 0
        self.g = 0
        self.h = 0
    def __eq__(self, other):
        return self.position == other.position
    def __lt__(self, other):
        return self.f < other.f  
    def __hash__(self):
        return hash(self.position)      
    
    
def h_func(node, end): # 목적지와 노드간의 거리를 계산하는 휴리스틱 함수
    return int(math.sqrt(abs(node.position[0] - end.position[0])**2 + abs(node.position[1] - end.position[1])**2)*10)

def Astar(maze, start_pos, end_pos, func):
    start_node = Node(start_pos)
    end_node = Node(end_pos)
    open = []
    closed_set = set()

    
    heapq.heappush(open, start_node)
    
    while True:
        present_node = heapq.heappop(open)
        if present_node in closed_set:
            continue
        
        if present_node == end_node:
            path = []
            while present_node:
                path.append(present_node)
                present_node = present_node.parent
            return path
        closed_set.add(present_node)
        func(present_node, open, closed_set)
        
        (x, y) = present_node.position
        neighbors = [(x-1, y-1), (x+1, y+1) ,(x-1, y+1), (x+1, y-1),(x, y-1), (x, y+1), (x+1, y),(x-1, y)]    
        
        for i in neighbors:
            if (0 <= i[0] < len(maze) and 0<= i[1] < len(maze[0]) and maze[i[0]][i[1]] == 0): # 미로안에 있고 움직일 수 있는 공간이면
                neighbor = Node(i, present_node)
                if neighbor in closed_set:
                    continue
                
                if abs(i[0]-x)==1 and abs(i[1]-y) == 1: #neighbor가 대각선이면 비용 14
                    neighbor.g = present_node.g + 14
                else: #neighbor가 수직/수평이면 비용 10
                    neighbor.g = present_node.g + 10
                
                neighbor.h = h_func(neighbor, end_node)
                neighbor.f = neighbor.g + neighbor.h  
                if any(o_node for o_node in open if neighbor == o_node and neighbor.g > o_node.g):
                    continue       
                
                heapq.heappush(open, neighbor)       
                
                
    


pygame.init()
grid_size = 40
total_width = len(maze[0])
total_height = len(maze)
screen = pygame.display.set_mode((total_width * grid_size, total_height * grid_size))
pygame.display.set_caption("A*")
font = pygame.font.Font(None, 24)
font_gh = pygame.font.Font(None, 14)    
colors = {
    "start": (255, 0, 0),
    "end": (0, 255, 0),
    "path": (0, 122, 255),
    "wall": (0, 0, 0),
    "empty": (255, 255, 255),
    "open": (0, 255, 255),
    "present": (255, 0, 255),
    "closed": (255, 255, 0)
}

def screen_texting(screen, node):
        text = font_gh.render(str(node.g), True, (0, 0, 0))
        screen.blit(text, (node.position[1] * grid_size  , node.position[0] * grid_size + 5))            
        text = font_gh.render(str(node.h), True, (0, 0, 0))
        screen.blit(text, (node.position[1] * grid_size + 20, node.position[0] * grid_size + 5))            
        text = font.render(str(node.f), True, (0, 0, 0))
        screen.blit(text, (node.position[1] * grid_size + 5, node.position[0] * grid_size + 20))    

def visualize(present_node, open, closed_set):
    for y in range(total_height):
        for x in range(total_width):
            if maze[y][x] == 1:
                color = colors["wall"]
            else:
                color = colors["empty"]
            pygame.draw.rect(screen, color, pygame.Rect(x * grid_size, y * grid_size, grid_size, grid_size))
    for node in closed_set:
        pygame.draw.rect(screen, colors["closed"], pygame.Rect(node.position[1] * grid_size, node.position[0] * grid_size, grid_size, grid_size))
        screen_texting(screen, node)
    
    for node in open:
        pygame.draw.rect(screen, colors["open"], pygame.Rect(node.position[1] * grid_size, node.position[0] * grid_size, grid_size, grid_size))
        screen_texting(screen, node)
    
    pygame.draw.rect(screen, colors["present"], pygame.Rect(present_node.position[1] * grid_size, present_node.position[0] * grid_size, grid_size, grid_size))
    pygame.draw.rect(screen, colors["start"], pygame.Rect(start_pos[1] * grid_size, start_pos[0] * grid_size, grid_size, grid_size))
    pygame.draw.rect(screen, colors["end"], pygame.Rect(end_pos[1] * grid_size, end_pos[0] * grid_size, grid_size, grid_size))
    pygame.display.flip()
    time.sleep(0.1)       



path = Astar(maze, start_pos, end_pos, visualize)

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            break
        
    #맵 생성
    for y in range(total_height):
        for x in range(total_width):
            if maze[y][x] == 1:
                color = colors["wall"]
            else:
                color = colors["empty"]
            pygame.draw.rect(screen, color, pygame.Rect(x * grid_size, y * grid_size, grid_size, grid_size))
            
    for node in path:
        pygame.draw.rect(screen, colors["path"], pygame.Rect(node.position[1] * grid_size, node.position[0] * grid_size, grid_size, grid_size))
        screen_texting(screen, node) 
    pygame.draw.rect(screen, colors["start"], pygame.Rect(start_pos[1] * grid_size, start_pos[0] * grid_size, grid_size, grid_size))
    pygame.draw.rect(screen, colors["end"], pygame.Rect(end_pos[1] * grid_size, end_pos[0] * grid_size, grid_size, grid_size))
    
    pygame.display.flip()
    
            

                
                
        
    
    
# maze = [
#     [0,0,1,0,0,0,0,0],
#     [0,0,1,0,0,0,0,0],
#     [0,0,1,0,0,0,0,0],
#     [0,0,1,0,0,0,0,0],
#     [0,0,1,0,1,1,0,0],
#     [0,0,1,0,1,0,0,0],
#     [0,0,1,0,1,0,0,0],
#     [0,0,0,0,1,0,0,0],
# ]    
