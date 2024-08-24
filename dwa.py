import pygame
import math
import numpy as np
import random

robot_radius = 1.0
max_speed = 1.0
min_speed = -0.5
max_omega = 40.0 * math.pi / 180.0 # 최대 각속도
max_a = 1.0
max_delta_omega = 40.0 * math.pi / 180.0 # 최대 각가속도
max_val = -10000
dt = 0.1

future_time = 2

# 가중치
alpha = 1.0
beta = 0.1
gamma = 0.1

# 환경 파라미터
end = np.array([5.0, 8.0])
obstacles = np.array([[5.0, 5.0], [3.0, 6.0], [7.0, 8.0]])

# Pygame 초기화
pygame.init()
screen = pygame.display.set_mode((800, 800))

colors = {
    "start": (255, 0, 0),
    "end": (0, 255, 0),
    "path": (0, 0, 255),
    "wall": (40, 40, 40),
    "empty": (255, 255, 255),
    "trees": (0, 0, 255),
    "robot": (255,255,0),
    "best_trajectory": (255, 0, 0)
}

def distance(a, b):
    return math.sqrt((a[0] - b[0])**2+(a[1] - b[1])**2) # 예시엔 -로했네
 
    
def generate_velocity_range(u, dt): # u = [v,w]
    return np.array([u[0]-max_a*dt, u[0]+max_a*dt, u[1]-max_delta_omega*dt ,u[1]+max_delta_omega*dt])
    
    
def create_dynamic_window(robot_states): # 일단은 장애물 말고 동적 제약조건만 가정
    velocity_constraint = [min_speed, max_speed, -max_omega, max_omega]
    u = [robot_states[2],robot_states[4]]
    velocity_range = generate_velocity_range(u,dt)
    # print(velocity_constraint)
    # print(velocity_range)
    dw = [max(velocity_constraint[0],velocity_range[0]), # 최대/최소 속도 제약조건 넘지 않도록 설정.
          min(velocity_constraint[1],velocity_range[1]),
          max(velocity_constraint[2],velocity_range[2]),
          min(velocity_constraint[3],velocity_range[3])]
    return dw
    
    
def velocity_sampling(dw): 
    i = 0
    j = 0
    sampling_number = 10
    sample_list = []
    while i <= sampling_number:
        while j <=sampling_number:
            sample_list.append((dw[0]+(dw[1]-dw[0])*i/sampling_number,dw[2]+(dw[3]-dw[2])*j/sampling_number))
            j+=1
        i+=1
        j=0
        
    return sample_list


def motion(x, u, dt):
    # x = [x, y, v, angle, omega]
    x[3] += u[1] * dt
    x[0] += u[0] * math.cos(x[3]) * dt
    x[1] += u[0] * math.sin(x[3]) * dt
    x[2] = u[0]
    x[4] = u[1]
    return x

def calc_trajectory(robot_states, v, omega):
    trajectory = np.array(robot_states)
    delta_t = dt 
    time = delta_t
    while time <= 2.0:
        robot_states = motion(np.copy(robot_states), [v, omega], delta_t)
        trajectory = np.vstack((trajectory, robot_states))
        time += dt
    return trajectory 
      
def objective_function(trajectory, end, obstacles): # 평가 함수. 목적함수가 최소가 되는 v,w쌍 찾기
    heading = distance(trajectory[-1],end)
    clearance = 0
    dist_list = []
    for ob in obstacles:
        for point in trajectory:
            pointxy = [point[0],point[1]]
            dist_list.append(distance(pointxy,ob))
            if min(dist_list) <= robot_radius:
                return float('inf')  # 충돌 시 큰 비용
            clearance += 1.0 / min(dist_list)
        dist_list = []
    velocity = trajectory[-1,2]
    ob = alpha * heading + beta * clearance + gamma * velocity
    
    return ob   
      
robot_states = np.array([1.0, 1.0, 0.0 ,0.0, 0.0]) # [x, y, v, angle, omega]
past_trajectory = robot_states
best_trajectory = []
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            break
    min_val = float("inf")
    dw = create_dynamic_window(robot_states)
    sample_list = velocity_sampling(dw)
    best_vw = []

    for sample in sample_list:
        trajectory = calc_trajectory(robot_states,sample[0],sample[1])
        ob = objective_function(trajectory, end, obstacles)
        if min_val > ob:
            min_val = ob
            best_trajectory = trajectory
            best_vw = [sample[0],sample[1]]

    #print(robot_states,", ",best_vw)  
    robot_states = motion(robot_states, best_vw, dt)
    past_trajectory = np.vstack((past_trajectory, robot_states))

    screen.fill((255, 255, 255))
    
    for ob in obstacles:
        pygame.draw.circle(screen, colors["wall"], (int(ob[0]*80 ), int(ob[1]*80 )), int(80))
    
    pygame.draw.circle(screen, colors["end"], (int(end[0] * 80), int(end[1] * 80)), 5)
    pygame.draw.circle(screen, colors["robot"], (int(robot_states[0] * 80), int(robot_states[1] * 80)), int(robot_radius*8))
    for point in past_trajectory:
        pygame.draw.circle(screen, colors["path"], (int(point[0] * 80), int(point[1] * 80)), 2)
    
    pygame.display.flip()
    
    if distance((robot_states[0],robot_states[1]),end) < 0.1:
        print("goal reached.")
        end = np.array([random.randint(0,10),random.randint(0,10)])

pygame.quit()
