import numpy as np
import pygame
import math


max_accel = 4.0
max_delta = np.pi/4

obstacles = [(440,400),(460,500),(542,420)]
obstacles_radius = 3

class KBM: # kinematic bicycle model
    def __init__(self, L,Lr, dt, max_v):
        self.L = L
        self.Lr = Lr
        self.dt = dt
        self.max_v = max_v
    
    def step(self, x,y,v,theta,delta,a):
        beta = np.arctan((self.Lr / self.L) * np.tan(delta))
        x += v * np.cos(theta + beta) * self.dt
        y += v * np.sin(theta + beta) * self.dt
        theta += (v / (self.L - self.Lr)) * np.sin(beta) * self.dt
        v += a * self.dt
        if v > self.max_v:
            v = self.max_v
        return x, y, v, theta



def create_race_track(): # path 생성

    start = (400, 400)
    straight_length = 25
    radius = 50
    num_points_curve = 50
    #직선
    straight1 = [(start[0] + i*4, start[1]) for i in range(straight_length)]
    straight2 = [(start[0] + i*4, start[1] + 2 * radius) for i in range(straight_length)]
    straight2.reverse()

    #커브
    theta1 = np.linspace(-np.pi/2, np.pi/2, num_points_curve)
    curve1 = [(start[0] + straight_length*4 + radius * np.cos(t), start[1] + radius * np.sin(t)+50) for t in theta1]
    theta2 = np.linspace(np.pi/2, np.pi/2*3, num_points_curve)
    curve2 = [(start[0]  + radius * np.cos(t), start[1] + 2 * radius + radius * np.sin(t)-50) for t in theta2]
    
    track = straight1 + curve1 + straight2 + curve2
    return track

# 비용함수
def cost_function(state, control, targets, count):
    pos = (state[0],state[1])
    A = 0.1
    B = 0.2
    C = 0.2
    
    heading = distance(pos,targets[count]) + distance(pos,targets[count+1]) + distance(pos,targets[count+2]) + distance(pos,targets[count+3])
    
    velocity = np.linalg.norm(control)
    
    clearance = 0
    for ob in obstacles:
        ob_distance = distance(pos,ob)
        if ob_distance < obstacles_radius:  #충돌하면 비용 크게
            clearance +=  1000
            
    return A * heading + B * velocity + C * clearance

def distance(a, b):
    return math.sqrt((a[0] - b[0])**2+(a[1] - b[1])**2) 


def mppi(state, targets, count, robot, N, num_samples, lambda_=1.0):
    #균등분포 난수
    #controls = np.random.uniform(low=[-max_accel, -max_delta], high=[max_accel, max_delta], size=(num_samples, N, 2)) 
    #정규분포 난수
    controls = np.random.normal(loc=0, scale=[max_accel, max_delta], size=(num_samples, N, 2)) 
    controls = np.clip(controls, [-max_accel, -max_delta], [max_accel, max_delta])
    
    costs = np.zeros(num_samples)
    trajectories = []

    for i in range(num_samples):
        trajectory = []
        x, y, v, theta = state
        for t in range(N):
            a, delta = controls[i, t]
            #x, y, theta, v = kinematic_bicycle_model(x, y, theta, v, delta, a, dt, L, Lr)
            x, y, v, theta = robot.step(x,y,v,theta,delta,a) 
            costs[i] += cost_function((x, y), (a, delta), targets, count)
            trajectory.append((x, y))
        trajectories.append(trajectory)
    weights = np.exp(-costs / lambda_)
    weights /= np.sum(weights)
    
    best_control = np.sum(weights[:, None, None] * controls, axis=0)
    return best_control, trajectories


if __name__ == "__main__":
    pygame.init()
    screen = pygame.display.set_mode((900, 800))
    clock = pygame.time.Clock()
    font = pygame.font.Font(None, 36)
    running = True
    
    x, y, v, theta = 399.0, 399.0, 0.0, -0.5
    L = 3.0 
    Lr = 1.5  # 후륜 축에서 차량 중심까지의 거리
    dt = 0.1
    N = 10
    max_v = 14.0
    robot = KBM(L,Lr,dt,max_v)
    robot_trajectory = []
    
    num_samples = 80
    targets = create_race_track()

    count = 1
    
    while True:
            
        screen.fill((255, 255, 255))
        xypos = [x,y]

        if targets[count] == targets[len(targets)-3]:
            count = 1

        state = (x, y, v, theta)
        best_control, trajectories = mppi(state, targets,count, robot, N, num_samples)
        a, delta = best_control[0]

        # 차량 상태 업데이트
        x, y, v, theta = robot.step(x,y,v,theta,delta,a)

        xy_to_goal = distance(xypos,targets[count])
        if xy_to_goal <= 1 or xy_to_goal >= distance(xypos,targets[count+1]):
            count += 1
            print("new target : ", targets[count])

    #시각화 코드
        # 화면 중심을 로봇 위치로 이동
        screen_center_x = screen.get_width() // 2
        screen_center_y = screen.get_height() // 2
        offset_x = screen_center_x - x * 20
        offset_y = screen_center_y - y * 20
        
        # 경로 및 차량 시각화
        for target in targets:
            pygame.draw.circle(screen, (0, 255, 0), (target[0] * 20+ offset_x, target[1] * 20+offset_y), 5)

        car_x = x * 20 + offset_x
        car_y = y * 20 + offset_y

        # 랜덤 샘플링한 궤적 시각화 (노란색)
        for trajectory in trajectories:
            for i in range(len(trajectory) - 1):
                start = (trajectory[i][0] * 20 + offset_x, trajectory[i][1] * 20 + offset_y)
                end = (trajectory[i + 1][0] * 20 + offset_x, trajectory[i + 1][1] * 20 + offset_y)
                pygame.draw.line(screen, (255, 255, 0), start, end)

        # 최적 궤적 시각화 (빨간색)
        best_trajectory = [(x, y)]
        temp_state = (x, y, v, theta)
        for t in range(N):
            a, delta = best_control[t]
            temp_state = robot.step(*temp_state,delta,a)
            best_trajectory.append((temp_state[0], temp_state[1]))

        for i in range(len(best_trajectory) - 1): # 최적의 궤적
            pygame.draw.circle(screen, (255, 0, 0), (best_trajectory[i][0] * 20 + offset_x, best_trajectory[i][1] * 20 + offset_y),2)
            
        for ob in obstacles: # 장애물
            pygame.draw.circle(screen,(0,0,0),(ob[0]*20+offset_x,ob[1]*20+offset_y),obstacles_radius*20)
        
        pygame.draw.circle(screen, (0, 0, 255), (car_x, car_y), 10) # 로봇
        screen.blit(font.render(f'{round(v,2)}', True, (0,0,0)), (car_x-20, car_y - 40)) # 속도 시각화
        pygame.draw.line(screen, (255, 0, 0), (car_x, car_y), (car_x + 15 * np.cos(theta), car_y + 15 * np.sin(theta)), 2) # 로봇 방향

        pygame.display.flip()
        clock.tick(30)

    pygame.quit()
