## 1. A*

목표까지의 최단 경로를 구하는 다익스트라(Dijkstra)알고리즘을 개선한 방식으로, 네트워크나 게임 혹은 로봇에 사용되는 Path Planning 알고리즘.

현재 노드에서 다음 노드까지의 비용을 g(x), 휴리스틱 함수를 h(x)로 할 때,<br/>
$f(x)=g(x)+h(x)$<br/>
가 최소가 되는 f(x)를 우선적으로 탐색하는 알고리즘이다.<br/>
$f(x) = g(x)$<br/>
일 때 다익스트라 알고리즘이 된다고 하니 다익스트라 알고리즘에 휴리스틱 함수를 추가해서 개선한 알고리즘이라고 볼 수 있겠다.

보통 2차원 격자 공간에서 구현할 때에는 휴리스틱 함수 h(x)를 해당 노드에서 목적지까지의 거리를 나타내는 함수로 작성한다.

이미 탐색이 끝난 노드는 closed 되었다고 하고, 탐색이 끝난 노드와 인접한 노드는 open된 노드라고 한다. 
이 모든 open된 노드들이 이진트리인 Heap에 들어가서, 가장 F값이 작은 노드를 우선으로 탐색한다.
파이썬에서는 heapq라는 라이브러리와 라이브러리 내 함수 heappush()와 heappop()을 사용해서 간단히 Heap을 사용할 수 있다. heappush는 힙 트리에 노드를 추가하고, heappop은 그곳에서 가장 작은 f값을 가진 노드부터 빠져나오도록 만들 수 있다.

pygame을 사용하여 시각화 하였다. 첫 번째 영상처럼 수평/수직 거리는 가중치 10, 대각선 거리는 대각선 거리를 계산한 후에 10을 곱하고 정수형으로 변환하였다. 즉 대각선으로 1칸 이동할 때 14의 비용이 든다. 


 
유튜브의 영상이 알고리즘 이해에 큰 도움이 되었다. 해당 영상들은 격자 공간에서 A* 알고리즘의 작동 방식을 보여주는 가장 좋은 예시라고 생각한다.

https://www.youtube.com/watch?v=-L-WgKMFuhE  <br/>
https://www.youtube.com/watch?v=QR3U1dgc5RE  





![smallmap](https://github.com/user-attachments/assets/2ead7314-4c27-4754-8250-e97fdfb9ae3b)

작은 공간에서 첫 번째 영상의 예시대로 구현해 보았다. 루프에 딜레이를 걸어 작동원리가 잘 보이게 하였다.


![bigmap](https://github.com/user-attachments/assets/63ef67fc-32de-4a6b-bd84-7fab0e1a32fb)

큰 공간에서는 다음과 같다.





유튜브의 두 번째 영상에서 볼 수 있듯이, 그리드의 크기가 커질수록 탐색해야 하는 범위가 늘어나다 보니 탐색에 큰 시간이 걸린다.

따라서 로봇공학에서는 RRT* 알고리즘을 개선한 방식도 Path Planning에 잘 사용하는 것으로 보인다.<br/>
그 전에 먼저 RRT 알고리즘에 대해서 알아보았다.

## 2. Rapidly-exploring Random Tree(RRT)

더 해상도가 큰 공간에서 path를 생성하기에 적합한 알고리즘<br/>
알고리즘 진행 순서는 다음과 같다.<br/>
1. 초기화     <br/>
2. 랜덤한 위치의 노드 생성<br/>
3. 랜덤한 노드와 가장 가까운 노드 찾기<br/>
4. 가까운 노드에서 랜덤노드 방향으로 step_size만큼 거리에 새 노드 생성<br/>
5. 충돌 검사 후 node_list에 append<br/>
6. 2번부터 반복<br/>

처음엔 4번 과정에서 랜덤 노드와의 거리를 직접 구해서 생성할 노드와 비교하는 방법을 사용했는데, step_size보다 거리가 작은 노드가 생성되는 경우가 많아 steer() 함수로 변경하여 랜덤 노드가 어떤 곳에 위치하던 step_size만큼의 거리에 새 노드가 생성되게 하였다. 

![rrt](https://github.com/user-attachments/assets/63f913eb-822b-4181-9e63-6e7fb5d1327a)

구현 모습이다. end_node를 찾으면 end_node부터 부모 노드를 차례대로 올라가며 start_node에 닿는다.




## 3. RRT*
RRT의 방식은 랜덤 샘플링 방식이기 때문에 최적의 경로를 구할 수 없다. 이 문제를 개선하기 위해 RRT*라는 알고리즘이 고안되었다.
기본적인 방식은 RRT와 같지만, 노드의 비용과 트리를 재구성하는 과정이 새로 추가되었다.

1. RRT에서는 랜덤 샘플링 노드와 가장 가까운 위치의 노드를 부모 노드로 바로 정했다면, RRT*는 일정 반경 안의 이웃 노드들과의 비용을 계산한 후, 가장 적은 비용의 노드를 부모 노드로 새로 정한다.
2. 이렇게 부모 노드를 새로 갱신했다면, 인접 노드들 또한 비용을 계산해서 새로 생성한 노드와의 비용이 더 작다면 새로 생성한 노드를 부모 노드로 갱신한다.

즉, 비용을 계산할 필요가 없던 RRT에 비용을 추가해서 최적 경로를 찾는 알고리즘이다. A*처럼 Node클래스에 g값을 추가한 다음, 부모 노드까지의 거리를 비용으로 저장하였다.

![rrt_star](https://github.com/user-attachments/assets/80a4beb0-7421-4a7f-8321-947e4664da55)

구현 모습이다. 새 path를 찾았을 때의 비용이 기존 path의 비용보다 낮을 시 path를 갱신하도록 하였다.

![rrt_star_big_step](https://github.com/user-attachments/assets/7d79aaff-378d-4c11-9118-76f729726776)


step_size와 search_radius를 더 크게 조절하면 부드러운 path 생성은 불가능하지만, 굉장히 빠른 초기 path 생성과 빠른 최적화를 확인할 수 있었다.

흥미로운 점은, 처음엔 마구잡이로 뻗치고 휘던 노드 트리가 시간이 지나면 알고리즘에 의해서 마치 자성체에 이끌리는 철가루 처럼 시작지점에서 시작해 여러갈래로 뻗어나가는 형태로 변화한다는 점이다. 

확실히 한 번 RRT* 트리를 만들어 놓으면 목적 지점을 어느 곳으로 움직이던지 시작지점에서부터 비교적 좋은 경로로 이동할 수 있겠다는 생각이 들었다.

이외에도 목적 지점에서도 랜덤 트리를 생성하여 시작지점에서 뻗어나온 트리와 만나면 path를 생성하는 알고리즘 등 여러 RRT*의 변형 알고리즘이 나온 것으로 보인다.

하지만 몇 가지 의문점이 생겼다.

  1. RRT* 로도 최적화에 시간이 조금 걸리는 것으로 보이는데, 차라리 고해상도 공간을 일정 크기의 그리드로 나누어서 A* 알고리즘을 사용하는게 더 좋지 않을까?<br/>
  2. RRT*로는 시작지점으로부터의 최적화만 진행됐는데, 시작지점의 위치를 변경하면 path planning 알고리즘 전체를 다시 돌려야 하는 건가?


## 4. Dynamic Window Approach (DWA)

Path planning은 Global planning과 Local planning의 두 종류로 나뉜다.<br/>
앞에서 구현한 알고리즘들은 global planning에 해당한다. 로봇의 시작 위치에서 목적지까지의 전체적인 경로를 설정하는 역할이다.

그와 다르게 local planning은 로봇이 이동하면서 기존에 없던 장애물이 새로 생기는 등의 동적 상황을 해결하기 위한 플랜이다.

그 중 DWA는 로봇의 현재 속도, 각속도 등의 데이터를 기반으로 동적 윈도우(Dynamic Window)를 생성해서, t초 앞에서 일어날 상황을 미리 판단하고 충돌을 회피하기 위한 알고리즘이다.

작동 방식은 다음과 같다.

    1. 현재 속도, 각속도로 가능한 속도 범위 생성
    2. dynamic window 생성. 로봇의 가속도, 각속도 등과 장애물 충돌 등을 고려
    3. 속도 샘플링
    4. 샘플링한 속도들에 대해서 가상 궤적 생성
    4. 각 궤적들을 목적함수로 평가
    5. 최적의 속도와 각속도 선정  

유튜브에서 작동 영상을 미리 확인하는게 알고리즘 이해와 시각화에 큰 도움이 되었다.<br/>
https://www.youtube.com/watch?v=Y14CAtCNBDE

1번의 속도 범위를 생성할 때에는 해당 공식을 이용한다.<br/>
![image](https://github.com/user-attachments/assets/b14c49cc-9c87-4b78-8289-4fd87a56faa9)

[Vmin, Vmax] = [V-amax*dt, V+amax*dt]<br/>
[Wmin, Wmax] = [W-αmax*dt, W+αmax*dt]<br/>

이 때, 로봇의 + 방향 최대가속도와 -방향 최대가속도는 같다고 가정하였다. 각가속도 또한 마찬가지다.


목적함수를 평가할 때는 다음 식을 사용한다.


![daw2](https://github.com/user-attachments/assets/139ec641-5db8-4e3b-803c-926fda18f67a)


구현 모습이다. 시각화를 위해서 위의 유튜브 영상과 같이, 샘플링한 궤적은 노란색, 장애물과 충돌하는 궤적은 파란색, 최적의 궤적은 빨간색으로 설정하였다.

장애물과 완전히 근접해 있을 시에 조금씩 멈추는 현상이 있다. 

개선할 점

로봇

## 5. Gloabl Planning과 Local Planning 결합

지금까지 Global Path Planning인 A*, RRT, RRT*와 Local Path Planning인 DWA를 구현하였다.

ROS2 의 네비게이션 패키지 등 현재 로봇 네비게이션 시스템에는 Global Planning과 Local Planning을 합친 시스템을 사용하는 것으로 보인다.

따라서 마지막으로 Global Planning과 Local Planning을 결합하여 


구현하면서 새로 공부한 파이썬 함수

set()<br/>
중복을 제거하고, 순서가 없는 집합형 자료구조. <br/>
closed된 노드를 open처럼 list로 관리하는게 아니라서 중복 검색을 방지하고, 새 노드 탐색이 굉장히 빨라짐

 __eq__(),   __lt__(), __hash__()<br/>
 Node 클래스 내부에서 정의하는 함수들인데, 이 노드들을 heapq에 넣고 서로 비교하기 위해서 사용된 함수들이다.
 
 __eq__()<br/>
 클래스에서 두 객체들이 같은지 확인하는 함수
 
 __lt__()<br/>
 less than의 뜻. 어떤게 더 작은지 비교하는 함수
 
 __hash__()<br/>
 노드의 해시값을 계산해서 set()등의 자료구조에서 사용할 수 있게 만든다.


 if any(o_node for o_node in open if neighbor == o_node and neighbor.g > o_node.g)
 
A*를 구현하면서 알게 된 최적화 방법. 현재의 neighbor 셀이 이미 open 힙 자료구조에 있던 셀이고 neighbor의 비용이 기존위치의 비용보다 크게 계산되면 그냥 무시하도록 하는 부분.
if any()와 그 속의 o_node for o_node in open 부분의 쓰임새를 알기까지 시간을 썻다.

DWA에서, robot_states라는 로봇의 상태를 저장하는 np.array형 변수가 있는데 이걸 여기저기 함수에 넣어서 사용했더니 calc_trajectory 함수에서 robot_state의 결과에 영향을 주기 시작했다. 
해결법으로 np.copy(robot_states)로 robot_states를 그대로 집어넣지 않고 복사해서 해결하였다. 오류의 원인과 해결법을 알 때까지 시간을 꽤 사용했다..


