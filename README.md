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

작은 공간에서 첫 번째 영상의 예시대로 구현해 보았다. 


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

출처 : https://jkimst.org/journal/view.php?doi=10.9766/KIMST.2021.24.1.061

Motion planning(path planning)은 Global planning과 Local planning의 두 종류로 나뉜다.<br/>
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

Dynamic Window에서 위에서 구한 식과, 기존 로봇의 최대속도/최대각속도를 비교해서 최대속도/최대각속도를 넘지 않도록 한다. 공식에선 이렇게 나타냈다.

$Vr = Vs \cap Va \cap Vd$


목적함수로 각 궤적을 평가할 때는 다음 식을 사용한다.

$G(v,w) = α \times heading(v,w) + β \times clearance(v,w) + γ \times velocity(v,w)$

heading은 궤적이 목표지점까지 얼마나 가까운지를 나타낸다.

clearance는 궤적이 장애물과 가장 가까운 거리로 우회한다. 단, 장애물과 부딫히는 궤적은 비용을 매우 크게 줘서 충돌을 방지한다.

velocity는 취할 수 있는 가장 높은 속도를 선택한다.



이때 α, β, γ는 가중치 이다.

![daw2](https://github.com/user-attachments/assets/139ec641-5db8-4e3b-803c-926fda18f67a)


구현 모습이다. 시각화를 위해서 위의 유튜브 영상과 같이, 샘플링한 궤적은 노란색, 장애물과 충돌하는 궤적은 파란색, 최적의 궤적은 빨간색으로 설정하였다.

또한 장애물의 위치를 이미 다 인식하고 있다고 가정하기 때문에 먼 거리에서부터 장애물을 회피하려는 움직임이 보였다. 실제 환경에서 적용할 때엔 센서를 통해서 실시간으로 장애물을 인식하는 방법이 새로 필요하겠다.


# Model Predictive Path Integral Control (MPPI)

Model Predictive Control 에 path intergral을 접목한 방식으로, 구현하려면 로봇의 동역학 모델과 MPC에 대한 지식이 필요했다.

기본적인 내용은 유튜브의 제어조교 영상을 보았다.<br/>
https://www.youtube.com/watch?v=dP8yzkznnK8

https://www.youtube.com/watch?v=zU9DxmNZ1ng

이후 추가적인 공부는 여러 블로그와 KOCW 강의를 참고하였다.

동역학 모델은 Kinematic Bicycle Model을 먼저 적용해보고, 이후에 질량과 관성을 추가한 Dynamic Bicycle Model을 적용하였다.

https://www.youtube.com/watch?v=6fyUnoRxPvs


논문에서의 pseudo code는 다음과 같다.

![12412ed21ed21d1d](https://github.com/user-attachments/assets/9f5a4180-dfa0-4e91-a5c0-e01509a86f64)


즉, 구현 방식은

  1. 변수 초기화, 로봇의 동역학 모델 로드
  2. 여러 랜덤한 제어 입력 u 생성
  3. 랜덤한 제어입력 u로 N개의 미래 time_step을 가진 sampling_numbers만큼의 가상 궤적 생성
  4. 각 가상 궤적을 비용함수로 평가
  5. 가중치 계산
  6. 가중치를 토대로 최적 제어입력 계산
  7. 로봇에 최적 제어입력 반영
  8. 2번부터 반복

DWA와 MPPI 모두 가상 궤적을 샘플링한 다음 N time step 만큼의 미래 시간을 예측해서 최적의 경로를 찾는다. 비용함수도 둘이 매우 비슷하지만, 세부적인 사항은 달랐다.

DWA의 구현엔 동역학 모델이 필요하지 않았다.

또한 DWA는 샘플링한 궤적중 비용이 가장 낮은 최적의 궤적 하나를 선택해 그 궤적대로 이동한다.

하지만 MPPI는 모든 랜덤한 샘플링 궤적의 가중치 합으로 새로운 최적 제어값 u를 계산한다.

![mppi3](https://github.com/user-attachments/assets/c7296403-b728-4788-8d27-42af13a3c919)

Kinematic Bicycle Model 적용


![mppi_dbm](https://github.com/user-attachments/assets/d6080866-2078-4484-9d3e-48c7be554e38)


Dynamic Bicycle Model 적용

정해진 path를 하나하나 나아가면서 장애물을 회피하는 본래의 목적은 어느정도 달성했지만 추가적인 개선이 좀 필요해 보인다.


1. 가끔씩 장애물을 회피하지 못하고 그대로 충돌하는 경우가 있다.
2. 가중치를 계산할 때 목표 위치가 너무 멀어지면 가상 궤적들의 비용이 매우 커져 오류가 발생한다. 가중치나 비용의 데이터 타입 문제로 예상한다.
3. DBM모델을 구성하는 식에서 Vx가 분모가 되는 경우가 많다. 즉, 로봇의 속도가 작아질수록 로봇의 상태가 갑자기 튀는 현상이 발생한다. DBM은 로봇이 일정 속도 이상일 때만 제대로 동작하는 것으로 생각한다.


# 고찰

dwa와 mppi 알고리즘 모두 비용함수의 계수를 적절히 설정해야 하는데, pid튜닝 처럼 경험적으로 계수를 조정해야 해서 시간을 꽤 썼다. 

DWA에서 참고한 문헌(https://jkimst.org/journal/view.php?doi=10.9766/KIMST.2021.24.1.061) 에는 강화학습을 이용하여 해당 계수를 튜닝하였다고 한다. mppi에도 비슷하게 적용이 가능할 것 같다.



# 구현하면서 새로 공부한 파이썬 함수

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

np.random.uniform : 균등분포로부터 랜덤한 값을 추출하는 함수<br/>
np.random.normal : 정규분포로부터 랜덤한 값을 추출하는 함수. 

DWA에서, robot_states라는 로봇의 상태를 저장하는 np.array형 변수가 있는데 이걸 여기저기 함수에 넣어서 사용했더니 calc_trajectory 함수에서 robot_state의 결과에 영향을 주기 시작했다. 
해결법으로 np.copy(robot_states)로 robot_states를 그대로 집어넣지 않고 복사해서 해결하였다. 오류의 원인과 해결법을 알 때까지 시간을 꽤 사용했다..



