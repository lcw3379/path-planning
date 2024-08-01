## 1. A*

목표까지의 최단 경로를 구하는 다익스트라(Dijkstra)알고리즘을 개선한 방식으로, 네트워크나 게임 혹은 로봇에 사용되는 Path Planning 알고리즘.

현재 노드에서 다음 노드까지의 비용을 g(x), 휴리스틱 함수를 h(x)로 할 때,
f(x)=g(x)+h(x)
가 최소가 되는 f(x)를 우선적으로 탐색하는 알고리즘이다.
f(x) = g(x)일 때 다익스트라 알고리즘이 된다고 하니 다익스트라 알고리즘에 휴리스틱 함수를 추가해서 개선한 알고리즘이라고 볼 수 있다.

보통 2차원 격자 공간에서 구현할 때에는 휴리스틱 함수 h(x)를 해당 노드에서 목적지까지의 거리를 나타내는 함수로 작성한다.

이미 탐색이 끝난 노드는 closed 되었다고 하고, 탐색이 끝난 노드와 인접한 노드는 open된 노드라고 한다. 
이 모든 open된 노드들이 이진트리인 Heap에 들어가서, 가장 F값이 작은 노드를 우선으로 탐색한다.
파이썬에서는 heapq라는 라이브러리와 라이브러리 내 함수 heappush()와 heappop()을 사용해서 간단히 Heap을 사용할 수 있다. heappush는 힙 트리에 노드를 추가하고, heappop은 그곳에서 가장 작은 f값을 가진 노드부터 빠져나오도록 만들 수 있다.

pygame을 사용하여 시각화 하였다. 첫 번째 영상처럼 수평/수직 거리는 가중치 10, 대각선 거리는 대각선 거리를 계산한 후에 10을 곱하고 정수형으로 변환하였다. 즉 대각선으로 1칸 이동할 때 14의 비용이 든다. 


 
유튜브의 영상이 알고리즘 이해에 큰 도움이 되었다. 해당 영상들은 격자 공간에서 A* 알고리즘을 보여주는 가장 좋은 예시라고 생각한다.

https://www.youtube.com/watch?v=-L-WgKMFuhE  <br/>
https://www.youtube.com/watch?v=QR3U1dgc5RE  



작은 공간에서 

![smallmap](https://github.com/user-attachments/assets/2ead7314-4c27-4754-8250-e97fdfb9ae3b)

큰 공간에서 

![bigmap](https://github.com/user-attachments/assets/63ef67fc-32de-4a6b-bd84-7fab0e1a32fb)







유튜브의 두 번째 영상에서 볼 수 있듯이, 그리드의 크기가 커질수록 탐색해야 하는 범위가 늘어나다 보니 탐색에 큰 시간이 걸린다.

따라서 로봇공학에서는 RRT와 RRT* 알고리즘을 Path Planning에 주로 사용하는 것으로 보인다.

## 2. RRT

더 큰 공간에서 path를 생성하기에 적합한 알고리즘<br/>
알고리즘 진행 순서는 다음과 같다.<br/>
1. 초기화     <br/>
2. 랜덤한 위치의 노드 생성<br/>
3. 랜덤한 노드와 가장 가까운 노드 찾기<br/>
4. 가까운 노드에서 랜덤노드 방향으로 step_size만큼 거리에 새 노드 생성<br/>
5. 충돌 검사 후 node_list에 append<br/>
6. 다음 랜덤 위치 설정<br/>

처음엔 4번 과정에서 삼각비를 이용하여 생성했는데, step_size보다 작은 노드가 생성되는 경우가 많아 steer() 함수로 변경하여 랜덤 노드가 어떤 곳에 위치하던 step_size만큼의 위치에 새 노드가 생성되게 하였다. 

![rrt](https://github.com/user-attachments/assets/63f913eb-822b-4181-9e63-6e7fb5d1327a)

구현 모습이다. end_node를 찾으면 end_node부터 부모 노드를 차례대로 올라가며 start_node에 닿는다.



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



