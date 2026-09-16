# ROS 2 PGM → Voronoi roadmap

`pgm_voronoi.py`는 지도에서 흰색 자유 공간의 Voronoi 중심선을 그리고,
A*/Dijkstra 입력으로 사용할 수 있는 무방향 그래프를 저장합니다. ROS 실행은 필요 없습니다.

## 실행

```bash
python3 -m pip install -r requirements.txt
python3 pgm_voronoi.py map/map2.pgm
```

기본 출력은 `output/map2_voronoi_overlay.png`, `output/map2_voronoi_skeleton.png`,
`output/map2_voronoi_graph.json`입니다. 원본 지도는 수정하지 않습니다.
overlay의 빨간 선이 Voronoi 경로이며, skeleton은 원본 크기의 흑백 시각화입니다.

PGM만 있고 해상도 정보가 없으면 **모든 거리 옵션과 그래프 거리는 픽셀 단위**입니다.
같은 이름의 `.yaml`/`.yml` 파일이 있으면 자동으로 읽습니다. YAML 자체를 입력해도 됩니다.

```bash
# 실제 지도 해상도가 0.05 m/pixel인 경우의 예시 (현재 PGM의 해상도를 가정하지 않음)
python3 pgm_voronoi.py map/map2.pgm --resolution 0.05 \
  --robot-radius 0.22 --safety-margin 0.05 --min-branch-length 0.3

# YAML의 image, resolution, origin, negate, free_thresh 사용
python3 pgm_voronoi.py /path/to/map.yaml --robot-radius 0.22 --safety-margin 0.05

# 픽셀 단위로 짧은 막다른 가지 정리
python3 pgm_voronoi.py map/map1.pgm --min-branch-length 8 --output-dir output/pruned
python3 pgm_voronoi.py --help
```

`--origin X Y YAW`로 원점과 회전(rad)을 지정할 수 있습니다. 해상도만 주면 원점은
`[0, 0, 0]`입니다. 실제 ROS map 좌표에 맞추려면 원래 YAML을 사용하는 것이 좋습니다.

## ROS 2 RViz에 표시

`voronoi_rviz.py`가 **JSON → visualization_msgs/msg/Marker (LINE_LIST)**로 변환하여
`/voronoi` 토픽에 빨간 선을 발행합니다. PNG는 일반 이미지 확인용입니다.

```bash
source /opt/ros/humble/setup.bash
cd /home/bjw/jwb_ws/src/voronoi

# PGM만으로 생성한 기존 JSON: 실제 지도에 맞는 해상도와 원점을 입력
# 아래 값은 사용자가 제시한 YAML의 예시이며 해당 PGM의 실제 값인지 확인해야 합니다.
python3 voronoi_rviz.py output/map_voronoi_graph.json \
  --resolution 0.05 --origin -12.9 -11.2 0

# JSON에 올바른 해상도/원점이 이미 있으면 추가 옵션 생략
python3 voronoi_rviz.py /path/to/metric_voronoi_graph.json
```

RViz에서 다음과 같이 설정합니다.

1. **Global Options → Fixed Frame: `map`**
2. **Add → Marker → Topic: `/voronoi`** (`MarkerArray`가 아닌 `Marker`)
3. 기존 지도와 겹쳐 보려면 **Map** 디스플레이의 Topic을 `/map`으로 설정

스크립트는 실행한 상태로 유지합니다. 1초 간격으로 재발행하며, QoS는 Reliable /
Transient Local입니다. 선 두께는 `--line-width 0.03`(m), 프레임은 `--frame-id map`으로
조절합니다. `--z 0.03`은 선을 지도보다 3cm 위에 놓아 겹침 현상을 줄입니다.

YAML 파일 자체는 필요하지 않습니다. 다만 기존 `/map`과 정확히 겹치려면 **그 지도와
동일한 resolution, origin, frame**이 필요합니다. 픽셀 단위 JSON에 해상도를 주지 않으면
명확한 오류로 종료하여 픽셀을 미터로 잘못 표시하지 않습니다. 표시 옵션은 원래 JSON이나
경로 생성 때 적용한 반경/여유 거리 필터를 변경하지 않습니다.

이 스크립트는 Voronoi 선만 발행합니다. 배경 지도는 기존 map_server의 `/map`을 사용합니다.
ROS 노드나 TF를 새로 연결하지 않아도 Fixed Frame과 Marker frame이 모두 `map`이면
Voronoi 선 자체는 표시할 수 있습니다. 다른 Fixed Frame을 쓰면 해당 프레임 간 TF가 필요합니다.

참고: [ROS 2 RViz Marker 설명](https://docs.ros.org/en/humble/Tutorials/Intermediate/RViz/Marker-Display-types/Marker-Display-types.html).

## 처리 방식

- 기본 `negate=0`, `free_thresh=0.196`: `(255 - gray) / 255 < free_thresh`인 픽셀만 자유 공간입니다.
  제공된 지도의 254는 free, 0과 205는 통과 불가로 처리합니다. 지도 밖도 통과 불가입니다.
- 장애물/unknown과 자유 공간이 접하는 경계 셀 중심을 SciPy Voronoi의 site로 사용합니다.
  같은 벽의 이웃 샘플에서 생기는 선을 줄이기 위해 site 간격이 2 px 미만인 ridge는 제외합니다.
  경사진 벽의 계단 모양 픽셀에서 생기는 잔가지를 줄이기 위해 끝 가지가 본 경로에
  붙는 선분의 중점에서 두 site를 바라본 각도가 `--min-site-angle`(기본 45°)보다
  작으면 해당 끝 가지 전체를 제거합니다. 이 필터는 순환 경로를 끊지 않습니다.
  `--min-site-angle 0`으로 각도 필터를 끌 수 있습니다.
  셀 경계를 샘플링한 **근사 medial axis**이며 정확한 연속 장애물의 medial axis는 아닙니다.
- 각 선분 전체의 장애물 여유 거리 하한을 계산합니다. 장애물 셀을 외접원으로 감싸는
  보수적 계산이므로 실제 셀 모서리까지의 거리보다 작을 수 있습니다.
  `robot-radius + safety-margin`보다 여유 거리가 큰 선분만 유지합니다.
  반경은 원형 footprint 또는 비원형 로봇의 외접 반경으로 지정합니다.
- `--min-branch-length`는 짧은 끝 가지를 반복 제거하며 기본값 0은 제거하지 않습니다.
  가지 정리, 각도 및 반경 필터는 좁은 통로나 일부 목적지로의 연결을 없앨 수 있습니다.
- trinary/scale YAML과 8-bit grayscale PGM을 지원합니다. scale의 중간 점유도 셀도
  free 기준을 만족하지 않으면 통과 불가입니다. raw mode는 지원하지 않습니다.

## 플래너에서 사용

JSON의 `map.units`는 `m` 또는 `px`입니다.

| 필드 | 의미 |
| --- | --- |
| `nodes[].id` | 노드 번호; nodes 배열 인덱스와 동일 |
| `nodes[].pixel` | `[u, v]`, 이미지 좌표; 정수 위치가 픽셀 중심 |
| `nodes[].position` | `[x, y]`, 아래 수식으로 변환된 좌표 |
| `nodes[].component` | 연결 성분 ID; 다른 성분끼리는 경로 없음 |
| `edges[].source`, `target` | 양방향 연결 노드 ID |
| `edges[].length` | A*/Dijkstra의 기본 비용으로 사용 가능한 선분 길이 |
| `edges[].min_clearance` | 선분 전체의 장애물까지 거리 하한; 이미 반경을 뺀 값은 아님 |

좌표 변환은 `local = [(u + 0.5), (height - v - 0.5)] * resolution`,
`position = origin.xy + R(origin.yaw) * local`입니다. 해상도가 없으면 배율 1을 사용합니다.

그래프는 선분의 중간 꺾임을 생략하지 않습니다. 경로 탐색 시 각 edge를 양방향으로
adjacency에 넣고 `length`를 비용으로 쓰면 됩니다. 시작/목표가 그래프 위에 없을 때는
**충돌 검사를 통과한 연결 선분**으로 그래프에 붙여야 합니다. 가까운 노드에 무조건
직선 연결하면 벽을 가로지를 수 있습니다. skeleton PNG는 시각화용으로,
픽셀 반올림 때문에 JSON 그래프와 연결성이 다를 수 있습니다.

현재 결과는 정적 지도용 경로망입니다. Nav2 planner plugin 연결, 시작/목표 연결,
동적 costmap 검사와 최종 footprint 검사는 아직 포함하지 않습니다.

검증: `python3 -m unittest -v` (좌표 변환, 통로 연결, 반경 필터, 모서리 접촉,
장애물 주위 순환 경로, 실제 장애물 셀에 대한 여유 거리 검사).

참고: [SciPy Voronoi](https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.Voronoi.html),
[Nav2 Humble 지도 로딩 구현](https://api.nav2.org/nav2-humble/html/map__io_8cpp_source.html).
