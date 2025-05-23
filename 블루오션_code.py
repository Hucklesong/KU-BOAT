import numpy as np
from sklearn.cluster import DBSCAN
import math

## LiDAR 데이터 전처리 및 장애물 탐지 모듈
## 이 코드는 LiDAR에서 수집된 데이터를 분석하고 DBSCAN을 사용하여 클러스터링을 수행하여 장애물을 감지하기 위해 작성되었습니다.
## DBSCAN은 밀도 기반 클러스터링 알고리즘으로, 점 군집의 밀도를 기준으로 장애물을 효과적으로 탐지할 수 있습니다.

def process_lidar_data_with_dbscan(lidar_data, eps=0.5, min_samples=5, max_distance=10.0, min_distance=0.1):
    """
    LiDAR 데이터를 DBSCAN으로 클러스터링하여 장애물 리스트를 반환합니다.

    Parameters:
    - lidar_data: LiDAR에서 수집된 (각도, 거리) 데이터 리스트 [(angle1, distance1), ...]
    - eps: DBSCAN의 거리 임계값 (클러스터링 범위)
    - min_samples: 클러스터로 간주하기 위한 최소 점 개수
    - max_distance: 처리할 최대 거리 (노이즈 필터링용)
    - min_distance: 처리할 최소 거리 (노이즈 필터링용)

    Returns:
    - obstacle_clusters: 클러스터링된 장애물 리스트 (각 클러스터의 중심점과 점 목록)
    """
    ## LiDAR 데이터를 직교 좌표계로 변환합니다.
    ## DBSCAN은 x, y 좌표를 기반으로 작동하므로, (각도, 거리)를 (x, y)로 변환합니다.
    points = []
    for angle, distance in lidar_data:
        if min_distance <= distance <= max_distance:  # 유효 거리 필터링
            x = distance * math.cos(math.radians(angle))
            y = distance * math.sin(math.radians(angle))
            points.append([x, y])

    ## NumPy 배열로 변환하여 처리 속도를 높입니다.
    points = np.array(points)

    ## DBSCAN을 사용하여 클러스터링을 수행합니다.
    ## eps는 두 점 사이의 최대 거리, min_samples는 클러스터를 이루는 최소 점 개수입니다.
    dbscan = DBSCAN(eps=eps, min_samples=min_samples)
    labels = dbscan.fit_predict(points)

    ## 클러스터링 결과를 기반으로 장애물 클러스터를 만듭니다.
    ## labels에서 -1은 노이즈로 간주되므로 제외합니다.
    obstacle_clusters = []
    for cluster_id in set(labels):
        if cluster_id == -1:
            continue  ## 노이즈는 무시합니다.

        ## 동일 클러스터 ID를 가진 점들을 모읍니다.
        cluster_points = points[labels == cluster_id]
        
        ## 클러스터의 중심점 계산
        cluster_center = cluster_points.mean(axis=0).tolist()
        obstacle_clusters.append({
            "center": cluster_center,
            "points": cluster_points.tolist()
        })

    return obstacle_clusters

## LiDAR 데이터를 -90° ~ +90°로 변환
## 기존 0° ~ 360° 데이터를 선박 기준 좌표계로 변환합니다.
def convert_lidar_angles(lidar_data, current_heading):
    """
    LiDAR 데이터를 선박 기준 -90° ~ +90°로 변환합니다.

    Parameters:
    - lidar_data: LiDAR에서 수집된 (각도, 거리) 데이터 리스트 [(angle1, distance1), ...]
    - current_heading: 현재 선박의 헤딩 방향 (°)

    Returns:
    - converted_data: 변환된 LiDAR 데이터 리스트 [(relative_angle, distance), ...]
    """
    converted_data = []
    for angle, distance in lidar_data:
        ## 상대 각도로 변환
        relative_angle = (angle - current_heading + 360) % 360
        if relative_angle > 180:
            relative_angle -= 360

        ## -90° ~ +90° 범위 내 각도만 포함
        if -90 <= relative_angle <= 90:
            converted_data.append((relative_angle, distance))

    return converted_data

## 장애물 없는 구간 리스트화 함수 수정
## -90° ~ +90° 기준으로 안전 구간을 계산합니다.
def find_safe_zones(obstacle_clusters, safety_margin=15):
    """
    장애물 없는 구간을 계산하여 리스트로 반환합니다.

    Parameters:
    - obstacle_clusters: 장애물 클러스터 리스트
    - safety_margin: 장애물 주변의 안전 각도 마진 (°)

    Returns:
    - safe_zones: 장애물이 없는 각도 구간 리스트 [[start, end], ...]
    """
    obstacles = []
    for cluster in obstacle_clusters:
        x, y = cluster['center']
        angle = math.degrees(math.atan2(y, x)) % 360  ## 장애물 중심 각도 계산
        ## -90° ~ +90° 범위로 변환
        relative_angle = (angle + 360) % 360
        if relative_angle > 180:
            relative_angle -= 360
        if -90 <= relative_angle <= 90:
            obstacles.append((relative_angle - safety_margin, relative_angle + safety_margin))

    obstacles.sort()  ## 각도를 기준으로 정렬

    safe_zones = []
    current_angle = -90

    for start, end in obstacles:
        if start > current_angle:
            safe_zones.append([current_angle, start])  ## 안전 구간 추가
        current_angle = max(current_angle, end)

    if current_angle < 90:
        safe_zones.append([current_angle, 90])  ## 마지막 구간 추가

    return safe_zones

## 최적의 안전 각도를 탐색
## 목표 헤딩과 각 안전 구간의 중심각도를 비교하여 최적 구간 선택
def find_safe_angle(current_heading, goal_heading, safe_zones):
    """
    장애물 없는 구간 중에서 최적의 안전 각도를 탐색합니다.

    Parameters:
    - current_heading: 현재 선박의 선수각 (°)
    - goal_heading: 목표 지점 각도 (°, 절대 좌표계)
    - safe_zones: 장애물이 없는 각도 구간 리스트 [[start, end], ...]

    Returns:
    - best_angle: 선택된 안전 각도
    """
    # 목표 헤딩을 상대 좌표계로 변환
    relative_goal_heading = (goal_heading - current_heading + 360) % 360
    if relative_goal_heading > 180:
        relative_goal_heading -= 360

    best_angle = None
    min_diff = float('inf')

    for zone in safe_zones:
        # 구간 중심각도 계산
        zone_center = (zone[0] + zone[1]) / 2
        # 목표 헤딩(상대 좌표)과 중심각도의 차이 (절댓값)
        diff_to_goal = abs(relative_goal_heading - zone_center)

        # 최소 차이 갱신
        if diff_to_goal < min_diff:
            min_diff = diff_to_goal
            best_angle = zone_center
        elif diff_to_goal == min_diff:
            # 동일한 차이인 경우 현재 헤딩과의 차이 비교
            diff_to_current = abs(current_heading - zone_center)
            if diff_to_current < min_diff:
                min_diff = diff_to_current
                best_angle = zone_center

    return best_angle

## 메인 알고리즘 루프 수정
## 기본적으로 목표 위치로 이동하고, 장애물이 탐지되면 회피 동작 수행
def main_navigation_loop():
    """
    메인 알고리즘 루프: 목표 지점을 향해 이동하며, 장애물이 있으면 회피 동작 수행.
    """
    current_lat, current_lon = 35.123, 129.321  ## 현재 GPS 좌표
    target_lat, target_lon = 35.124, 129.322  ## 목표 GPS 좌표

    while True:
        ## 목표 지점까지의 거리와 헤딩 계산
        distance, goal_heading = calculate_distance_and_heading(current_lat, current_lon, target_lat, target_lon)

        ## 목표 지점 도달 여부 확인
        if distance < 0.5:  ## 0.5m 이내면 도달로 간주
            print("목표 지점에 도달!")
            break

        current_heading = 90  ## 현재 헤딩 값 (예시)

        ## LiDAR 데이터 생성 및 변환
        lidar_data = [(angle, np.random.uniform(0.1, 2.0)) for angle in range(0, 360, 5)]
        converted_lidar_data = convert_lidar_angles(lidar_data, current_heading)

        ## 장애물 클러스터링
        obstacle_clusters = process_lidar_data_with_dbscan(converted_lidar_data)

        ## 장애물이 없는 경우 목표 헤딩으로 이동
        if not obstacle_clusters:
            safe_angle = goal_heading
            thrust_command = 1.0  ## 최대 속도
            steering_command = 0  ## 직진
            print(f"장애물이 없어 목표로 직진: Safe Angle = {safe_angle:.2f}°")
        else:
            ## 장애물이 있는 경우 회피 동작 수행
            safe_zones = find_safe_zones(obstacle_clusters)

            print("이동 가능한 구간:")
            for i, zone in enumerate(safe_zones):
                print(f"구간 {i+1}: [{zone[0]:.1f}°~{zone[1]:.1f}°]")

            ## 최적의 안전 각도 탐색
            safe_angle = find_safe_angle(current_heading, goal_heading, safe_zones)

            print(f"선택된 구간 중심각도: {safe_angle:.1f}°")

            ## 제어 명령 생성
            thrust_command, steering_command = control_boat(current_heading, safe_angle)

        ## 제어 명령 출력##모터값 제어 코드는 따로 외부 클래스로 정의
        print(f"Distance to Target: {distance:.2f}m, Goal Heading: {goal_heading:.2f}, Safe Angle: {safe_angle:.2f}, Thrust: {thrust_command:.2f}, Steering: {steering_command:.2f}")

        ## 현재 헤딩 업데이트 (가상 환경에서는 실제 GPS와 IMU 데이터를 사용)
        current_heading = (current_heading + steering_command * 10) % 360

if __name__ == "__main__":
    main_navigation_loop()
