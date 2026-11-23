"""학습·평가·GUI가 공유하는 보조 없는 주행 설정."""
import os


def unassisted_enabled():
    return os.environ.get("YEOUIDO_UNASSISTED") == "1"


def unassisted_config(road, action):
    road = {**road, "signal_assist": False, "obey_speed_limit": False,
            "safety_shield": False, "lane_change_mode": 0,
            "route_lane_guidance": False, "navigation_observation": True,
            "reroute": {**road.get("reroute", {}), "enabled": False}}
    action = {**action, "lane_change_safety": {
        **action.get("lane_change_safety", {}), "enabled": False}}
    return road, action


def direction_index(direction):
    """지도상의 직진·좌·우·유턴. 안전성·우선권·정답 차로는 포함하지 않는다."""
    return {"s": 0, "l": 1, "L": 1, "r": 2, "R": 2, "t": 3}.get(direction)


def navigation_features(net, edge_id, lane_index, route, route_index, offsets):
    """도로 단위 다음 회전 4개 + 주변 5개 차로의 모든 연결 방향 20개.

    차로 연결은 목적지와 무관한 정적 지도 정보이며, 추천 차로를 계산하지 않는다.
    교차로 내부/알 수 없는 도로에서는 모두 0, 최종 도로에서는 다음 회전만 0이다.
    """
    import numpy as np
    out = np.zeros(4 + 4 * len(offsets), dtype=np.float32)
    if edge_id.startswith(":") or not net.hasEdge(edge_id):
        return out
    edge = net.getEdge(edge_id)
    if 0 <= route_index < len(route) - 1:
        next_id = route[route_index + 1]
        for target, connections in edge.getOutgoing().items():
            if target.getID() == next_id:
                for connection in connections:
                    k = direction_index(connection.getDirection())
                    if k is not None:
                        out[k] = 1.0
    lanes = edge.getLanes()
    for i, offset in enumerate(offsets):
        li = lane_index + offset
        if 0 <= li < len(lanes):
            for connection in lanes[li].getOutgoing():
                k = direction_index(connection.getDirection())
                if k is not None:
                    out[4 + 4 * i + k] = 1.0
    return out
