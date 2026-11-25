"""Ground Truth 기반 '모의 안전판단' 로직 (ROS 의존성 없음).

⚠️ 실제 판단 알고리즘(AI 탐지 결과 기반)이 들어오기 전까지 Gazebo 파트를
   단독으로 시험하기 위한 테스트용 스텁이다. 출력 형식(decision JSON)은
   실제 판단 프로그램도 그대로 따르도록 docs/INTERFACE.md 에 규격화되어 있다.
"""

import math
from typing import Dict, List, Optional

LAND_OK = 'LAND_OK'          # 착륙 가능
HOLD = 'HOLD'                # 대기 (Hover)
RESTRICTED = 'RESTRICTED'    # 접근 제한
NO_LAND = 'NO_LAND'          # 착륙 금지

SEVERITY = {LAND_OK: 0, HOLD: 1, RESTRICTED: 2, NO_LAND: 3}
LABEL_KO = {LAND_OK: '착륙 가능', HOLD: '대기', RESTRICTED: '접근 제한', NO_LAND: '착륙 금지'}

DEFAULT_RULES = {
    'person_radius_m': 5.0,     # 패드 중심 반경 내 사람 → HOLD
    'vehicle_radius_m': 8.0,    # 패드 중심 반경 내 차량 → RESTRICTED
    'fod_margin_m': 0.5,        # 패드 사각형 + 여유폭 내 FOD → NO_LAND
}


def evaluate(objects: List[dict], pad: dict, rules: Optional[Dict] = None) -> dict:
    """objects: [{'id','type','x','y',...}] (World ENU), pad: {'x','y','size_m'}"""
    r = dict(DEFAULT_RULES)
    if rules:
        r.update(rules)
    px, py = float(pad.get('x', 0.0)), float(pad.get('y', 0.0))
    half = float(pad.get('size_m', 3.0)) / 2.0
    reasons = []
    for o in objects:
        t = o.get('type')
        dx, dy = float(o['x']) - px, float(o['y']) - py
        dist = math.hypot(dx, dy)
        if t == 'person' and dist <= r['person_radius_m']:
            reasons.append({'decision': HOLD, 'type': t, 'id': o.get('id'), 'dist_m': round(dist, 2)})
        elif t == 'vehicle' and dist <= r['vehicle_radius_m']:
            reasons.append({'decision': RESTRICTED, 'type': t, 'id': o.get('id'), 'dist_m': round(dist, 2)})
        elif t == 'fod' and max(abs(dx), abs(dy)) <= half + r['fod_margin_m']:
            reasons.append({'decision': NO_LAND, 'type': t, 'id': o.get('id'), 'dist_m': round(dist, 2)})
    decision = LAND_OK
    for rs in reasons:
        if SEVERITY[rs['decision']] > SEVERITY[decision]:
            decision = rs['decision']
    return {'decision': decision, 'label_ko': LABEL_KO[decision], 'reasons': reasons}
