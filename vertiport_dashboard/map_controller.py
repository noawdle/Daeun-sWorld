"""미터 단위의 로컬 지도 좌표. 외부 좌표계 변환은 공급자에서 담당합니다."""

from pathlib import Path

import streamlit.components.v1 as components


MAP_LIMIT = 10.0
MAX_ALTITUDE = 20.0
_map = components.declare_component("vertiport_map", path=str(Path(__file__).with_name("map_component")))


def local_target(x, y, z):
    values = (x, y, z)
    if not all(type(value) in (float, int) for value in values):
        raise ValueError("유한한 로컬 좌표를 입력해 주세요.")
    if not (-MAP_LIMIT <= x <= MAP_LIMIT and -MAP_LIMIT <= y <= MAP_LIMIT and 0 <= z <= MAX_ALTITUDE):
        raise ValueError("지도 범위는 X/Y ±10 m, Z 0~20 m입니다.")
    return tuple(round(value, 2) for value in values)


def render_map(position, target, path, pad, handled_click, *, twin=None):
    return _map(position=position, target=target, path=path, pad=pad,
                handled_click=handled_click, limit=MAP_LIMIT, twin=twin,
                key="vertiport_map", default=None)
