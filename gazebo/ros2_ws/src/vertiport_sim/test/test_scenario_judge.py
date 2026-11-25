"""scenario_core / judge_core 테스트."""
import os
import sys

import yaml

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from vertiport_sim import judge_core as jc  # noqa: E402
from vertiport_sim import scenario_core as sc  # noqa: E402

CFG = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), '..', 'config', 'scenarios.yaml')))
PAD = {'x': 0.0, 'y': 0.0, 'size_m': 3.0}


def objs(name):
    return [sc.parse_object(d, CFG['models']) for d in (CFG['scenarios'][name]['objects'] or [])]


def test_all_scenarios_parse_and_expected_decision():
    expected = {'S0_clear': jc.LAND_OK, 'S1_person': jc.HOLD, 'S2_vehicle': jc.RESTRICTED,
                'S3_fod': jc.NO_LAND, 'S4_fod_edge': jc.NO_LAND, 'S5_mixed': jc.NO_LAND,
                'S6_person_far': jc.LAND_OK}
    for name, exp in expected.items():
        res = jc.evaluate([o.to_gt() for o in objs(name)], PAD)
        assert res['decision'] == exp, (name, res)


def test_sequences_reference_existing_scenarios():
    for seq, steps in CFG['sequences'].items():
        for s in steps:
            if 'apply' in s:
                assert s['apply'] in CFG['scenarios'], (seq, s)


def test_diff_scene():
    cur = {o.id: o for o in objs('S5_mixed')}
    remove, spawn, move = sc.diff_scene(cur, objs('S1_person'))
    assert {o.id for o in remove} == {'vehicle_1', 'fod_1'} and not spawn and not move
    moved = sc.parse_object({'id': 'person_1', 'type': 'person', 'x': 1, 'y': 1}, CFG['models'])
    remove, spawn, move = sc.diff_scene({'person_1': objs('S1_person')[0]}, [moved])
    assert not remove and not spawn and [o.id for o in move] == ['person_1']


def test_requests_and_dry_run():
    gz = sc.GzClient('vertiport', dry_run=True)
    o = objs('S3_fod')[0]
    assert gz.create(o)[0]
    req = gz.calls[-1][-1]
    assert '"' not in req.split('sdf: "')[1].split('" name:')[0], 'SDF 안에 큰따옴표 금지'
    assert 'name: "hz_fod_1"' in req and 'model://vp_fod' in req
    assert gz.calls[-1][3] == '/world/vertiport/create'
    gz.remove(o.gz_name)
    assert gz.calls[-1][-1] == 'name: "hz_fod_1" type: MODEL'


def test_bad_objects():
    for bad in ({'type': 'ufo', 'id': 'a', 'x': 0, 'y': 0}, {'type': 'fod', 'id': '', 'x': 0, 'y': 0},
                {'type': 'fod', 'id': 'a b', 'x': 0, 'y': 0}, {'type': 'fod', 'id': 'a', 'x': 'nan', 'y': 0}):
        try:
            sc.parse_object(bad, CFG['models'])
        except ValueError:
            continue
        raise AssertionError(bad)


def test_sequence_runner():
    r = sc.SequenceRunner('t', CFG['sequences']['demo_person_hold'])
    assert r.step(0.0, 0.0) == 'S0_clear'
    assert r.step(0.5, 0.0) is None          # 지상: 이륙 전에는 트리거되지 않음
    assert r.step(0.8, 4.0) is None          # 이륙 완료, 아직 하강 전
    assert r.step(1.0, 3.5) is None
    assert r.step(2.0, 2.9) == 'S1_person'
    assert r.step(3.0, 2.9) is None          # wait_s 8 시작
    assert r.step(10.9, 2.9) is None
    assert r.step(11.1, 2.9) == 'S0_clear'
    assert r.step(12.0, 2.9) is None and r.done


if __name__ == '__main__':
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            fn()
            print('PASS', name)
