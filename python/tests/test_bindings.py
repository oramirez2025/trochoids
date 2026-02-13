import math

import pytest
import trochoids


def _state(x, y, z, psi):
    s = trochoids.XYZPsiState()
    s.x = float(x)
    s.y = float(y)
    s.z = float(z)
    s.psi = float(psi)
    return s


def test_api_symbols_exist():
    assert hasattr(trochoids, "XYZPsiState")
    assert hasattr(trochoids, "VerticalConstraints")
    assert hasattr(trochoids, "VerticalPlanInfo")
    assert hasattr(trochoids, "VerticalPlanningCase")
    assert hasattr(trochoids, "get_trochoid_path")
    assert hasattr(trochoids, "get_trochoid_path_numerical")
    assert hasattr(trochoids, "get_trochoid_path_3d")


def test_invalid_wind_length_raises_value_error():
    start = _state(0.0, 0.0, 110.0, 0.0)
    goal = _state(500.0, 0.0, 110.0, 0.0)

    with pytest.raises(ValueError, match="wind must contain exactly 3 elements"):
        trochoids.get_trochoid_path(start, goal, [0.0, 0.0], 15.0, 0.02)


def test_get_trochoid_path_smoke():
    start = _state(0.0, 0.0, 110.0, 0.0)
    goal = _state(500.0, 0.0, 110.0, 0.0)

    valid, path = trochoids.get_trochoid_path(start, goal, [0.3, 0.5, 0.0], 15.0, 0.02, 10.0)

    assert isinstance(valid, bool)
    assert valid
    assert len(path) > 1
    assert math.isclose(path[0].x, start.x, abs_tol=1e-6)
    assert math.isclose(path[0].y, start.y, abs_tol=1e-6)
    assert math.isclose(path[-1].x, goal.x, abs_tol=1e-2)
    assert math.isclose(path[-1].y, goal.y, abs_tol=1e-2)


def test_get_trochoid_path_numerical_smoke():
    start = _state(0.0, 0.0, 110.0, 0.0)
    goal = _state(500.0, 0.0, 110.0, 0.0)

    valid, path = trochoids.get_trochoid_path_numerical(
        start,
        goal,
        [0.3, 0.5, 0.0],
        15.0,
        0.02,
        False,
        10.0,
    )

    assert isinstance(valid, bool)
    assert valid
    assert len(path) > 1


def test_get_trochoid_path_3d_smoke():
    start = _state(0.0, 0.0, 100.0, 0.2)
    goal = _state(2200.0, 400.0, 180.0, 0.2)

    constraints = trochoids.VerticalConstraints()
    constraints.max_climb_rate = 2.0
    constraints.max_descent_rate = 2.0

    valid, path, info = trochoids.get_trochoid_path_3d(
        start,
        goal,
        [4.0, -2.0, 0.0],
        40.0,
        0.02,
        constraints,
    )

    assert isinstance(valid, bool)
    assert valid
    assert len(path) > 1
    assert info.valid
    assert info.vertical_feasible
    assert info.case_used == trochoids.VerticalPlanningCase.DIRECT_PROFILE
    assert math.isclose(path[-1].z, goal.z, abs_tol=1e-6)
