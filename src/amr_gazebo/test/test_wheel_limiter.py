import math
import pytest
from amr_gazebo.wheel_limiter import WheelLimiter


def test_combined_limits_and_centripetal_bound():
    f = WheelLimiter(timeout=100)
    f.step(0); f.command(4, 12, 0)
    for i in range(1, 201):
        old = (f.left, f.right)
        v, w = f.step(i * .02)
        assert max(abs(f.left), abs(f.right)) <= 1 + 1e-12
        assert max(abs(f.left-old[0]), abs(f.right-old[1])) <= .01 + 1e-12
        assert abs(v*w) <= 1 + 1e-12


def test_reverse_and_timeout():
    f = WheelLimiter(timeout=.2)
    f.step(0); f.command(.5, 0, 0); f.step(.1)
    assert f.left == pytest.approx(.05)
    f.command(-.5, 0, .1); f.step(.2)
    assert f.left == pytest.approx(0)
    f.step(.3); assert f.left == pytest.approx(-.05)
    f.step(.4); assert f.left == pytest.approx(0)


def test_clock_reset_and_large_gap():
    f = WheelLimiter(timeout=5, max_dt=.1)
    f.step(1); f.command(1, 0, 1); f.step(1.1)
    before = f.left
    f.step(0); assert f.left == before and f.last_command is None
    f.step(100); assert abs(f.left-before) <= .05 + 1e-12
    with pytest.raises(ValueError): f.command(math.nan, 0, 100)


def test_command_before_first_tick_is_preserved():
    f = WheelLimiter()
    f.command(.2, 0, 1.0)
    f.step(1.0)
    f.step(1.02)
    assert f.left == pytest.approx(.01)


def test_calibrated_turn_uses_effective_wheel_track():
    f = WheelLimiter()
    f.step(0)
    f.command(0, .5, 0)
    assert f.target == pytest.approx((-.1375, .1375))
    for now in (.1, .2, .3):
        previous = (f.left, f.right)
        v, w = f.step(now)
        assert max(abs(f.left - previous[0]),
                   abs(f.right - previous[1])) <= .05 + 1e-12
    assert v == pytest.approx(0)
    assert w == pytest.approx(.5)


def test_saturation_and_coupled_deceleration():
    f = WheelLimiter(timeout=100)
    f.step(0); f.command(2, 4, 0)
    assert max(map(abs, f.target)) == pytest.approx(1)
    for i in range(1, 151):
        f.step(i * .02)
    f.command(0, 0, 3)
    prior = (f.left, f.right)
    f.step(3.02)
    assert max(abs(f.left-prior[0]), abs(f.right-prior[1])) <= .0100000001


@pytest.mark.parametrize('linear,angular', [(1., 0.), (0., 2.), (.3, .8), (-.3, -.8)])
def test_body_acceleration_respects_both_wheel_limits(linear, angular):
    f = WheelLimiter(timeout=100)
    previous_v, previous_w = f.step(0.)
    f.command(linear, angular, 0.)
    for i in range(1, 101):
        v, w = f.step(i * .02)
        acceleration = (v - previous_v) / .02
        angular_acceleration = (w - previous_w) / .02
        assert abs(acceleration - f.track * angular_acceleration / 2) <= .5 + 1e-10
        assert abs(acceleration + f.track * angular_acceleration / 2) <= .5 + 1e-10
        if i == 1 and angular == 0:
            assert acceleration == pytest.approx(.5)
        if i == 1 and linear == 0:
            assert angular_acceleration == pytest.approx(2 * .5 / .55)
        previous_v, previous_w = v, w
