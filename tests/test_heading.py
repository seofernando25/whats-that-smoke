from whats_that_smoke.heading import RelativeHeading


def calibrated_heading():
    heading = RelativeHeading()
    for index in range(301):
        result = heading.update([0.0, 0.0, -1.0], [0.0, 0.0, 0.2], index / 100, moving=False)
    assert result["heading_calibrated"]
    assert result["heading_deg"] == 0.0
    return heading


def test_heading_zeroes_after_three_stationary_seconds():
    heading = calibrated_heading()
    assert heading.snapshot()["heading_status"] == "relative · ready"


def test_heading_integrates_actual_clockwise_gyro_rotation_while_driving():
    heading = calibrated_heading()
    for index in range(1, 101):
        result = heading.update([0.0, 0.0, -1.0], [0.0, 0.0, 90.2], 3.0 + index / 100, moving=True)
    assert abs(result["heading_deg"] - 90.0) < 0.01
    assert not result["heading_stationary"]


def test_rezero_waits_for_stationary_state():
    heading = calibrated_heading()
    heading.request_calibration()
    result = heading.update([0.0, 0.0, -1.0], [0.0, 0.0, 0.2], 3.01, moving=True)
    assert not result["heading_calibrated"]
    assert result["heading_status"] == "stop car · hold still 3s"


def test_sample_gap_does_not_integrate_unknown_rotation():
    heading = calibrated_heading()
    result = heading.update([0.0, 0.0, -1.0], [0.0, 0.0, 90.2], 3.3, moving=True)
    assert result["heading_deg"] == 0.0
    assert result["heading_status"] == "sample gap · heading held"


def test_disconnect_invalidates_estimate_until_rezeroed():
    heading = calibrated_heading()
    heading.disconnected()
    result = heading.snapshot()
    assert not result["heading_calibrated"]
    assert result["heading_status"] == "imu-disconnected"
