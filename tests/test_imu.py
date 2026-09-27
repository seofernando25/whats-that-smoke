from whats_that_smoke.imu import parse_imu_line


def test_parse_imu_line():
    parsed = parse_imu_line(b'{"type":"imu","a":[1,2,3],"g":[4,5,6]}')
    assert parsed == ([1.0, 2.0, 3.0], [4.0, 5.0, 6.0])


def test_parse_imu_line_rejects_bad_messages():
    assert parse_imu_line(b'not-json') is None
    assert parse_imu_line(b'{"type":"other","a":[1,2,3],"g":[4,5,6]}') is None
