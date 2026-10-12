"""TEL duty as compare counts reads back as the Q15 column earlier
recordings carry."""

import pandas as pd

from oscnb.datasets import drive_ticks, with_duty_q15

META = {"pwm_period_ticks": 1200}


def test_counts_at_the_goal_read_as_the_goal():
    cmd = [8520, 8520, -14745, 0]
    counts = [int(drive_ticks(8520, 1200)), 200, -int(drive_ticks(14745, 1200)), 0]
    df = with_duty_q15(pd.DataFrame({"cmd_duty_q15": cmd, "duty_counts": counts}), META)
    assert df.duty_q15.tolist() == [8520, round(200 * 32768 / 1200), -14745, 0]


def test_empty_cells_stay_empty():
    df = pd.DataFrame({"cmd_duty_q15": [100, 100], "duty_counts": [None, 4]})
    df = with_duty_q15(df, META)
    assert df.duty_q15.isna().tolist() == [True, False]


def test_a_q15_recording_passes_through():
    df = pd.DataFrame({"cmd_duty_q15": [8520], "duty_q15": [8519]})
    assert with_duty_q15(df.copy(), {}).duty_q15.tolist() == [8519]
