import json
import pathlib

import numpy as np

from atom_calibration.calibration.tf_derivator import (
    timeFloatToStamp,
    timeStampToFloat,
    getTFToDeriveList,
)


def test_time_float_to_stamp():
    t = 1234.567891234

    assert timeFloatToStamp(t) == {"secs": 1234, "nsecs": 567891234}


def test_time_stamp_to_float():

    stamp = {"secs": 1234, "nsecs": 567891234}

    assert timeStampToFloat(stamp) == 1234.567891234


def test_getTFToDeriveList():

    script_dir = pathlib.Path(__file__).parent

    with open(script_dir / "test_data/getTFToDeriveList.json") as f:
        input_dataset = json.load(f)

    tf_list = input_dataset["continuous_data"]["transforms"]

    from_frame = "world"
    to_frame = "imu_link"
    t = 2650.636300000
    n = 3

    expected_data = [
        {
            "parent": "world",
            "child": "imu_link",
            "trans": np.array([[-1.5506525], [1.18418197], [0.8566985]]),
            "quat": np.array(
                [1.42682789e-05, 8.67348849e-09, 3.49097182e-04, 9.99999939e-01]
            ),
            "stamp": {"nsecs": 637000000, "secs": 2650},
        },
        {
            "parent": "world",
            "child": "imu_link",
            "trans": np.array([[-1.5506525], [1.18418197], [0.8566985]]),
            "quat": np.array(
                [1.42682789e-05, 8.67348849e-09, 3.49097182e-04, 9.99999939e-01]
            ),
            "stamp": {"nsecs": 617000000, "secs": 2650},
        },
        {
            "parent": "world",
            "child": "imu_link",
            "trans": np.array([[-1.5506525], [1.18418197], [0.8566985]]),
            "quat": np.array(
                [1.42682789e-05, 8.67348849e-09, 3.49097182e-04, 9.99999939e-01]
            ),
            "stamp": {"nsecs": 657000000, "secs": 2650},
        },
    ]

    res = getTFToDeriveList(tf_list, from_frame, to_frame, t, n)

    # Sort the lists so assertion ignores list order
    expected_data = sorted(expected_data, key=lambda x: x["stamp"]["nsecs"])
    res = sorted(res, key=lambda x: x["stamp"]["nsecs"])

    # np.testing.assert_array_almost_equal(res, expected_data)
    assert res == expected_data
