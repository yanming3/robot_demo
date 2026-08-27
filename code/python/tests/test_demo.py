import argparse
import csv
import json
from robot_arm_demo.demo import run_demo


def test_writes_csv_output(tmp_path) -> None:
    output_path = tmp_path / "trajectory.csv"
    args = argparse.Namespace(
        link_1=1.0,
        link_2=1.0,
        target=(1.2, 0.8),
        steps=8,
        elbow_up=False,
        output=str(output_path),
    )

    exit_code = run_demo(args)

    assert exit_code == 0
    with output_path.open(newline="", encoding="utf-8") as output_file:
        rows = list(csv.DictReader(output_file))

    assert len(rows) == 9
    assert rows[0]["step"] == "0"
    assert rows[0]["end_x"] == "2.0"
    assert rows[-1]["step"] == "8"
    assert rows[-1]["end_x"] == "1.2"
    assert rows[-1]["end_y"] == "0.8"


def test_writes_json_output(tmp_path) -> None:
    output_path = tmp_path / "trajectory.json"
    args = argparse.Namespace(
        link_1=1.0,
        link_2=1.0,
        target=(1.2, 0.8),
        steps=8,
        elbow_up=False,
        output=str(output_path),
    )

    exit_code = run_demo(args)

    assert exit_code == 0
    with output_path.open(newline="", encoding="utf-8") as output_file:
        rows = json.load(output_file)

    assert len(rows) == 9
    assert rows[0]["step"] == 0
    assert rows[0]["end_x"] == 2.0
    assert rows[-1]["step"] == 8
    assert rows[-1]["end_x"] == 1.2
    assert rows[-1]["end_y"] == 0.8
