"""
Tests for the dataclass-driven parameter helpers and the math helpers.
"""

import math
from dataclasses import dataclass, field
from types import SimpleNamespace

import pytest

from mobile_base.mathutil import clamp, wrap_angle
from mobile_base.params import declare_params, read_params


@dataclass
class Sample:
    """
    Dataclass with one field of each type the helpers coerce.
    """

    rate_hz: float = 2.0
    enabled: bool = True
    colors: list[str] = field(default_factory=lambda: ["red"])
    name: str = "ferb"


class FakeNode:
    """
    Minimal stand-in for rclpy.node.Node: stores parameters in a dict.
    """

    def __init__(self):
        self.values = {}

    def declare_parameter(self, name, default):
        self.values[name] = default

    def get_parameter(self, name):
        return SimpleNamespace(value=self.values[name])


def test_declare_params_uses_field_names_and_defaults():
    node = FakeNode()
    declare_params(node, Sample())
    assert node.values == {
        "rate_hz": 2.0,
        "enabled": True,
        "colors": ["red"],
        "name": "ferb",
    }


def test_declare_params_rejects_non_dataclass():
    with pytest.raises(TypeError):
        declare_params(FakeNode(), {"rate_hz": 2.0})


def test_read_params_round_trip():
    node = FakeNode()
    declare_params(node, Sample())
    assert read_params(node, Sample()) == Sample()


def test_read_params_coerces_to_the_default_type():
    node = FakeNode()
    declare_params(node, Sample())
    node.values["rate_hz"] = 5  # int from YAML into a float field
    node.values["colors"] = ("blue", "green")  # sequence into a list
    result = read_params(node, Sample())
    assert result.rate_hz == 5.0 and isinstance(result.rate_hz, float)
    assert result.colors == ["blue", "green"]


def test_clamp_limits_both_ends():
    assert clamp(5.0, -1.0, 1.0) == 1.0
    assert clamp(-5.0, -1.0, 1.0) == -1.0
    assert clamp(0.25, -1.0, 1.0) == 0.25


def test_wrap_angle_stays_in_range():
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert wrap_angle(3 * math.pi / 2) == pytest.approx(-math.pi / 2)
    assert wrap_angle(-3 * math.pi / 2) == pytest.approx(math.pi / 2)
