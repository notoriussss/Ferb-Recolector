"""
Dataclass-driven ROS parameter helpers. Pure logic, no rclpy.

A parameter is named once, as a dataclass field. The same dataclass gives
the declared defaults and receives the values read back from the node, so
names cannot drift between `declare_parameter` and `get_parameter` calls.
The `node` argument only needs `declare_parameter` and `get_parameter`.
"""

from dataclasses import fields, is_dataclass
from typing import Any


def declare_params(node: Any, defaults: Any) -> None:
    """
    Declare one ROS parameter per field of the `defaults` dataclass.

    The field name is the parameter name and the field value its default.
    """
    if not is_dataclass(defaults):
        raise TypeError(f"expected a dataclass instance, got {defaults!r}")
    for field in fields(defaults):
        node.declare_parameter(field.name, getattr(defaults, field.name))


def read_params[ParamsT](node: Any, defaults: ParamsT) -> ParamsT:
    """
    Build a new dataclass of the same type as `defaults` from the node's
    current parameter values.

    Values are coerced to the type of the default (an int set from YAML into
    a float field becomes a float, a sequence into a list).
    """
    values = {}
    for field in fields(defaults):
        default = getattr(defaults, field.name)
        value = node.get_parameter(field.name).value
        values[field.name] = _coerce(value, default)
    return type(defaults)(**values)


def _coerce(value: Any, default: Any) -> Any:
    """
    Convert `value` to the type of `default` when that is safe to do.
    """
    if isinstance(default, bool):
        return bool(value)
    if isinstance(default, float):
        return float(value)
    if isinstance(default, list):
        return list(value)
    return value
