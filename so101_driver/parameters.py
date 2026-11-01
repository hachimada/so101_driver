"""Numeric parameters that accept 30 for 30.0, and 20.0 for 20, from YAML."""

from rcl_interfaces.msg import ParameterDescriptor

# Humble rejects an INTEGER override for a DOUBLE parameter and vice versa. Numbers are
# therefore declared with dynamic typing and normalized here. A list that mixes the two,
# such as [-1, 1.0], is still rejected by the YAML parser before any node code runs.


def _to_number(name, value, kind):
    if type(value) not in (int, float):
        raise ValueError(f"{name} must be a number, got {value!r}")
    if kind is int:
        if not float(value).is_integer():
            raise ValueError(f"{name} must be an integer, got {value!r}")
        return int(value)
    return float(value)


def declare_number(node, name, default):
    """Declare a read-only number, or a list of floats, and return its normalized value."""
    descriptor = ParameterDescriptor(read_only=True, dynamic_typing=True)
    value = node.declare_parameter(name, default, descriptor).value
    if isinstance(default, list):
        if isinstance(value, (str, bytes)) or not hasattr(value, "__iter__"):
            raise ValueError(f"{name} must be a list of numbers, got {value!r}")
        return [_to_number(name, item, float) for item in value]
    return _to_number(name, value, type(default))
