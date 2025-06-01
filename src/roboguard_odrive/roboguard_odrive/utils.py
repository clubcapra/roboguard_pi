from math import pi
from typing import Dict, List, Sized
from diagnostic_msgs.msg import KeyValue


def rad2rev(rad: float) -> float:
    return rad / (2 * pi)


def rev2rad(rev: float) -> float:
    return rev * 2 * pi


def verifyLengthMatch(*values: Sized) -> bool:
    return all([len(values[0]) == len(v) for v in values])


def dict2keyvalues(values: Dict[str, str]) -> List[KeyValue]:
    return list([KeyValue(key=k, value=v) for k, v in values.items()])


def yesno(value: bool) -> str:
    return 'yes' if value else 'no'
