from math import pi
from typing import Sized, TypeVar

Revs = float
Radians = float
Meters = float

def rad2rev(rad: Radians) -> Revs:
    return rad / (2 * pi)


def rev2rad(rev: Revs) -> Radians:
    return rev * 2 * pi


def verifyLengthMatch(*values: Sized) -> bool:
    return all([len(values[0]) == len(v) for v in values])

_T = TypeVar('_T', int, float, complex)

def clamp(minVal: _T, maxVal: _T, val: _T) -> _T:
    return max(minVal, min(maxVal, val))

def sign(val: _T) -> _T:
    return -1 if val < 0 else 1

def bounce(minValue: float, maxValue: float, fac: float) -> float:
    m = fac % 1
    f = 1 - (abs(m - 0.5) * 2.0)
    return minValue + (maxValue - minValue) * f

def niceFloat(value: float, numDigits: int = 3, just:int = 6) -> str:
    return str(round(value, numDigits)).ljust(just)