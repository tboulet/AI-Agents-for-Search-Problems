from typing import Callable


def manhattan_distance(a, b):
    """
    Manhattan distance between two points.
    """
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def counted(f : Callable) -> Callable:
    def wrapped(*args, **kwargs):
        wrapped.calls += 1
        return f(*args, **kwargs)
    
    wrapped.calls = 0
    return wrapped 