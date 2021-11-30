from collections import deque
from itertools import islice

class CircularBuffer(deque):
    def __init__(self, capacity):
        super().__init__(maxlen=capacity)

    def __getitem__(self, index):
        if isinstance(index, slice):
            size = len(self)

            start = index.start or 0
            stop = index.stop or size
            if start < 0 or stop < 0:
                start = max(start + size, 0)
                stop = max(stop + size, 0)
            return islice(self, start, stop, index.step)
        elif isinstance(index, int):
            return super().__getitem__(index)
        else:
            raise TypeError("Invalid index")


