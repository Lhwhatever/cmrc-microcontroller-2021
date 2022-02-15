class CircularBuffer:

    def __init__(self, capacity):
        self._capacity = capacity
        self._index = 0
        self._data = [None] * self._capacity
        self._length = 0

    def append(self, entry):
        self._data[self._index] = entry
        self._index = (self._index + 1) % self._capacity
        self._length = min(self._length + 1, self._capacity)

    def __len__(self):
        return self._length

    def __getitem__(self, index):
        if isinstance(index, slice):
            size = len(self)
            start = max(index.start or -size, -size)
            stop = min(index.stop or 0, 0)

            if start > 0 or stop > 0:
                raise IndexError("cannot index circular buffer with positive indices")

            i0 = (self._index + start) % self._capacity
            i1 = (self._index + stop) % self._capacity

            if start == stop or i0 < i1:
                return self._data[i0:i1]
            else:
                return self._data[i0:] + self._data[:i1]

        elif isinstance(index, int):
            if index >= 0 or index < -len(self):
                raise IndexError("circular buffer index out of range")
            else:
                return self._data[(len(self) + index) % self._capacity]
        else:
            raise TypeError("Invalid index")
