class CircularBuffer:
    def __init__(self, capacity):
        self._index = 0
        self._capacity = capacity
        self._data = [None] * self._capacity

    def push(self, x):
        self._data[self._index] = x
        self._index += 1

    def __getitem__(self, key):
        if isinstance(key, slice):
            if key.start >= 0 or key.stop >= 0 or key.start < -self._capacity < -self._capacity:
                raise IndexError("Index out of bounds")

            return [self._data[self._index + i] for i in range(*key.indices(self._capacity))]

        elif isinstance(key, int):
            # gets the x-th item from the back
            if key >= 0 or key < -self._capacity:
                raise IndexError("Index out of bounds")
            index = (self._index + key) % self._capacity
            return self._data[index]
        else:
            raise TypeError(f"Incompatible type for key")
