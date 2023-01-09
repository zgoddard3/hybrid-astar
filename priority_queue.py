from heapq import heappush, heappop

class PriorityQueue:
    def __init__(self, key_fn=lambda x : x):
        self._queue = []
        self._count = 0
        self._key_fn = key_fn
        self._lookup = dict()

    def push(self, priority, value):
        if self._key_fn(value) in self._lookup:
            self.remove(value)
        item = [priority, self._count, value]
        heappush(self._queue, item)
        self._lookup[self._key_fn(value)] = item
        self._count += 1
    
    def pop(self):
        while self:
            value = heappop(self._queue)[2]
            if value is not None:
                del self._lookup[self._key_fn(value)]
                return value
        raise KeyError("Priority queue is empty!")
    
    def remove(self, value):
        item = self._lookup.pop(self._key_fn(value))
        item[2] = None 
    
    def __len__(self):
        return len(self._lookup)
    
    def clear(self):
        self._queue.clear()
        self._count = 0
        self._lookup.clear()

if __name__ == "__main__":

    print("Expected Output:\n")
    print("Second")
    print("Fifth")
    print("First")
    print("Third")
    print("Fourth")

    queue = PriorityQueue()

    queue.push(3, "First")
    queue.push(1, "Second")
    queue.push(100, "Third")
    queue.push(100, "Fourth")
    queue.push(4, "Fifth")
    queue.push(2, "Fifth")

    print("\nOutput:\n")

    while queue:
        print(queue.pop())

    