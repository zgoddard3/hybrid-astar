from heapq import heappush, heappop

class PriorityQueue:
    def __init__(self):
        self._queue = []
        self._count = 0

    def push(self, priority, item):
        heappush(self._queue, (priority, self._count, item))
        self._count += 1
    
    def pop(self):
        return heappop(self._queue)[2]
    
    def __len__(self):
        return len(self._queue)

if __name__ == "__main__":

    print("Expected Output:\n")
    print("Second")
    print("First")
    print("Third")
    print("Fourth")

    queue = PriorityQueue()

    queue.push(3, "First")
    queue.push(1, "Second")
    queue.push(100, "Third")
    queue.push(100, "Fourth")

    print("\nOutput:\n")

    while queue:
        print(queue.pop())

    