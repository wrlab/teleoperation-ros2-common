class ActivityTracker:
    def __init__(self, node, interval_s: float = 1.0):
        self._node = node
        self._sub = set()
        self._pub = set()
        self._timer = node.create_timer(interval_s, self._tick)

    def mark_sub(self, topic: str):
        self._sub.add(topic)

    def mark_pub(self, topic: str):
        self._pub.add(topic)

    def _tick(self):
        if self._sub:
            topics = ', '.join(sorted(self._sub))
            self._node.get_logger().info(f"[Activity] sub: {topics}")
        if self._pub:
            topics = ', '.join(sorted(self._pub))
            self._node.get_logger().info(f"[Activity] pub: {topics}")
        self._sub.clear()
        self._pub.clear()


