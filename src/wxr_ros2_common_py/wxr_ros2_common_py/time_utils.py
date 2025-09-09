def now_sec_nsec(node):
    now = node.get_clock().now()
    return now.seconds_nanoseconds()

def now_sec_of_day(node):
    sec, nsec = now_sec_nsec(node)
    sec_of_day = sec % 86400
    return sec_of_day, nsec

def stamp_to_float(sec: int, nsec: int) -> float:
    return float(sec) + float(nsec) / 1e9


