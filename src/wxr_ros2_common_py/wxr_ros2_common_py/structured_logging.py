from typing import Dict, Any, Iterable

def _format_params(params: Dict[str, Any]) -> str:
    if not params:
        return "(none)"
    return ", ".join(f"{k}={v}" for k, v in params.items())

def log_startup(node, params: Dict[str, Any], endpoints: Iterable[Dict[str, Any]]):
    """Print startup logs: params and endpoints with QoS details.
    endpoints: [{ 'dir': 'sub'|'pub', 'topic': str, 'type': str, 'qos': QoSProfile }]
    endpoints: [{ 'dir': 'service'|'client', 'service': str, 'type': str, 'qos': QoSProfile }]

    """
    node.get_logger().info(f"[Param]: {_format_params(params)}")
    for ep in endpoints:
        direction = ep.get('dir', '').lower()

        if direction == 'sub':
            tag = '[Sub]'
            topic = ep.get('topic', '')
        elif direction == 'pub':
            tag = '[Pub]'
            topic = ep.get('topic', '')
        elif direction == 'server':
            tag = '[Service-Server]'
            topic = ep.get('service', '')
        elif direction == 'client':
            tag = '[Service-Client]'
            topic = ep.get('service', '')

        #tag = '[Sub]' if direction == 'sub' else '[Pub]'
        #topic = ep.get('topic', '')

        mtype = ep.get('type', '')
        qos = ep.get('qos', None)
        from .qos_utils import qos_to_multiline
        node.get_logger().info(
            f"{tag}: {topic} [{mtype}] qos={qos_to_multiline(qos)}"
        )

def log_latency(node, label: str, start_time_sec: float, end_time_sec: float, file_handle=None):
    latency = end_time_sec - start_time_sec
    msg = f"[Latency] {label}: {latency:.6f} sec"
    node.get_logger().info(msg)
    if file_handle is not None:
        file_handle.write(msg + "\n")
        file_handle.flush()


