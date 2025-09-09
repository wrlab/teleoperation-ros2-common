def qos_to_multiline(qos_profile) -> str:
    try:
        def _name(value):
            return getattr(value, 'name', str(value))
        lines = []
        if hasattr(qos_profile, 'history'):
            lines.append(f"\n    history: {_name(qos_profile.history)}")
        if hasattr(qos_profile, 'depth'):
            lines.append(f"\n    depth: {getattr(qos_profile, 'depth', '')}")
        if hasattr(qos_profile, 'reliability'):
            lines.append(f"\n    reliability: {_name(qos_profile.reliability)}")
        if hasattr(qos_profile, 'durability'):
            lines.append(f"\n    durability: {_name(qos_profile.durability)}")
        if hasattr(qos_profile, 'liveliness'):
            lines.append(f"\n    liveliness: {_name(qos_profile.liveliness)}")
        return ''.join(lines) if lines else "\n    (no qos fields)"
    except Exception:
        return "\n    (error printing qos)"


