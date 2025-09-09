def get_bool_param(node, name: str, default: bool = False) -> bool:
    node.declare_parameter(name, default)
    return node.get_parameter(name).value

def get_enum_param(node, name: str, allowed, default):
    node.declare_parameter(name, default)
    value = node.get_parameter(name).value
    if value not in allowed:
        node.get_logger().warn(f"Invalid {name}='{value}', falling back to '{default}'")
        return default
    return value


