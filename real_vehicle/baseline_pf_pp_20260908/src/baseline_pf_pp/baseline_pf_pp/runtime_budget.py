"""Fixed-configuration runtime: retain parameters, remove unused remote services."""

PARAMETER_SERVICES = ('describe_parameters', 'get_parameter_types', 'get_parameters',
                      'list_parameters', 'set_parameters', 'set_parameters_atomically')


def freeze_remote_parameters(node):
    """Only remove this node's standard parameter services, never map/services.

    CLI/YAML overrides have already been applied; local get_parameter remains
    available. Runtime reconfiguration is intentionally not used by this stack.
    """
    prefix=node.get_namespace().rstrip('/')+'/'+node.get_name()+'/'
    # Foxy stores the originally supplied (often relative) service name.
    expected={prefix+name for name in PARAMETER_SERVICES}
    expected.update(node.get_name()+'/'+name for name in PARAMETER_SERVICES)
    removed=[]
    for service in list(node.services):
        if service.srv_name in expected:
            name=service.srv_name
            if not node.destroy_service(service):raise RuntimeError('Cannot freeze '+name)
            removed.append(name)
    return removed
