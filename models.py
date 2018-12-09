def path_decompiler(lines):
    """
    decompiles a path from its geometry configuration into a pure list of tuples
    :param  lines:      list in geometric form according to osmnx 'geometry' feature
    :return new_path:   list of tuples
    """
    path = []
    for line in lines:
        for point in line:
            path.append(point)
    return path
