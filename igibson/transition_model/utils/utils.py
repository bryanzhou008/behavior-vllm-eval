def convert_windows_to_linux_path(path):
    path=path.replace('\v', '/v')
    path=path.replace('\t', '/t')
    path=path.replace('\b', '/b')
    path=path.replace('\a', '/a')
    return path.replace('\\', '/')