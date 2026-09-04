"""パス解決ヘルパ（フルパスを書かずに済ませるための共通処理）."""
import os

from ament_index_python.packages import get_package_share_directory


def pkg_path(*parts, package='waypoint_tools'):
    """package の share ディレクトリ配下のパスを返す."""
    return os.path.join(get_package_share_directory(package), *parts)


def resolve_path(value, base_package='waypoint_tools'):
    """パス文字列を解決する。

    - ``pkg://<package>/<rel>``: その package の share ディレクトリ基準
    - ``~/...``: ホーム展開
    - 相対パス: ``base_package`` の share ディレクトリ基準
    - 絶対パス: そのまま
    - 空文字: そのまま
    """
    if not value:
        return value
    if value.startswith('pkg://'):
        package, _, rel = value[len('pkg://'):].partition('/')
        return os.path.join(get_package_share_directory(package), rel)
    value = os.path.expanduser(value)
    if os.path.isabs(value):
        return value
    return os.path.join(get_package_share_directory(base_package), value)
