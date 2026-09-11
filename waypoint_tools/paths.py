"""パス解決ヘルパ（フルパスを書かずに済ませるための共通処理）."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory


def pkg_path(*parts, package='waypoint_tools'):
    """package の share ディレクトリ配下のパスを返す."""
    return os.path.join(get_package_share_directory(package), *parts)


def source_path(*parts):
    """waypoint_tools のソース配下を返す（install へは保存しない）."""
    # ソースからの実行と colcon --symlink-install に対応する。
    for parent in Path(__file__).resolve().parents:
        if (parent / 'package.xml').is_file() and (parent / 'config').is_dir():
            return str(parent.joinpath(*parts))

    # 通常の install と --merge-install の両方で workspace/src を探す。
    share = Path(get_package_share_directory('waypoint_tools'))
    for parent in share.parents:
        candidate = parent / 'src' / 'waypoint_tools'
        if (candidate / 'package.xml').is_file():
            return str(candidate.joinpath(*parts))
    raise FileNotFoundError(
        'waypoint_tools のソースが見つかりません。'
        'workspace/src/waypoint_tools を配置するか、絶対パスを指定してください。')


def resolve_path(value, base_package='waypoint_tools'):
    """パス文字列を解決する。

    - ``pkg://<package>/<rel>``: その package の share ディレクトリ基準
    - ``~/...``: ホーム展開
    - 相対パス: waypoint_tools はソース、それ以外は share 基準
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
    if base_package == 'waypoint_tools':
        return source_path(value)
    return os.path.join(get_package_share_directory(base_package), value)
