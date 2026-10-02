"""パス解決ヘルパ（フルパスを書かずに済ませるための共通処理）."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.frontend.parse_substitution import parse_substitution
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.parameter_descriptions import ParameterFile
import yaml


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

    - ``$(find-pkg-share <package>)/<rel>``: load_params() /
      expand_substs() で展開済みの絶対パスとして渡ってくる
    - ``~/...``: ホーム展開
    - 相対パス: waypoint_tools はソース、それ以外は share 基準
    - 絶対パス: そのまま
    - 空文字: そのまま
    """
    if not value:
        return value
    value = os.path.expanduser(value)
    if os.path.isabs(value):
        return value
    if base_package == 'waypoint_tools':
        return source_path(value)
    return os.path.join(get_package_share_directory(base_package), value)


def expand_substs(context, value):
    """launch 引数の文字列に含まれる ``$(find-pkg-share ...)`` などを展開する."""
    if not value:
        return value
    return perform_substitutions(context, parse_substitution(value))


def load_params(context, params_file):
    """params YAML を読み、waypoint_tools の ros__parameters を返す.

    ``$(find-pkg-share <package>)`` などの launch 置換を展開してから読む。
    """
    param_file = ParameterFile(params_file, allow_substs=True)
    try:
        with open(param_file.evaluate(context), 'r') as yaml_file:
            config = yaml.safe_load(yaml_file) or {}
    finally:
        param_file.cleanup()

    if 'waypoint_tools' in config:
        return config['waypoint_tools'].get('ros__parameters', {})
    return config.get('ros__parameters', config)


def as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).lower() in ('1', 'true', 'yes', 'on')


class LaunchParams:
    """params YAML の値を、同名の launch 引数による上書き込みで取り出す.

    既定値は持たない。launch 引数が空で、params YAML にキーが無い
    （または値が空）ならエラーにする。
    """

    def __init__(self, context, params_file):
        self._context = context
        self._params_file = params_file
        self._params = load_params(context, params_file)

    def raw(self, key):
        # 宣言していない launch 引数は「上書きなし」とみなす。
        override = expand_substs(
            self._context,
            LaunchConfiguration(key, default='').perform(self._context))
        if override:
            return override
        if key not in self._params:
            raise RuntimeError(
                f'{self._params_file}: parameter "{key}" is not set.')
        value = self._params[key]
        if value is None or value == '':
            raise RuntimeError(
                f'{self._params_file}: parameter "{key}" is empty.')
        return value

    def str(self, key):
        return str(self.raw(key))

    def path(self, key):
        return resolve_path(self.str(key))

    def bool(self, key):
        return as_bool(self.raw(key))

    def float(self, key):
        return self._convert(float, key)

    def int(self, key):
        return self._convert(int, key)

    def _convert(self, type_, key):
        value = self.raw(key)
        try:
            return type_(value)
        except (TypeError, ValueError):
            raise RuntimeError(
                f'parameter "{key}": {value!r} is not {type_.__name__}.')


def declare_overrides(keys):
    """params YAML のキーを上書きするための launch 引数（空 = 上書きなし）."""
    return [DeclareLaunchArgument(key, default_value='') for key in keys]
