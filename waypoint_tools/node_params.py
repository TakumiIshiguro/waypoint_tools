"""ノードのパラメータ宣言ヘルパ（既定値を持たせない）."""
from rclpy.exceptions import ParameterUninitializedException
from rclpy.parameter import Parameter

STRING = Parameter.Type.STRING
BOOL = Parameter.Type.BOOL
INTEGER = Parameter.Type.INTEGER
DOUBLE = Parameter.Type.DOUBLE


def require_parameters(node, specs):
    """specs ({名前: 型}) を既定値なしで宣言し、値の dict を返す.

    1 つでも未設定なら、未設定の名前をすべて挙げて RuntimeError を投げる。
    """
    node.declare_parameters('', list(specs.items()))
    values = {}
    missing = []
    for name in specs:
        try:
            values[name] = node.get_parameter(name).value
        except ParameterUninitializedException:
            missing.append(name)
    if missing:
        raise RuntimeError(
            f'{node.get_name()}: parameter not set: {", ".join(missing)} '
            '(config/params/waypoint_tools_params.yaml を確認してください)')
    return values
