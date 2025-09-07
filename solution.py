from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any


@dataclass
class Solution:
    """全ソルバー共通の解表現。

    - routes: 車両ID -> ノード列（必ず [0, ..., 0] で始終）
    - objective_value: 目的関数値（距離最小なら総距離と同値でも良い）
    - total_distance: 総走行距離（objective_valueと同一でも可）
    - status: 'Optimal', 'Feasible', 'Infeasible' など
    - is_feasible: 実行可能性
    - runtime_s: 実行時間（秒）
    - solver_name: ソルバー名（例: 'MIP(CBC)', 'Savings'）
    - meta: 任意メタ情報（パラメータ等）
    """

    routes: Dict[int, List[int]]
    objective_value: float
    total_distance: Optional[float] = None
    status: str = "Unknown"
    is_feasible: bool = True
    runtime_s: Optional[float] = None
    solver_name: Optional[str] = None
    meta: Dict[str, Any] = field(default_factory=dict)

    # 後方互換: dict風アクセスを一部サポート
    def __getitem__(self, key: str):
        if key == "routes":
            return self.routes
        if key == "total_distance":
            return self.total_distance if self.total_distance is not None else self.objective_value
        if key == "objective_value":
            return self.objective_value
        if key == "status":
            return self.status
        if key == "runtime_s":
            return self.runtime_s
        if key == "solver_name":
            return self.solver_name
        raise KeyError(key)

    def num_vehicles_used(self) -> int:
        return sum(1 for r in self.routes.values() if len(r) > 2)

    def ratio_to(self, reference_value: float) -> Optional[float]:
        if reference_value and reference_value > 0:
            return self.objective_value / reference_value
        return None