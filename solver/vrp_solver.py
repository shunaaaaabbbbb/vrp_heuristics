from instance import Instance, Customer, Vehicle
from solution import Solution

class VRPSolver:
    """VRPを解く基底クラス"""
    def __init__(self, instance: Instance):
        self.instance = instance
        self.customers_with_depot: list[Customer] = instance.customers_with_depot  # デポを含む全地点
        self.customers: list[Customer] = instance.customers  # 顧客のみ
        self.distances = instance.distances  # NumPy距離行列 (shape: (N+1, N+1))
        self.capacity: int = instance.capacity  # 車両容量
        self.vehicles: list[Vehicle] = instance.vehicles  # 車両オブジェクトのリスト
        self.vehicle_ids: list[int] = [v.id for v in self.vehicles]  # ソルバー用の車両ID
        self.solution: Solution|None = None
    
    def solve(self):
        """VRPを解く(抽象メソッド)"""
        pass

    def _make_solution(self, routes: dict[int, list[int]], total_distance: float, *, status: str, runtime_s: float, solver_name: str) -> Solution:
        """共通のSolution組み立てヘルパー"""
        return Solution(
            routes=routes,
            objective_value=total_distance,
            total_distance=total_distance,
            status=status,
            runtime_s=runtime_s,
            solver_name=solver_name,
        )
