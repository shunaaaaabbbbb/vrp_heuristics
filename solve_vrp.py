import pulp
import time
from instance import Instance, Customer
from typing import List, Dict, Tuple, Optional
from solution import Solution

class VRPSolver:
    """VRPを解く基底クラス"""
    def __init__(self, instance: Instance):
        self.instance = instance
        self.V: List[Customer] = instance.customers_with_depot  # デポを含む全地点
        self.V_0: List[Customer] = instance.customers  # 顧客のみ
        self.D: Dict[Tuple[int, int], float] = instance.distances  # 距離行列
        self.Q: int = instance.capacity  # 車両容量
        self.K: List[int] = instance.vehicles  # 車両リスト
        self.solution: Optional[Solution] = None
    
    def solve(self):
        """VRPを解く(抽象メソッド)"""
        pass

    def _make_solution(self, routes: Dict[int, List[int]], total_distance: float, *, status: str, runtime_s: float, solver_name: str) -> Solution:
        """共通のSolution組み立てヘルパー"""
        return Solution(
            routes=routes,
            objective_value=total_distance,
            total_distance=total_distance,
            status=status,
            runtime_s=runtime_s,
            solver_name=solver_name,
        )


class VRPSolverMIP(VRPSolver):
    """MTZ制約を用いたVRPのMIPソルバー"""
    
    def model_mtz(self):
        """MTZ制約を用いてVRPをMIPで解く"""
        self.model = pulp.LpProblem("VRP_MTZ", pulp.LpMinimize)
        
        # 決定変数
        # x[i,j,k] = 1 if 車両kが地点iから地点jに移動する
        self.x = pulp.LpVariable.dicts(
            "x",
            ((i.id, j.id, k) for i in self.V for j in self.V for k in self.K),
            cat="Binary",
        )
        
        # u[i] = MTZ制約用の変数（地点iの訪問順序を表す）
        self.u = pulp.LpVariable.dicts(
            "u",
            (i.id for i in self.V_0),
            lowBound=0, upBound=self.Q, cat="Continuous"
        )
        
        # 目的関数：総移動距離の最小化
        self.model += pulp.lpSum([self.D[(i.id, j.id)] * self.x[i.id, j.id, k] 
                           for i in self.V for j in self.V for k in self.K 
                           if i.id != j.id])
        
        # 制約1：各顧客はちょうど1台の車両によって訪問される
        for customer in self.V_0:
            self.model += pulp.lpSum([self.x[i.id, customer.id, k] 
                               for i in self.V for k in self.K 
                               if i.id != customer.id]) == 1
        
        # 制約2：各車両はデポから出発し、デポに戻る
        for k in self.K:
            # デポからの出発
            self.model += pulp.lpSum([self.x[0, j.id, k] for j in self.V_0]) == 1
            # デポへの帰還
            self.model += pulp.lpSum([self.x[i.id, 0, k] for i in self.V_0]) == 1
        
        # 制約3：フロー保存制約（各地点での入出のバランス）
        for customer in self.V_0:
            for k in self.K:
                self.model += (
                    pulp.lpSum([self.x[i.id, customer.id, k] for i in self.V if i.id != customer.id])
                    ==
                    pulp.lpSum([self.x[customer.id, j.id, k] for j in self.V if j.id != customer.id])
                )
        
        # 制約4：MTZ制約（部分巡回路除去 + 容量制約）
        for i in self.V_0:
            for j in self.V_0:
                if i.id != j.id:
                    self.model += self.u[i.id] - self.u[j.id] + self.Q * pulp.lpSum([self.x[i.id, j.id, k] for k in self.K]) <= self.Q - j.demand
        # 容量の下限（需要以上）
        for i in self.V_0:
            self.model += self.u[i.id] >= i.demand
    def solve(self):
        # モデル未構築なら構築
        if not hasattr(self, "model"):
            self.model_mtz()
        # ソルバー実行
        start_time = time.time()
        self.model.solve(pulp.PULP_CBC_CMD(msg=0))
        runtime_s = time.time() - start_time
        status_str = pulp.LpStatus[self.model.status]
        
        # 解の解析
        if self.model.status in (pulp.LpStatusOptimal, pulp.LpStatusNotSolved, pulp.LpStatusFeasible):
            routes, total_distance = self._extract_routes_and_distance()
            try:
                obj_val = float(pulp.value(self.model.objective))
            except Exception:
                obj_val = total_distance
            total_distance = obj_val if obj_val is not None else total_distance
            sol = self._make_solution(routes, total_distance, status=status_str, runtime_s=runtime_s, solver_name="MIP(CBC)")
            self.solution = sol
            return sol
        else:
            raise Exception(f"最適解が見つかりませんでした。ステータス: {status_str}")

    def _extract_routes_and_distance(self) -> Tuple[Dict[int, List[int]], float]:
        """モデルのxからルートと総距離を抽出"""
        routes: Dict[int, List[int]] = {}
        total_distance = 0.0
        for k in self.K:
            route: List[int] = [0]
            current = 0
            while True:
                next_customer = None
                for customer in self.V:
                    key = (current, customer.id, k)
                    if key in self.x and self.x[key].varValue == 1:
                        next_customer = customer
                        break
                if next_customer is None:
                    break
                if next_customer.id != 0:
                    route.append(next_customer.id)
                total_distance += self.D[(current, next_customer.id)]
                current = next_customer.id
                if current == 0:
                    break
            if route:
                routes[k] = route
            route.append(0)
        return routes, total_distance

    def print_solution(self):
        """解を表示"""
        if self.solution is None:
            print("解がありません。")
            return
        
        print("=== VRP解（MTZ制約） ===")
        print(f"総移動距離: {self.solution.total_distance:.2f}")
        print(f"使用車両数: {self.solution.num_vehicles_used()}")
        print("\n各車両のルート:")
        
        for k, route in self.solution.routes.items():
            route_str = " -> ".join(map(str, route))
            print(f"車両{k}: {route_str}")


# テスト用のメイン関数
if __name__ == "__main__":
    from instance import Instance
    import random
    random.seed(42)
    # 小さな問題例でテスト
    print("VRP問題例を作成中...")
    instance = Instance(num_customers=8, num_vehicles=2, capacity=20)
    
    print(f"顧客数: {instance.num_customers}")
    print(f"車両数: {instance.num_vehicles}")
    print(f"車両容量: {instance.capacity}")
    print(f"顧客の需要: {[c.demand for c in instance.customers]}")
    
    print("\nMTZ制約を用いたVRPソルバーで解を求めています...")
    solver = VRPSolverMIP(instance)
    solution = solver.solve()
    print(solution.routes)
    
    if solution:
        solver.print_solution()
    else:
        print("解が見つかりませんでした。")

    import matplotlib.pyplot as plt
    plt.scatter([c.x for c in instance.customers_with_depot], [c.y for c in instance.customers_with_depot])
    for i in range(len(instance.customers_with_depot)):
        plt.annotate(f"{instance.customers_with_depot[i].id}", (instance.customers_with_depot[i].x, instance.customers_with_depot[i].y))
    for k, route in solution.routes.items():
        plt.plot([instance.customers_with_depot[i].x for i in route], [instance.customers_with_depot[i].y for i in route], label=f"車両{k}")
    plt.legend()
    plt.show()