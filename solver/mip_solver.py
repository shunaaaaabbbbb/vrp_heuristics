import pulp
import time
from .vrp_solver import VRPSolver


class VRPSolverMIP(VRPSolver):
    """MTZ制約を用いたVRPのMIPソルバー"""
    
    def model_mtz(self):
        """MTZ制約を用いてVRPをMIPで解く"""
        self.model = pulp.LpProblem("VRP_MTZ", pulp.LpMinimize)
        
        # 決定変数
        # x[i,j,k] = 1 if 車両kが地点iから地点jに移動する
        self.x = pulp.LpVariable.dicts(
            "x",
            ((i.id, j.id, k_id) for i in self.customers_with_depot for j in self.customers_with_depot for k_id in self.vehicle_ids),
            cat="Binary",
        )
        
        # u[i] = MTZ制約用の変数（地点iの訪問順序を表す）
        self.u = pulp.LpVariable.dicts(
            "u",
            (i.id for i in self.customers),
            lowBound=0, upBound=self.capacity, cat="Continuous"
        )
        
        # 目的関数：総移動距離の最小化
        self.model += pulp.lpSum([
            self.distances[i.id, j.id] * self.x[i.id, j.id, k_id]
            for i in self.customers_with_depot for j in self.customers_with_depot for k_id in self.vehicle_ids
            if i.id != j.id
        ])
        
        # 制約1：各顧客はちょうど1台の車両によって訪問される
        for customer in self.customers:
            self.model += pulp.lpSum([
                self.x[i.id, customer.id, k_id]
                for i in self.customers_with_depot for k_id in self.vehicle_ids
                if i.id != customer.id
            ]) == 1
        
        # 制約2：各車両はデポから出発し、デポに戻る
        for k_id in self.vehicle_ids:
            # デポからの出発
            self.model += pulp.lpSum([self.x[0, j.id, k_id] for j in self.customers_with_depot]) == 1
            # デポへの帰還
            self.model += pulp.lpSum([self.x[i.id, 0, k_id] for i in self.customers_with_depot]) == 1
        
        # 制約3：フロー保存制約（各地点での入出のバランス）
        for customer in self.customers_with_depot:
            for k_id in self.vehicle_ids:
                self.model += (
                    pulp.lpSum([self.x[i.id, customer.id, k_id] for i in self.customers_with_depot if i.id != customer.id])
                    ==
                    pulp.lpSum([self.x[customer.id, j.id, k_id] for j in self.customers_with_depot if j.id != customer.id])
                )
        
        # 制約4：MTZ制約（部分巡回路除去 + 容量制約）
        for i in self.customers:
            for j in self.customers:
                if i.id != j.id:
                    self.model += self.u[i.id] - self.u[j.id] + self.capacity * pulp.lpSum([self.x[i.id, j.id, k_id] for k_id in self.vehicle_ids]) <= self.capacity - j.demand
        # 容量の下限（需要以上）
        for i in self.customers:
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
        if self.model.status == pulp.LpStatusOptimal:
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

    def _extract_routes_and_distance(self) -> tuple[dict[int, list[int]], float]:
        """モデルのxからルートと総距離を抽出"""
        routes: dict[int, list[int]] = {}
        total_distance = 0.0
        for k_id in self.vehicle_ids:
            route: list[int] = [0]
            current = 0
            while True:
                next_customer = None
                for customer in self.customers_with_depot:
                    key = (current, customer.id, k_id)
                    if key in self.x and self.x[key].varValue == 1:
                        next_customer = customer
                        break
                if next_customer is None:
                    break
                if next_customer.id != 0:
                    route.append(next_customer.id)
                total_distance += self.distances[current, next_customer.id]
                current = next_customer.id
                if current == 0:
                    break
            if route:
                routes[k_id] = route
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
