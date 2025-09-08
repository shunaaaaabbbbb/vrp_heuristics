from .vrp_solver import VRPSolver
from instance import Customer
import math
import time


class SweepSolver(VRPSolver):
    """Sweep法でVRPを解くクラス"""
    def solve(self):
        """Sweep法でVRPを解く（角度順に1顧客ずつ車両に割当、容量超で次車両）"""
        start = time.time()

        sorted_customers = self._sorted_customers_by_angle()
        routes, total_distance = self._build_routes_from_sorted(sorted_customers)

        runtime = time.time() - start
        status = "Feasible" if len(sorted_customers) == sum(len(r) - 2 for r in routes.values()) else "Partial"
        sol = self._make_solution(routes, total_distance, status=status, runtime_s=runtime, solver_name="Sweep")
        self.solution = sol
        return sol
    
    def _demand(self, cid: int) -> float:
        # 顧客ID -> 需要
        # Vは [depot(0)] + customers(1..N) なので、idに一致するものを引く
        # 規模が小さいため線形検索で充分
        for c in self.customers_with_depot:
            if c.id == cid:
                return c.demand
        return 0.0
    
    def print_solution(self):
        if self.solution is None:
            print("解がありません。")
            return
        print("=== VRP解（Sweep） ===")
        print(f"総移動距離: {self.solution.total_distance:.2f}")
        print(f"使用車両数: {self.solution.num_vehicles_used()}")
        print("\n各車両のルート:")
        for k, route in self.solution.routes.items():
            route_str = " -> ".join(map(str, route))
            print(f"車両{k}: {route_str}")
    
    def compute_angle_from_depot(self, customer: Customer) -> float:
        """デポから顧客への角度を0～2πの間で計算"""
        return math.atan2(customer.y - self.instance.depot.y, customer.x - self.instance.depot.x) % (2 * math.pi)

    # ===== 内部ヘルパー =====
    def _sorted_customers_by_angle(self) -> list[Customer]:
        """顧客のみを角度で昇順にソートして返す"""
        customers = self.customers_with_depot
        customers.sort(key=lambda c: self.compute_angle_from_depot(c))
        return customers

    def _build_routes_from_sorted(self, sorted_customers: list[Customer]) -> tuple[dict[int, list[int]], float]:
        """角度順顧客列から容量制約を守って順に割当て、ルートと距離を返す"""
        routes: dict[int, list[int]] = {}
        total_distance: float = 0.0

        vehicle_index = 0
        customer_index = 0
        num_customers = len(sorted_customers)

        while customer_index < num_customers and vehicle_index < len(self.vehicles):
            vehicle = self.vehicles[vehicle_index]
            capacity_left = vehicle.capacity
            current_node = 0
            route: list[int] = [0]

            # 現在の車両に入るだけ詰める
            while customer_index < num_customers:
                customer = sorted_customers[customer_index]
                if self._demand(customer.id) <= capacity_left:
                    # 追加
                    route.append(customer.id)
                    total_distance += self.distances[current_node, customer.id]
                    capacity_left -= self._demand(customer.id)
                    current_node = customer.id
                    customer_index += 1
                else:
                    # 次の車両へ交代
                    break

            # ルートをクローズ
            total_distance += self.distances[current_node, 0]
            route.append(0)
            routes[vehicle.id] = route

            vehicle_index += 1

        return routes, total_distance

class SweepNearestSolver(SweepSolver):
    """Sweep法を角度起点 + 最近傍で改良"""

    def solve(self):
        """Sweep法でVRPを解く（角度順に1顧客ずつ車両に割当、容量超で次車両）最近傍で改良"""
        start = time.time()

        sorted_customers = self._sorted_customers_by_angle()
        routes, total_distance = self._build_routes_from_sorted(sorted_customers)

        runtime = time.time() - start
        status = "Feasible" if len(sorted_customers) == sum(len(r) - 2 for r in routes.values()) else "Partial"
        sol = self._make_solution(routes, total_distance, status=status, runtime_s=runtime, solver_name="SweepNearest")
        self.solution = sol
        return sol
    # 角度順の候補列は顧客のみ（デポ除外）
    def _sorted_customers_by_angle(self) -> list[Customer]:
        return sorted(self.customers_with_depot, key=lambda c: self.compute_angle_from_depot(c))

    def _build_routes_from_sorted(self, sorted_customers: list[Customer]) -> tuple[dict[int, list[int]], float]:
        """各車両: 開始は角度最小の未訪問、以降は最近傍で容量限界まで追加"""
        routes: dict[int, list[int]] = {}
        total_distance: float = 0.0

        # 未訪問集合（ID）と補助マップ
        unvisited: set[int] = {c.id for c in self.customers_with_depot}
        angle_of: dict[int, float] = {c.id: self.compute_angle_from_depot(c) for c in self.customers_with_depot}

        vehicle_index = 0
        while unvisited and vehicle_index < len(self.vehicles):
            v = self.vehicles[vehicle_index]
            capacity_left = v.capacity
            current_id = 0
            route: list[int] = [0]

            # ルート開始: 角度が最小の未訪問を選ぶ（容量に入るもの）
            start_id = self._select_start_customer(unvisited, angle_of, capacity_left)
            if start_id is None:
                # どれも容量に入らない
                routes[v.id] = [0, 0]
                vehicle_index += 1
                continue

            # 追加
            route.append(start_id)
            total_distance += self.distances[current_id, start_id]
            capacity_left -= self._demand(start_id)
            current_id = start_id
            unvisited.remove(start_id)

            # 以降は最近傍で埋める
            while True:
                next_id = self._select_nearest_feasible(current_id, unvisited, capacity_left)
                if next_id is None:
                    break
                route.append(next_id)
                total_distance += self.distances[current_id, next_id]
                capacity_left -= self._demand(next_id)
                current_id = next_id
                unvisited.remove(next_id)

            # クローズ
            total_distance += self.distances[current_id, 0]
            route.append(0)
            routes[v.id] = route
            vehicle_index += 1

        return routes, total_distance

    # ===== NN用の補助関数 =====
    def _select_start_customer(self, unvisited: set[int], angle_of: dict[int, float], capacity_left: float) -> int | None:
        """未訪問のうち角度が最小で容量に入る顧客IDを返す"""
        candidates = [cid for cid in unvisited if self._demand(cid) <= capacity_left]
        if not candidates:
            return None
        return min(candidates, key=lambda cid: angle_of[cid])

    def _select_nearest_feasible(self, current_id: int, unvisited: set[int], capacity_left: float) -> int | None:
        """現在位置から最近傍で容量に入る未訪問の顧客IDを返す"""
        candidates = [cid for cid in unvisited if self._demand(cid) <= capacity_left]
        if not candidates:
            return None
        return min(candidates, key=lambda cid: self.distances[current_id, cid])