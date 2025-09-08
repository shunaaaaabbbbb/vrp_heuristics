from .vrp_solver import VRPSolver
import time
from typing import Set


class NNSolver(VRPSolver):
    """最近傍(Nearest Neighbor)で貪欲にルートを作る簡易ソルバー"""

    def solve(self):
        start = time.time()

        unvisited: Set[int] = {c.id for c in self.customers_with_depot}
        routes: dict[int, list[int]] = {}
        total_distance: float = 0.0

        # 各車両ごとにデポから開始して、積載が許す限り最近傍の顧客を追加
        for v in self.vehicles:
            capacity_left = v.capacity
            current = 0
            route: list[int] = [0]

            while True:
                # 追加可能な候補の中で最近傍
                candidates = [cid for cid in unvisited if self._demand(cid) <= capacity_left]
                if not candidates:
                    break
                next_id = min(candidates, key=lambda cid: self.distances[current, cid])

                # ルートに追加
                route.append(next_id)
                total_distance += self.distances[current, next_id]
                capacity_left -= self._demand(next_id)
                current = next_id
                unvisited.remove(next_id)

            # デポへ戻る
            total_distance += self.distances[current, 0]
            route.append(0)
            routes[v.id] = route

            if not unvisited:
                break

        runtime = time.time() - start
        status = "Feasible" if not unvisited else "Partial"
        sol = self._make_solution(routes, total_distance, status=status, runtime_s=runtime, solver_name="NN")
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
        print("=== VRP解（NN） ===")
        print(f"総移動距離: {self.solution.total_distance:.2f}")
        print(f"使用車両数: {self.solution.num_vehicles_used()}")
        print("\n各車両のルート:")
        for k, route in self.solution.routes.items():
            route_str = " -> ".join(map(str, route))
            print(f"車両{k}: {route_str}")