import math
from typing import List

class Customer:
    def __init__(self, x: float, y: float, demand: float, idx: int):
        self.x = x
        self.y = y
        self.demand = demand
        self.idx = idx

class Depot:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

def polar_angle(depot: Depot, customer: Customer) -> float:
    dx = customer.x - depot.x
    dy = customer.y - depot.y
    return math.atan2(dy, dx) % (2 * math.pi)  # 0〜2πに正規化

def distance(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])

def sweep_algorithm(customers: List[Customer], depot: Depot, capacity: float):
    # 角度ソート
    sorted_customers = sorted(customers, key=lambda c: polar_angle(depot, c))

    solution = []
    route = []
    load = 0

    for c in sorted_customers:
        if load + c.demand <= capacity:
            route.append(c)
            load += c.demand
        else:
            solution.append(route)
            route = [c]
            load = c.demand
    if route:
        solution.append(route)

    return solution
def route_distance(route: List[Customer], depot: Depot) -> float:
    """デポを含むルート距離"""
    dist = 0.0
    prev = (depot.x, depot.y)
    for c in route:
        dist += math.hypot(prev[0] - c.x, prev[1] - c.y)
        prev = (c.x, c.y)
    dist += math.hypot(prev[0] - depot.x, prev[1] - depot.y)
    return dist

def two_opt(route: List[Customer], depot: Depot) -> List[Customer]:
    """2-optでルート改善"""
    best = route[:]
    improved = True
    while improved:
        improved = False
        for i in range(1, len(best) - 1):
            for j in range(i + 1, len(best)):
                if j - i == 1:
                    continue
                new_route = best[:]
                new_route[i:j] = best[j - 1:i - 1:-1]
                if route_distance(new_route, depot) < route_distance(best, depot):
                    best = new_route
                    improved = True
        route = best
    return best

def sweep_nearest(customers: List[Customer], depot: Depot, capacity: float):
    # Step 0–1: 顧客を角度でソート
    sorted_customers = sorted(customers, key=lambda c: polar_angle(depot, c))
    unrouted = set(sorted_customers)
    solution = []

    while unrouted:
        # Step 2: 未割当の中で最小角度の顧客
        current = min(unrouted, key=lambda c: polar_angle(depot, c))
        route = [current]
        load = current.demand
        unrouted.remove(current)

        # Step 3: Nearest Neighborで拡張
        while True:
            candidates = [c for c in unrouted if load + c.demand <= capacity]
            if not candidates:
                break
            next_cust = min(
                candidates,
                key=lambda c: distance((route[-1].x, route[-1].y), (c.x, c.y))
            )
            route.append(next_cust)
            load += next_cust.demand
            unrouted.remove(next_cust)

        # Step 5: 2-optで改善
        route = two_opt(route, depot)
        solution.append(route)

    return solution

import math
import random
from typing import List, Tuple

class Customer:
    def __init__(self, x: float, y: float, demand: int, idx: int):
        self.x = x
        self.y = y
        self.demand = demand
        self.idx = idx

class Depot:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

def distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])

def route_distance(route: List[Customer], depot: Depot) -> float:
    dist = 0.0
    prev = (depot.x, depot.y)
    for c in route:
        dist += distance(prev, (c.x, c.y))
        prev = (c.x, c.y)
    dist += distance(prev, (depot.x, depot.y))
    return dist

def total_cost(solution: List[List[Customer]], depot: Depot, capacity: int) -> float:
    cost = 0.0
    penalty = 0.0
    for route in solution:
        cost += route_distance(route, depot)
        load = sum(c.demand for c in route)
        if load > capacity:
            penalty += (load - capacity) * 1000  # 超過需要には大きな罰則
    return cost + penalty

def initial_solution(customers: List[Customer], capacity: int) -> List[List[Customer]]:
    solution = []
    route = []
    load = 0
    for c in customers:
        if load + c.demand <= capacity:
            route.append(c)
            load += c.demand
        else:
            solution.append(route)
            route = [c]
            load = c.demand
    if route:
        solution.append(route)
    return solution

def neighbor(solution: List[List[Customer]]) -> List[List[Customer]]:
    new_solution = [r[:] for r in solution]  # deepcopy
    if random.random() < 0.5:
        # 顧客を別ルートへ移動
        if len(new_solution) >= 2:
            r1, r2 = random.sample(range(len(new_solution)), 2)
            if new_solution[r1]:
                c = random.choice(new_solution[r1])
                new_solution[r1].remove(c)
                pos = random.randint(0, len(new_solution[r2]))
                new_solution[r2].insert(pos, c)
    else:
        # 同じルート内で顧客を入れ替え
        r = random.choice(new_solution)
        if len(r) >= 2:
            i, j = random.sample(range(len(r)), 2)
            r[i], r[j] = r[j], r[i]
    return new_solution

def simulated_annealing(customers: List[Customer], depot: Depot, capacity: int,
                        T=1000, alpha=0.995, iter_per_temp=100):
    current = initial_solution(customers, capacity)
    best = current
    best_cost = total_cost(best, depot, capacity)

    while T > 1e-3:
        for _ in range(iter_per_temp):
            candidate = neighbor(current)
            delta = total_cost(candidate, depot, capacity) - total_cost(current, depot, capacity)
            if delta < 0 or random.random() < math.exp(-delta / T):
                current = candidate
                if total_cost(current, depot, capacity) < best_cost:
                    best = current
                    best_cost = total_cost(best, depot, capacity)
        T *= alpha
    return best, best_cost


import matplotlib.pyplot as plt

def plot_solution(solution, depot: Depot):
    colors = plt.cm.tab10.colors  # 10色のカラーマップ
    
    plt.figure(figsize=(7, 7))
    # デポを赤い★で描画
    plt.scatter(depot.x, depot.y, c="red", marker="*", s=200, label="Depot")
    
    for i, route in enumerate(solution):
        xs = [depot.x] + [c.x for c in route] + [depot.x]
        ys = [depot.y] + [c.y for c in route] + [depot.y]
        
        plt.plot(xs, ys, marker="o", color=colors[i % len(colors)], label=f"Route {i+1}")
        for c in route:
            plt.text(c.x+0.1, c.y+0.1, str(c.idx))  # 顧客番号を表示
    
    plt.title("Sweep Algorithm Routes")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.show()


import random
random.seed(42)
def make_instance(num_customers: int, capacity: int):
    depot = Depot(0, 0)
    customers = [Customer(random.uniform(-10, 10), random.uniform(-10, 10), random.uniform(1, 10), i) for i in range(1, num_customers+1)]
    return customers, depot, capacity

customers, depot, capacity = make_instance(100, 100)
solution_sa, cost_sa = simulated_annealing(customers, depot, capacity)

print(f"SA cost: {cost_sa}")

plot_solution(solution_sa, depot)
