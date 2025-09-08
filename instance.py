import random
import numpy as np
import matplotlib.pyplot as plt

random.seed(42)


class Customer:
    """顧客を表すクラス"""
    def __init__(self, id: int, demand: float, x: float, y: float):
        self.id = id
        self.demand = demand
        self.x = x
        self.y = y


class Vehicle:
    """車両を表すクラス"""
    def __init__(self, id: int, capacity: float):
        self.id = id
        self.capacity = capacity
        self.remaining_capacity = capacity
        self.route = [0]  # デポから開始

    def add_customer(self, customer: Customer):
        """顧客をルートに追加（容量チェック付き）"""
        if self.remaining_capacity >= customer.demand:
            self.route.append(customer.id)
            self.remaining_capacity -= customer.demand
            return True
        return False

    def close_route(self):
        """ルートをデポに戻して閉じる"""
        if self.route[-1] != 0:
            self.route.append(0)


class Instance:
    """問題例を表すクラス"""
    def __init__(self, num_customers: int, num_vehicles: int, capacity: int):
        self.num_customers = num_customers
        self.num_vehicles = num_vehicles
        self.capacity = capacity

        # 顧客を生成（IDは1..N、デポはID=0）
        self.customers = self.create_customers(num_customers)
        self.depot = Customer(0, 0, 0, 0)
        self.customers_with_depot = [self.depot] + self.customers

        # 車両を生成（Vehicleオブジェクトのリスト）
        self.vehicles = self.create_vehicles(num_vehicles)

        # 距離行列（NumPy配列）を計算
        self.distances = self.compute_distances()

    def create_customers(self, num_customers: int) -> list[Customer]:
        return [Customer(i, random.uniform(1, 10), random.uniform(-10, 10), random.uniform(-10, 10)) for i in range(1, num_customers + 1)]
    
    def create_vehicles(self, num_vehicles: int) -> list[Vehicle]:
        return [Vehicle(i, self.capacity) for i in range(1, num_vehicles + 1)]
    
    def compute_distances(self) -> np.ndarray:
        """任意の2点間の距離を計算してNumPy配列に格納"""
        n = self.num_customers + 1
        distances = np.zeros((n, n))
        for i in range(n):
            for j in range(n):
                distances[i, j] = self.distance(self.customers_with_depot[i], self.customers_with_depot[j])
        return distances

    def distance(self, customer1: Customer, customer2: Customer) -> float:
        """2点間のユークリッド距離を計算"""
        return ((customer1.x - customer2.x) ** 2 + (customer1.y - customer2.y) ** 2) ** 0.5

    def plot_instance(self):
        """顧客とデポを可視化"""
        xs = [c.x for c in self.customers]
        ys = [c.y for c in self.customers]
        plt.scatter(xs, ys, c="blue", label="Customers")
        plt.scatter([self.depot.x], [self.depot.y], c="red", marker="s", label="Depot")
        plt.legend()
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.title("VRP Instance")
        plt.show()

