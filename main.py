from instance import Instance
from solver import VRPSolverMIP, NNSolver, SweepSolver, SweepNearestSolver
from plot_graph import save_solution_plot
import datetime
import os


def main():
    # 問題設定
    num_customers = 30
    num_vehicles = 10
    capacity = 20
    # インスタンス生成
    instance = Instance(num_customers, num_vehicles, capacity)

    # 実行するソルバー（MIP と NN）
    solvers = [
        #VRPSolverMIP(instance),
        NNSolver(instance),
        SweepSolver(instance),
        SweepNearestSolver(instance),
    ]

    now = datetime.datetime.now()
    date_str = now.strftime("%Y%m%d%H%M%S")
    dated_dir = os.path.join("results", date_str)
    os.makedirs(dated_dir, exist_ok=True)

    for solver in solvers:
        solution = solver.solve()
        solver.print_solution()
        save_solution_plot(solution, instance, out_path=dated_dir)


if __name__ == "__main__":
    main()