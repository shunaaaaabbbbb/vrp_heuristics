import os
import matplotlib.pyplot as plt
from instance import Instance

def save_solution_plot(solution, instance: Instance, out_path: str) -> None:
    # 実行ごとに: results/YYYYMMDD/ に保存し、ファイル名に時刻＋ソルバー名を付与
    solver_tag = (solution.solver_name or "solver").replace(" ", "_")
    out_path = os.path.join(out_path, f"{solver_tag}.png")

    id_to_customer = {c.id: c for c in instance.customers_with_depot}
    depot = id_to_customer[0]

    plt.figure(figsize=(7, 7))
    # デポ
    plt.scatter(depot.x, depot.y, c="red", marker="*", s=200, label="Depot")

    colors = plt.cm.tab20.colors
    for i, (k, route) in enumerate(sorted(solution.routes.items())):
        if len(route) < 2:
            continue
        xs = [id_to_customer[n].x for n in route]
        ys = [id_to_customer[n].y for n in route]
        color = colors[i % len(colors)]
        plt.plot(xs, ys, marker="o", color=color, label=f"Veh {k}")

    # 顧客点（重ね描画・識別用）
    cust_x = [c.x for c in instance.customers]
    cust_y = [c.y for c in instance.customers]
    plt.scatter(cust_x, cust_y, c="blue", s=20, alpha=0.6)

    plt.title(f"Routes: {solution.solver_name} | dist={solution.total_distance:.2f}")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.tight_layout()
    plt.savefig(out_path, dpi=200)
    plt.close()
    print(f"Saved route plot: {out_path}")
