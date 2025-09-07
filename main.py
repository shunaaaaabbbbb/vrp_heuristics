from instance import Instance

if __name__ == "__main__":
    instance = Instance(10, 2)
    print(instance.customers[0].id, instance.customers[0].demand, instance.customers[0].x, instance.customers[0].y)
    print(instance.distances[(0, 1)])