from gurobipy import *
import read_data
import numpy as np
import math

depot_num = 1  # 仓库数
customer_num = 10  # 客户数
vehicle_num = 3  # 总车辆数
unit_vehicle_cost = 1  # 单位车辆行驶成本
vehicle_capacity = 20  # 车的最大容量
vehicle_max_distance = 200  # 车辆的最大行驶距离（曼哈顿距离会不会比较合理？）
total_node_num = customer_num + depot_num  # 所有的节点的数量

infile = "1P1.DAT"
f = 10

x = np.zeros((total_node_num, total_node_num, vehicle_num))
x_list = x.tolist()
u = np.zeros((vehicle_num,total_node_num))
u_list = u.tolist()

if __name__ == '__main__':
    infile = read_data.loadDatadet(infile, f)  # 读取数据

    delivery = {}

    for i in range(total_node_num):
        delivery[i] = infile[i][9]

    VRP_Mode = Model("normal VRP")

    # 创建决策变量x
    for i in range(total_node_num):
        for j in range(total_node_num):
            for k in range(vehicle_num):
                x_list[i][j][k] = VRP_Mode.addVar(vtype=GRB.BINARY, name="x_" + str(i) + "_" + str(j) + "_" + str(k))

    # MTZ子环约束决策变量
    for k in range(vehicle_num):
        for i in range(total_node_num):
            u_list[k][i] = VRP_Mode.addVar(lb=1,ub=GRB.INFINITY,obj=0.0,vtype=GRB.CONTINUOUS, name="u_" + str(k) + "_" + str(i))

    dist = [(lambda z: [math.sqrt(math.pow(infile[z][1] - infile[g][1], 2)  # 计算各点间距离
                                  + math.pow(infile[z][2] - infile[g][2], 2))
                        for g in range(0, len(infile))])(i)
            for i in range(0, len(infile))]

    obj_lr = LinExpr()
    for i in range(total_node_num):
        for j in range(total_node_num):
            for k in range(vehicle_num):
                obj_lr += dist[i][j] * x_list[i][j][k] * unit_vehicle_cost
    VRP_Mode.setObjective(obj_lr, GRB.MINIMIZE)

    # 保证每个客户点都能够被服务
    for j in range(depot_num, total_node_num):
        constr1_lr = LinExpr()
        for i in range(total_node_num):
            for k in range(vehicle_num):
                if i != j:
                    constr1_lr += x_list[i][j][k]
        VRP_Mode.addConstr(constr1_lr == 1, name="C1" + "_" + str(j))

    # 保证每个客户点不能有车辆停留
    for i in range(depot_num, total_node_num):
        constr2_lr = LinExpr()
        for j in range(total_node_num):
            for k in range(vehicle_num):
                if i != j:
                    constr2_lr += x_list[i][j][k]
        VRP_Mode.addConstr(constr2_lr == 1, name="C2" + "_" + str(i))

    # 防止车辆由i点出发到i点
    constr3_lr = LinExpr()
    for i in range(total_node_num):
        for k in range(vehicle_num):
            constr3_lr += x_list[i][i][k]
    VRP_Mode.addConstr(constr3_lr == 0, name="C3" + "_" + str(i) + "_" + str(k))

    # 送货路线与回程路线相同
    for j in range(depot_num, total_node_num):
        constr4_lr = LinExpr()
        for k in range(vehicle_num):
            for i in range(total_node_num):
                constr4_lr += x_list[i][j][k]
                constr4_lr -= x_list[j][i][k]
        VRP_Mode.addConstr(constr4_lr == 0, name="C4" + "_" + str(j))

    # 所有的车辆都要从仓库发出
    for k in range(vehicle_num):
        constr5_lr = LinExpr()
        for j in range(depot_num, total_node_num):
            constr5_lr += x_list[0][j][k]
        VRP_Mode.addConstr(constr5_lr == 1, name="C5" + "_" + str(k))

    # 所有车辆配送完后都需要回到仓库点
    for k in range(vehicle_num):
        constr6_lr = LinExpr()
        for i in range(depot_num, total_node_num):
            constr6_lr += x_list[i][0][k]
        VRP_Mode.addConstr(constr6_lr == 1, name="C6" + "_" + str(k))

    # 车辆返回仓库(无效约束)
    for k in range(vehicle_num):
        constr7_lr = LinExpr()
        for i in range(depot_num, total_node_num):
            for j in range(depot_num, total_node_num):
                constr7_lr += x_list[0][j][k]
                constr7_lr -= x_list[i][0][k]
        VRP_Mode.addConstr(constr7_lr == 0, name="C7" + "_" + str(k))

    # MTZ子环约束
    for k in range(vehicle_num):
        for i in range(total_node_num):
            for j in range(total_node_num):
                if i != j:
                    VRP_Mode.addConstr(u_list[k][i] - u_list[k][j] + 1 - (total_node_num * (1 - x_list[i][j][k])) <= 0,
                                       name="C8" + "_" + str(k) + "_" + str(i) + "_" + str(j))



    # 车辆运货不能超过车辆的最大载重
    for k in range(vehicle_num):
        constr9_lr = LinExpr()
        for i in range(total_node_num):
            for j in range(depot_num, total_node_num):
                constr9_lr += delivery[j] * x_list[i][j][k]
        VRP_Mode.addConstr(constr9_lr <= vehicle_capacity, name="C9" + "_" + str(k))

    # 车辆运输不能超过车辆的最长运输距离
    for k in range(vehicle_num):
        constr10_lr = LinExpr()
        for i in range(total_node_num):
            for j in range(total_node_num):
                constr10_lr += dist[i][j] * x_list[i][j][k]
        VRP_Mode.addConstr(constr10_lr <= vehicle_max_distance, name="C10" + "_" + str(k))

    VRP_Mode.update()
    # Gurobi param
    VRP_Mode.setParam('OutputFlag', 0)

    VRP_Mode.optimize()

    VRP_Mode.write('normal VRP.lp')

    print(VRP_Mode.getObjective())
    print("最优化值", VRP_Mode.objVal)

    # # 车辆的返回路线和发货路线一样
    # for i in range():
    #     for j in range():
