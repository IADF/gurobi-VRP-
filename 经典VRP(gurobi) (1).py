from gurobipy import *
import numpy as np
import math



model = Model()   # Model是一个类，实例化一个名为model的对象
x = {}
b = {}
target_amount = 11       # 客户点总量
trunk_amount = 3        # 车辆总量
time = (target_amount - 1) * 20             # 程序运行时间
unit_cost = 1          # 单位距离行驶成本
max_capacity = 20       # 车辆最大容量
MD = 200                 # 车辆最大行驶距离
pickup = {}               # 客户点取货量
delivery = {}             # 客户点送货量

def loadDatadet(infile, k):              #导入数据集
    fo = open(infile, 'r')
    temp2 =[]
    temp3 = []
    dataset = []
    for line in fo.readlines():
        temp1 = line.replace('\n', '')
        temp1 = temp1.replace('    ', ' ')
        temp1 = temp1.replace('   ', ' ')
        temp1 = temp1.replace('  ', ' ')
        temp1 = temp1.strip()
        temp2.append(temp1)
    for i in range(0, len(temp2)):
        temp3 = temp2[i].split()
        dataset.append(temp3)
    for i in range(101):
        for j in range(k):
            dataset[i][j] = int(dataset[i][j])
    return dataset

infile = "1P1.DAT"
k = 10
infile = loadDatadet(infile, k)

dist = [(lambda z:[math.sqrt(math.pow(infile[z][1] - infile[g][1], 2)         #计算各点间距离
                             + math.pow(infile[z][2] - infile[g][2], 2))
                   for g in range(0, len(infile))])(i)
        for i in range(0, len(infile))]

for i in range(target_amount):
    delivery[i] = infile[i][9]

for i in range(target_amount):
    for j in range(target_amount):
        for k in range(trunk_amount):
            x[i, j, k] = model.addVar(vtype=GRB.BINARY, name='x_' + str(i) + str(j) + str(k))

#流平衡约束
for i in range(0, target_amount):
    b[i] = model.addVar(lb=1.0, ub=GRB.INFINITY, obj=0.0, vtype=GRB.CONTINUOUS, name="u[%s]" % i)

# model.Params.timeLimit = time

model.setObjective(quicksum(dist[i][j] * x[i, j, k] * unit_cost for i in range(target_amount) for j in range(target_amount) for k in range(trunk_amount)), GRB.MINIMIZE)

for j in range(1, target_amount):
    model.addConstr(quicksum(x[i, j, k] for i in range(target_amount) for k in range(trunk_amount) if i != j) == 1, name='c11')
for i in range(1, target_amount):
    model.addConstr(quicksum(x[i, j, k] for j in range(target_amount) for k in range(trunk_amount) if i != j) == 1, name='c12')


model.addConstr(quicksum(x[i, j, k] for i in range(target_amount) for j in range(target_amount) for k in range(trunk_amount) if i == j) == 0, name='c2')

for j in range(1, target_amount):
    for k in range(trunk_amount):
        model.addConstr(quicksum(x[i, j, k] for i in range(target_amount)) - quicksum(x[j, i, k] for i in range(target_amount)) == 0, name='c3')

for k in range(trunk_amount):
    model.addConstr(quicksum(x[0, j, k] for j in range(1, target_amount)) == 1, name='c4')
for k in range(trunk_amount):
    model.addConstr(quicksum(x[i, 0, k] for i in range(1, target_amount)) == 1, name='c5')

for k in range(trunk_amount):
    model.addConstr(quicksum(x[0, j, k] for j in range(1, target_amount)) == quicksum(x[i, 0, k] for i in range(1, target_amount)),name='c61')


for k in range(trunk_amount):
    model.addConstr(quicksum(dist[i][j] * x[i, j, k] for i in range(target_amount) for j in range(target_amount)) <= MD, name='c6')

for i in range(1, target_amount):
    for j in range(2, target_amount+1):
        if i != j:
            if j == target_amount:
                j = 1
            model.addConstr(b[i]+1-(target_amount-1)*(1-quicksum(x[i, j, k] for k in range(trunk_amount))) <= b[j], name='c7')


for k in range(trunk_amount):

    for i in range(target_amount):
        for j in range(target_amount):
            if i != j:
                model.addConstr(b[i] - b[j] + 1 - (target_amount * (1 - x[i,j,k])) <= 0,
                            name="C8" + "_" + str(k) + "_" + str(i) + "_" + str(j))


for k in range(trunk_amount):
    model.addConstr(quicksum(delivery[j] * x[i, j, k] for i in range(target_amount) for j in range(1, target_amount)) <= max_capacity,name='c9')


model.optimize()
print(model.getAttr(GRB.Attr.NumConstrs))  # 获取约束数量
print("Runtime:")
print(model.getAttr(GRB.Attr.Runtime))  # 获取时间
result_X = model.getAttr('X', x)   #获得x变量的值
print('Obj: %g' % model.objVal)

for i in range(target_amount):
    for j in range(target_amount):
        for k in range(trunk_amount):
            if result_X[i,j,k] > 0.9:
                result_X[i,j,k] = 1
            else:
                result_X[i,j,k] = 0
            if result_X[i,j,k] > 0:
                print("x", i, j,k ,result_X[i, j,k])
    model.write('model VRP.lp')

    

# print('Obj: ', model.getObjective())
# constrs = model.getConstrs()####获取约束相关属性
# print (constrs)
# for i in range(2):
#  print(model.getRow(constrs[i]))
