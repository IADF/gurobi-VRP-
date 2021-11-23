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