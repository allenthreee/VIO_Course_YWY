#!/usr/bin/python
import matplotlib.pyplot as plt
# filename = "lambda_data.txt"
filename1 = "alg1_lambda_data.txt"
X1, Y1 = [],[]
for line in open(filename1, 'r'):
    value = [float (s) for s in line.split()]
    X1.append(value[0])
    Y1.append(value[1])

filename3 = "alg3_lambda_data.txt"
X3, Y3 = [],[]
for line in open(filename3, 'r'):
    value = [float (s) for s in line.split()]
    X3.append(value[0])
    Y3.append(value[1])

plt.plot(X1, Y1, color="g", linestyle="-", marker="^", linewidth=1, label="algorithm 1")
plt.plot(X3, Y3, color="r", linestyle="--", marker="x", linewidth=1, label="algorithm 3")

plt.legend(loc='upper right', bbox_to_anchor=(0.95, 0.95))
# plt.title("lambda data")
plt.title("mix lambda data")
plt.xlabel("iterations")
plt.ylabel("lambda")
plt.savefig("./mix_lambda_line_chart")
plt.show()
