import matplotlib.pyplot as plt
from typing import List

plt.rcParams["font.family"] = "serif"
plt.rcParams['pdf.fonttype'] = 42
plt.rcParams['ps.fonttype'] = 42

# Set default font size
plt.rc('font', size=20)

# def unbounded_knapsack_dp(wgt: list[int], val: list[float], cap: int) -> float:
def unbounded_knapsack_dp(wgt: List[int], val: List[float], cap: int) -> float:

    """完全背包：动态规划"""
    n = len(wgt)
    
    # 初始化 dp 表和用于记录选中物品的表
    dp = [[0.0] * (cap + 1) for _ in range(n + 1)]
    selected_items = [[0] * (cap + 1) for _ in range(n + 1)]


    # 状态转移
    for i in range(1, n + 1):
        for c in range(1, cap + 1):
            if wgt[i - 1] > c:
                # 若超过背包容量，则不选物品 i
                dp[i][c] = dp[i - 1][c]
                selected_items[i][c] = selected_items[i - 1][c]
            else:
                # 不选和选物品 i 这两种方案的较大值
                if dp[i - 1][c] > dp[i][c - wgt[i - 1]] + val[i - 1]:
                    dp[i][c] = dp[i - 1][c]
                    selected_items[i][c] = selected_items[i - 1][c]
                else:
                    dp[i][c] = dp[i][c - wgt[i - 1]] + val[i - 1]
                    selected_items[i][c] = i

    # 打印选中的物品
    current_capacity = cap
    items_selected = []
    for i in range(n, 0, -1):
        while current_capacity >= wgt[i - 1] and selected_items[i][current_capacity] == i:
            items_selected.append(i)
            current_capacity -= wgt[i - 1]

    # print("Selected items:", items_selected[::-1])
    temp_list = items_selected[::-1]

    mapping = {1: 'A', 2: 'B', 3: 'C'}

    new_list = [mapping[item] for item in temp_list]

    count_a = new_list.count('A')
    count_b = new_list.count('B')
    count_c = new_list.count('C')

    return dp[n][cap], count_a, count_b, count_c


# Example usage
print("calculating the number of UAV.....")
weights = [5, 7, 15] # battery capacity 1000 mah
values = [0.05, 0.08, 0.18]  # total coverage 0.01 km*km
capacity = 113 #  1000 mah
result, count_a, count_b, count_c = unbounded_knapsack_dp(weights, values, capacity)
print(f"Count of UAV A: {count_a}")
print(f"Count of UAV B: {count_b}")
print(f"Count of UAV C: {count_c}")
print("Max battery:", capacity*1000, " mah")
print("Maximum coverage area:", result*0.01, "km^2")

capacity_list = list(range(100, 151))
custom_ticks = list(range(100, 151, 5))
#[100, 110, 120, 130, 140, 150]

result_list = []
a_list = []
b_list = []
c_list = []
for i in capacity_list:
    print(i)
    result, count_a, count_b, count_c = unbounded_knapsack_dp(weights, values, i)
    result_list.append(result)
    a_list.append(count_a)
    b_list.append(count_b)
    c_list.append(count_c)

fig, ax1 = plt.subplots(figsize=(18, 12))


ax1.plot(capacity_list, a_list, label='UAV A')
ax1.plot(capacity_list, b_list, label='UAV B')
ax1.plot(capacity_list, c_list, label='UAV C')
# plt.plot(capacity_list, result_list, label='Cover Area')
plt.grid(True)

ax1.set_xlabel('Total Battery Capacity (*1000 mah)')
ax1.set_ylabel('Number', color='k')
# Create a secondary y-axis for B_list
ax2 = ax1.twinx()
ax2.plot(capacity_list, result_list, label='Cover Area', color='red')
ax2.set_ylabel('Cover Area (km^2)', color='red')
ax2.tick_params('y', colors='red')


# plt.xlabel('Capacity (*1000 mah)')
# plt.ylabel('Number')
plt.title('Number of UAVs vs. Total Battery Capacity')


# Set integer ticks on both axes
#plt.xticks(capacity_list)
plt.xticks(custom_ticks)

# plt.yticks(range(int(min(min(a_list), min(b_list))), int(max(max(a_list), max(b_list))) + 1))
# plt.legend()

# Add legends
ax1.legend(loc='upper left')
# ax2.legend(loc='upper right')
ax2.legend(loc='upper right', bbox_to_anchor=(0.9, 1.0))  # x y
plt.grid(True)

plt.tight_layout()
plt.show()
##################################################

print("calculating the number of AUV.....")
weights = [25, 35, 50] # battery capacity 1000 mah 
values = [0.20, 0.36, 0.40]  # total coverage  km*km
capacity = 120 #  1000 mah
result, count_a, count_b, count_c = unbounded_knapsack_dp(weights, values, capacity)
print(f"Count of AUV A: {count_a}")
print(f"Count of AUV B: {count_b}")
print(f"Count of AUV C: {count_c}")
print("Max battery:", capacity*1000, " mah")
print("Maximum coverage area:", result*0.01, "km^2")


capacity_list = list(range(100, 151))
# Set custom x-axis ticks
#custom_ticks = [100, 110, 120, 130, 140, 150]
custom_ticks = list(range(100, 151, 5))

result_list = []
a_list = []
b_list = []
c_list = []
for i in capacity_list:
    print(i)
    result, count_a, count_b, count_c = unbounded_knapsack_dp(weights, values, i)
    result_list.append(result)
    a_list.append(count_a)
    b_list.append(count_b)
    c_list.append(count_c)

fig, ax1 = plt.subplots(figsize=(18, 12))

ax1.plot(capacity_list, a_list, label='AUV A')
ax1.plot(capacity_list, b_list, label='AUV B')
ax1.plot(capacity_list, c_list, label='AUV C')
# plt.plot(capacity_list, result_list, label='Cover Area')
plt.grid(True)

ax1.set_xlabel('Total Battery Capacity (*1000 mah)')
ax1.set_ylabel('Number', color='k')
# Create a secondary y-axis for B_list
ax2 = ax1.twinx()
ax2.plot(capacity_list, result_list, label='Cover Area', color='red')
ax2.set_ylabel('Cover Area (km^2)', color='red')
ax2.tick_params('y', colors='red')

# Set integer ticks on both axes
# plt.xticks(capacity_list)
plt.xticks(custom_ticks)

plt.title('Number of AUV vs. Total Battery Capacity')
# plt.yticks(range(int(min(min(a_list), min(b_list))), int(max(max(a_list), max(b_list))) + 1))
# plt.legend()

# Add legends
ax1.legend(loc='upper left')
# ax2.legend(loc='upper right')
# Adjust the position of the legend slightly towards the middle
ax2.legend(loc='upper right', bbox_to_anchor=(1.0, 0.9))  # x y

plt.grid(True)





plt.tight_layout()
plt.show()


