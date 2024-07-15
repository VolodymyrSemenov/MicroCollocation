str_ = ""

# for i in range(8):
#     for j in range(5):
#         str_ = str_ + f'Export["h{i+1}_{j+1}.txt", h{i+1}[[{j+1}]]]\n'
# for i in range(8):
#     for j in range(5):
#         str_ = str_ + f'Export["lag_h{i+1}_{j+1}.txt", lagrange[h{i+1}[[{j+1}]]]]\n'
# for i in range(8):
#     for j in range(5):
#         str_ = str_ + f'Export["hess_h{i+1}_{j+1}.txt", hess[h{i+1}[[{j+1}]]]]\n'

for i in range(462):
    # str_ += f"iRow[{i}] = JACOBIAN_X_{i};\njCol[{i}] = JACOBIAN_Y_{i};\n" 
    str_ += f"values[{i}] = JACOBIAN_VAL_{i};\n" 

print(str_)