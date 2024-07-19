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

for i in range(400,843): #843
    str_ += f"iRow[{i}] = HESSIAN_X_{i};\njCol[{i}] = HESSIAN_Y_{i};\n" 
    #str_ += f"values[{i}] = HESSIAN_VAL_{i};\n" 

print(str_)