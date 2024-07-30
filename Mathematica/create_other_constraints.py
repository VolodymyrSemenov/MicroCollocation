import os

variables = [11, 20, 29, 38, 47, 19]
constants = ['leg', 'legDot', 'theta', 'thetaDot', 'ia0', 'leg']

def writeable_file(file_name):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    file_path = os.path.join(script_dir, file_name)
    return open(file_path, 'w+')

for i in range(6):
    base_filename = f'h0_{1+i}.txt'
    lagrangian_filename = f'lag_h0_{1+i}.txt'
    hessian_filename = f'hess_h0_{1+i}.txt'
    f1 = writeable_file(base_filename)
    f1.write(f'x{variables[i]} - {constants[i]}')

    f2 = writeable_file(lagrangian_filename)
    lag = ['0']*55
    lag[variables[i] - 1] = '1'
    f2.write('\n'.join(lag))

    f3 = writeable_file(hessian_filename)
    f3.write('\n'.join([f"{{{', '.join(['0']*55) }}}"]*55))


