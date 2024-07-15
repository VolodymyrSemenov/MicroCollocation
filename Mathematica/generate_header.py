import re
from util import open_file_in_same_directory

class Constraint:
    counter = 1 # initial value
    def __init__(self, name):
        self.name = self.counter
        Constraint.counter += 1 # To give each constraint its own name
        self.base = self.parse_equation(open_file_in_same_directory(name))
        self.lagrangian = self.parse_lagrangian("lag_" + name)
        self.hessian = self.parse_hessian("hess_" + name)

    def __repr__(self):
        return str("Constraint" + str(self.name))

    def parse_equation(self, equation): 
        def replace_subscript(text):
            inner_word = text.group(1)
            return ''.join(inner_word.split(", "))

        def replace_x(text):
            inner_word = text.group(0)
            return f'x[{int(inner_word[1:]) - 1}]'
        
        def replace_sin(text):
            inner_word = text.group(1)
            return f'std::sin({inner_word})'

        def replace_cos(text):
            inner_word = text.group(1)
            return f'std::cos({inner_word})'

        def replace_power(text):
            base = text.group(1)
            power = text.group(2)
            return f'std::pow({base},{power})'

        def replace_all_complex_powers(string):
            i = re.search(r"\)\^(\d+)", string)
            while i:
                bad_end = i.end()
                bad_start = i.start()
                parenthesis = 1
                while parenthesis != 0:
                    bad_start -= 1
                    letter = i.string[bad_start]
                    if letter == ')':
                        parenthesis += 1
                    elif letter == '(':
                        parenthesis -= 1
                part_to_change = string[bad_start:bad_end].split('^')

                string = string[:bad_start] + f'std::pow({part_to_change[0]},{part_to_change[1]})' + string[bad_end:] 
                i = re.search(r"\)\^(\d+)", string)

            return string

        equation = re.sub(r"Subscript\[([^\]]*)\]", replace_subscript, equation)
        equation = re.sub(r"([^ )(*+-/]+)\^(\d+)", replace_power, equation)
        equation = re.sub(r"([^ )(*+-/]+)\^\((-\d)\)", replace_power, equation)
        equation = replace_all_complex_powers(equation)
        equation = re.sub(r"Cos\[([^\]]*)\]", replace_cos, equation)
        equation = re.sub(r"Sin\[([^\]]*)\]", replace_sin, equation)
        equation = re.sub(r"(x\d+)", replace_x, equation)
        return equation

    def parse_lagrangian(self, file):
        lagrangians = list(open_file_in_same_directory(file).split('\n'))
        for i in range(len(lagrangians)):
            lagrangians[i] = self.parse_equation(lagrangians[i])
        return lagrangians

    def parse_hessian(self, file):
        parsed = []
        for row in self.parse_equation(open_file_in_same_directory(file)).split('\n'):
            parsed.append([])
            for col in row[1:-1].split(', '):
                parsed[-1].append(col)
        return parsed

class Defines:
    def __init__(self):
        self.defines = {} # name -> str

    def add_cost_function(self, cost_function):
        self.defines["COST_FUNCTION"] = cost_function.base
        for i in range(len(cost_function.lagrangian)):
            self.defines[f'GRAD_F{i}'] = cost_function.lagrangian[i]

    def add_constraints(self, constraints):
        for i in range(len(constraints)):
            self.defines[f"CONSTRAINT_{i}"] = constraints[i].base
        non_zeros = 0
        for i in range(len(constraints)):
            for j in range(len(constraints[i].lagrangian)):
                val = constraints[i].lagrangian[j]
                if val != '0':
                    self.defines[f'JACOBIAN_X_{non_zeros}'] = i
                    self.defines[f'JACOBIAN_Y_{non_zeros}'] = j
                    self.defines[f'JACOBIAN_VAL_{non_zeros}'] = val
                    non_zeros += 1
                    

        self.defines["JACOBIAN_NONZERO"] = non_zeros

    def add_hessian_of_lagrangian(self, constraints, cost_function):
        non_zeros = 0
        non_zero_list = []
        self.defines["HESSIAN_NONZERO"] = non_zeros

    def return_defines(self):
        return '\n'.join(f'#define {key} {val}' for key, val in self.defines.items())


def generate_header(constraints, cost_function):
    define_list = Defines()
    define_list.add_cost_function(cost_function)
    define_list.add_constraints(constraints)
    define_list.add_hessian_of_lagrangian(constraints, cost_function)

    with open("formulas.hpp", "w+") as f:
        f.write(define_list.return_defines())

        

if __name__ == '__main__':
    
    constraints = []
    for i in range(1, 6+1):
        constraints.append(Constraint(f'h{0}_{i}.txt'))
    for i in range(1, 8+1):
        for j in range(1, 5+1):
            constraints.append(Constraint(f'h{i}_{j}.txt'))

    cost_function = Constraint("cost_function.txt") # Not a constraint but same formatting used
    
    generate_header(constraints, cost_function)