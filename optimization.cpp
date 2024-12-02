#include <iostream>
#include <cmath>

#include "/util/Ipopt/include/coin-or/IpIpoptApplication.hpp"
#include "/util/Ipopt/include/coin-or/IpTNLP.hpp"

#include "optimization.hpp"
#include "globals.hpp"
#include "formulas.hpp"


using namespace Ipopt;
  
// constructor
Collocation::Collocation(bool printiterate) {
   printiterate_ = printiterate;
}

// destructor
Collocation::~Collocation() {}

// returns the size of the problem
bool Collocation::get_nlp_info(
    Index&          variableCount,
    Index&          constraintCount,
    Index&          jacobianNonZeroCount,
    Index&          hessianNonZeroCount,
    IndexStyleEnum& index_style
    ) {
    variableCount = 50;

    constraintCount = 40;
    jacobianNonZeroCount = JACOBIAN_NONZERO;
    hessianNonZeroCount = HESSIAN_NONZERO;
    index_style = TNLP::C_STYLE;

    return true;
}

    // returns the variable bounds
    bool Collocation::get_bounds_info(
    Index   variableCount,
    Number* variableLowerBounds,
    Number* variableUpperBounds,
    Index   constraintCount,
    Number* constraintLowerBounds,
    Number* constraintUpperBounds
    ) {
    const double lowerBoundPattern[6] = {-1.0 * vmax, 0.5 * lz, -10, theta - 0.1, 0, -0.5};
    const double upperBoundPattern[6] = {1.0 * vmax,  1.0 * lz, 10,  pi - theta + 0.1, 100, 0.5};

    
    // Put in bounds for x1
    variableLowerBounds[0] = 0.0;
    variableUpperBounds[0] = 1.0;

    for (Index i = 1; i < 10; i++) {
        variableLowerBounds[i] = -1.0 * vmax;
        variableUpperBounds[i] = 1.0 * vmax;
    }

    for (Index i = 10; i < variableCount; i++) {
        variableLowerBounds[i] = lowerBoundPattern[(i-2) / 8];
        variableUpperBounds[i] = upperBoundPattern[(i-2) / 8]; 
    }

    // Update bounds that don't fit pattern
    variableLowerBounds[25] = 0.0;

    variableLowerBounds[33] = pi / 2; 

    // All Constraints are Equality constraints to 0
    for (int i = 0; i < constraintCount; i++) {
        constraintLowerBounds[i] = constraintUpperBounds[i] = 0;
    }

    return true;
}


// returns the initial point for the problem
bool Collocation::get_starting_point(
    Index   variableCount,
    bool    init_x,
    Number* x,
    bool    init_z,
    Number* z_L,
    Number* z_U,
    Index   constraintCount,
    bool    init_lambda,
    Number* lambda
    ) {
    // x[0] = 0.5;
    // x[1] = 0.0;
    // x[2] = 0.1 * vmax;
    // x[3] = 0.2 * vmax;
    // x[4] = 0.3 * vmax;
    // x[5] = 0.4 * vmax;
    // x[6] = 0.5 * vmax;
    // x[7] = 0.6 * vmax;
    // x[8] = 0.7 * vmax;
    // x[9] = 0.8 * vmax;
    // x[10] = lz * 0.95;
    // x[11] = lz * 0.90;
    // x[12] = lz * 0.85;
    // x[13] = lz * 0.80;
    // x[14] = lz * 0.85;
    // x[15] = lz * 0.90;
    // x[16] = lz * 0.95;
    // x[17] = lz;
    // x[18] = legDot * 0.75;
    // x[19] = legDot * 0.5;
    // x[20] = legDot * 0.25;
    // x[21] = 0.0;
    // x[22] = legDot * -0.25;
    // x[23] = legDot * -0.50;
    // x[24] = legDot * -0.75;
    // x[25] = legDot * -1;
    // x[26] = (1.0/8) * pi - theta;
    // x[27] = (2.0/8) * pi - theta;
    // x[28] = (3.0/8) * pi - theta;
    // x[29] = (4.0/8) * pi - theta;
    // x[30] = (5.0/8) * pi - theta;
    // x[31] = (6.0/8) * pi - theta;
    // x[32] = (7.0/8) * pi - theta;
    // x[33] = pi - theta;
    // x[34] = thetaDot;
    // x[35] = thetaDot;
    // x[36] = thetaDot;
    // x[37] = thetaDot;
    // x[38] = thetaDot;
    // x[39] = thetaDot;
    // x[40] = thetaDot;
    // x[41] = thetaDot;
    // x[42] = 0.1;
    // x[43] = 0.1;
    // x[44] = 0.1;
    // x[45] = 0.1;
    // x[46] = 0.1;
    // x[47] = 0.1;
    // x[48] = 0.1;
    // x[49] = 0.1;
    x[0] = 0.0850791;
    x[1] = 0.860168;
    x[2] = 0.848687;
    x[3] = 0.922777;
    x[4] = 1.12963;
    x[5] = 1.17345;
    x[6] = 1.14967;
    x[7] = 1.07011;
    x[8] = 0.965738;
    x[9] = 0.865917;
    x[10] = 0.157015;
    x[11] = 0.145692;
    x[12] = 0.137714;
    x[13] = 0.134176;
    x[14] = 0.135433;
    x[15] = 0.141075;
    x[16] = 0.15002;
    x[17] = 0.16072;
    x[18] = -1.17288;
    x[19] = -0.929819;
    x[20] = -0.553136;
    x[21] = -0.106674;
    x[22] = 0.336615;
    x[23] = 0.707198;
    x[24] = 0.950342;
    x[25] = 1.03405;
    x[26] = 1.34147;
    x[27] = 1.37266;
    x[28] = 1.4062;
    x[29] = 1.44183;
    x[30] = 1.4778;
    x[31] = 1.512;
    x[32] = 1.5431;
    x[33] = 1.5708;
    x[34] = 2.81434;
    x[35] = 3.04436;
    x[36] = 3.26734;
    x[37] = 3.40185;
    x[38] = 3.32774;
    x[39] = 3.08236;
    x[40] = 2.76149;
    x[41] = 2.45532;
    x[42] = -0.5;
    x[43] = -0.5;
    x[44] = 0.499996;
    x[45] = 0.499999;
    x[46] = 0.5;
    x[47] = 0.5;
    x[48] = 0.5;
    x[49] = 0.499998;

    return true;
}


// returns the value of the objective function
bool Collocation::eval_f(
    Index         variableCount,
    const Number* x,
    bool          new_x,
    Number&       obj_value
    ) {
    obj_value = COST_FUNCTION;
    
    return true;
}


 // return the gradient of the objective function grad_{x} f(x)
bool Collocation::eval_grad_f(
    Index         variableCount,
    const Number* x,
    bool          new_x,
    Number*       grad_f
    ) {
    grad_f[0] = GRAD_F0;
    grad_f[1] = GRAD_F1;
    grad_f[2] = GRAD_F2;
    grad_f[3] = GRAD_F3;
    grad_f[4] = GRAD_F4;
    grad_f[5] = GRAD_F5;
    grad_f[6] = GRAD_F6;
    grad_f[7] = GRAD_F7;
    grad_f[8] = GRAD_F8;
    grad_f[9] = GRAD_F9;
    grad_f[10] = GRAD_F10;
    grad_f[11] = GRAD_F11;
    grad_f[12] = GRAD_F12;
    grad_f[13] = GRAD_F13;
    grad_f[14] = GRAD_F14;
    grad_f[15] = GRAD_F15;
    grad_f[16] = GRAD_F16;
    grad_f[17] = GRAD_F17;
    grad_f[18] = GRAD_F18;
    grad_f[19] = GRAD_F19;
    grad_f[20] = GRAD_F20;
    grad_f[21] = GRAD_F21;
    grad_f[22] = GRAD_F22;
    grad_f[23] = GRAD_F23;
    grad_f[24] = GRAD_F24;
    grad_f[25] = GRAD_F25;
    grad_f[26] = GRAD_F26;
    grad_f[27] = GRAD_F27;
    grad_f[28] = GRAD_F28;
    grad_f[29] = GRAD_F29;
    grad_f[30] = GRAD_F30;
    grad_f[31] = GRAD_F31;
    grad_f[32] = GRAD_F32;
    grad_f[33] = GRAD_F33;
    grad_f[34] = GRAD_F34;
    grad_f[35] = GRAD_F35;
    grad_f[36] = GRAD_F36;
    grad_f[37] = GRAD_F37;
    grad_f[38] = GRAD_F38;
    grad_f[39] = GRAD_F39;
    grad_f[40] = GRAD_F40;
    grad_f[41] = GRAD_F41;
    grad_f[42] = GRAD_F42;
    grad_f[43] = GRAD_F43;
    grad_f[44] = GRAD_F44;
    grad_f[45] = GRAD_F45;
    grad_f[46] = GRAD_F46;
    grad_f[47] = GRAD_F47;
    grad_f[48] = GRAD_F48;
    grad_f[49] = GRAD_F49;

    return true;
}


// return the value of the constraints: g(x)
bool Collocation::eval_g(
    Index         variableCount,
    const Number* x,
    bool          new_x,
    Index         constraintCount,
    Number*       constraint // Constraint Values Out
    ) {

    constraint[0] = CONSTRAINT_0;
    constraint[1] = CONSTRAINT_1;
    constraint[2] = CONSTRAINT_2;
    constraint[3] = CONSTRAINT_3;
    constraint[4] = CONSTRAINT_4;
    constraint[5] = CONSTRAINT_5;
    constraint[6] = CONSTRAINT_6;
    constraint[7] = CONSTRAINT_7;
    constraint[8] = CONSTRAINT_8;
    constraint[9] = CONSTRAINT_9;
    constraint[10] = CONSTRAINT_10;
    constraint[11] = CONSTRAINT_11;
    constraint[12] = CONSTRAINT_12;
    constraint[13] = CONSTRAINT_13;
    constraint[14] = CONSTRAINT_14;
    constraint[15] = CONSTRAINT_15;
    constraint[16] = CONSTRAINT_16;
    constraint[17] = CONSTRAINT_17;
    constraint[18] = CONSTRAINT_18;
    constraint[19] = CONSTRAINT_19;
    constraint[20] = CONSTRAINT_20;
    constraint[21] = CONSTRAINT_21;
    constraint[22] = CONSTRAINT_22;
    constraint[23] = CONSTRAINT_23;
    constraint[24] = CONSTRAINT_24;
    constraint[25] = CONSTRAINT_25;
    constraint[26] = CONSTRAINT_26;
    constraint[27] = CONSTRAINT_27;
    constraint[28] = CONSTRAINT_28;
    constraint[29] = CONSTRAINT_29;
    constraint[30] = CONSTRAINT_30;
    constraint[31] = CONSTRAINT_31;
    constraint[32] = CONSTRAINT_32;
    constraint[33] = CONSTRAINT_33;
    constraint[34] = CONSTRAINT_34;
    constraint[35] = CONSTRAINT_35;
    constraint[36] = CONSTRAINT_36;
    constraint[37] = CONSTRAINT_37;
    constraint[38] = CONSTRAINT_38;
    constraint[39] = CONSTRAINT_39;
    return true;
}

void Collocation::finalize_solution(
    SolverReturn               status,
    Index                      n,
    const Number*              x,
    const Number*              z_L,
    const Number*              z_U,
    Index                      m,
    const Number*              g,
    const Number*              lambda,
    Number                     obj_value,
    const IpoptData*           ip_data,
    IpoptCalculatedQuantities* ip_cq
    ) {
    std::cout << std::endl << std::endl << "Solution of the primal variables, x" << std::endl;
    for (Index i = 0; i < n; i++) {
        std::cout << "x[" << i << "] = " << x[i] << std::endl;
    }

    std::cout << std::endl << std::endl << "Objective value" << std::endl;
    std::cout << "f(x*) = " << obj_value << std::endl;
}

bool Collocation::intermediate_callback(
    AlgorithmMode              mode,
    Index                      iter,
    Number                     obj_value,
    Number                     inf_pr,
    Number                     inf_du,
    Number                     mu,
    Number                     d_norm,
    Number                     regularization_size,
    Number                     alpha_du,
    Number                     alpha_pr,
    Index                      ls_trials,
    const IpoptData*           ip_data,
    IpoptCalculatedQuantities* ip_cq
    ) {
    return true;
}
