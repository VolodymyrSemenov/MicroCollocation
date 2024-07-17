#include <iostream>
#include <cassert>
#include <numbers>
#include <cmath>

#include "/usr/local/include/coin-or/IpIpoptApplication.hpp"
#include "/usr/local/include/coin-or/IpTNLP.hpp"

#include "formulas.hpp"
#include "main.hpp"

using namespace Ipopt;

// Constants
double pi = 3.141592653;    //
double g = 9.81;            //

// Robot Parameters
double lz = 0.17;           // 
double kz = 1699;           // 
double b = 5.0;               //
double m = 1.2;             // 

// Initial Condition
double leg = 0.17;          //
double legDot = -1.2369;    //
double theta = 1.3129;      //
double thetaDot = 2.5559;   //
double iaz = 0.2;           //  

// Motor Parameters
double vmax = 18;           //
double Ra = 0.135;          //
double R = 33.0625;         //
double kt = 0.0098;         //
double kb = kt;             //
double c = 0.0000016; //* std::pow(10.0, -6);      
double J = 0.00000183; //* std::pow(10.0, -6);      
double La = 0.0000166; //* std::pow(10.0, -5);     


int main(int argc, char** argv) {
   // Create a new instance of your nlp
   //  (use a SmartPtr, not raw)
   SmartPtr<TNLP> mynlp = new Collocation();

   // Create a new instance of IpoptApplication
   //  (use a SmartPtr, not raw)
   // We are using the factory, since this allows us to compile this
   // example with an Ipopt Windows DLL
   SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

   // Change some options
   // Note: The following choices are only examples, they might not be
   //       suitable for your optimization problem.
   //app->Options()->SetNumericValue("tol", 10e-5);
   app->Options()->SetNumericValue("constr_viol_tol", 10e-3);
   app->Options()->SetIntegerValue("max_iter", 5000);
   app->Options()->SetStringValue("hessian_approximation", "limited-memory");
   //app->Options()->SetStringValue("mu_strategy", "adaptive");
   //app->Options()->SetStringValue("output_file", "ipopt.out");

   // Initialize the IpoptApplication and process the options
   ApplicationReturnStatus status;
   status = app->Initialize();
   if (status != Solve_Succeeded) {
      std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
      return (int) status;
   }

   // Ask Ipopt to solve the problem
   status = app->OptimizeTNLP(mynlp);

   if (status == Solve_Succeeded) {
      std::cout << std::endl << std::endl << "*** The problem solved!" << std::endl;
   } else {
      std::cout << std::endl << std::endl << "*** The problem FAILED!" << std::endl;
   }

   // As the SmartPtrs go out of scope, the reference count
   // will be decremented and the objects will automatically
   // be deleted.

   return (int) status;
}


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
    // 55 variables, x1 through x55
    variableCount = 55;

    constraintCount = 46;

    // jacobian contains 8 nonzeros
    jacobianNonZeroCount = JACOBIAN_NONZERO;

    // we only need the lower left corner and middle diagonal (since it is symmetric)
    hessianNonZeroCount = HESSIAN_NONZERO;

    // use C style indexing (0-based)
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
    assert(variableCount == 55);
    assert(constraintCount == 47);
    
    double lowerBoundPattern[6] = {-1.0 * vmax, 0.5 * lz, -10, theta - 0.1, 0, -0.5}; 
    double upperBoundPattern[6] = {1.0 * vmax,  1.0 * lz, 10,  pi - theta + 0.1, 100, 0.5};


    // Set variable bounds x2-x55. Indices of 1-54
    for (Index i = 1; i < variableCount; i++) {
        variableLowerBounds[i] = lowerBoundPattern[(i-1) / 9];
        variableUpperBounds[i] = upperBoundPattern[(i-1) / 9];
    }
    
    // Put in bounds for x1
    variableLowerBounds[0] = 0.0;
    variableUpperBounds[0] = 1.0;

    // Update bounds that don't fit pattern
    variableUpperBounds[19] = 0.0;
    variableLowerBounds[27] = 0.0;

    variableLowerBounds[36] = pi / 2; 

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
    // Here, we assume we only have starting values for x, if you code
    // your own NLP, you can provide starting values for the dual variables
    // if you wish
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);

    // initialize to the given starting point
 /*
x[0] = 0.0600000000000000;
x[1] = 0;
x[2] = 1;
x[3] = 2;
x[4] = 3;
x[5] = 4;
x[6] = 5;
x[7] = 6;
x[8] = 7;
x[9] = 8;
x[10] = 0.153000000000000;
x[11] = 0.153000000000000;
x[12] = 0.153000000000000;
x[13] = 0.153000000000000;
x[14] = 0.153000000000000;
x[15] = 0.153000000000000;
x[16] = 0.153000000000000;
x[17] = 0.153000000000000;
x[18] = 0.153000000000000;
x[19] = -1.23693101005372;
x[20] = -0.927698257540290;
x[21] = -0.618465505026860;
x[22] = -0.309232752513430;
x[23] = 0;
x[24] = 0.309232752513430;
x[25] = 0.618465505026860;
x[26] = 0.927698257540290;
x[27] = 1.23693101005372;
x[28] = 1.31288888888889;
x[29] = 1.37736574836539;
x[30] = 1.44184260784189;
x[31] = 1.50631946731840;
x[32] = 1.57079632679490;
x[33] = 1.63527318627140;
x[34] = 1.69975004574790;
x[35] = 1.76422690522440;
x[36] = 1.82870376470090;
x[37] = 5.11181335096876;
x[38] = 5.11181335096876;
x[39] = 5.11181335096876;
x[40] = 5.11181335096876;
x[41] = 5.11181335096876;
x[42] = 5.11181335096876;
x[43] = 5.11181335096876;
x[44] = 5.11181335096876;
x[45] = 5.11181335096876;
x[46] = 0.49;
x[47] = 0.49;
x[48] = 0.49;
x[49] = 0.49;
x[50] = 0.49;
x[51] = 0.49;
x[52] = 0.49;
x[53] = 0.49;
x[54] = 0.49;
*/
    // x[0] = 0.0942; //0.5;
    // x[1] = -0.3501; //0.0;
    // x[2] = -0.1494; //0.1 * vmax;
    // x[3] = 0.2306; //0.2 * vmax;
    // x[4] = 0.769; //0.3 * vmax;
    // x[5] = 1.4717; //0.4 * vmax;
    // x[6] = 2.1133; //0.5 * vmax;
    // x[7] = 2.3532; //0.6 * vmax;
    // x[8] = 2.1004; //0.7 * vmax;
    // x[9] = 1.7377;// 0.8 * vmax;
    // x[10] = 0.17; //lz;
    // x[11] = 0.1557; //lz * 0.95;
    // x[12] = 0.1435; //lz * 0.90;
    // x[13] = 0.1357; //lz * 0.85;
    // x[14] = 0.1335; //lz * 0.80;
    // x[15] = 0.1371; //lz * 0.85;
    // x[16] = 0.1456; //lz * 0.90;
    // x[17] = 0.1573; //lz * 0.95;
    // x[18] = 0.17; //lz;
    // x[19] = -1.2369; //legDot;
    // x[20] = -1.1594; //legDot * 0.75;
    // x[21] = -0.8721; //legDot * 0.5;
    // x[22] = -0.4355; //legDot * 0.25;
    // x[23] = 0.0634; //0.0;
    // x[24] = 0.5317; //legDot * -0.25;
    // x[25] = 0.8876; //legDot * -0.50;
    // x[26] = 1.0706;// legDot * -0.75;
    // x[27] = 1.0502; //legDot * -1;
    // x[28] = 1.3129; //theta;
    // x[29] = 1.3381; //(1.0/8) * pi - theta;
    // x[30] = 1.3561; //(2.0/8) * pi - theta;
    // x[31] = 1.3717; //(3.0/8) * pi - theta;
    // x[32] = 1.3916; //(4.0/8) * pi - theta;
    // x[33] = 1.4215; //(5.0/8) * pi - theta;
    // x[34] = 1.4639; //(6.0/8) * pi - theta;
    // x[35] = 1.5156; //(7.0/8) * pi - theta;
    // x[36] = 1.5709; //pi - theta;
    // x[37] = 2.5559; //thetaDot;
    // x[38] = 1.7784; //thetaDot;
    // x[39] = 1.3464;//thetaDot;
    // x[40] = 1.4073; //thetaDot;
    // x[41] = 2.0348; //thetaDot;
    // x[42] = 3.0654; //thetaDot;
    // x[43] = 4.0524; //thetaDot;
    // x[44] = 4.599;// thetaDot;
    // x[45] = 4.6992;// thetaDot;
    // x[46] = 0.2; //0.1;
    // x[47] = 2.5048; //0.1;
    // x[48] = 5.4354; //0.1;
    // x[49] = 8.4664; //0.1;
    // x[50] = 11.4481; //0.1;
    // x[51] = 13.0869; //0.1;
    // x[52] = 11.9215; //0.1;
    // x[53] = 8.2269; //0.1;
    // x[54] = 4.8566; //0.1;

    x[0] = 0.5;
    x[1] = 0.0;
    x[2] = 0.1 * vmax;
    x[3] = 0.2 * vmax;
    x[4] = 0.3 * vmax;
    x[5] = 0.4 * vmax;
    x[6] = 0.5 * vmax;
    x[7] = 0.6 * vmax;
    x[8] = 0.7 * vmax;
    x[9] = 0.8 * vmax;
    x[10] = lz;
    x[11] = lz * 0.95;
    x[12] = lz * 0.90;
    x[13] = lz * 0.85;
    x[14] = lz * 0.80;
    x[15] = lz * 0.85;
    x[16] = lz * 0.90;
    x[17] = lz * 0.95;
    x[18] = lz;
    x[19] = legDot;
    x[20] = legDot * 0.75;
    x[21] = legDot * 0.5;
    x[22] = legDot * 0.25;
    x[23] = 0.0;
    x[24] = legDot * -0.25;
    x[25] = legDot * -0.50;
    x[26] = legDot * -0.75;
    x[27] = legDot * -1;
    x[28] = theta;
    x[29] = (1.0/8) * pi - theta;
    x[30] = (2.0/8) * pi - theta;
    x[31] = (3.0/8) * pi - theta;
    x[32] = (4.0/8) * pi - theta;
    x[33] = (5.0/8) * pi - theta;
    x[34] = (6.0/8) * pi - theta;
    x[35] = (7.0/8) * pi - theta;
    x[36] = pi - theta;
    x[37] = thetaDot;
    x[38] = thetaDot;
    x[39] = thetaDot;
    x[40] = thetaDot;
    x[41] = thetaDot;
    x[42] = thetaDot;
    x[43] = thetaDot;
    x[44] = thetaDot;
    x[45] = thetaDot;
    x[46] = 0.1;
    x[47] = 0.1;
    x[48] = 0.1;
    x[49] = 0.1;
    x[50] = 0.1;
    x[51] = 0.1;
    x[52] = 0.1;
    x[53] = 0.1;
    x[54] = 0.1;

    return true;
}


// returns the value of the objective function
bool Collocation::eval_f(
    Index         variableCount,
    const Number* x,
    bool          new_x,
    Number&       obj_value
    ) {
    assert(variableCount == 55);

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
    assert(variableCount == 55);
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
    grad_f[50] = GRAD_F50;
    grad_f[51] = GRAD_F51;
    grad_f[52] = GRAD_F52;
    grad_f[53] = GRAD_F53;
    grad_f[54] = GRAD_F54;

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
    assert(variableCount == 55);
    assert(constraintCount == 46);

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
    constraint[40] = CONSTRAINT_40;
    constraint[41] = CONSTRAINT_41;
    constraint[42] = CONSTRAINT_42;
    constraint[43] = CONSTRAINT_43;
    constraint[44] = CONSTRAINT_44;
    constraint[45] = CONSTRAINT_45;
    return true;
}


// return the structure or values of the Jacobian
bool Collocation::eval_jac_g(
    Index         variableCount,
    const Number* x,
    bool          new_x,
    Index         constraintCount,
    Index         nele_jac,
    Index*        iRow,
    Index*        jCol,
    Number*       values
    ) {
    assert(variableCount == 55);
    assert(constraintCount == 46);

    if (values == NULL) {
        // return the structure of the Jacobian
        iRow[0] = JACOBIAN_X_0;
        jCol[0] = JACOBIAN_Y_0;
        iRow[1] = JACOBIAN_X_1;
        jCol[1] = JACOBIAN_Y_1;
        iRow[2] = JACOBIAN_X_2;
        jCol[2] = JACOBIAN_Y_2;
        iRow[3] = JACOBIAN_X_3;
        jCol[3] = JACOBIAN_Y_3;
        iRow[4] = JACOBIAN_X_4;
        jCol[4] = JACOBIAN_Y_4;
        iRow[5] = JACOBIAN_X_5;
        jCol[5] = JACOBIAN_Y_5;
        iRow[6] = JACOBIAN_X_6;
        jCol[6] = JACOBIAN_Y_6;
        iRow[7] = JACOBIAN_X_7;
        jCol[7] = JACOBIAN_Y_7;
        iRow[8] = JACOBIAN_X_8;
        jCol[8] = JACOBIAN_Y_8;
        iRow[9] = JACOBIAN_X_9;
        jCol[9] = JACOBIAN_Y_9;
        iRow[10] = JACOBIAN_X_10;
        jCol[10] = JACOBIAN_Y_10;
        iRow[11] = JACOBIAN_X_11;
        jCol[11] = JACOBIAN_Y_11;
        iRow[12] = JACOBIAN_X_12;
        jCol[12] = JACOBIAN_Y_12;
        iRow[13] = JACOBIAN_X_13;
        jCol[13] = JACOBIAN_Y_13;
        iRow[14] = JACOBIAN_X_14;
        jCol[14] = JACOBIAN_Y_14;
        iRow[15] = JACOBIAN_X_15;
        jCol[15] = JACOBIAN_Y_15;
        iRow[16] = JACOBIAN_X_16;
        jCol[16] = JACOBIAN_Y_16;
        iRow[17] = JACOBIAN_X_17;
        jCol[17] = JACOBIAN_Y_17;
        iRow[18] = JACOBIAN_X_18;
        jCol[18] = JACOBIAN_Y_18;
        iRow[19] = JACOBIAN_X_19;
        jCol[19] = JACOBIAN_Y_19;
        iRow[20] = JACOBIAN_X_20;
        jCol[20] = JACOBIAN_Y_20;
        iRow[21] = JACOBIAN_X_21;
        jCol[21] = JACOBIAN_Y_21;
        iRow[22] = JACOBIAN_X_22;
        jCol[22] = JACOBIAN_Y_22;
        iRow[23] = JACOBIAN_X_23;
        jCol[23] = JACOBIAN_Y_23;
        iRow[24] = JACOBIAN_X_24;
        jCol[24] = JACOBIAN_Y_24;
        iRow[25] = JACOBIAN_X_25;
        jCol[25] = JACOBIAN_Y_25;
        iRow[26] = JACOBIAN_X_26;
        jCol[26] = JACOBIAN_Y_26;
        iRow[27] = JACOBIAN_X_27;
        jCol[27] = JACOBIAN_Y_27;
        iRow[28] = JACOBIAN_X_28;
        jCol[28] = JACOBIAN_Y_28;
        iRow[29] = JACOBIAN_X_29;
        jCol[29] = JACOBIAN_Y_29;
        iRow[30] = JACOBIAN_X_30;
        jCol[30] = JACOBIAN_Y_30;
        iRow[31] = JACOBIAN_X_31;
        jCol[31] = JACOBIAN_Y_31;
        iRow[32] = JACOBIAN_X_32;
        jCol[32] = JACOBIAN_Y_32;
        iRow[33] = JACOBIAN_X_33;
        jCol[33] = JACOBIAN_Y_33;
        iRow[34] = JACOBIAN_X_34;
        jCol[34] = JACOBIAN_Y_34;
        iRow[35] = JACOBIAN_X_35;
        jCol[35] = JACOBIAN_Y_35;
        iRow[36] = JACOBIAN_X_36;
        jCol[36] = JACOBIAN_Y_36;
        iRow[37] = JACOBIAN_X_37;
        jCol[37] = JACOBIAN_Y_37;
        iRow[38] = JACOBIAN_X_38;
        jCol[38] = JACOBIAN_Y_38;
        iRow[39] = JACOBIAN_X_39;
        jCol[39] = JACOBIAN_Y_39;
        iRow[40] = JACOBIAN_X_40;
        jCol[40] = JACOBIAN_Y_40;
        iRow[41] = JACOBIAN_X_41;
        jCol[41] = JACOBIAN_Y_41;
        iRow[42] = JACOBIAN_X_42;
        jCol[42] = JACOBIAN_Y_42;
        iRow[43] = JACOBIAN_X_43;
        jCol[43] = JACOBIAN_Y_43;
        iRow[44] = JACOBIAN_X_44;
        jCol[44] = JACOBIAN_Y_44;
        iRow[45] = JACOBIAN_X_45;
        jCol[45] = JACOBIAN_Y_45;
        iRow[46] = JACOBIAN_X_46;
        jCol[46] = JACOBIAN_Y_46;
        iRow[47] = JACOBIAN_X_47;
        jCol[47] = JACOBIAN_Y_47;
        iRow[48] = JACOBIAN_X_48;
        jCol[48] = JACOBIAN_Y_48;
        iRow[49] = JACOBIAN_X_49;
        jCol[49] = JACOBIAN_Y_49;
        iRow[50] = JACOBIAN_X_50;
        jCol[50] = JACOBIAN_Y_50;
        iRow[51] = JACOBIAN_X_51;
        jCol[51] = JACOBIAN_Y_51;
        iRow[52] = JACOBIAN_X_52;
        jCol[52] = JACOBIAN_Y_52;
        iRow[53] = JACOBIAN_X_53;
        jCol[53] = JACOBIAN_Y_53;
        iRow[54] = JACOBIAN_X_54;
        jCol[54] = JACOBIAN_Y_54;
        iRow[55] = JACOBIAN_X_55;
        jCol[55] = JACOBIAN_Y_55;
        iRow[56] = JACOBIAN_X_56;
        jCol[56] = JACOBIAN_Y_56;
        iRow[57] = JACOBIAN_X_57;
        jCol[57] = JACOBIAN_Y_57;
        iRow[58] = JACOBIAN_X_58;
        jCol[58] = JACOBIAN_Y_58;
        iRow[59] = JACOBIAN_X_59;
        jCol[59] = JACOBIAN_Y_59;
        iRow[60] = JACOBIAN_X_60;
        jCol[60] = JACOBIAN_Y_60;
        iRow[61] = JACOBIAN_X_61;
        jCol[61] = JACOBIAN_Y_61;
        iRow[62] = JACOBIAN_X_62;
        jCol[62] = JACOBIAN_Y_62;
        iRow[63] = JACOBIAN_X_63;
        jCol[63] = JACOBIAN_Y_63;
        iRow[64] = JACOBIAN_X_64;
        jCol[64] = JACOBIAN_Y_64;
        iRow[65] = JACOBIAN_X_65;
        jCol[65] = JACOBIAN_Y_65;
        iRow[66] = JACOBIAN_X_66;
        jCol[66] = JACOBIAN_Y_66;
        iRow[67] = JACOBIAN_X_67;
        jCol[67] = JACOBIAN_Y_67;
        iRow[68] = JACOBIAN_X_68;
        jCol[68] = JACOBIAN_Y_68;
        iRow[69] = JACOBIAN_X_69;
        jCol[69] = JACOBIAN_Y_69;
        iRow[70] = JACOBIAN_X_70;
        jCol[70] = JACOBIAN_Y_70;
        iRow[71] = JACOBIAN_X_71;
        jCol[71] = JACOBIAN_Y_71;
        iRow[72] = JACOBIAN_X_72;
        jCol[72] = JACOBIAN_Y_72;
        iRow[73] = JACOBIAN_X_73;
        jCol[73] = JACOBIAN_Y_73;
        iRow[74] = JACOBIAN_X_74;
        jCol[74] = JACOBIAN_Y_74;
        iRow[75] = JACOBIAN_X_75;
        jCol[75] = JACOBIAN_Y_75;
        iRow[76] = JACOBIAN_X_76;
        jCol[76] = JACOBIAN_Y_76;
        iRow[77] = JACOBIAN_X_77;
        jCol[77] = JACOBIAN_Y_77;
        iRow[78] = JACOBIAN_X_78;
        jCol[78] = JACOBIAN_Y_78;
        iRow[79] = JACOBIAN_X_79;
        jCol[79] = JACOBIAN_Y_79;
        iRow[80] = JACOBIAN_X_80;
        jCol[80] = JACOBIAN_Y_80;
        iRow[81] = JACOBIAN_X_81;
        jCol[81] = JACOBIAN_Y_81;
        iRow[82] = JACOBIAN_X_82;
        jCol[82] = JACOBIAN_Y_82;
        iRow[83] = JACOBIAN_X_83;
        jCol[83] = JACOBIAN_Y_83;
        iRow[84] = JACOBIAN_X_84;
        jCol[84] = JACOBIAN_Y_84;
        iRow[85] = JACOBIAN_X_85;
        jCol[85] = JACOBIAN_Y_85;
        iRow[86] = JACOBIAN_X_86;
        jCol[86] = JACOBIAN_Y_86;
        iRow[87] = JACOBIAN_X_87;
        jCol[87] = JACOBIAN_Y_87;
        iRow[88] = JACOBIAN_X_88;
        jCol[88] = JACOBIAN_Y_88;
        iRow[89] = JACOBIAN_X_89;
        jCol[89] = JACOBIAN_Y_89;
        iRow[90] = JACOBIAN_X_90;
        jCol[90] = JACOBIAN_Y_90;
        iRow[91] = JACOBIAN_X_91;
        jCol[91] = JACOBIAN_Y_91;
        iRow[92] = JACOBIAN_X_92;
        jCol[92] = JACOBIAN_Y_92;
        iRow[93] = JACOBIAN_X_93;
        jCol[93] = JACOBIAN_Y_93;
        iRow[94] = JACOBIAN_X_94;
        jCol[94] = JACOBIAN_Y_94;
        iRow[95] = JACOBIAN_X_95;
        jCol[95] = JACOBIAN_Y_95;
        iRow[96] = JACOBIAN_X_96;
        jCol[96] = JACOBIAN_Y_96;
        iRow[97] = JACOBIAN_X_97;
        jCol[97] = JACOBIAN_Y_97;
        iRow[98] = JACOBIAN_X_98;
        jCol[98] = JACOBIAN_Y_98;
        iRow[99] = JACOBIAN_X_99;
        jCol[99] = JACOBIAN_Y_99;
        iRow[100] = JACOBIAN_X_100;
        jCol[100] = JACOBIAN_Y_100;
        iRow[101] = JACOBIAN_X_101;
        jCol[101] = JACOBIAN_Y_101;
        iRow[102] = JACOBIAN_X_102;
        jCol[102] = JACOBIAN_Y_102;
        iRow[103] = JACOBIAN_X_103;
        jCol[103] = JACOBIAN_Y_103;
        iRow[104] = JACOBIAN_X_104;
        jCol[104] = JACOBIAN_Y_104;
        iRow[105] = JACOBIAN_X_105;
        jCol[105] = JACOBIAN_Y_105;
        iRow[106] = JACOBIAN_X_106;
        jCol[106] = JACOBIAN_Y_106;
        iRow[107] = JACOBIAN_X_107;
        jCol[107] = JACOBIAN_Y_107;
        iRow[108] = JACOBIAN_X_108;
        jCol[108] = JACOBIAN_Y_108;
        iRow[109] = JACOBIAN_X_109;
        jCol[109] = JACOBIAN_Y_109;
        iRow[110] = JACOBIAN_X_110;
        jCol[110] = JACOBIAN_Y_110;
        iRow[111] = JACOBIAN_X_111;
        jCol[111] = JACOBIAN_Y_111;
        iRow[112] = JACOBIAN_X_112;
        jCol[112] = JACOBIAN_Y_112;
        iRow[113] = JACOBIAN_X_113;
        jCol[113] = JACOBIAN_Y_113;
        iRow[114] = JACOBIAN_X_114;
        jCol[114] = JACOBIAN_Y_114;
        iRow[115] = JACOBIAN_X_115;
        jCol[115] = JACOBIAN_Y_115;
        iRow[116] = JACOBIAN_X_116;
        jCol[116] = JACOBIAN_Y_116;
        iRow[117] = JACOBIAN_X_117;
        jCol[117] = JACOBIAN_Y_117;
        iRow[118] = JACOBIAN_X_118;
        jCol[118] = JACOBIAN_Y_118;
        iRow[119] = JACOBIAN_X_119;
        jCol[119] = JACOBIAN_Y_119;
        iRow[120] = JACOBIAN_X_120;
        jCol[120] = JACOBIAN_Y_120;
        iRow[121] = JACOBIAN_X_121;
        jCol[121] = JACOBIAN_Y_121;
        iRow[122] = JACOBIAN_X_122;
        jCol[122] = JACOBIAN_Y_122;
        iRow[123] = JACOBIAN_X_123;
        jCol[123] = JACOBIAN_Y_123;
        iRow[124] = JACOBIAN_X_124;
        jCol[124] = JACOBIAN_Y_124;
        iRow[125] = JACOBIAN_X_125;
        jCol[125] = JACOBIAN_Y_125;
        iRow[126] = JACOBIAN_X_126;
        jCol[126] = JACOBIAN_Y_126;
        iRow[127] = JACOBIAN_X_127;
        jCol[127] = JACOBIAN_Y_127;
        iRow[128] = JACOBIAN_X_128;
        jCol[128] = JACOBIAN_Y_128;
        iRow[129] = JACOBIAN_X_129;
        jCol[129] = JACOBIAN_Y_129;
        iRow[130] = JACOBIAN_X_130;
        jCol[130] = JACOBIAN_Y_130;
        iRow[131] = JACOBIAN_X_131;
        jCol[131] = JACOBIAN_Y_131;
        iRow[132] = JACOBIAN_X_132;
        jCol[132] = JACOBIAN_Y_132;
        iRow[133] = JACOBIAN_X_133;
        jCol[133] = JACOBIAN_Y_133;
        iRow[134] = JACOBIAN_X_134;
        jCol[134] = JACOBIAN_Y_134;
        iRow[135] = JACOBIAN_X_135;
        jCol[135] = JACOBIAN_Y_135;
        iRow[136] = JACOBIAN_X_136;
        jCol[136] = JACOBIAN_Y_136;
        iRow[137] = JACOBIAN_X_137;
        jCol[137] = JACOBIAN_Y_137;
        iRow[138] = JACOBIAN_X_138;
        jCol[138] = JACOBIAN_Y_138;
        iRow[139] = JACOBIAN_X_139;
        jCol[139] = JACOBIAN_Y_139;
        iRow[140] = JACOBIAN_X_140;
        jCol[140] = JACOBIAN_Y_140;
        iRow[141] = JACOBIAN_X_141;
        jCol[141] = JACOBIAN_Y_141;
        iRow[142] = JACOBIAN_X_142;
        jCol[142] = JACOBIAN_Y_142;
        iRow[143] = JACOBIAN_X_143;
        jCol[143] = JACOBIAN_Y_143;
        iRow[144] = JACOBIAN_X_144;
        jCol[144] = JACOBIAN_Y_144;
        iRow[145] = JACOBIAN_X_145;
        jCol[145] = JACOBIAN_Y_145;
        iRow[146] = JACOBIAN_X_146;
        jCol[146] = JACOBIAN_Y_146;
        iRow[147] = JACOBIAN_X_147;
        jCol[147] = JACOBIAN_Y_147;
        iRow[148] = JACOBIAN_X_148;
        jCol[148] = JACOBIAN_Y_148;
        iRow[149] = JACOBIAN_X_149;
        jCol[149] = JACOBIAN_Y_149;
        iRow[150] = JACOBIAN_X_150;
        jCol[150] = JACOBIAN_Y_150;
        iRow[151] = JACOBIAN_X_151;
        jCol[151] = JACOBIAN_Y_151;
        iRow[152] = JACOBIAN_X_152;
        jCol[152] = JACOBIAN_Y_152;
        iRow[153] = JACOBIAN_X_153;
        jCol[153] = JACOBIAN_Y_153;
        iRow[154] = JACOBIAN_X_154;
        jCol[154] = JACOBIAN_Y_154;
        iRow[155] = JACOBIAN_X_155;
        jCol[155] = JACOBIAN_Y_155;
        iRow[156] = JACOBIAN_X_156;
        jCol[156] = JACOBIAN_Y_156;
        iRow[157] = JACOBIAN_X_157;
        jCol[157] = JACOBIAN_Y_157;
        iRow[158] = JACOBIAN_X_158;
        jCol[158] = JACOBIAN_Y_158;
        iRow[159] = JACOBIAN_X_159;
        jCol[159] = JACOBIAN_Y_159;
        iRow[160] = JACOBIAN_X_160;
        jCol[160] = JACOBIAN_Y_160;
        iRow[161] = JACOBIAN_X_161;
        jCol[161] = JACOBIAN_Y_161;
        iRow[162] = JACOBIAN_X_162;
        jCol[162] = JACOBIAN_Y_162;
        iRow[163] = JACOBIAN_X_163;
        jCol[163] = JACOBIAN_Y_163;
        iRow[164] = JACOBIAN_X_164;
        jCol[164] = JACOBIAN_Y_164;
        iRow[165] = JACOBIAN_X_165;
        jCol[165] = JACOBIAN_Y_165;
        iRow[166] = JACOBIAN_X_166;
        jCol[166] = JACOBIAN_Y_166;
        iRow[167] = JACOBIAN_X_167;
        jCol[167] = JACOBIAN_Y_167;
        iRow[168] = JACOBIAN_X_168;
        jCol[168] = JACOBIAN_Y_168;
        iRow[169] = JACOBIAN_X_169;
        jCol[169] = JACOBIAN_Y_169;
        iRow[170] = JACOBIAN_X_170;
        jCol[170] = JACOBIAN_Y_170;
        iRow[171] = JACOBIAN_X_171;
        jCol[171] = JACOBIAN_Y_171;
        iRow[172] = JACOBIAN_X_172;
        jCol[172] = JACOBIAN_Y_172;
        iRow[173] = JACOBIAN_X_173;
        jCol[173] = JACOBIAN_Y_173;
        iRow[174] = JACOBIAN_X_174;
        jCol[174] = JACOBIAN_Y_174;
        iRow[175] = JACOBIAN_X_175;
        jCol[175] = JACOBIAN_Y_175;
        iRow[176] = JACOBIAN_X_176;
        jCol[176] = JACOBIAN_Y_176;
        iRow[177] = JACOBIAN_X_177;
        jCol[177] = JACOBIAN_Y_177;
        iRow[178] = JACOBIAN_X_178;
        jCol[178] = JACOBIAN_Y_178;
        iRow[179] = JACOBIAN_X_179;
        jCol[179] = JACOBIAN_Y_179;
        iRow[180] = JACOBIAN_X_180;
        jCol[180] = JACOBIAN_Y_180;
        iRow[181] = JACOBIAN_X_181;
        jCol[181] = JACOBIAN_Y_181;
        iRow[182] = JACOBIAN_X_182;
        jCol[182] = JACOBIAN_Y_182;
        iRow[183] = JACOBIAN_X_183;
        jCol[183] = JACOBIAN_Y_183;
        iRow[184] = JACOBIAN_X_184;
        jCol[184] = JACOBIAN_Y_184;
        iRow[185] = JACOBIAN_X_185;
        jCol[185] = JACOBIAN_Y_185;
        iRow[186] = JACOBIAN_X_186;
        jCol[186] = JACOBIAN_Y_186;
        iRow[187] = JACOBIAN_X_187;
        jCol[187] = JACOBIAN_Y_187;
        iRow[188] = JACOBIAN_X_188;
        jCol[188] = JACOBIAN_Y_188;
        iRow[189] = JACOBIAN_X_189;
        jCol[189] = JACOBIAN_Y_189;
        iRow[190] = JACOBIAN_X_190;
        jCol[190] = JACOBIAN_Y_190;
        iRow[191] = JACOBIAN_X_191;
        jCol[191] = JACOBIAN_Y_191;
        iRow[192] = JACOBIAN_X_192;
        jCol[192] = JACOBIAN_Y_192;
        iRow[193] = JACOBIAN_X_193;
        jCol[193] = JACOBIAN_Y_193;
        iRow[194] = JACOBIAN_X_194;
        jCol[194] = JACOBIAN_Y_194;
        iRow[195] = JACOBIAN_X_195;
        jCol[195] = JACOBIAN_Y_195;
        iRow[196] = JACOBIAN_X_196;
        jCol[196] = JACOBIAN_Y_196;
        iRow[197] = JACOBIAN_X_197;
        jCol[197] = JACOBIAN_Y_197;
        iRow[198] = JACOBIAN_X_198;
        jCol[198] = JACOBIAN_Y_198;
        iRow[199] = JACOBIAN_X_199;
        jCol[199] = JACOBIAN_Y_199;
        iRow[200] = JACOBIAN_X_200;
        jCol[200] = JACOBIAN_Y_200;
        iRow[201] = JACOBIAN_X_201;
        jCol[201] = JACOBIAN_Y_201;
        iRow[202] = JACOBIAN_X_202;
        jCol[202] = JACOBIAN_Y_202;
        iRow[203] = JACOBIAN_X_203;
        jCol[203] = JACOBIAN_Y_203;
        iRow[204] = JACOBIAN_X_204;
        jCol[204] = JACOBIAN_Y_204;
        iRow[205] = JACOBIAN_X_205;
        jCol[205] = JACOBIAN_Y_205;
        iRow[206] = JACOBIAN_X_206;
        jCol[206] = JACOBIAN_Y_206;
        iRow[207] = JACOBIAN_X_207;
        jCol[207] = JACOBIAN_Y_207;
        iRow[208] = JACOBIAN_X_208;
        jCol[208] = JACOBIAN_Y_208;
        iRow[209] = JACOBIAN_X_209;
        jCol[209] = JACOBIAN_Y_209;
        iRow[210] = JACOBIAN_X_210;
        jCol[210] = JACOBIAN_Y_210;
        iRow[211] = JACOBIAN_X_211;
        jCol[211] = JACOBIAN_Y_211;
        iRow[212] = JACOBIAN_X_212;
        jCol[212] = JACOBIAN_Y_212;
        iRow[213] = JACOBIAN_X_213;
        jCol[213] = JACOBIAN_Y_213;
        iRow[214] = JACOBIAN_X_214;
        jCol[214] = JACOBIAN_Y_214;
        iRow[215] = JACOBIAN_X_215;
        jCol[215] = JACOBIAN_Y_215;
        iRow[216] = JACOBIAN_X_216;
        jCol[216] = JACOBIAN_Y_216;
        iRow[217] = JACOBIAN_X_217;
        jCol[217] = JACOBIAN_Y_217;
        iRow[218] = JACOBIAN_X_218;
        jCol[218] = JACOBIAN_Y_218;
        iRow[219] = JACOBIAN_X_219;
        jCol[219] = JACOBIAN_Y_219;
        iRow[220] = JACOBIAN_X_220;
        jCol[220] = JACOBIAN_Y_220;
        iRow[221] = JACOBIAN_X_221;
        jCol[221] = JACOBIAN_Y_221;
        iRow[222] = JACOBIAN_X_222;
        jCol[222] = JACOBIAN_Y_222;
        iRow[223] = JACOBIAN_X_223;
        jCol[223] = JACOBIAN_Y_223;
        iRow[224] = JACOBIAN_X_224;
        jCol[224] = JACOBIAN_Y_224;
        iRow[225] = JACOBIAN_X_225;
        jCol[225] = JACOBIAN_Y_225;
        iRow[226] = JACOBIAN_X_226;
        jCol[226] = JACOBIAN_Y_226;
        iRow[227] = JACOBIAN_X_227;
        jCol[227] = JACOBIAN_Y_227;
        iRow[228] = JACOBIAN_X_228;
        jCol[228] = JACOBIAN_Y_228;
        iRow[229] = JACOBIAN_X_229;
        jCol[229] = JACOBIAN_Y_229;
        iRow[230] = JACOBIAN_X_230;
        jCol[230] = JACOBIAN_Y_230;
        iRow[231] = JACOBIAN_X_231;
        jCol[231] = JACOBIAN_Y_231;
        iRow[232] = JACOBIAN_X_232;
        jCol[232] = JACOBIAN_Y_232;
        iRow[233] = JACOBIAN_X_233;
        jCol[233] = JACOBIAN_Y_233;
        iRow[234] = JACOBIAN_X_234;
        jCol[234] = JACOBIAN_Y_234;
        iRow[235] = JACOBIAN_X_235;
        jCol[235] = JACOBIAN_Y_235;
        iRow[236] = JACOBIAN_X_236;
        jCol[236] = JACOBIAN_Y_236;
        iRow[237] = JACOBIAN_X_237;
        jCol[237] = JACOBIAN_Y_237;
        iRow[238] = JACOBIAN_X_238;
        jCol[238] = JACOBIAN_Y_238;
        iRow[239] = JACOBIAN_X_239;
        jCol[239] = JACOBIAN_Y_239;
        iRow[240] = JACOBIAN_X_240;
        jCol[240] = JACOBIAN_Y_240;
        iRow[241] = JACOBIAN_X_241;
        jCol[241] = JACOBIAN_Y_241;
        iRow[242] = JACOBIAN_X_242;
        jCol[242] = JACOBIAN_Y_242;
        iRow[243] = JACOBIAN_X_243;
        jCol[243] = JACOBIAN_Y_243;
        iRow[244] = JACOBIAN_X_244;
        jCol[244] = JACOBIAN_Y_244;
        iRow[245] = JACOBIAN_X_245;
        jCol[245] = JACOBIAN_Y_245;
        iRow[246] = JACOBIAN_X_246;
        jCol[246] = JACOBIAN_Y_246;
        iRow[247] = JACOBIAN_X_247;
        jCol[247] = JACOBIAN_Y_247;
        iRow[248] = JACOBIAN_X_248;
        jCol[248] = JACOBIAN_Y_248;
        iRow[249] = JACOBIAN_X_249;
        jCol[249] = JACOBIAN_Y_249;
        iRow[250] = JACOBIAN_X_250;
        jCol[250] = JACOBIAN_Y_250;
        iRow[251] = JACOBIAN_X_251;
        jCol[251] = JACOBIAN_Y_251;
        iRow[252] = JACOBIAN_X_252;
        jCol[252] = JACOBIAN_Y_252;
        iRow[253] = JACOBIAN_X_253;
        jCol[253] = JACOBIAN_Y_253;
        iRow[254] = JACOBIAN_X_254;
        jCol[254] = JACOBIAN_Y_254;
        iRow[255] = JACOBIAN_X_255;
        jCol[255] = JACOBIAN_Y_255;
        iRow[256] = JACOBIAN_X_256;
        jCol[256] = JACOBIAN_Y_256;
        iRow[257] = JACOBIAN_X_257;
        jCol[257] = JACOBIAN_Y_257;
        iRow[258] = JACOBIAN_X_258;
        jCol[258] = JACOBIAN_Y_258;
        iRow[259] = JACOBIAN_X_259;
        jCol[259] = JACOBIAN_Y_259;
        iRow[260] = JACOBIAN_X_260;
        jCol[260] = JACOBIAN_Y_260;
        iRow[261] = JACOBIAN_X_261;
        jCol[261] = JACOBIAN_Y_261;
        iRow[262] = JACOBIAN_X_262;
        jCol[262] = JACOBIAN_Y_262;
        iRow[263] = JACOBIAN_X_263;
        jCol[263] = JACOBIAN_Y_263;
        iRow[264] = JACOBIAN_X_264;
        jCol[264] = JACOBIAN_Y_264;
        iRow[265] = JACOBIAN_X_265;
        jCol[265] = JACOBIAN_Y_265;
        iRow[266] = JACOBIAN_X_266;
        jCol[266] = JACOBIAN_Y_266;
        iRow[267] = JACOBIAN_X_267;
        jCol[267] = JACOBIAN_Y_267;
        iRow[268] = JACOBIAN_X_268;
        jCol[268] = JACOBIAN_Y_268;
        iRow[269] = JACOBIAN_X_269;
        jCol[269] = JACOBIAN_Y_269;
        iRow[270] = JACOBIAN_X_270;
        jCol[270] = JACOBIAN_Y_270;
        iRow[271] = JACOBIAN_X_271;
        jCol[271] = JACOBIAN_Y_271;
        iRow[272] = JACOBIAN_X_272;
        jCol[272] = JACOBIAN_Y_272;
        iRow[273] = JACOBIAN_X_273;
        jCol[273] = JACOBIAN_Y_273;
        iRow[274] = JACOBIAN_X_274;
        jCol[274] = JACOBIAN_Y_274;
        iRow[275] = JACOBIAN_X_275;
        jCol[275] = JACOBIAN_Y_275;
        iRow[276] = JACOBIAN_X_276;
        jCol[276] = JACOBIAN_Y_276;
        iRow[277] = JACOBIAN_X_277;
        jCol[277] = JACOBIAN_Y_277;
        iRow[278] = JACOBIAN_X_278;
        jCol[278] = JACOBIAN_Y_278;
        iRow[279] = JACOBIAN_X_279;
        jCol[279] = JACOBIAN_Y_279;
        iRow[280] = JACOBIAN_X_280;
        jCol[280] = JACOBIAN_Y_280;
        iRow[281] = JACOBIAN_X_281;
        jCol[281] = JACOBIAN_Y_281;
        iRow[282] = JACOBIAN_X_282;
        jCol[282] = JACOBIAN_Y_282;
        iRow[283] = JACOBIAN_X_283;
        jCol[283] = JACOBIAN_Y_283;
        iRow[284] = JACOBIAN_X_284;
        jCol[284] = JACOBIAN_Y_284;
        iRow[285] = JACOBIAN_X_285;
        jCol[285] = JACOBIAN_Y_285;
        iRow[286] = JACOBIAN_X_286;
        jCol[286] = JACOBIAN_Y_286;
        iRow[287] = JACOBIAN_X_287;
        jCol[287] = JACOBIAN_Y_287;
        iRow[288] = JACOBIAN_X_288;
        jCol[288] = JACOBIAN_Y_288;
        iRow[289] = JACOBIAN_X_289;
        jCol[289] = JACOBIAN_Y_289;
        iRow[290] = JACOBIAN_X_290;
        jCol[290] = JACOBIAN_Y_290;
        iRow[291] = JACOBIAN_X_291;
        jCol[291] = JACOBIAN_Y_291;
        iRow[292] = JACOBIAN_X_292;
        jCol[292] = JACOBIAN_Y_292;
        iRow[293] = JACOBIAN_X_293;
        jCol[293] = JACOBIAN_Y_293;
        iRow[294] = JACOBIAN_X_294;
        jCol[294] = JACOBIAN_Y_294;
        iRow[295] = JACOBIAN_X_295;
        jCol[295] = JACOBIAN_Y_295;
        iRow[296] = JACOBIAN_X_296;
        jCol[296] = JACOBIAN_Y_296;
        iRow[297] = JACOBIAN_X_297;
        jCol[297] = JACOBIAN_Y_297;
        iRow[298] = JACOBIAN_X_298;
        jCol[298] = JACOBIAN_Y_298;
        iRow[299] = JACOBIAN_X_299;
        jCol[299] = JACOBIAN_Y_299;
        iRow[300] = JACOBIAN_X_300;
        jCol[300] = JACOBIAN_Y_300;
        iRow[301] = JACOBIAN_X_301;
        jCol[301] = JACOBIAN_Y_301;
        iRow[302] = JACOBIAN_X_302;
        jCol[302] = JACOBIAN_Y_302;
        iRow[303] = JACOBIAN_X_303;
        jCol[303] = JACOBIAN_Y_303;
        iRow[304] = JACOBIAN_X_304;
        jCol[304] = JACOBIAN_Y_304;
        iRow[305] = JACOBIAN_X_305;
        jCol[305] = JACOBIAN_Y_305;
        iRow[306] = JACOBIAN_X_306;
        jCol[306] = JACOBIAN_Y_306;
        iRow[307] = JACOBIAN_X_307;
        jCol[307] = JACOBIAN_Y_307;
        iRow[308] = JACOBIAN_X_308;
        jCol[308] = JACOBIAN_Y_308;
        iRow[309] = JACOBIAN_X_309;
        jCol[309] = JACOBIAN_Y_309;
        iRow[310] = JACOBIAN_X_310;
        jCol[310] = JACOBIAN_Y_310;
        iRow[311] = JACOBIAN_X_311;
        jCol[311] = JACOBIAN_Y_311;
        iRow[312] = JACOBIAN_X_312;
        jCol[312] = JACOBIAN_Y_312;
        iRow[313] = JACOBIAN_X_313;
        jCol[313] = JACOBIAN_Y_313;
        iRow[314] = JACOBIAN_X_314;
        jCol[314] = JACOBIAN_Y_314;
        iRow[315] = JACOBIAN_X_315;
        jCol[315] = JACOBIAN_Y_315;
        iRow[316] = JACOBIAN_X_316;
        jCol[316] = JACOBIAN_Y_316;
        iRow[317] = JACOBIAN_X_317;
        jCol[317] = JACOBIAN_Y_317;
        iRow[318] = JACOBIAN_X_318;
        jCol[318] = JACOBIAN_Y_318;
        iRow[319] = JACOBIAN_X_319;
        jCol[319] = JACOBIAN_Y_319;
        iRow[320] = JACOBIAN_X_320;
        jCol[320] = JACOBIAN_Y_320;
        iRow[321] = JACOBIAN_X_321;
        jCol[321] = JACOBIAN_Y_321;
        iRow[322] = JACOBIAN_X_322;
        jCol[322] = JACOBIAN_Y_322;
        iRow[323] = JACOBIAN_X_323;
        jCol[323] = JACOBIAN_Y_323;
        iRow[324] = JACOBIAN_X_324;
        jCol[324] = JACOBIAN_Y_324;
        iRow[325] = JACOBIAN_X_325;
        jCol[325] = JACOBIAN_Y_325;
        iRow[326] = JACOBIAN_X_326;
        jCol[326] = JACOBIAN_Y_326;
        iRow[327] = JACOBIAN_X_327;
        jCol[327] = JACOBIAN_Y_327;
        iRow[328] = JACOBIAN_X_328;
        jCol[328] = JACOBIAN_Y_328;
        iRow[329] = JACOBIAN_X_329;
        jCol[329] = JACOBIAN_Y_329;
        iRow[330] = JACOBIAN_X_330;
        jCol[330] = JACOBIAN_Y_330;
        iRow[331] = JACOBIAN_X_331;
        jCol[331] = JACOBIAN_Y_331;
        iRow[332] = JACOBIAN_X_332;
        jCol[332] = JACOBIAN_Y_332;
        iRow[333] = JACOBIAN_X_333;
        jCol[333] = JACOBIAN_Y_333;
        iRow[334] = JACOBIAN_X_334;
        jCol[334] = JACOBIAN_Y_334;
        iRow[335] = JACOBIAN_X_335;
        jCol[335] = JACOBIAN_Y_335;
        iRow[336] = JACOBIAN_X_336;
        jCol[336] = JACOBIAN_Y_336;
        iRow[337] = JACOBIAN_X_337;
        jCol[337] = JACOBIAN_Y_337;
        iRow[338] = JACOBIAN_X_338;
        jCol[338] = JACOBIAN_Y_338;
        iRow[339] = JACOBIAN_X_339;
        jCol[339] = JACOBIAN_Y_339;
        iRow[340] = JACOBIAN_X_340;
        jCol[340] = JACOBIAN_Y_340;
        iRow[341] = JACOBIAN_X_341;
        jCol[341] = JACOBIAN_Y_341;
        iRow[342] = JACOBIAN_X_342;
        jCol[342] = JACOBIAN_Y_342;
        iRow[343] = JACOBIAN_X_343;
        jCol[343] = JACOBIAN_Y_343;
        iRow[344] = JACOBIAN_X_344;
        jCol[344] = JACOBIAN_Y_344;
        iRow[345] = JACOBIAN_X_345;
        jCol[345] = JACOBIAN_Y_345;
        iRow[346] = JACOBIAN_X_346;
        jCol[346] = JACOBIAN_Y_346;
        iRow[347] = JACOBIAN_X_347;
        jCol[347] = JACOBIAN_Y_347;
        iRow[348] = JACOBIAN_X_348;
        jCol[348] = JACOBIAN_Y_348;
        iRow[349] = JACOBIAN_X_349;
        jCol[349] = JACOBIAN_Y_349;
        iRow[350] = JACOBIAN_X_350;
        jCol[350] = JACOBIAN_Y_350;
        iRow[351] = JACOBIAN_X_351;
        jCol[351] = JACOBIAN_Y_351;
        iRow[352] = JACOBIAN_X_352;
        jCol[352] = JACOBIAN_Y_352;
        iRow[353] = JACOBIAN_X_353;
        jCol[353] = JACOBIAN_Y_353;
        iRow[354] = JACOBIAN_X_354;
        jCol[354] = JACOBIAN_Y_354;
        iRow[355] = JACOBIAN_X_355;
        jCol[355] = JACOBIAN_Y_355;
        iRow[356] = JACOBIAN_X_356;
        jCol[356] = JACOBIAN_Y_356;
        iRow[357] = JACOBIAN_X_357;
        jCol[357] = JACOBIAN_Y_357;
        iRow[358] = JACOBIAN_X_358;
        jCol[358] = JACOBIAN_Y_358;
        iRow[359] = JACOBIAN_X_359;
        jCol[359] = JACOBIAN_Y_359;
        iRow[360] = JACOBIAN_X_360;
        jCol[360] = JACOBIAN_Y_360;
        iRow[361] = JACOBIAN_X_361;
        jCol[361] = JACOBIAN_Y_361;
        iRow[362] = JACOBIAN_X_362;
        jCol[362] = JACOBIAN_Y_362;
        iRow[363] = JACOBIAN_X_363;
        jCol[363] = JACOBIAN_Y_363;
        iRow[364] = JACOBIAN_X_364;
        jCol[364] = JACOBIAN_Y_364;
        iRow[365] = JACOBIAN_X_365;
        jCol[365] = JACOBIAN_Y_365;
        iRow[366] = JACOBIAN_X_366;
        jCol[366] = JACOBIAN_Y_366;
        iRow[367] = JACOBIAN_X_367;
        jCol[367] = JACOBIAN_Y_367;
        iRow[368] = JACOBIAN_X_368;
        jCol[368] = JACOBIAN_Y_368;
        iRow[369] = JACOBIAN_X_369;
        jCol[369] = JACOBIAN_Y_369;
        iRow[370] = JACOBIAN_X_370;
        jCol[370] = JACOBIAN_Y_370;
        iRow[371] = JACOBIAN_X_371;
        jCol[371] = JACOBIAN_Y_371;
        iRow[372] = JACOBIAN_X_372;
        jCol[372] = JACOBIAN_Y_372;
        iRow[373] = JACOBIAN_X_373;
        jCol[373] = JACOBIAN_Y_373;
        iRow[374] = JACOBIAN_X_374;
        jCol[374] = JACOBIAN_Y_374;
        iRow[375] = JACOBIAN_X_375;
        jCol[375] = JACOBIAN_Y_375;
        iRow[376] = JACOBIAN_X_376;
        jCol[376] = JACOBIAN_Y_376;
        iRow[377] = JACOBIAN_X_377;
        jCol[377] = JACOBIAN_Y_377;
        iRow[378] = JACOBIAN_X_378;
        jCol[378] = JACOBIAN_Y_378;
        iRow[379] = JACOBIAN_X_379;
        jCol[379] = JACOBIAN_Y_379;
        iRow[380] = JACOBIAN_X_380;
        jCol[380] = JACOBIAN_Y_380;
        iRow[381] = JACOBIAN_X_381;
        jCol[381] = JACOBIAN_Y_381;
        iRow[382] = JACOBIAN_X_382;
        jCol[382] = JACOBIAN_Y_382;
        iRow[383] = JACOBIAN_X_383;
        jCol[383] = JACOBIAN_Y_383;
        iRow[384] = JACOBIAN_X_384;
        jCol[384] = JACOBIAN_Y_384;
        iRow[385] = JACOBIAN_X_385;
        jCol[385] = JACOBIAN_Y_385;
        iRow[386] = JACOBIAN_X_386;
        jCol[386] = JACOBIAN_Y_386;
        iRow[387] = JACOBIAN_X_387;
        jCol[387] = JACOBIAN_Y_387;
        iRow[388] = JACOBIAN_X_388;
        jCol[388] = JACOBIAN_Y_388;
        iRow[389] = JACOBIAN_X_389;
        jCol[389] = JACOBIAN_Y_389;
        iRow[390] = JACOBIAN_X_390;
        jCol[390] = JACOBIAN_Y_390;
        iRow[391] = JACOBIAN_X_391;
        jCol[391] = JACOBIAN_Y_391;
        iRow[392] = JACOBIAN_X_392;
        jCol[392] = JACOBIAN_Y_392;
        iRow[393] = JACOBIAN_X_393;
        jCol[393] = JACOBIAN_Y_393;
        iRow[394] = JACOBIAN_X_394;
        jCol[394] = JACOBIAN_Y_394;
        iRow[395] = JACOBIAN_X_395;
        jCol[395] = JACOBIAN_Y_395;
        iRow[396] = JACOBIAN_X_396;
        jCol[396] = JACOBIAN_Y_396;
        iRow[397] = JACOBIAN_X_397;
        jCol[397] = JACOBIAN_Y_397;
        iRow[398] = JACOBIAN_X_398;
        jCol[398] = JACOBIAN_Y_398;
        iRow[399] = JACOBIAN_X_399;
        jCol[399] = JACOBIAN_Y_399;
        iRow[400] = JACOBIAN_X_400;
        jCol[400] = JACOBIAN_Y_400;
        iRow[401] = JACOBIAN_X_401;
        jCol[401] = JACOBIAN_Y_401;
        iRow[402] = JACOBIAN_X_402;
        jCol[402] = JACOBIAN_Y_402;
        iRow[403] = JACOBIAN_X_403;
        jCol[403] = JACOBIAN_Y_403;
        iRow[404] = JACOBIAN_X_404;
        jCol[404] = JACOBIAN_Y_404;
        iRow[405] = JACOBIAN_X_405;
        jCol[405] = JACOBIAN_Y_405;
        iRow[406] = JACOBIAN_X_406;
        jCol[406] = JACOBIAN_Y_406;
        iRow[407] = JACOBIAN_X_407;
        jCol[407] = JACOBIAN_Y_407;
        iRow[408] = JACOBIAN_X_408;
        jCol[408] = JACOBIAN_Y_408;
        iRow[409] = JACOBIAN_X_409;
        jCol[409] = JACOBIAN_Y_409;
        iRow[410] = JACOBIAN_X_410;
        jCol[410] = JACOBIAN_Y_410;
        iRow[411] = JACOBIAN_X_411;
        jCol[411] = JACOBIAN_Y_411;
        iRow[412] = JACOBIAN_X_412;
        jCol[412] = JACOBIAN_Y_412;
        iRow[413] = JACOBIAN_X_413;
        jCol[413] = JACOBIAN_Y_413;
        iRow[414] = JACOBIAN_X_414;
        jCol[414] = JACOBIAN_Y_414;
        iRow[415] = JACOBIAN_X_415;
        jCol[415] = JACOBIAN_Y_415;
        iRow[416] = JACOBIAN_X_416;
        jCol[416] = JACOBIAN_Y_416;
        iRow[417] = JACOBIAN_X_417;
        jCol[417] = JACOBIAN_Y_417;
        iRow[418] = JACOBIAN_X_418;
        jCol[418] = JACOBIAN_Y_418;
        iRow[419] = JACOBIAN_X_419;
        jCol[419] = JACOBIAN_Y_419;
        iRow[420] = JACOBIAN_X_420;
        jCol[420] = JACOBIAN_Y_420;
        iRow[421] = JACOBIAN_X_421;
        jCol[421] = JACOBIAN_Y_421;
        iRow[422] = JACOBIAN_X_422;
        jCol[422] = JACOBIAN_Y_422;
        iRow[423] = JACOBIAN_X_423;
        jCol[423] = JACOBIAN_Y_423;
        iRow[424] = JACOBIAN_X_424;
        jCol[424] = JACOBIAN_Y_424;
        iRow[425] = JACOBIAN_X_425;
        jCol[425] = JACOBIAN_Y_425;
        iRow[426] = JACOBIAN_X_426;
        jCol[426] = JACOBIAN_Y_426;
        iRow[427] = JACOBIAN_X_427;
        jCol[427] = JACOBIAN_Y_427;
        iRow[428] = JACOBIAN_X_428;
        jCol[428] = JACOBIAN_Y_428;
        iRow[429] = JACOBIAN_X_429;
        jCol[429] = JACOBIAN_Y_429;
        iRow[430] = JACOBIAN_X_430;
        jCol[430] = JACOBIAN_Y_430;
        iRow[431] = JACOBIAN_X_431;
        jCol[431] = JACOBIAN_Y_431;
        iRow[432] = JACOBIAN_X_432;
        jCol[432] = JACOBIAN_Y_432;
        iRow[433] = JACOBIAN_X_433;
        jCol[433] = JACOBIAN_Y_433;
        iRow[434] = JACOBIAN_X_434;
        jCol[434] = JACOBIAN_Y_434;
        iRow[435] = JACOBIAN_X_435;
        jCol[435] = JACOBIAN_Y_435;
        iRow[436] = JACOBIAN_X_436;
        jCol[436] = JACOBIAN_Y_436;
        iRow[437] = JACOBIAN_X_437;
        jCol[437] = JACOBIAN_Y_437;
        iRow[438] = JACOBIAN_X_438;
        jCol[438] = JACOBIAN_Y_438;
        iRow[439] = JACOBIAN_X_439;
        jCol[439] = JACOBIAN_Y_439;
        iRow[440] = JACOBIAN_X_440;
        jCol[440] = JACOBIAN_Y_440;
        iRow[441] = JACOBIAN_X_441;
        jCol[441] = JACOBIAN_Y_441;
        iRow[442] = JACOBIAN_X_442;
        jCol[442] = JACOBIAN_Y_442;
        iRow[443] = JACOBIAN_X_443;
        jCol[443] = JACOBIAN_Y_443;
        iRow[444] = JACOBIAN_X_444;
        jCol[444] = JACOBIAN_Y_444;
        iRow[445] = JACOBIAN_X_445;
        jCol[445] = JACOBIAN_Y_445;
        iRow[446] = JACOBIAN_X_446;
        jCol[446] = JACOBIAN_Y_446;
        iRow[447] = JACOBIAN_X_447;
        jCol[447] = JACOBIAN_Y_447;
        iRow[448] = JACOBIAN_X_448;
        jCol[448] = JACOBIAN_Y_448;
        iRow[449] = JACOBIAN_X_449;
        jCol[449] = JACOBIAN_Y_449;
        iRow[450] = JACOBIAN_X_450;
        jCol[450] = JACOBIAN_Y_450;
        iRow[451] = JACOBIAN_X_451;
        jCol[451] = JACOBIAN_Y_451;
        iRow[452] = JACOBIAN_X_452;
        jCol[452] = JACOBIAN_Y_452;
        iRow[453] = JACOBIAN_X_453;
        jCol[453] = JACOBIAN_Y_453;
        iRow[454] = JACOBIAN_X_454;
        jCol[454] = JACOBIAN_Y_454;
        iRow[455] = JACOBIAN_X_455;
        jCol[455] = JACOBIAN_Y_455;
        iRow[456] = JACOBIAN_X_456;
        jCol[456] = JACOBIAN_Y_456;
        iRow[457] = JACOBIAN_X_457;
        jCol[457] = JACOBIAN_Y_457;
        iRow[458] = JACOBIAN_X_458;
        jCol[458] = JACOBIAN_Y_458;
        iRow[459] = JACOBIAN_X_459;
        jCol[459] = JACOBIAN_Y_459;
        iRow[460] = JACOBIAN_X_460;
        jCol[460] = JACOBIAN_Y_460;
        iRow[461] = JACOBIAN_X_461;
        jCol[461] = JACOBIAN_Y_461;
    } else {
        values[0] = JACOBIAN_VAL_0;
        values[1] = JACOBIAN_VAL_1;
        values[2] = JACOBIAN_VAL_2;
        values[3] = JACOBIAN_VAL_3;
        values[4] = JACOBIAN_VAL_4;
        values[5] = JACOBIAN_VAL_5;
        values[6] = JACOBIAN_VAL_6;
        values[7] = JACOBIAN_VAL_7;
        values[8] = JACOBIAN_VAL_8;
        values[9] = JACOBIAN_VAL_9;
        values[10] = JACOBIAN_VAL_10;
        values[11] = JACOBIAN_VAL_11;
        values[12] = JACOBIAN_VAL_12;
        values[13] = JACOBIAN_VAL_13;
        values[14] = JACOBIAN_VAL_14;
        values[15] = JACOBIAN_VAL_15;
        values[16] = JACOBIAN_VAL_16;
        values[17] = JACOBIAN_VAL_17;
        values[18] = JACOBIAN_VAL_18;
        values[19] = JACOBIAN_VAL_19;
        values[20] = JACOBIAN_VAL_20;
        values[21] = JACOBIAN_VAL_21;
        values[22] = JACOBIAN_VAL_22;
        values[23] = JACOBIAN_VAL_23;
        values[24] = JACOBIAN_VAL_24;
        values[25] = JACOBIAN_VAL_25;
        values[26] = JACOBIAN_VAL_26;
        values[27] = JACOBIAN_VAL_27;
        values[28] = JACOBIAN_VAL_28;
        values[29] = JACOBIAN_VAL_29;
        values[30] = JACOBIAN_VAL_30;
        values[31] = JACOBIAN_VAL_31;
        values[32] = JACOBIAN_VAL_32;
        values[33] = JACOBIAN_VAL_33;
        values[34] = JACOBIAN_VAL_34;
        values[35] = JACOBIAN_VAL_35;
        values[36] = JACOBIAN_VAL_36;
        values[37] = JACOBIAN_VAL_37;
        values[38] = JACOBIAN_VAL_38;
        values[39] = JACOBIAN_VAL_39;
        values[40] = JACOBIAN_VAL_40;
        values[41] = JACOBIAN_VAL_41;
        values[42] = JACOBIAN_VAL_42;
        values[43] = JACOBIAN_VAL_43;
        values[44] = JACOBIAN_VAL_44;
        values[45] = JACOBIAN_VAL_45;
        values[46] = JACOBIAN_VAL_46;
        values[47] = JACOBIAN_VAL_47;
        values[48] = JACOBIAN_VAL_48;
        values[49] = JACOBIAN_VAL_49;
        values[50] = JACOBIAN_VAL_50;
        values[51] = JACOBIAN_VAL_51;
        values[52] = JACOBIAN_VAL_52;
        values[53] = JACOBIAN_VAL_53;
        values[54] = JACOBIAN_VAL_54;
        values[55] = JACOBIAN_VAL_55;
        values[56] = JACOBIAN_VAL_56;
        values[57] = JACOBIAN_VAL_57;
        values[58] = JACOBIAN_VAL_58;
        values[59] = JACOBIAN_VAL_59;
        values[60] = JACOBIAN_VAL_60;
        values[61] = JACOBIAN_VAL_61;
        values[62] = JACOBIAN_VAL_62;
        values[63] = JACOBIAN_VAL_63;
        values[64] = JACOBIAN_VAL_64;
        values[65] = JACOBIAN_VAL_65;
        values[66] = JACOBIAN_VAL_66;
        values[67] = JACOBIAN_VAL_67;
        values[68] = JACOBIAN_VAL_68;
        values[69] = JACOBIAN_VAL_69;
        values[70] = JACOBIAN_VAL_70;
        values[71] = JACOBIAN_VAL_71;
        values[72] = JACOBIAN_VAL_72;
        values[73] = JACOBIAN_VAL_73;
        values[74] = JACOBIAN_VAL_74;
        values[75] = JACOBIAN_VAL_75;
        values[76] = JACOBIAN_VAL_76;
        values[77] = JACOBIAN_VAL_77;
        values[78] = JACOBIAN_VAL_78;
        values[79] = JACOBIAN_VAL_79;
        values[80] = JACOBIAN_VAL_80;
        values[81] = JACOBIAN_VAL_81;
        values[82] = JACOBIAN_VAL_82;
        values[83] = JACOBIAN_VAL_83;
        values[84] = JACOBIAN_VAL_84;
        values[85] = JACOBIAN_VAL_85;
        values[86] = JACOBIAN_VAL_86;
        values[87] = JACOBIAN_VAL_87;
        values[88] = JACOBIAN_VAL_88;
        values[89] = JACOBIAN_VAL_89;
        values[90] = JACOBIAN_VAL_90;
        values[91] = JACOBIAN_VAL_91;
        values[92] = JACOBIAN_VAL_92;
        values[93] = JACOBIAN_VAL_93;
        values[94] = JACOBIAN_VAL_94;
        values[95] = JACOBIAN_VAL_95;
        values[96] = JACOBIAN_VAL_96;
        values[97] = JACOBIAN_VAL_97;
        values[98] = JACOBIAN_VAL_98;
        values[99] = JACOBIAN_VAL_99;
        values[100] = JACOBIAN_VAL_100;
        values[101] = JACOBIAN_VAL_101;
        values[102] = JACOBIAN_VAL_102;
        values[103] = JACOBIAN_VAL_103;
        values[104] = JACOBIAN_VAL_104;
        values[105] = JACOBIAN_VAL_105;
        values[106] = JACOBIAN_VAL_106;
        values[107] = JACOBIAN_VAL_107;
        values[108] = JACOBIAN_VAL_108;
        values[109] = JACOBIAN_VAL_109;
        values[110] = JACOBIAN_VAL_110;
        values[111] = JACOBIAN_VAL_111;
        values[112] = JACOBIAN_VAL_112;
        values[113] = JACOBIAN_VAL_113;
        values[114] = JACOBIAN_VAL_114;
        values[115] = JACOBIAN_VAL_115;
        values[116] = JACOBIAN_VAL_116;
        values[117] = JACOBIAN_VAL_117;
        values[118] = JACOBIAN_VAL_118;
        values[119] = JACOBIAN_VAL_119;
        values[120] = JACOBIAN_VAL_120;
        values[121] = JACOBIAN_VAL_121;
        values[122] = JACOBIAN_VAL_122;
        values[123] = JACOBIAN_VAL_123;
        values[124] = JACOBIAN_VAL_124;
        values[125] = JACOBIAN_VAL_125;
        values[126] = JACOBIAN_VAL_126;
        values[127] = JACOBIAN_VAL_127;
        values[128] = JACOBIAN_VAL_128;
        values[129] = JACOBIAN_VAL_129;
        values[130] = JACOBIAN_VAL_130;
        values[131] = JACOBIAN_VAL_131;
        values[132] = JACOBIAN_VAL_132;
        values[133] = JACOBIAN_VAL_133;
        values[134] = JACOBIAN_VAL_134;
        values[135] = JACOBIAN_VAL_135;
        values[136] = JACOBIAN_VAL_136;
        values[137] = JACOBIAN_VAL_137;
        values[138] = JACOBIAN_VAL_138;
        values[139] = JACOBIAN_VAL_139;
        values[140] = JACOBIAN_VAL_140;
        values[141] = JACOBIAN_VAL_141;
        values[142] = JACOBIAN_VAL_142;
        values[143] = JACOBIAN_VAL_143;
        values[144] = JACOBIAN_VAL_144;
        values[145] = JACOBIAN_VAL_145;
        values[146] = JACOBIAN_VAL_146;
        values[147] = JACOBIAN_VAL_147;
        values[148] = JACOBIAN_VAL_148;
        values[149] = JACOBIAN_VAL_149;
        values[150] = JACOBIAN_VAL_150;
        values[151] = JACOBIAN_VAL_151;
        values[152] = JACOBIAN_VAL_152;
        values[153] = JACOBIAN_VAL_153;
        values[154] = JACOBIAN_VAL_154;
        values[155] = JACOBIAN_VAL_155;
        values[156] = JACOBIAN_VAL_156;
        values[157] = JACOBIAN_VAL_157;
        values[158] = JACOBIAN_VAL_158;
        values[159] = JACOBIAN_VAL_159;
        values[160] = JACOBIAN_VAL_160;
        values[161] = JACOBIAN_VAL_161;
        values[162] = JACOBIAN_VAL_162;
        values[163] = JACOBIAN_VAL_163;
        values[164] = JACOBIAN_VAL_164;
        values[165] = JACOBIAN_VAL_165;
        values[166] = JACOBIAN_VAL_166;
        values[167] = JACOBIAN_VAL_167;
        values[168] = JACOBIAN_VAL_168;
        values[169] = JACOBIAN_VAL_169;
        values[170] = JACOBIAN_VAL_170;
        values[171] = JACOBIAN_VAL_171;
        values[172] = JACOBIAN_VAL_172;
        values[173] = JACOBIAN_VAL_173;
        values[174] = JACOBIAN_VAL_174;
        values[175] = JACOBIAN_VAL_175;
        values[176] = JACOBIAN_VAL_176;
        values[177] = JACOBIAN_VAL_177;
        values[178] = JACOBIAN_VAL_178;
        values[179] = JACOBIAN_VAL_179;
        values[180] = JACOBIAN_VAL_180;
        values[181] = JACOBIAN_VAL_181;
        values[182] = JACOBIAN_VAL_182;
        values[183] = JACOBIAN_VAL_183;
        values[184] = JACOBIAN_VAL_184;
        values[185] = JACOBIAN_VAL_185;
        values[186] = JACOBIAN_VAL_186;
        values[187] = JACOBIAN_VAL_187;
        values[188] = JACOBIAN_VAL_188;
        values[189] = JACOBIAN_VAL_189;
        values[190] = JACOBIAN_VAL_190;
        values[191] = JACOBIAN_VAL_191;
        values[192] = JACOBIAN_VAL_192;
        values[193] = JACOBIAN_VAL_193;
        values[194] = JACOBIAN_VAL_194;
        values[195] = JACOBIAN_VAL_195;
        values[196] = JACOBIAN_VAL_196;
        values[197] = JACOBIAN_VAL_197;
        values[198] = JACOBIAN_VAL_198;
        values[199] = JACOBIAN_VAL_199;
        values[200] = JACOBIAN_VAL_200;
        values[201] = JACOBIAN_VAL_201;
        values[202] = JACOBIAN_VAL_202;
        values[203] = JACOBIAN_VAL_203;
        values[204] = JACOBIAN_VAL_204;
        values[205] = JACOBIAN_VAL_205;
        values[206] = JACOBIAN_VAL_206;
        values[207] = JACOBIAN_VAL_207;
        values[208] = JACOBIAN_VAL_208;
        values[209] = JACOBIAN_VAL_209;
        values[210] = JACOBIAN_VAL_210;
        values[211] = JACOBIAN_VAL_211;
        values[212] = JACOBIAN_VAL_212;
        values[213] = JACOBIAN_VAL_213;
        values[214] = JACOBIAN_VAL_214;
        values[215] = JACOBIAN_VAL_215;
        values[216] = JACOBIAN_VAL_216;
        values[217] = JACOBIAN_VAL_217;
        values[218] = JACOBIAN_VAL_218;
        values[219] = JACOBIAN_VAL_219;
        values[220] = JACOBIAN_VAL_220;
        values[221] = JACOBIAN_VAL_221;
        values[222] = JACOBIAN_VAL_222;
        values[223] = JACOBIAN_VAL_223;
        values[224] = JACOBIAN_VAL_224;
        values[225] = JACOBIAN_VAL_225;
        values[226] = JACOBIAN_VAL_226;
        values[227] = JACOBIAN_VAL_227;
        values[228] = JACOBIAN_VAL_228;
        values[229] = JACOBIAN_VAL_229;
        values[230] = JACOBIAN_VAL_230;
        values[231] = JACOBIAN_VAL_231;
        values[232] = JACOBIAN_VAL_232;
        values[233] = JACOBIAN_VAL_233;
        values[234] = JACOBIAN_VAL_234;
        values[235] = JACOBIAN_VAL_235;
        values[236] = JACOBIAN_VAL_236;
        values[237] = JACOBIAN_VAL_237;
        values[238] = JACOBIAN_VAL_238;
        values[239] = JACOBIAN_VAL_239;
        values[240] = JACOBIAN_VAL_240;
        values[241] = JACOBIAN_VAL_241;
        values[242] = JACOBIAN_VAL_242;
        values[243] = JACOBIAN_VAL_243;
        values[244] = JACOBIAN_VAL_244;
        values[245] = JACOBIAN_VAL_245;
        values[246] = JACOBIAN_VAL_246;
        values[247] = JACOBIAN_VAL_247;
        values[248] = JACOBIAN_VAL_248;
        values[249] = JACOBIAN_VAL_249;
        values[250] = JACOBIAN_VAL_250;
        values[251] = JACOBIAN_VAL_251;
        values[252] = JACOBIAN_VAL_252;
        values[253] = JACOBIAN_VAL_253;
        values[254] = JACOBIAN_VAL_254;
        values[255] = JACOBIAN_VAL_255;
        values[256] = JACOBIAN_VAL_256;
        values[257] = JACOBIAN_VAL_257;
        values[258] = JACOBIAN_VAL_258;
        values[259] = JACOBIAN_VAL_259;
        values[260] = JACOBIAN_VAL_260;
        values[261] = JACOBIAN_VAL_261;
        values[262] = JACOBIAN_VAL_262;
        values[263] = JACOBIAN_VAL_263;
        values[264] = JACOBIAN_VAL_264;
        values[265] = JACOBIAN_VAL_265;
        values[266] = JACOBIAN_VAL_266;
        values[267] = JACOBIAN_VAL_267;
        values[268] = JACOBIAN_VAL_268;
        values[269] = JACOBIAN_VAL_269;
        values[270] = JACOBIAN_VAL_270;
        values[271] = JACOBIAN_VAL_271;
        values[272] = JACOBIAN_VAL_272;
        values[273] = JACOBIAN_VAL_273;
        values[274] = JACOBIAN_VAL_274;
        values[275] = JACOBIAN_VAL_275;
        values[276] = JACOBIAN_VAL_276;
        values[277] = JACOBIAN_VAL_277;
        values[278] = JACOBIAN_VAL_278;
        values[279] = JACOBIAN_VAL_279;
        values[280] = JACOBIAN_VAL_280;
        values[281] = JACOBIAN_VAL_281;
        values[282] = JACOBIAN_VAL_282;
        values[283] = JACOBIAN_VAL_283;
        values[284] = JACOBIAN_VAL_284;
        values[285] = JACOBIAN_VAL_285;
        values[286] = JACOBIAN_VAL_286;
        values[287] = JACOBIAN_VAL_287;
        values[288] = JACOBIAN_VAL_288;
        values[289] = JACOBIAN_VAL_289;
        values[290] = JACOBIAN_VAL_290;
        values[291] = JACOBIAN_VAL_291;
        values[292] = JACOBIAN_VAL_292;
        values[293] = JACOBIAN_VAL_293;
        values[294] = JACOBIAN_VAL_294;
        values[295] = JACOBIAN_VAL_295;
        values[296] = JACOBIAN_VAL_296;
        values[297] = JACOBIAN_VAL_297;
        values[298] = JACOBIAN_VAL_298;
        values[299] = JACOBIAN_VAL_299;
        values[300] = JACOBIAN_VAL_300;
        values[301] = JACOBIAN_VAL_301;
        values[302] = JACOBIAN_VAL_302;
        values[303] = JACOBIAN_VAL_303;
        values[304] = JACOBIAN_VAL_304;
        values[305] = JACOBIAN_VAL_305;
        values[306] = JACOBIAN_VAL_306;
        values[307] = JACOBIAN_VAL_307;
        values[308] = JACOBIAN_VAL_308;
        values[309] = JACOBIAN_VAL_309;
        values[310] = JACOBIAN_VAL_310;
        values[311] = JACOBIAN_VAL_311;
        values[312] = JACOBIAN_VAL_312;
        values[313] = JACOBIAN_VAL_313;
        values[314] = JACOBIAN_VAL_314;
        values[315] = JACOBIAN_VAL_315;
        values[316] = JACOBIAN_VAL_316;
        values[317] = JACOBIAN_VAL_317;
        values[318] = JACOBIAN_VAL_318;
        values[319] = JACOBIAN_VAL_319;
        values[320] = JACOBIAN_VAL_320;
        values[321] = JACOBIAN_VAL_321;
        values[322] = JACOBIAN_VAL_322;
        values[323] = JACOBIAN_VAL_323;
        values[324] = JACOBIAN_VAL_324;
        values[325] = JACOBIAN_VAL_325;
        values[326] = JACOBIAN_VAL_326;
        values[327] = JACOBIAN_VAL_327;
        values[328] = JACOBIAN_VAL_328;
        values[329] = JACOBIAN_VAL_329;
        values[330] = JACOBIAN_VAL_330;
        values[331] = JACOBIAN_VAL_331;
        values[332] = JACOBIAN_VAL_332;
        values[333] = JACOBIAN_VAL_333;
        values[334] = JACOBIAN_VAL_334;
        values[335] = JACOBIAN_VAL_335;
        values[336] = JACOBIAN_VAL_336;
        values[337] = JACOBIAN_VAL_337;
        values[338] = JACOBIAN_VAL_338;
        values[339] = JACOBIAN_VAL_339;
        values[340] = JACOBIAN_VAL_340;
        values[341] = JACOBIAN_VAL_341;
        values[342] = JACOBIAN_VAL_342;
        values[343] = JACOBIAN_VAL_343;
        values[344] = JACOBIAN_VAL_344;
        values[345] = JACOBIAN_VAL_345;
        values[346] = JACOBIAN_VAL_346;
        values[347] = JACOBIAN_VAL_347;
        values[348] = JACOBIAN_VAL_348;
        values[349] = JACOBIAN_VAL_349;
        values[350] = JACOBIAN_VAL_350;
        values[351] = JACOBIAN_VAL_351;
        values[352] = JACOBIAN_VAL_352;
        values[353] = JACOBIAN_VAL_353;
        values[354] = JACOBIAN_VAL_354;
        values[355] = JACOBIAN_VAL_355;
        values[356] = JACOBIAN_VAL_356;
        values[357] = JACOBIAN_VAL_357;
        values[358] = JACOBIAN_VAL_358;
        values[359] = JACOBIAN_VAL_359;
        values[360] = JACOBIAN_VAL_360;
        values[361] = JACOBIAN_VAL_361;
        values[362] = JACOBIAN_VAL_362;
        values[363] = JACOBIAN_VAL_363;
        values[364] = JACOBIAN_VAL_364;
        values[365] = JACOBIAN_VAL_365;
        values[366] = JACOBIAN_VAL_366;
        values[367] = JACOBIAN_VAL_367;
        values[368] = JACOBIAN_VAL_368;
        values[369] = JACOBIAN_VAL_369;
        values[370] = JACOBIAN_VAL_370;
        values[371] = JACOBIAN_VAL_371;
        values[372] = JACOBIAN_VAL_372;
        values[373] = JACOBIAN_VAL_373;
        values[374] = JACOBIAN_VAL_374;
        values[375] = JACOBIAN_VAL_375;
        values[376] = JACOBIAN_VAL_376;
        values[377] = JACOBIAN_VAL_377;
        values[378] = JACOBIAN_VAL_378;
        values[379] = JACOBIAN_VAL_379;
        values[380] = JACOBIAN_VAL_380;
        values[381] = JACOBIAN_VAL_381;
        values[382] = JACOBIAN_VAL_382;
        values[383] = JACOBIAN_VAL_383;
        values[384] = JACOBIAN_VAL_384;
        values[385] = JACOBIAN_VAL_385;
        values[386] = JACOBIAN_VAL_386;
        values[387] = JACOBIAN_VAL_387;
        values[388] = JACOBIAN_VAL_388;
        values[389] = JACOBIAN_VAL_389;
        values[390] = JACOBIAN_VAL_390;
        values[391] = JACOBIAN_VAL_391;
        values[392] = JACOBIAN_VAL_392;
        values[393] = JACOBIAN_VAL_393;
        values[394] = JACOBIAN_VAL_394;
        values[395] = JACOBIAN_VAL_395;
        values[396] = JACOBIAN_VAL_396;
        values[397] = JACOBIAN_VAL_397;
        values[398] = JACOBIAN_VAL_398;
        values[399] = JACOBIAN_VAL_399;
        values[400] = JACOBIAN_VAL_400;
        values[401] = JACOBIAN_VAL_401;
        values[402] = JACOBIAN_VAL_402;
        values[403] = JACOBIAN_VAL_403;
        values[404] = JACOBIAN_VAL_404;
        values[405] = JACOBIAN_VAL_405;
        values[406] = JACOBIAN_VAL_406;
        values[407] = JACOBIAN_VAL_407;
        values[408] = JACOBIAN_VAL_408;
        values[409] = JACOBIAN_VAL_409;
        values[410] = JACOBIAN_VAL_410;
        values[411] = JACOBIAN_VAL_411;
        values[412] = JACOBIAN_VAL_412;
        values[413] = JACOBIAN_VAL_413;
        values[414] = JACOBIAN_VAL_414;
        values[415] = JACOBIAN_VAL_415;
        values[416] = JACOBIAN_VAL_416;
        values[417] = JACOBIAN_VAL_417;
        values[418] = JACOBIAN_VAL_418;
        values[419] = JACOBIAN_VAL_419;
        values[420] = JACOBIAN_VAL_420;
        values[421] = JACOBIAN_VAL_421;
        values[422] = JACOBIAN_VAL_422;
        values[423] = JACOBIAN_VAL_423;
        values[424] = JACOBIAN_VAL_424;
        values[425] = JACOBIAN_VAL_425;
        values[426] = JACOBIAN_VAL_426;
        values[427] = JACOBIAN_VAL_427;
        values[428] = JACOBIAN_VAL_428;
        values[429] = JACOBIAN_VAL_429;
        values[430] = JACOBIAN_VAL_430;
        values[431] = JACOBIAN_VAL_431;
        values[432] = JACOBIAN_VAL_432;
        values[433] = JACOBIAN_VAL_433;
        values[434] = JACOBIAN_VAL_434;
        values[435] = JACOBIAN_VAL_435;
        values[436] = JACOBIAN_VAL_436;
        values[437] = JACOBIAN_VAL_437;
        values[438] = JACOBIAN_VAL_438;
        values[439] = JACOBIAN_VAL_439;
        values[440] = JACOBIAN_VAL_440;
        values[441] = JACOBIAN_VAL_441;
        values[442] = JACOBIAN_VAL_442;
        values[443] = JACOBIAN_VAL_443;
        values[444] = JACOBIAN_VAL_444;
        values[445] = JACOBIAN_VAL_445;
        values[446] = JACOBIAN_VAL_446;
        values[447] = JACOBIAN_VAL_447;
        values[448] = JACOBIAN_VAL_448;
        values[449] = JACOBIAN_VAL_449;
        values[450] = JACOBIAN_VAL_450;
        values[451] = JACOBIAN_VAL_451;
        values[452] = JACOBIAN_VAL_452;
        values[453] = JACOBIAN_VAL_453;
        values[454] = JACOBIAN_VAL_454;
        values[455] = JACOBIAN_VAL_455;
        values[456] = JACOBIAN_VAL_456;
        values[457] = JACOBIAN_VAL_457;
        values[458] = JACOBIAN_VAL_458;
        values[459] = JACOBIAN_VAL_459;
        values[460] = JACOBIAN_VAL_460;
        values[461] = JACOBIAN_VAL_461;
    }

    return true;
}


//return the structure or values of the Hessian of the Lagrangian
// bool Collocation::eval_h(
//     Index         n,
//     const Number* x,
//     bool          new_x,
//     Number        obj_factor,
//     Index         m,
//     const Number* lambda,
//     bool          new_lambda,
//     Index         nele_hess,
//     Index*        iRow,
//     Index*        jCol,
//     Number*       values
//     ) {
//     assert(n == 4);
//     assert(m == 2);

//     if (values == NULL) {
//         // return the structure. This is a symmetric matrix, fill the lower left
//         // triangle only.

//     } else {
//         // return the values. This is a symmetric matrix, fill the lower left
//         // triangle only

//         // fill the objective portion
    
//     }

//     return true;
// }


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
    // here is where we would store the solution to variables, or write to a file, etc
    // so we could use the solution.

    // For this example, we write the solution to the console
    std::cout << std::endl << std::endl << "Solution of the primal variables, x" << std::endl;
    for (Index i = 0; i < n; i++) {
        std::cout << "x[" << i << "] = " << x[i] << std::endl;
    }

    std::cout << std::endl << std::endl << "Objective value" << std::endl;
    std::cout << "f(x*) = " << obj_value << std::endl;

    std::cout << std::endl << "Final value of the constraints:" << std::endl;
    for (Index i = 0; i < m; i++) {
        std::cout << "g(" << i << ") = " << g[i] << std::endl;
    }
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