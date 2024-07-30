#include <iostream>
#include <chrono>

#include "/usr/local/include/coin-or/IpIpoptApplication.hpp"
#include "/usr/local/include/coin-or/IpTNLP.hpp"

#include "optimization.hpp"
#include "globals.hpp"

using namespace Ipopt;  

double leg = 0.17;  
double legDot = -1.2369;    
double theta = 1.3129;      
double thetaDot = 2.5559;   
double iaz = 0.2;  

int main(int argc, char** argv) {
    // Create a new instance of your nlp
    //  (use a SmartPtr, not raw)
    SmartPtr<TNLP> mynlp = new Collocation();

    // Create a new instance of IpoptApplication
    //  (use a SmartPtr, not raw)
    // We are using the factory, since this allows us to compile this
    // example with an Ipopt Windows DLL
    SmartPtr<IpoptApplication> app = IpoptApplicationFactory();

    app->Options()->SetNumericValue("tol", 10e-5);
    app->Options()->SetNumericValue("constr_viol_tol", 10e-3);
    //app->Options()->SetStringValue("hessian_approximation", "limited-memory");
    //app->Options()->SetIntegerValue("max_iter", 0);

    // Initialize the IpoptApplication and process the options
    ApplicationReturnStatus status;
    status = app->Initialize();
    if (status != Solve_Succeeded) {
        std::cout << std::endl << std::endl << "*** Error during initialization!" << std::endl;
        return (int) status;
    }

    auto start = std::chrono::high_resolution_clock::now();
    // Ask Ipopt to solve the problem
    status = app->OptimizeTNLP(mynlp);

    if (status == Solve_Succeeded) {
        std::cout << std::endl << std::endl << "*** The problem solved!" << std::endl;
    } else {
        std::cout << std::endl << std::endl << "*** The problem FAILED!" << std::endl;
    }

    auto stop = std::chrono::high_resolution_clock::now();
    

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
    std::cout << "Actual time for optimization: " << duration.count() << std::endl;
    return (int) status;
}
