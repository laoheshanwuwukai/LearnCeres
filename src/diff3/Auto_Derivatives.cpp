
#include "Auto_Derivatives.h"


struct Auto_Derivatives_CostFunctor{
    template<typename T>
    bool operator()(const T* const x , T* residual) const{
        residual[0] = 10.0 - x[0];
        return true;
    }
};

void Auto_Derivatives( ){


    //first: The variable to solve for with its initial value
    double initial_x = 5.0f;
    double x = initial_x;

    //second: Build the problem
    ceres::Problem Auto_Derivatives_Problem;

    //third : Set up the only cost function (also known as residual).
    //        This uses auto-differentition to obtain the dericative(jacobian);
    ceres::CostFunction* Auto_Derivatives_CostFunction = 
        new ceres::AutoDiffCostFunction<Auto_Derivatives_CostFunctor, 1 , 1 >
        (new Auto_Derivatives_CostFunctor);

    Auto_Derivatives_Problem.AddResidualBlock(Auto_Derivatives_CostFunction,nullptr , &x);

    //Fourth : Run the Solver
    ceres::Solver::Options Auto_Derivatives_Options;
    Auto_Derivatives_Options.linear_solver_type = ceres::DENSE_QR;
    Auto_Derivatives_Options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary Auto_Derivatives_Summary;
    ceres::Solve(Auto_Derivatives_Options, 
                &Auto_Derivatives_Problem,
                &Auto_Derivatives_Summary);

    std::cout<<"*****Auto differentition******"<<std::endl;
    std::cout<<Auto_Derivatives_Summary.BriefReport()<<"\n";
    std::cout<<"x: "<<initial_x<<" -> "<<x<<std::endl;
    return ;

}