#include "Numeric_Derivatives.h"

struct NumericDiffCostFunctor{
    bool operator()(const double * const x , double* residual) const{
        residual[0] = 10.0 - x[0];
        return true;
    }
};

void Numeric_Derivatives(){
    //first
    const double initial_x = 5.0;
    double x = initial_x;

    ceres::CostFunction* NumericDiffCostFunction = 
    new ceres::NumericDiffCostFunction<NumericDiffCostFunctor , ceres::CENTRAL , 1 ,1 >(
        new NumericDiffCostFunctor
    );


    ceres::Problem NumericDiffProblem;
    NumericDiffProblem.AddResidualBlock(NumericDiffCostFunction , 
                                        nullptr , 
                                        &x);

    ceres::Solver::Options NumericDiffOptions;
    ceres::Solver::Summary NumericDiffSummary;

    NumericDiffOptions.linear_solver_type = ceres::DENSE_QR;
    NumericDiffOptions.minimizer_progress_to_stdout = true;

    ceres::Solve(NumericDiffOptions , 
                    &NumericDiffProblem , 
                    &NumericDiffSummary);

    std::cout<<"*****Numeric_Derivatives*****"<<std::endl;
    std::cout<<NumericDiffSummary.BriefReport()<<std::endl;
    std::cout<<" x: "<<initial_x<<"-->"<<x<<std::endl;

    return ;
}