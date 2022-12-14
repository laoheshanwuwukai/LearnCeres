#include "Analytic_Derivatives.h"



class Analytic_Derivatives_CostFunctor
    : public ceres::SizedCostFunction<  1,/*number of risiduals */
                                        1 /* size of first parameter*/>{
public:
    virtual ~Analytic_Derivatives_CostFunctor(){}
    virtual bool Evaluate(double const* const* parameters , 
                        double * residuals , double** jacobians)const override{
        const double x = parameters[0][0];
        residuals[0] = 10-x;

        //Compute the Jacobian if asked for
        if(jacobians != nullptr && jacobians[0]!=nullptr){
            jacobians[0][0] = -1;
        }
        return true;
    }
};

void Analytic_Derivatives(){


    const double initial_x = 0.5;
    double x = initial_x;

    ceres::Problem Analytic_Derivates_Problem ;
    ceres::CostFunction* Analytic_Derivates_CostFunction =
                    new Analytic_Derivatives_CostFunctor;

    Analytic_Derivates_Problem.AddResidualBlock(
        Analytic_Derivates_CostFunction,nullptr,&x);

    ceres::Solver::Options Analytic_Derivates_Option;
    ceres::Solver::Summary Analytic_Derivates_Summary;
    Analytic_Derivates_Option.minimizer_progress_to_stdout = true;
    
    ceres::Solve(Analytic_Derivates_Option, &Analytic_Derivates_Problem, &Analytic_Derivates_Summary);
    std::cout<<std::endl;
    std::cout<<"************Analytic_Derivatives**************"<<std::endl;
    std::cout<<Analytic_Derivates_Summary.BriefReport()<<std::endl;
    return ;
}