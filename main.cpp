#include <vector>
#include <iostream>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <sophus/so3.hpp>
#include <ceres/cost_function.h>
#include <ceres/local_parameterization.h>
#include <ceres/sized_cost_function.h>
#include <ceres/problem.h>
#include <ceres/solver.h>
#include <opencv2/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <glog/logging.h>

#include <yaml-cpp/yaml.h>

#include "global_defination/global_defination.h"
#include "data.cpp"
typedef Eigen::Matrix<double , 6 ,1> Vector6d;
using std::cout;
using std::endl;

static Eigen::Matrix3d skew( Eigen::Vector3d t){
    Eigen::Matrix3d t_hat ;

    t_hat <<  0  , -t(2) , t(1),
            t(2) ,   0   , -t(0),
            -t(1),  t(0) , 0;
    return t_hat;
}

bool CreateRealPoints(std::vector<Eigen::Vector3d>& real_points){
    real_points.clear();
    real_points.resize(4);
    real_points[0] = Eigen::Vector3d(1,1,1);
    real_points[1] = Eigen::Vector3d(5,6,7);
    real_points[2] = Eigen::Vector3d(3,4,5);
    real_points[3] = Eigen::Vector3d(2,6,9);

    return true;    
}
/*
Eigen::Quaternioned::coeffs
0.39036
0.58554
0.19518
0.68313 w
*/

const double q_param[4] = {0.39036 , 
0.58554 , 
0.19518 , 
0.68313};

const double t_param[3] = {1,2,3};

double init_param[7] = {0,0,0,1,0,0,0};

    /**
     * @brief 全局优化变量为7纬，分别是4纬的四元数和三维的旋转变量，
     * 但是描述旋转四元数存在冗余约束，所以实际的局部优化变量是6纬，如何在全局变量上更新迭代算出来的增量是
     * ceres::LocalParameteriztion 中的虚函数Plus(...)做的事情，从7纬到6纬是ComputeJacobians(...)
     * 做的事情；
     * 
     * 这里拿到6纬的增量，是SE3上的李代数，将旋转和平移恢复，需要考虑转角趋近于0的情况；
     * 
     * @param delta 
     * @param delta_q 
     * @param delta_t 
     */
    static void GetTransformse3(const Eigen::Matrix<double ,6 , 1>& delta, 
                        Eigen::Quaterniond & delta_q , 
                        Eigen::Vector3d& delta_t){
        Eigen::Vector3d omega(delta.data());
        Eigen::Vector3d upslion(delta.data()+3);
        Eigen::Matrix3d omega_skew = skew(omega); //反对称矩阵

        
        double theta = omega.norm(); //角度
        double half_theta = theta/2;

        double imag_factor;
        double real_factor = cos(half_theta);
        if(theta<1e-10){  //如果theta太小了，那么需要将  sin(theta/2) /theta 泰勒展开求虚部的系数
            double theta_pow2 = theta*theta;
            double theta_pow4 = theta_pow2*theta_pow2;
            imag_factor = 0.5 - 0.020833 * theta_pow2 + 0.0002604 * theta_pow4;
        } else{ //正常情况 delta_q = cos(theta/2)  sin(theta/2)*n 这里n是单位向量，omega = theta*n 
            imag_factor = sin(half_theta) /theta;
        }

        delta_q = Eigen::Quaterniond(real_factor , 
                                imag_factor * omega.x() , 
                                imag_factor * omega.y() , 
                                imag_factor * omega.z());
        
        /*李代数中的 upslion 不是平移向量t，t实际上是 J*upslion
        J = (sin(theta) /theta )*I +
            (1 - sin(theta)/theta)*n*(n.transpose())+
            (1-cos(theta))/theta * a.skew();
        经过 n.skew() * n.skew() = n*n.transpose() - I;的化简
        */
        Eigen::Matrix3d J;
        if(theta<1e-10){
            J = delta_q.matrix();
        }else{
            Eigen::Matrix3d omega_skew_pow2 = omega_skew * omega_skew;
            J = Eigen::Matrix3d::Identity() +
                (1-cos(theta))/(theta*theta)*omega_skew +
                (theta - sin(theta))/(pow(theta,3)) * omega_skew_pow2;
        }

        delta_t = J*upslion;

        return ;
    }

class myCostFunctor: public ceres::SizedCostFunction<   1,  //Size of residuals
                                                7>{ //Size of paramblocks
public:
    myCostFunctor(Eigen::Vector3d pnta_ , Eigen::Vector3d pntb_):pnta(pnta_) , pntb(pntb_){}
    virtual bool Evaluate(double const * const * parameter , 
                            double * residuals , 
                            double ** jacobians) const{

    Eigen::Map<const Eigen::Quaterniond> q_const(parameter[0]);
    Eigen::Map<const Eigen::Vector3d> t_const(parameter[0] + 4);
    Eigen::Vector3d pntb_est = q_const * pnta + t_const;
    Eigen::Vector3d nu = pntb_est - pntb;
    double nu_norm = nu.norm();

    residuals[0] = nu_norm;

    if(jacobians!=nullptr && jacobians[0]!=nullptr){
        Eigen::Matrix<double ,3 , 6> third = Eigen::Matrix<double , 3 , 6>::Zero();
        third.block<3 ,3>(0 , 0) = - skew(pntb_est);
        third.block<3, 3>(0 , 3) = Eigen::Matrix3d::Identity();

        Eigen::Map<Eigen::Matrix<double , 1 , 7> > J(jacobians[0]);
        J.setZero();
        J.block<1,6>(0 , 0 ) = nu.transpose() / nu_norm * third;
    }
    return true;
    }

private:
    Eigen::Vector3d pnta , pntb;
};

class MyLocalParam: public ceres::LocalParameterization{
public:
    MyLocalParam(){}
    virtual ~MyLocalParam(){}

    virtual int GlobalSize() const { return 7;}
    virtual int LocalSize()  const { return 6;}

    virtual bool Plus(const double * x , //sizeof x is 7
                    const double *delta, //sizeof delta is 6
                    double * x_plus_delta) const{ //size of x_plus_delta is 7
    
        Eigen::Map<const Eigen::Quaterniond> x_q(x);
        Eigen::Map<const Eigen::Vector3d> x_t(x+4);
        
        //恢复出delta的q和t，要处理角度很小的情况；
        Eigen::Quaterniond delta_q;
        Eigen::Vector3d delta_t;
        
        Eigen::Map<const Eigen::Matrix<double ,6 ,1 > > temp(delta);

        GetTransformse3(temp , delta_q , delta_t);
        
        Eigen::Map<Eigen::Quaterniond> q_plus_delta(x_plus_delta);
        Eigen::Map<Eigen::Vector3d> t_plus_delta(x_plus_delta + 4);

        q_plus_delta = delta_q * x_q;
        t_plus_delta = delta_q * x_t + delta_t;

        return true;    
    }

    // finish virtual function CpmputeJacobians() 
    bool ComputeJacobian(const double *x, double *jacobian) const
    {
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> j(jacobian);
        (j.topRows(6)).setIdentity();
        (j.bottomRows(1)).setZero();

        return true;
    }
};

cv::Mat Eigen2Opencv33(const Eigen::Matrix3d input ){
    cv::Mat ret;
    ret = (cv::Mat_<double>(3,3)<<
    input(0,0) , input(0,1) , input(0,2),
    input(1,0) , input(1,1) , input(1,2),
    input(2,0) , input(2,1) , input(2,2));

    return ret;

}

Eigen::Vector3d Eigen2Opencv31(cv::Mat input){
    Eigen::Vector3d temp ; 
    temp(0,0) = input.at<double>(0,0) ;
    temp(1,0) = input.at<double>(1,0) ;
    temp(2,0) = input.at<double>(2,0) ;
    return temp;
}

Eigen::Matrix4d inv44(const Eigen::Matrix4d input){
    Eigen::Matrix4d output = Eigen::Matrix4d::Identity();
    Eigen::Matrix3d R = (input.block<3,3>(0,0));
    Eigen::Matrix<double,3,1> t = input.block<3,1>(0,3);

    output.block<3,3>(0,0) = R.transpose();
    output.block<3,1>(0,3) = -1*(R.transpose())*t;

    return output ;
}
int main(int argc , char ** argv){

    // //TODO 
    std::vector< Eigen::Matrix4d > cameradata =HandEye_data::getCameradata();
    std::vector< Eigen::Matrix4d > robotdata = HandEye_data::getRobotdata();
    //-------------compute t ---------------------
    // int n = cameradata.size();
    // int k = (n*n -n)/2;
    // Eigen::MatrixXd C(3*k , 3);
    // Eigen::MatrixXd d(3*k , 1);
    // Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
    // // std::vector< Eigen::Matrix4d> A , B;
    // int idx = 0;
    // Eigen::Matrix3d myR =HandEye_data::getans();
    // for(int i = 0 ; i<n-1 ; i++){
    //     for(int j = i+1 ; j<n ; j++,idx++){
    //         Eigen::Matrix4d Hgij= inv44(robotdata[j]) * robotdata[i];
    //         Eigen::Matrix4d Hcij= cameradata[j] * inv44(cameradata[i]);

    //         Eigen::Matrix3d Rgij = Hgij.block<3,3>(0,0);
    //         Eigen::Matrix<double,3,1> tgij = Hgij.block<3,1>(0,3);
    //         Eigen::Matrix<double,3,1> tcij = Hcij.block<3,1>(0,3);

    //         Eigen::Matrix3d I_tgij = I3 - Rgij;
    //         C.block<3,3>(3*idx , 0) = I_tgij;
    //         Eigen::Matrix<double ,3,1> A_RB = tgij -myR*tcij;

    //         d.block<3,1>(3*idx,0) = A_RB; 
    //     }
    // }
    // Eigen::Matrix<double ,3,1> myt;
    // myt = C.bdcSvd(Eigen::ComputeThinU|Eigen::ComputeThinV).solve(d);
    // std::cout<<myt<<std::endl;
    // -------------end of compute t------------


   // std::cout<<C.rows()<<" * "<<C.cols()<<std::endl;
    // std::cout<<d.rows()<<" * "<<d.cols()<<std::endl;

    
    // -------------compute R------------------------
    std::vector<Eigen::Vector3d> RA; //Robot pose 
    std::vector<Eigen::Vector3d> RB;  //Camera pose

    for(int i= 0 ; i<cameradata.size()-1 ; i++){
        for(int j = i+1 ; j<cameradata.size() ; j++){
            Eigen::Matrix3d hgij = 
            (robotdata[j].block<3,3>(0,0).transpose())
            * (robotdata[i].block<3,3>(0,0));

            Eigen::Matrix3d hcij = 
            (cameradata[j].block<3,3>(0,0)) *
            (cameradata[i].block<3,3>(0,0).transpose());

            cv::Mat tempRa = Eigen2Opencv33(hgij);
            cv::Mat tempRb = Eigen2Opencv33(hcij);

            //TODO use cv::Rodrigues change R to rotation vector
            cv::Mat tempa , tempb ;
            cv::Rodrigues(tempRa , tempa);
            cv::Rodrigues(tempRb , tempb);
            RA.push_back( Eigen2Opencv31(tempa));
            RB.push_back( Eigen2Opencv31(tempb));
        }
    }
    // ----------------End of compute R---------------


    
    // google::InitGoogleLogging(argv[0]);
    // FLAGS_alsologtostderr = true;
    // FLAGS_log_dir = global_defination::WORK_SPACE_PATH+"/log";
    std::string config_file_path = global_defination::WORK_SPACE_PATH +"/config/config.yaml";
    YAML::Node config_node = YAML::LoadFile(config_file_path);
    /*********数据**********/
    Eigen::Map<const Eigen::Quaterniond> q(q_param);
    Eigen::Map<const Eigen::Vector3d> t(t_param);

    // std::vector<Eigen::Vector3d> pa , pb(4);
    // CreateRealPoints(pa);

    // for(int i= 0 ; i<pa.size() ; i++){
    //     pb[i] = q*pa[i]+t;
    // }

    /****************/
    ceres::Problem problem;
    ceres::LocalParameterization* localparam = new MyLocalParam();
    problem.AddParameterBlock(init_param , 7 , localparam);

    for(int i = 0 ; i<RA.size(); i++){
        ceres::CostFunction * percostfunction = new myCostFunctor(RB[i] , RA[i]);
        problem.AddResidualBlock(percostfunction , nullptr , init_param);
    }
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    int max_num_iterations = config_node["Options"]["max_num_iterations"].as<int>();
    LOG(INFO)<<"Options::max_num_iterations: "<<max_num_iterations;

    options.max_num_iterations =max_num_iterations;

    ceres::Solve(options , &problem , &summary);


    cout<<summary.BriefReport()<<endl;


    LOG(INFO)<<summary.BriefReport()<<endl;

    Eigen::Map<Eigen::Quaterniond> q_ans(init_param);
    Eigen::Map<Eigen::Vector3d> t_ans(init_param+4);
    cout<<"Rotation_ans: \n"<<q.matrix()<<endl;
    cout<<"Translation_ans: \n"<<t.transpose()<<endl;

    cout<<"Rotation: \n"<<q_ans.matrix()<<endl;
    cout<<"Translation: \n"<<t_ans.transpose()<<endl;

    
    return 0;
}