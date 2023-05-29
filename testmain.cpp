#include <iostream>
#include "data.cpp"
#include "Eigen/Dense"
#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <random>

cv::Mat Matrix2Mat(Eigen::MatrixXd input ){
    int row = input.rows();
    int col = input.cols();
    cv::Mat output(row , col , CV_64F);
    for(int i = 0 ; i<row ; i++){
        for(int j = 0 ; j<col ; j++){
            output.at<double>(i , j ) = input(i,j);
        }
    }
    return output;
}

cv::Mat Eigen2Opencv33(const Eigen::Matrix3d input ){
    cv::Mat ret;
    ret = (cv::Mat_<double>(3,3)<<
    input(0,0) , input(0,1) , input(0,2),
    input(1,0) , input(1,1) , input(1,2),
    input(2,0) , input(2,1) , input(2,2));

    return ret;
}
cv::Mat Eigen2Opencv31( Eigen::Matrix<double,3,1> input){
    cv::Mat ret;
    ret = (cv::Mat_<double>(3,1)<<
    input(0,1) , input(1,0) , input(2,0)
    );
    return ret;
}
cv::Mat Eigen2Opencv44(const Eigen::Matrix4d & input){
    cv::Mat ret;
    ret = cv::Mat::zeros(4,4,CV_64F);
    for(int i = 0 ; i<4 ; i++){
        for(int j = 0 ; j<4 ; j++){
            ret.at<double>(i , j) = input(i,j);
        }
    }
    return ret;
}

void cvRttoH(cv::Mat &H , const cv::Mat& R , const cv::Mat&t){
    H = cv::Mat::eye(4,4,CV_64F);
    R.copyTo(H(cv::Rect(0,0,3,3)));
    t.copyTo(H(cv::Rect(3,0,1,3)));
}

void cvHtoRt(const cv::Mat H , cv::Mat& R , cv::Mat& t){
    R = H(cv::Rect(0,0,3,3));
    t = H(cv::Rect(3,0,1,3));
}

void manyHtoRt(const std::vector<cv::Mat> & H , 
                std::vector<cv::Mat> &R , 
                std::vector<cv::Mat> &t){
    int n = H.size();
    R.clear() ; t.clear();
    for(int i = 0 ; i< n ; i++){
        cv::Mat Ri , ti;
        cvHtoRt(H[i] , Ri , ti);
        R.push_back(Ri);
        t.push_back(ti);
    }
}

cv::Mat invH(cv::Mat H){
    cv::Mat ans = cv::Mat::eye(4,4,CV_64F);
    cv::Mat R,t;
    cv::Mat invR , invt;
    cvHtoRt(H , R ,t);
    invR = R.t();
    invt = -1 * invR *t ;
    invR.copyTo(ans(cv::Rect(0,0,3,3) ));
    invt.copyTo(ans(cv::Rect(3,0,1,3)));
    return ans;
}


Eigen::Matrix3d generateRandomOrthogonalMatrix3D()
{
    Eigen::Quaterniond quaternion = Eigen::Quaterniond::UnitRandom();
    Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();
    return rotation_matrix;
}

Eigen::Vector3d generateRandomVector3D()
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(-100, 100);

    Eigen::Matrix<double , 3,1> random_vector;
    random_vector << dist(gen),
                    dist(gen),
                    std::abs(dist(gen));
    
    return random_vector;
}


int main(int argc , char ** argv){

    Robot Hcg_ans = Robot(50 , 100 , 150 ,30 , 0 , 0);
    Robot Hbt_ans = Robot(500 , 300 , 2000 , 0 , 0 , 0);
    cv::Mat cvHcg_ans = Matrix2Mat(Hcg_ans.T);
    cv::Mat cvHbt_ans = Matrix2Mat(Hbt_ans.T);
    cv::Mat cvRcg_ans , cvtcg_ans , cvRbt_ans , cvtbt_ans;

    cvHtoRt(cvHcg_ans , cvRcg_ans , cvtcg_ans);
    cvHtoRt(cvHbt_ans , cvRbt_ans , cvtbt_ans);    

    std::vector<cv::Mat> Rcs , Rgs , tcs , tgs;
    int data_number = 3;
    for(int i = 0 ; i<data_number ; i++){
        Eigen::Matrix3d tempR = generateRandomOrthogonalMatrix3D();
        Eigen::Matrix<double ,3,1> tempt = generateRandomVector3D();
        
        cv::Mat Rc , Rg , tc , tg , Hg , Hc;
        Rc = Matrix2Mat(tempR);
        tc = Matrix2Mat(tempt);
        Rcs.push_back(Rc);
        tcs.push_back(tc);
        cvRttoH(Hc , Rc , tc);
        
        cv::Mat tempH = cvHcg_ans * (Hc * cvHbt_ans);
        Hg = invH(tempH);
        // std::cout<<Hg<<std::endl;
        cvHtoRt(Hg , Rg , tg);
        Rgs.push_back(Rg);
        tgs.push_back(tg);
        // std::cout<<Rc <<tc<<std::endl;
        // std::cout<<Rg <<tg<<std::endl;
    }

    cv::Mat cvRpark , cvtpark;
    cv::calibrateHandEye(Rgs , tgs , Rcs , tcs , cvRpark , cvtpark , cv::CALIB_HAND_EYE_PARK);

    std::cout<<cvRpark<<std::endl;
    std::cout<<cvtpark<<std::endl; 
    std::cout<<cvHcg_ans<<std::endl;

    return 0;

}