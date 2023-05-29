#include <iostream>
#include "data.cpp"
#include "Eigen/Dense"
#include <opencv2/calib3d.hpp>

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

void cvRttoH(cv::Mat H , const cv::Mat& R , const cv::Mat&t){
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


void testHcij(std::vector<cv::Mat>& Hcij ){
    Robot a1(10 , 15 , 20  , 20 , 30 ,30);
    Robot a2(30 , 45 , 45  , 30 , 45 ,45);
    Robot a3(60 , 60 , 60  , 45 , 60 ,60);
    
    cv::Mat cva1 = Eigen2Opencv44(a1.T);
    cv::Mat cva2 = Eigen2Opencv44(a2.T);
    cv::Mat cva3 = Eigen2Opencv44(a3.T);

    Hcij.push_back(cva1);
    Hcij.push_back(cva2);
    Hcij.push_back(cva3);
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

std::vector<cv::Mat> testHgij(const cv::Mat T ,std::vector<cv::Mat>& Hcij ){
    int n = Hcij.size();
    std::vector<cv::Mat> ret;
    cv::Mat T_inv = invH(T);
    for(int i = 0 ; i<n ; i++){

        cv::Mat temp = (T * Hcij[i]) * T_inv;
        ret.push_back(temp);
    }
    return ret;
}


int main(int argc , char ** argv){

    Robot Hcg_ans = Robot(50 , 100 , 150 ,30 , 0 , 0);
    cv::Mat cvHcg_ans = Matrix2Mat(Hcg_ans.T);

    std::vector<cv::Mat> Hcij ,Hgij;
    testHcij(Hcij);
    Hgij = testHgij(cvHcg_ans , Hcij);

    std::vector<cv::Mat> Rcij , Rgij , tcij , tgij;
    for(int i = 0 ; i< Hcij.size() ; i++){
        cv::Mat tempRc , temptc , tempRg , temptg;
        cvHtoRt(Hcij[i] , tempRc , temptc);
        Rcij.push_back(tempRc);
        tcij.push_back(temptc);
        cvHtoRt(Hgij[i] , tempRg , temptg);
        Rgij.push_back(tempRg);
        tgij.push_back(temptg);
    }

    cv::Mat cvRpark , cvtpark;
    cv::calibrateHandEye(Rgij , tgij , Rcij , tcij , cvRpark , cvtpark , cv::CALIB_HAND_EYE_PARK);
    
    cv::Mat cvinvT_ans = invH(cvHcg_ans);

    // std::cout<<cvRpark<<std::endl;
    // std::cout<<cvtpark<<std::endl;
    // std::cout<<cvHcg_ans<<std::endl;
    // std::cout<< Hgij[0] * cvHcg_ans<<std::endl;
    // std::cout<< cvHcg_ans * Hcij[0]<<std::endl;

    return 0;

}