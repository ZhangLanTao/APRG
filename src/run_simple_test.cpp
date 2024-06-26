#include <iostream>
#include "open3d/Open3D.h"
#include <opencv2/opencv.hpp>
//#define EIGEN_USE_MKL_ALL
#include <Eigen/Dense>

using namespace std;
//    auto t0 = std::chrono::high_resolution_clock::now();
//    auto t1 = std::chrono::high_resolution_clock::now();
//    auto t2 = std::chrono::high_resolution_clock::now();
//    auto t3 = std::chrono::high_resolution_clock::now();
//    auto time_elapsed1 = std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count();
//    auto time_elapsed2 = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
//    auto time_elapsed3 = std::chrono::duration_cast<std::chrono::microseconds>(t3-t2).count();
//    cout << "t1 t2 t3: " <<  time_elapsed1 << "    " << time_elapsed2 << "    " << time_elapsed3 << endl;

//
//float sum(const Eigen::MatrixXf & mat){
//    return mat.sum();
//}
//
//float sum_1(const Eigen::MatrixXf & mat){
//    float ret = 0;
//    #pragma omp parallel for reduction(+:ret)
//    for (int j = 0; j < mat.rows(); ++j) {
//        for (int k = 0; k < mat.cols(); ++k) {
//            ret += mat(j, k);
//        }
//    }
//    return ret;
//}
//
//float control(int i){
//    return i+0.1;
//}
//
//int compare_sum(){
//    auto t0 = std::chrono::high_resolution_clock::now();
//    auto t1 = t0;
//    long time_elapsed;
//    Eigen::MatrixXf mat(640, 480);
//    mat.setRandom();
//
//    // 调库
//    float result = 0;
//    t0 = std::chrono::high_resolution_clock::now();
//    for (int i = 0; i < 100; ++i) {
//        auto ret = sum(mat);
//        result += ret;
//    }
//    t1 = std::chrono::high_resolution_clock::now();
//    time_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
//    cout << "result:" <<result<<endl;
//    cout<<"time: "<<time_elapsed<<endl;
//
//    // 手写
//    result = 0;
//    t0 = std::chrono::high_resolution_clock::now();
//    for (int i = 0; i < 100; ++i) {
//        auto ret = sum_1(mat);
//        result += ret;
//    }
//    t1 = std::chrono::high_resolution_clock::now();
//    time_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
//    cout << "result:" <<result<<endl;
//    cout<<"time: "<<time_elapsed<<endl;
//
//    result = 0;
//    t0 = std::chrono::high_resolution_clock::now();
//    for (int i = 0; i < 100; ++i) {
//        auto ret = control(i);
//        result += ret;
//    }
//    t1 = std::chrono::high_resolution_clock::now();
//    time_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
//    std::cout<<"time: "<<time_elapsed<<endl;
//}
//
//
//int main(){
//    Eigen::MatrixXf mat(4, 4);
//    mat << 1, 2, 3, 4,
//            5, 6, 7, 8,
//            9, 10, 11, 12,
//            13, 14, 15, 16;
//
//    const auto& subMatRef = mat.block<2, 2>(1, 2);
//
//    std::cout << "Modified matrix:\n" << mat << std::endl;
//
//}

//void my_copy(const std::vector<int> &A, const std::vector<int> &B ,const std::vector<int> &C){
//
//
//    auto t0 = std::chrono::steady_clock::now();
//    std::vector<int> X = A;
//    auto t1 = std::chrono::steady_clock::now();
//    std::vector<int> Y = B;
//    auto t2 = std::chrono::steady_clock::now();
//    std::vector<int> Z = C;
//    auto t3 = std::chrono::steady_clock::now();
//
//    auto time_elapsed1 = std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count();
//    auto time_elapsed2 = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
//    auto time_elapsed3 = std::chrono::duration_cast<std::chrono::microseconds>(t3-t2).count();
//    auto time_elapsed4 = std::chrono::duration_cast<std::chrono::microseconds>(t3-t0).count();
//
//    cout << "Process: " << time_elapsed1<< "   " << time_elapsed2 << "   " <<time_elapsed3 << "us" << endl;
//    cout << "All: " << time_elapsed4 << "us" << endl;
//
//}

//void my_copy(const cv::Mat &A, const cv::Mat &B ,const cv::Mat &C){
//    cv::Mat X =  cv::Mat::zeros(640, 480, CV_32F);
//    cv::Mat Y =  cv::Mat::zeros(640, 480, CV_32F);
//    cv::Mat Z =  cv::Mat::zeros(640, 480, CV_32F);
//
//
//    auto t0 = std::chrono::steady_clock::now();
//    A.copyTo(X);
//    auto t1 = std::chrono::steady_clock::now();
//    B.copyTo(Y);
//    auto t2 = std::chrono::steady_clock::now();
//    C.copyTo(Z);
//    auto t3 = std::chrono::steady_clock::now();
//
//    auto time_elapsed1 = std::chrono::duration_cast<std::chrono::microseconds>(t1-t0).count();
//    auto time_elapsed2 = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
//    auto time_elapsed3 = std::chrono::duration_cast<std::chrono::microseconds>(t3-t2).count();
//    auto time_elapsed4 = std::chrono::duration_cast<std::chrono::microseconds>(t3-t0).count();
//
//    cout << "Process: " << time_elapsed1<< "   " << time_elapsed2 << "   " <<time_elapsed3 << "us" << endl;
//    cout << "All: " << time_elapsed4 << "us" << endl;
//
//}


//int main(){
//    vector<int> test;
//    test.push_back(1);
//    cout<< test.capacity()<<"   ";
//    test.push_back(1);
//    cout<< test.capacity()<<"   ";
//    test.push_back(1);
//    cout<< test.capacity()<<"   ";
//    test.push_back(1);
//    cout<< test.capacity()<<"   ";
//    test.push_back(1);
//    cout<< test.capacity()<<"   ";
//    test.push_back(1);
//    cout<< test.capacity()<<"   ";
//    test.push_back(1);
//    cout<< test.capacity()<<"   ";
//    test.push_back(1);
//    cout<< test.capacity()<<"   ";
//    test.push_back(1);
//    cout<< test.capacity()<<"   ";
//    test.push_back(1);
//    cout<< test.capacity()<<"   ";
//}

int main(){
    int i = INT_MAX-2;
    for (int j = 0; j < 10; ++j) {
        ++i;
        cout<<i<<endl;
    }
    return 0;
}