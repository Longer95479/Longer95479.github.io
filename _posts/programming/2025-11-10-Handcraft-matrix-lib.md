---
layout: post
title:  "Handcraft Your Own Matrix Library"
date:   2025-11-10 22:30:00 +0800
tags: 
  - programming
  - math 
categories:
  - programming
  - math 
---

视觉惯性里程计的前后端的一些计算，涉及矩阵的运算，一些问题的求解，最终都会转变为方程组的求解，
也与矩阵的运算或线性代数相关，想要从零实现一个 VIO 的前端，主要涉及到 *多视角几何（MVS）*，
而 MVS 中的问题的求解绝大部分也都是转换为方程组的求解，因此从零实现一个矩阵运算库就变成了必要条件。

本文将记录笔者用 C++ 实现一个简单的矩阵运算库（类似于 Eigen 库）的过程。

实现的功能有：

- 矩阵多种方式的初始化
- 矩阵的加减乘，以及标量除法
- 矩阵的转置、L2 范数
- QR分解、SVD分解


最后贴出单文件的源码：

```c++
#include <iostream>
#include <vector>
#include <cmath>
#include <map>

template<typename T, int rows, int cols>
class Matrix {
public:
    Matrix() 
    {
        allocate_mem();

        for (int i = 0; i < rows_; i++) {
            for (int j = 0; j < cols_; j++) {
                data_ptrs_[i][j] = 0;
            }
        }
    }

    Matrix(const Matrix<T, rows, cols>& rhs) 
    : start_row_(rhs.start_row_),
      start_col_(rhs.start_col_),
      rows_(rhs.rows_),
      cols_(rhs.cols_)
    {
        allocate_mem();
        for (int i = 0; i < rows_; i++) {
            for (int j = 0; j < cols_; j++) {
                data_ptrs_[i][j] = rhs.data_ptrs_[i][j];
            }
        }
    }

    Matrix(Matrix<T, rows, cols>&& rhs) 
    : start_row_(rhs.start_row_),
      start_col_(rhs.start_col_),
      rows_(rhs.rows_),
      cols_(rhs.cols_)
    {
        data_ptrs_ = rhs.data_ptrs_;
        rhs.data_ptrs_ = nullptr;
    }

    Matrix(const std::vector<std::vector<T>>& data) 
    {
        if (data.size() != rows_ || data[0].size() != cols_)
            throw "SIZE_ERROR";

        allocate_mem();
        for (int i = 0; i < rows_; i++) {
            for (int j = 0; j < cols_; j++) {
                (*this)(i, j) = data[i][j];
            }
        }
    }

    Matrix(const std::vector<std::vector<T>>&& data) 
    {
        if (data.size() != rows_ || data[0].size() != cols_)
            throw "SIZE_ERROR";

        allocate_mem();
        for (int i = 0; i < rows_; i++) {
            for (int j = 0; j < cols_; j++) {
                (*this)(i, j) = data[i][j];
            }
        }
    }

    Matrix(T** ref_ptrs, int start_row, int start_col, int blk_rows, int blk_cols)
    : start_row_(start_row),
      start_col_(start_col),
      rows_(blk_rows),
      cols_(blk_cols),
      data_ptrs_(ref_ptrs),
      is_block_(true)
    {}


    ~Matrix() 
    { 
        if (data_ptrs_ && !is_block_) {
            for (int i = 0; i < rows_; i++) {
                delete[] data_ptrs_[i];
            }
            delete[] data_ptrs_;
        }
    }

    T& operator () (const int i, const int j)
    {
        if (data_ptrs_ && i < rows_ && j < cols_)
            return data_ptrs_[i + start_row_][j + start_col_];
        else
            throw "OVER_ACCESE_ERROR";
    }

    const T& operator () (const int i, const int j) const
    {
        if (data_ptrs_ && i < rows_ && j < cols_)
            return data_ptrs_[i + start_row_][j + start_col_];
        else 
            throw "OVER_ACCESE_ERROR";
    }

    friend std::ostream& operator << (std::ostream& os, const Matrix& mat)
    {
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                printf("%10.4f\t", mat(i, j));
            }
            os << std::endl;
        }
        return os;
    }

    // deep copy, not support shallow copy yet (having double free bug)
    void operator = (const Matrix<T, rows, cols>& rhs)
    {
        for (int i = 0; i < rows_; i++) {
            for (int j = 0; j < cols_; j++) {
                (*this)(i, j) = rhs(i, j);
            }
        }
    }
    
    void operator = (Matrix<T, rows, cols>&& rhs)
    {
        // NOTE HERE!
        if (is_block_ != rhs.is_block_) {
            for (int i = 0; i < rows_; i++) {
                for (int j = 0; j < cols_; j++) {
                    (*this)(i, j) = rhs(i, j);
                }
            }
        }
        else {
            data_ptrs_ = rhs.data_ptrs_;
            rhs.data_ptrs_ = nullptr; // NOTE: if don't have this, double free will happend in ~Matrix()
            start_row_ = rhs.start_row_; 
            start_col_ = rhs.start_col_; 
            rows_ = rhs.rows_;
            cols_ = rhs.cols_;
        }
    }
 

    Matrix operator + (const Matrix<T, rows, cols>& other)
    {
        Matrix<T, rows, cols> tmp_mat;
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                tmp_mat(i, j) = (*this)(i, j) + other(i, j);
            }
        }
        return tmp_mat;
    }

    Matrix operator - (const Matrix<T, rows, cols>& other)
    {
        Matrix<T, rows, cols> tmp_mat;
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                tmp_mat(i, j) = (*this)(i, j) - other(i, j);
            }
        }
        return tmp_mat;
    }

    template<int p>
    Matrix<T, rows, p> operator * (const Matrix<T, cols, p>& other)
    {
        Matrix<T, rows, p> tmp_mat;
        T sum = 0;
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < p; j++) {
                sum = 0;
                for (int k = 0; k < cols; k++) {
                    sum += (*this)(i, k) * other(k, j);
                }
                tmp_mat(i, j) = sum;
            }
        }
        return tmp_mat;
    }

    Matrix<T, rows, cols> operator * (T scalar)
    {
        Matrix<T, rows, cols> tmp_mat;
        for (int i = 0; i < rows_; i++) {
            for (int j = 0; j < cols_; j++) {
                tmp_mat(i, j) = (*this)(i, j) * scalar;
            }
        }
        return tmp_mat;
    }

    Matrix<T, rows, cols> operator / (T scalar)
    {
        if (std::abs(scalar - .0) < 1e-9)
            throw "ZERO_DEVIDE_ERROR";
        Matrix<T, rows, cols> tmp_mat;
        for (int i = 0; i < rows_; i++) {
            for (int j = 0; j < cols_; j++) {
                tmp_mat(i, j) = (*this)(i, j) / scalar;
            }
        }
        return tmp_mat;
    }

    template<int blk_rows, int blk_cols>
    Matrix<T, blk_rows, blk_cols> block(int start_row, int start_col)
    {
        return Matrix<T, blk_rows, blk_cols>(data_ptrs_, start_row, start_col, blk_rows, blk_cols);
    }

    template<int blk_rows, int blk_cols>
    Matrix<T, blk_rows, blk_cols> block(int start_row, int start_col) const
    {
        return Matrix<T, blk_rows, blk_cols>(data_ptrs_, start_row, start_col, blk_rows, blk_cols);
    }

    Matrix<T, 1, cols> row(int i)
    {
        return block<1, cols>(i, 0);
    }

    Matrix<T, rows, 1> col(int j)
    {
        return block<rows, 1>(0, j);
    }
    
    Matrix<T, cols, rows> transpose()
    {
        Matrix<T, cols, rows> tmp_mat;
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                tmp_mat(j, i) = (*this)(i, j);
            }
        }
        return tmp_mat;
    }

    T norm()
    {
        T n = 0;
        for (int i = 0; i < rows_; i++) {
            for (int j = 0; j < cols_; j++) {
                n += (*this)(i, j) * (*this)(i, j);
            }
        }
        return std::sqrt(n);
    }

    void svdDecompose(const int iter_nums,
                      Matrix<T, rows, rows>* U, 
                      Matrix<T, rows, cols>* C,
                      Matrix<T, cols, cols>* V)
    {
        // check rows >= cols
        if constexpr (rows < cols) {
            throw "ERROR_ROWS_LESS_THAN_COLS";
        }
        
        Matrix<T, rows, cols> A = (*this); // Call copy constructor instead of move constructor.
        std::pair<Matrix<T, rows, rows>, Matrix<T, rows, cols>> U_and_R0;
        std::pair<Matrix<T, cols, cols>, Matrix<T, cols, cols>> V_and_CT;
        Matrix<T, rows, cols> R0;
        Matrix<T, cols, cols> RT;
        Matrix<T, rows, cols> C0;

        for (int i = 0; i < rows; i++) {
            (*U)(i, i) = 1;
        }
        for (int i = 0; i < cols; i++) {
            (*V)(i, i) = 1;
        }

        for (int i = 0; i < iter_nums; i++) {

            U_and_R0 = A.qrDecompose();

            *U = (*U) * U_and_R0.first;
            R0 = U_and_R0.second;
            RT = R0.template block<cols, cols>(0, 0).transpose();

            V_and_CT = RT.qrDecompose();

            *V = (*V) * V_and_CT.first;
            C0.template block<cols, cols>(0, 0) = V_and_CT.second.transpose();
            A = C0;
        }

        *C = A;

    }

    std::pair<Matrix<T, rows, rows>, Matrix<T, rows, cols>> qrDecompose()
    {
        // check rows >= cols and (TODO) cols rank full
        if constexpr (rows < cols) {
            throw "ERROR_ROWS_LESS_THAN_COLS";
        }

        // Hs = ae
        // H = I - wwT, w = norm(s - ae), a = norm(s)
        // H(0) = I, A(0) = A
        // H(i), A(i) <- H(i-1), A(i-1) :
        // s(i) = A(i-1).block<rows-i, 1>(i, i)
        // e(i) = Matrix<T, rows-i, rows-i>::eyes()
        // H(i) = (I - 2*wwT) * H(i-1)

        Matrix<T, rows, rows> I;
        for (int i = 0; i < rows_; i++) {
            I(i, i) = 1;
        }
        return calQR<0>(I, (*this));
    }

    template<int i>
    std::pair<Matrix<T, rows, rows>, Matrix<T, rows, cols>> calQR(const Matrix<T, rows, rows>& H, 
                                                                  const Matrix<T, rows, cols>& A)
    {
        // std::cout << "i: " << i << std::endl;
        // std::cout << "H: " << std::endl << H << std::endl;
        // std::cout << "A: " << std::endl << A << std::endl;

        Matrix<T, rows, rows> H_tmp;
        for (int k = 0; k < i; k++) {
            H_tmp(k, k) = 1;
        }

        Matrix<T, rows - i, rows - i> I;
        for (int k = 0; k < rows - i; k++) {
            I(k, k) = 1;
        }

        Matrix<T, rows - i, 1> s, w, e;
        s = A.template block<rows - i, 1>(i, i);
        e(0, 0) = 1;
        w = s - e * s.norm();
        if (0 != w.norm()) w = w / w.norm();

        H_tmp.template block<rows - i, rows - i>(i, i) = I - w * w.transpose() * 2;

        // NOTE HERE: using template param i to replace for() loop in meta programming.
        if constexpr (i == cols - 1) 
            return {(H_tmp * H).transpose(), H_tmp * A};
        else 
            return calQR<i + 1>(H_tmp * H, H_tmp * A);
    }

    Matrix<T, cols, 1> solveUsingHouseholderQR(const Matrix<T, rows, 1>& b)
    {
        // check rows >= cols
        if constexpr (rows < cols) {
            throw "ERROR_ROWS_LESS_THAN_COLS";
        }

        // Ax = b
        // ATAx = ATb
        // [RT 0] [R] x = [RT 0] QTb
        //        [0]
        // Rx = e, e is first cols_ elements of QTb.

        Matrix<T, cols, 1> x;
        auto QR = qrDecompose();
        auto Q = QR.first;
        auto R = QR.second;
        auto QTb = Q.transpose() * b;

        for (int i = cols_ - 1; i >= 0; i--) {
            T sum = 0;
            for (int j = cols_ - 1; j > i; j--) {
                sum += R(i, j) * x(j, 0);
            }
            x(i, 0) = (QTb(i, 0) - sum) / R(i, i);
        }

        return x;
    }


private:
    T** data_ptrs_ = nullptr;
    int start_row_ = 0;
    int start_col_ = 0;
    int rows_ = rows;
    int cols_ = cols;
    bool is_block_ = false;

    void allocate_mem()
    {
        data_ptrs_ = new T*[rows_];
        for (int i = 0; i < rows_; i++) {
            data_ptrs_[i] = new T[cols_];
        }
    }
};


void test(void)
{
    Matrix<double, 4, 3> A({{1, 2, 3},
                            {4, 5, 6},
                            {7, 8, 9},
                            {10, 11, 12}});
    
    Matrix<double, 4, 3> B;
    B = A;

    Matrix<double, 4, 3> C(B);

    A(1, 1) = 13;

    std::cout << "A:" << std::endl << A << std::endl;
    std::cout << "B:" << std::endl << B << std::endl;
    std::cout << "C:" << std::endl << C << std::endl;
    std::cout << "B.block<2, 1>(1, 1)" << std::endl << B.block<2, 1>(1, 1) << std::endl;

    B.block<2, 1>(1, 1) = Matrix<double, 2, 1>({{954}, {79}});
    std::cout << B << std::endl;

    std::cout << "A + B:" << std::endl << A + B << std::endl;
    std::cout << "A - B:" << std::endl << A - B << std::endl;
    Matrix<double, 3, 3> D;
    D = A.transpose() * A;
    std::cout << "D = A.transpose() * A:" << std::endl << D << std::endl;
    std::cout << "D.row(1): " << std::endl << D.row(1) << std::endl;
    std::cout << "D.col(1): " << std::endl << D.col(1) << std::endl;
    std::cout << "D.col(1).norm(): " << std::endl << D.col(1).norm() << std::endl << std::endl;
    std::cout << "D.col(1) / D.col(1).norm(): " << std::endl << D.col(1) / D.col(1).norm() << std::endl;
            
    auto QR = A.qrDecompose();
    auto Q = QR.first;
    auto R = QR.second;
    auto b = Matrix<double, 4, 1>({{3}, {6}, {9}, {12}});
    auto x = A.solveUsingHouseholderQR(b);

    std::cout << "Q:" << std::endl << Q << std::endl;
    std::cout << "R:" << std::endl << R << std::endl;
    std::cout << "QR:" << std::endl << Q * R << std::endl;
    std::cout << "QTQ:" << std::endl << Q.transpose() * Q << std::endl;

    std::cout << "x:" << std::endl << x << std::endl;
    std::cout << "Ax:" << std::endl << A * x << std::endl;
    std::cout << "b:" << std::endl << b << std::endl;

    Matrix<double, 4, 4> U;
    Matrix<double, 4, 3> C0;
    Matrix<double, 3, 3> V;
    A.svdDecompose(10, &U, &C0, &V);
    std::cout << "U:" << std::endl << U << std::endl;
    std::cout << "C:" << std::endl << C0 << std::endl;
    std::cout << "V:" << std::endl << V << std::endl;
    std::cout << "UCVT:" << std::endl << U * C0 * V.transpose() << std::endl;
    std::cout << "UTU:" << std::endl << U.transpose() * U << std::endl;
    std::cout << "VTV:" << std::endl << V.transpose() * V << std::endl;

}


int main(void)
{
    try {
        test();
    }
    catch (const char* e) {
        std::cout << e << std::endl;
    }

    return 0;
}
```
