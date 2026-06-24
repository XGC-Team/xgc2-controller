/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef MINCO_HPP
#define MINCO_HPP

#include <Eigen/Eigen>
#include <cmath>
#include <vector>

#include "multirotor_reference_trajectory/core/minco_backend/trajectory.hpp"

namespace minco {

// The banded system class is used for solving
// banded linear system Ax=b efficiently.
// A is an N*N band matrix with lower band width lowerBw
// and upper band width upperBw.
// Banded LU factorization has O(N) time complexity.
/**
 * @brief 带状矩阵系统类，用于高效求解带状线性系统 Ax=b
 *
 * 带状矩阵是一种稀疏矩阵，其非零元素集中在主对角线附近的带状区域内。
 * A 是一个 N×N 的带状矩阵，具有下带宽 lowerBw 和上带宽 upperBw。
 * 带状 LU 分解的时间复杂度为 O(N)，远优于一般矩阵的 O(N³)。
 */
class BandedSystem {
   public:
    // The size of A, as well as the lower/upper
    // banded width p/q are needed
    /**
     * @brief 创建带状矩阵系统
     * @param n 矩阵大小 N×N
     * @param p 下带宽（主对角线下方的非零带宽度）
     * @param q 上带宽（主对角线上方的非零带宽度）
     *
     * 带状矩阵存储优化：只存储非零带状区域，总大小为 N×(p+q+1)
     */
    inline void create(const int& n, const int& p, const int& q) {
        // In case of re-creating before destroying
        // 防止重复创建导致内存泄漏，先销毁已有数据
        destroy();
        N = n;
        lowerBw = p;
        upperBw = q;
        int actualSize = N * (lowerBw + upperBw + 1);
        ptrData = new double[actualSize];
        std::fill_n(ptrData, actualSize, 0.0);
        return;
    }

    /**
     * @brief 销毁带状矩阵系统，释放内存
     */
    inline void destroy() {
        if (ptrData != nullptr) {
            delete[] ptrData;
            ptrData = nullptr;
        }
        return;
    }

   private:
    int N;        // 矩阵维度
    int lowerBw;  // 下带宽
    int upperBw;  // 上带宽
    // Compulsory nullptr initialization here
    double* ptrData = nullptr;  // 带状矩阵数据指针（必须初始化为空指针）

   public:
    // Reset the matrix to zero
    /**
     * @brief 将矩阵所有元素重置为零
     */
    inline void reset(void) {
        std::fill_n(ptrData, N * (lowerBw + upperBw + 1), 0.0);
        return;
    }

    // The band matrix is stored as suggested in "Matrix Computation"
    /**
     * @brief 访问带状矩阵元素（常量版本）
     * @param i 行索引
     * @param j 列索引
     * @return 矩阵元素的常量引用
     *
     * 存储方式参考《Matrix Computation》：
     * 映射公式：ptrData[(i-j+upperBw)*N+j]
     */
    inline const double& operator()(const int& i, const int& j) const {
        return ptrData[(i - j + upperBw) * N + j];
    }

    /**
     * @brief 访问带状矩阵元素（可修改版本）
     * @param i 行索引
     * @param j 列索引
     * @return 矩阵元素的引用
     */
    inline double& operator()(const int& i, const int& j) {
        return ptrData[(i - j + upperBw) * N + j];
    }

    // This function conducts banded LU factorization in place
    // Note that NO PIVOT is applied on the matrix "A" for efficiency!!!
    /**
     * @brief 原位带状 LU 分解
     *
     * 对带状矩阵 A 进行 LU 分解，分解结果存储在原矩阵中。
     * 注意：为了效率，不使用主元选择（NO PIVOT）！
     * 这要求矩阵必须是良态的（well-conditioned）。
     */
    inline void factorizeLU() {
        int iM, jM;
        double cVl;
        // 对每一列进行 LU 分解
        for (int k = 0; k <= N - 2; k++) {
            iM = std::min(k + lowerBw, N - 1);  // 下带宽限制的最大行索引
            cVl = operator()(k, k);             // 主元素
            // 计算 L 矩阵的第 k 列（在主对角线以下）
            for (int i = k + 1; i <= iM; i++) {
                if (operator()(i, k) != 0.0) {
                    operator()(i, k) /= cVl;  // L[i,k] = A[i,k] / A[k,k]
                }
            }
            jM = std::min(k + upperBw, N - 1);  // 上带宽限制的最大列索引
            // 更新 U 矩阵的剩余部分
            for (int j = k + 1; j <= jM; j++) {
                cVl = operator()(k, j);
                if (cVl != 0.0) {
                    for (int i = k + 1; i <= iM; i++) {
                        if (operator()(i, k) != 0.0) {
                            // A[i,j] -= L[i,k] * U[k,j]
                            operator()(i, j) -= operator()(i, k) * cVl;
                        }
                    }
                }
            }
        }
        return;
    }

    // This function solves Ax=b, then stores x in b
    // The input b is required to be N*m, i.e.,
    // m vectors to be solved.
    /**
     * @brief 求解线性系统 Ax=b，解存储在 b 中
     * @param b 输入右端项，输出解向量。维度为 N×m（可同时求解 m 个右端项）
     *
     * 使用 LU 分解求解：
     * 1. 前向替换求解 Ly=b
     * 2. 后向替换求解 Ux=y
     */
    template <typename EIGENMAT>
    inline void solve(EIGENMAT& b) const {
        int iM;
        // 前向替换：求解 Ly = b（L 为单位下三角矩阵）
        for (int j = 0; j <= N - 1; j++) {
            iM = std::min(j + lowerBw, N - 1);
            for (int i = j + 1; i <= iM; i++) {
                if (operator()(i, j) != 0.0) {
                    b.row(i) -= operator()(i, j) * b.row(j);
                }
            }
        }
        // 后向替换：求解 Ux = y（U 为上三角矩阵）
        for (int j = N - 1; j >= 0; j--) {
            b.row(j) /= operator()(j, j);  // 除以对角元素
            iM = std::max(0, j - upperBw);
            for (int i = iM; i <= j - 1; i++) {
                if (operator()(i, j) != 0.0) {
                    b.row(i) -= operator()(i, j) * b.row(j);
                }
            }
        }
        return;
    }

    // This function solves ATx=b, then stores x in b
    // The input b is required to be N*m, i.e.,
    // m vectors to be solved.
    /**
     * @brief 求解伴随（转置）线性系统 A^T x=b，解存储在 b 中
     * @param b 输入右端项，输出解向量。维度为 N×m
     *
     * 用于梯度反向传播，求解转置系统：
     * 1. 前向替换求解 U^T y = b
     * 2. 后向替换求解 L^T x = y
     */
    template <typename EIGENMAT>
    inline void solveAdj(EIGENMAT& b) const {
        int iM;
        // 前向替换：求解 U^T y = b（U^T 为下三角矩阵）
        for (int j = 0; j <= N - 1; j++) {
            b.row(j) /= operator()(j, j);
            iM = std::min(j + upperBw, N - 1);
            for (int i = j + 1; i <= iM; i++) {
                if (operator()(j, i) != 0.0) {
                    b.row(i) -= operator()(j, i) * b.row(j);
                }
            }
        }
        // 后向替换：求解 L^T x = y（L^T 为上三角矩阵）
        for (int j = N - 1; j >= 0; j--) {
            iM = std::max(0, j - lowerBw);
            for (int i = iM; i <= j - 1; i++) {
                if (operator()(j, i) != 0.0) {
                    b.row(i) -= operator()(j, i) * b.row(j);
                }
            }
        }
        return;
    }
};

// MINCO for s=2 and non-uniform time
/**
 * @brief MINCO 类：最小控制（Minimum Control）轨迹表示，s=2，非均匀时间
 *
 * MINCO (Minimum Control) 是一种稀疏的多项式轨迹表示方法。
 * s=2 表示优化目标为加速度（2阶导数）的平方积分最小。
 * 非均匀时间：每段轨迹的持续时间可以不同。
 *
 * 多项式阶数：2s+1 = 5（五次多项式）
 * 边界条件：位置(P)和速度(V)
 */
class MINCO_S2NU {
   public:
    MINCO_S2NU() = default;
    // 析构函数：销毁带状矩阵系统
    ~MINCO_S2NU() {
        A.destroy();
    }

   private:
    int N;                               // 轨迹段数
    Eigen::Matrix<double, 3, 2> headPV;  // 起始状态：位置(P)和速度(V)，3维空间
    Eigen::Matrix<double, 3, 2> tailPV;  // 终止状态：位置(P)和速度(V)，3维空间
    BandedSystem A;                      // 带状线性系统矩阵
    Eigen::MatrixX3d b;                  // 多项式系数矩阵（4N×3）
    Eigen::VectorXd T1;                  // 每段时间 t
    Eigen::VectorXd T2;                  // 每段时间的平方 t²
    Eigen::VectorXd T3;                  // 每段时间的三次方 t³

   public:
    /**
     * @brief 设置边界条件
     * @param headState 起始状态 [位置, 速度]（3×2矩阵）
     * @param tailState 终止状态 [位置, 速度]（3×2矩阵）
     * @param pieceNum 轨迹段数
     *
     * 三次多项式(s=2)有4个系数，N段轨迹共4N个系数需要求解。
     * 带状矩阵带宽为4（上下各4），因为连续性约束只涉及相邻段。
     */
    inline void setConditions(const Eigen::Matrix<double, 3, 2>& headState,
                              const Eigen::Matrix<double, 3, 2>& tailState, const int& pieceNum) {
        N = pieceNum;
        headPV = headState;
        tailPV = tailState;
        A.create(4 * N, 4, 4);  // 创建 4N×4N 的带状矩阵，带宽为4
        b.resize(4 * N, 3);     // 系数矩阵：4个系数/段 × N段 × 3个维度
        T1.resize(N);
        T2.resize(N);
        T3.resize(N);
        return;
    }

    /**
     * @brief 设置轨迹参数并求解多项式系数
     * @param inPs 中间路径点（3×(N-1)矩阵）
     * @param ts 每段轨迹的时间分配（N维向量）
     *
     * 构建并求解线性系统 Ac=b，其中：
     * - A 为约束矩阵（边界条件 + 连续性约束）
     * - c 为多项式系数（待求解）
     * - b 为约束值（边界状态 + 中间路径点）
     */
    inline void setParameters(const Eigen::Matrix3Xd& inPs, const Eigen::VectorXd& ts) {
        T1 = ts;
        T2 = T1.cwiseProduct(T1);  // 逐元素平方
        T3 = T2.cwiseProduct(T1);  // 逐元素三次方

        A.reset();
        b.setZero();

        // 起始边界条件：位置和速度
        A(0, 0) = 1.0;                         // p(0) = c0
        A(1, 1) = 1.0;                         // v(0) = c1
        b.row(0) = headPV.col(0).transpose();  // 起始位置
        b.row(1) = headPV.col(1).transpose();  // 起始速度

        // 中间段约束：加速度连续性 + 位置约束
        for (int i = 0; i < N - 1; i++) {
            // 加速度连续性：a_i(T_i) = a_{i+1}(0)
            A(4 * i + 2, 4 * i + 2) = 2.0;
            A(4 * i + 2, 4 * i + 3) = 6.0 * T1(i);
            A(4 * i + 2, 4 * i + 6) = -2.0;

            // 第 i 段末端位置：p_i(T_i) = inPs[i]
            A(4 * i + 3, 4 * i) = 1.0;
            A(4 * i + 3, 4 * i + 1) = T1(i);
            A(4 * i + 3, 4 * i + 2) = T2(i);
            A(4 * i + 3, 4 * i + 3) = T3(i);

            // 位置连续性：p_i(T_i) = p_{i+1}(0)
            A(4 * i + 4, 4 * i) = 1.0;
            A(4 * i + 4, 4 * i + 1) = T1(i);
            A(4 * i + 4, 4 * i + 2) = T2(i);
            A(4 * i + 4, 4 * i + 3) = T3(i);
            A(4 * i + 4, 4 * i + 4) = -1.0;

            // 速度连续性：v_i(T_i) = v_{i+1}(0)
            A(4 * i + 5, 4 * i + 1) = 1.0;
            A(4 * i + 5, 4 * i + 2) = 2.0 * T1(i);
            A(4 * i + 5, 4 * i + 3) = 3.0 * T2(i);
            A(4 * i + 5, 4 * i + 5) = -1.0;

            b.row(4 * i + 3) = inPs.col(i).transpose();  // 中间路径点
        }

        // 终止边界条件：位置和速度
        A(4 * N - 2, 4 * N - 4) = 1.0;
        A(4 * N - 2, 4 * N - 3) = T1(N - 1);
        A(4 * N - 2, 4 * N - 2) = T2(N - 1);
        A(4 * N - 2, 4 * N - 1) = T3(N - 1);
        A(4 * N - 1, 4 * N - 3) = 1.0;
        A(4 * N - 1, 4 * N - 2) = 2 * T1(N - 1);
        A(4 * N - 1, 4 * N - 1) = 3 * T2(N - 1);

        b.row(4 * N - 2) = tailPV.col(0).transpose();  // 终止位置
        b.row(4 * N - 1) = tailPV.col(1).transpose();  // 终止速度

        // 求解线性系统
        A.factorizeLU();  // LU 分解
        A.solve(b);       // 求解系数 b

        return;
    }

    /**
     * @brief 获取轨迹对象
     * @param traj 输出轨迹（阶数为3的轨迹，即三次多项式）
     *
     * 将系数转换为标准轨迹格式，系数按降幂排列。
     */
    inline void getTrajectory(Trajectory<3>& traj) const {
        traj.clear();
        traj.reserve(N);
        for (int i = 0; i < N; i++) {
            traj.emplace_back(
                T1(i),
                b.block<4, 3>(4 * i, 0).transpose().rowwise().reverse());  // 反转系数顺序为降幂排列
        }
        return;
    }

    /**
     * @brief 计算轨迹能量（加速度的平方积分）
     * @param energy 输出能量值
     *
     * 对于 s=2，能量为加速度平方的积分：∫a²dt
     * 由于 a(t) = 2c₂ + 6c₃t，积分可以解析计算。
     */
    inline void getEnergy(double& energy) const {
        energy = 0.0;
        for (int i = 0; i < N; i++) {
            energy += 4.0 * b.row(4 * i + 2).squaredNorm() * T1(i) +
                      12.0 * b.row(4 * i + 2).dot(b.row(4 * i + 3)) * T2(i) +
                      12.0 * b.row(4 * i + 3).squaredNorm() * T3(i);
        }
        return;
    }

    /**
     * @brief 获取多项式系数
     * @return 系数矩阵（4N×3）
     */
    inline const Eigen::MatrixX3d& getCoeffs(void) const {
        return b;
    }

    /**
     * @brief 计算能量对多项式系数的偏导数
     * @param gdC 输出梯度矩阵（4N×3）
     *
     * ∂E/∂cᵢ，用于优化算法中的梯度计算。
     */
    inline void getEnergyPartialGradByCoeffs(Eigen::MatrixX3d& gdC) const {
        gdC.resize(4 * N, 3);
        for (int i = 0; i < N; i++) {
            // ∂E/∂c₃：三次项系数的梯度
            gdC.row(4 * i + 3) = 12.0 * b.row(4 * i + 2) * T2(i) + 24.0 * b.row(4 * i + 3) * T3(i);
            // ∂E/∂c₂：二次项系数的梯度
            gdC.row(4 * i + 2) = 8.0 * b.row(4 * i + 2) * T1(i) + 12.0 * b.row(4 * i + 3) * T2(i);
            // c₀和c₁不影响加速度，梯度为零
            gdC.block<2, 3>(4 * i, 0).setZero();
        }
        return;
    }

    /**
     * @brief 计算能量对时间分配的偏导数
     * @param gdT 输出梯度向量（N维）
     *
     * ∂E/∂Tᵢ，用于时间优化。
     */
    inline void getEnergyPartialGradByTimes(Eigen::VectorXd& gdT) const {
        gdT.resize(N);
        for (int i = 0; i < N; i++) {
            gdT(i) = 4.0 * b.row(4 * i + 2).squaredNorm() +
                     24.0 * b.row(4 * i + 2).dot(b.row(4 * i + 3)) * T1(i) +
                     36.0 * b.row(4 * i + 3).squaredNorm() * T2(i);
        }
        return;
    }

    /**
     * @brief 梯度反向传播
     * @param partialGradByCoeffs 能量对系数的偏导数（4N×3）
     * @param partialGradByTimes 能量对时间的偏导数（N维）
     * @param gradByPoints 输出：能量对路径点的梯度（3×(N-1)）
     * @param gradByTimes 输出：能量对时间的完整梯度（N维）
     *
     * 使用伴随方法（adjoint method）计算梯度，将系数梯度传播到路径点和时间。
     * 这是自动微分中的反向模式（reverse mode）。
     */
    inline void propogateGrad(const Eigen::MatrixX3d& partialGradByCoeffs,
                              const Eigen::VectorXd& partialGradByTimes,
                              Eigen::Matrix3Xd& gradByPoints, Eigen::VectorXd& gradByTimes)

    {
        gradByPoints.resize(3, N - 1);
        gradByTimes.resize(N);
        Eigen::MatrixX3d adjGrad = partialGradByCoeffs;
        A.solveAdj(adjGrad);  // 求解伴随系统 A^T λ = ∂E/∂c

        // 从伴随变量中提取路径点梯度
        for (int i = 0; i < N - 1; i++) {
            gradByPoints.col(i) = adjGrad.row(4 * i + 3).transpose();
        }

        Eigen::Matrix<double, 4, 3> B1;
        Eigen::Matrix<double, 2, 3> B2;
        // 计算时间梯度（隐式依赖）
        for (int i = 0; i < N - 1; i++) {
            // negative jerk（负急动度）
            B1.row(0) = -6.0 * b.row(i * 4 + 3);

            // negative velocity（负速度）
            B1.row(1) = -(b.row(i * 4 + 1) + 2.0 * T1(i) * b.row(i * 4 + 2) +
                          3.0 * T2(i) * b.row(i * 4 + 3));
            B1.row(2) = B1.row(1);

            // negative acceleration（负加速度）
            B1.row(3) = -(2.0 * b.row(i * 4 + 2) + 6.0 * T1(i) * b.row(i * 4 + 3));

            gradByTimes(i) = B1.cwiseProduct(adjGrad.block<4, 3>(4 * i + 2, 0)).sum();
        }

        // 最后一段的时间梯度
        // negative velocity（负速度）
        B2.row(0) = -(b.row(4 * N - 3) + 2.0 * T1(N - 1) * b.row(4 * N - 2) +
                      3.0 * T2(N - 1) * b.row(4 * N - 1));

        // negative acceleration（负加速度）
        B2.row(1) = -(2.0 * b.row(4 * N - 2) + 6.0 * T1(N - 1) * b.row(4 * N - 1));

        gradByTimes(N - 1) = B2.cwiseProduct(adjGrad.block<2, 3>(4 * N - 2, 0)).sum();

        gradByTimes += partialGradByTimes;  // 加上直接时间依赖的梯度
    }
};

// MINCO for s=3 and non-uniform time
/**
 * @brief MINCO 类：最小控制轨迹表示，s=3，非均匀时间
 *
 * s=3 表示优化目标为急动度（3阶导数，jerk）的平方积分最小。
 * 多项式阶数：2s+1 = 7（七次多项式）
 * 边界条件：位置(P)、速度(V)和加速度(A)
 */
class MINCO_S3NU {
   public:
    MINCO_S3NU() = default;
    // 析构函数：销毁带状矩阵系统
    ~MINCO_S3NU() {
        A.destroy();
    }

   private:
    int N;                    // 轨迹段数
    Eigen::Matrix3d headPVA;  // 起始状态：位置、速度、加速度（3×3矩阵）
    Eigen::Matrix3d tailPVA;  // 终止状态：位置、速度、加速度（3×3矩阵）
    BandedSystem A;           // 带状线性系统矩阵
    Eigen::MatrixX3d b;       // 多项式系数矩阵（6N×3）
    Eigen::VectorXd T1;       // 每段时间 t
    Eigen::VectorXd T2;       // t²
    Eigen::VectorXd T3;       // t³
    Eigen::VectorXd T4;       // t⁴
    Eigen::VectorXd T5;       // t⁵

   public:
    /**
     * @brief 设置边界条件
     * @param headState 起始状态 [位置, 速度, 加速度]（3×3矩阵）
     * @param tailState 终止状态 [位置, 速度, 加速度]（3×3矩阵）
     * @param pieceNum 轨迹段数
     *
     * 五次多项式(s=3)有6个系数，N段轨迹共6N个系数需要求解。
     */
    inline void setConditions(const Eigen::Matrix3d& headState, const Eigen::Matrix3d& tailState,
                              const int& pieceNum) {
        N = pieceNum;
        headPVA = headState;
        tailPVA = tailState;
        A.create(6 * N, 6, 6);  // 创建 6N×6N 的带状矩阵，带宽为6
        b.resize(6 * N, 3);     // 系数矩阵：6个系数/段 × N段 × 3个维度
        T1.resize(N);
        T2.resize(N);
        T3.resize(N);
        T4.resize(N);
        T5.resize(N);
        return;
    }

    /**
     * @brief 设置轨迹参数并求解多项式系数
     * @param inPs 中间路径点（3×(N-1)矩阵）
     * @param ts 每段轨迹的时间分配（N维向量）
     *
     * 构建约束矩阵：边界条件(PVA) + 连续性约束(PVA + jerk) + 中间路径点
     */
    inline void setParameters(const Eigen::Matrix3Xd& inPs, const Eigen::VectorXd& ts) {
        T1 = ts;
        T2 = T1.cwiseProduct(T1);  // t²
        T3 = T2.cwiseProduct(T1);  // t³
        T4 = T2.cwiseProduct(T2);  // t⁴
        T5 = T4.cwiseProduct(T1);  // t⁵

        A.reset();
        b.setZero();

        // 起始边界条件：位置、速度、加速度
        A(0, 0) = 1.0;                          // p(0) = c0
        A(1, 1) = 1.0;                          // v(0) = c1
        A(2, 2) = 2.0;                          // a(0) = 2c2
        b.row(0) = headPVA.col(0).transpose();  // 起始位置
        b.row(1) = headPVA.col(1).transpose();  // 起始速度
        b.row(2) = headPVA.col(2).transpose();  // 起始加速度

        // 中间段约束：急动度连续性 + snap连续性 + 位置约束 + 速度、加速度连续性
        for (int i = 0; i < N - 1; i++) {
            // 急动度连续性：j_i(T_i) = j_{i+1}(0)
            // j(t) = 6c₃ + 24c₄t + 60c₅t²
            A(6 * i + 3, 6 * i + 3) = 6.0;
            A(6 * i + 3, 6 * i + 4) = 24.0 * T1(i);
            A(6 * i + 3, 6 * i + 5) = 60.0 * T2(i);
            A(6 * i + 3, 6 * i + 9) = -6.0;
            // Snap连续性：s_i(T_i) = s_{i+1}(0)
            // snap = 24c₄ + 120c₅t
            A(6 * i + 4, 6 * i + 4) = 24.0;
            A(6 * i + 4, 6 * i + 5) = 120.0 * T1(i);
            A(6 * i + 4, 6 * i + 10) = -24.0;
            // 第 i 段末端位置：p_i(T_i) = inPs[i]
            // p(t) = c₀ + c₁t + c₂t² + c₃t³ + c₄t⁴ + c₅t⁵
            A(6 * i + 5, 6 * i) = 1.0;
            A(6 * i + 5, 6 * i + 1) = T1(i);
            A(6 * i + 5, 6 * i + 2) = T2(i);
            A(6 * i + 5, 6 * i + 3) = T3(i);
            A(6 * i + 5, 6 * i + 4) = T4(i);
            A(6 * i + 5, 6 * i + 5) = T5(i);
            // 位置连续性：p_i(T_i) = p_{i+1}(0)
            A(6 * i + 6, 6 * i) = 1.0;
            A(6 * i + 6, 6 * i + 1) = T1(i);
            A(6 * i + 6, 6 * i + 2) = T2(i);
            A(6 * i + 6, 6 * i + 3) = T3(i);
            A(6 * i + 6, 6 * i + 4) = T4(i);
            A(6 * i + 6, 6 * i + 5) = T5(i);
            A(6 * i + 6, 6 * i + 6) = -1.0;
            // 速度连续性：v_i(T_i) = v_{i+1}(0)
            // v(t) = c₁ + 2c₂t + 3c₃t² + 4c₄t³ + 5c₅t⁴
            A(6 * i + 7, 6 * i + 1) = 1.0;
            A(6 * i + 7, 6 * i + 2) = 2 * T1(i);
            A(6 * i + 7, 6 * i + 3) = 3 * T2(i);
            A(6 * i + 7, 6 * i + 4) = 4 * T3(i);
            A(6 * i + 7, 6 * i + 5) = 5 * T4(i);
            A(6 * i + 7, 6 * i + 7) = -1.0;
            // 加速度连续性：a_i(T_i) = a_{i+1}(0)
            // a(t) = 2c₂ + 6c₃t + 12c₄t² + 20c₅t³
            A(6 * i + 8, 6 * i + 2) = 2.0;
            A(6 * i + 8, 6 * i + 3) = 6 * T1(i);
            A(6 * i + 8, 6 * i + 4) = 12 * T2(i);
            A(6 * i + 8, 6 * i + 5) = 20 * T3(i);
            A(6 * i + 8, 6 * i + 8) = -2.0;

            b.row(6 * i + 5) = inPs.col(i).transpose();  // 中间路径点
        }

        // 终止边界条件：位置、速度、加速度
        A(6 * N - 3, 6 * N - 6) = 1.0;
        A(6 * N - 3, 6 * N - 5) = T1(N - 1);
        A(6 * N - 3, 6 * N - 4) = T2(N - 1);
        A(6 * N - 3, 6 * N - 3) = T3(N - 1);
        A(6 * N - 3, 6 * N - 2) = T4(N - 1);
        A(6 * N - 3, 6 * N - 1) = T5(N - 1);
        A(6 * N - 2, 6 * N - 5) = 1.0;
        A(6 * N - 2, 6 * N - 4) = 2 * T1(N - 1);
        A(6 * N - 2, 6 * N - 3) = 3 * T2(N - 1);
        A(6 * N - 2, 6 * N - 2) = 4 * T3(N - 1);
        A(6 * N - 2, 6 * N - 1) = 5 * T4(N - 1);
        A(6 * N - 1, 6 * N - 4) = 2;
        A(6 * N - 1, 6 * N - 3) = 6 * T1(N - 1);
        A(6 * N - 1, 6 * N - 2) = 12 * T2(N - 1);
        A(6 * N - 1, 6 * N - 1) = 20 * T3(N - 1);

        b.row(6 * N - 3) = tailPVA.col(0).transpose();  // 终止位置
        b.row(6 * N - 2) = tailPVA.col(1).transpose();  // 终止速度
        b.row(6 * N - 1) = tailPVA.col(2).transpose();  // 终止加速度

        // 求解线性系统
        A.factorizeLU();  // LU 分解
        A.solve(b);       // 求解系数 b

        return;
    }

    /**
     * @brief 获取轨迹对象
     * @param traj 输出轨迹（阶数为5的轨迹，即五次多项式）
     */
    inline void getTrajectory(Trajectory<5>& traj) const {
        traj.clear();
        traj.reserve(N);
        for (int i = 0; i < N; i++) {
            traj.emplace_back(T1(i), b.block<6, 3>(6 * i, 0).transpose().rowwise().reverse());
        }
        return;
    }

    /**
     * @brief 计算轨迹能量（急动度的平方积分）
     * @param energy 输出能量值
     *
     * 对于 s=3，能量为急动度平方的积分：∫j²dt
     * j(t) = 6c₃ + 24c₄t + 60c₅t²
     */
    inline void getEnergy(double& energy) const {
        energy = 0.0;
        for (int i = 0; i < N; i++) {
            energy += 36.0 * b.row(6 * i + 3).squaredNorm() * T1(i) +
                      144.0 * b.row(6 * i + 4).dot(b.row(6 * i + 3)) * T2(i) +
                      192.0 * b.row(6 * i + 4).squaredNorm() * T3(i) +
                      240.0 * b.row(6 * i + 5).dot(b.row(6 * i + 3)) * T3(i) +
                      720.0 * b.row(6 * i + 5).dot(b.row(6 * i + 4)) * T4(i) +
                      720.0 * b.row(6 * i + 5).squaredNorm() * T5(i);
        }
        return;
    }

    /**
     * @brief 获取多项式系数
     * @return 系数矩阵（6N×3）
     */
    inline const Eigen::MatrixX3d& getCoeffs(void) const {
        return b;
    }

    /**
     * @brief 计算能量对多项式系数的偏导数
     * @param gdC 输出梯度矩阵（6N×3）
     *
     * ∂E/∂cᵢ，用于优化算法中的梯度计算。
     */
    inline void getEnergyPartialGradByCoeffs(Eigen::MatrixX3d& gdC) const {
        gdC.resize(6 * N, 3);
        for (int i = 0; i < N; i++) {
            // ∂E/∂c₅：五次项系数的梯度
            gdC.row(6 * i + 5) = 240.0 * b.row(6 * i + 3) * T3(i) +
                                 720.0 * b.row(6 * i + 4) * T4(i) +
                                 1440.0 * b.row(6 * i + 5) * T5(i);
            // ∂E/∂c₄：四次项系数的梯度
            gdC.row(6 * i + 4) = 144.0 * b.row(6 * i + 3) * T2(i) +
                                 384.0 * b.row(6 * i + 4) * T3(i) +
                                 720.0 * b.row(6 * i + 5) * T4(i);
            // ∂E/∂c₃：三次项系数的梯度
            gdC.row(6 * i + 3) = 72.0 * b.row(6 * i + 3) * T1(i) +
                                 144.0 * b.row(6 * i + 4) * T2(i) +
                                 240.0 * b.row(6 * i + 5) * T3(i);
            // c₀, c₁, c₂不影响急动度，梯度为零
            gdC.block<3, 3>(6 * i, 0).setZero();
        }
        return;
    }

    /**
     * @brief 计算能量对时间分配的偏导数
     * @param gdT 输出梯度向量（N维）
     *
     * ∂E/∂Tᵢ，用于时间优化。
     */
    inline void getEnergyPartialGradByTimes(Eigen::VectorXd& gdT) const {
        gdT.resize(N);
        for (int i = 0; i < N; i++) {
            gdT(i) = 36.0 * b.row(6 * i + 3).squaredNorm() +
                     288.0 * b.row(6 * i + 4).dot(b.row(6 * i + 3)) * T1(i) +
                     576.0 * b.row(6 * i + 4).squaredNorm() * T2(i) +
                     720.0 * b.row(6 * i + 5).dot(b.row(6 * i + 3)) * T2(i) +
                     2880.0 * b.row(6 * i + 5).dot(b.row(6 * i + 4)) * T3(i) +
                     3600.0 * b.row(6 * i + 5).squaredNorm() * T4(i);
        }
        return;
    }

    /**
     * @brief 梯度反向传播
     * @param partialGradByCoeffs 能量对系数的偏导数（6N×3）
     * @param partialGradByTimes 能量对时间的偏导数（N维）
     * @param gradByPoints 输出：能量对路径点的梯度（3×(N-1)）
     * @param gradByTimes 输出：能量对时间的完整梯度（N维）
     *
     * 使用伴随方法计算梯度，将系数梯度传播到路径点和时间。
     */
    inline void propogateGrad(const Eigen::MatrixX3d& partialGradByCoeffs,
                              const Eigen::VectorXd& partialGradByTimes,
                              Eigen::Matrix3Xd& gradByPoints, Eigen::VectorXd& gradByTimes)

    {
        gradByPoints.resize(3, N - 1);
        gradByTimes.resize(N);
        Eigen::MatrixX3d adjGrad = partialGradByCoeffs;
        A.solveAdj(adjGrad);  // 求解伴随系统 A^T λ = ∂E/∂c

        // 从伴随变量中提取路径点梯度
        for (int i = 0; i < N - 1; i++) {
            gradByPoints.col(i) = adjGrad.row(6 * i + 5).transpose();
        }

        Eigen::Matrix<double, 6, 3> B1;
        Eigen::Matrix3d B2;
        // 计算时间梯度（隐式依赖）
        for (int i = 0; i < N - 1; i++) {
            // negative velocity（负速度）
            B1.row(2) = -(b.row(i * 6 + 1) + 2.0 * T1(i) * b.row(i * 6 + 2) +
                          3.0 * T2(i) * b.row(i * 6 + 3) + 4.0 * T3(i) * b.row(i * 6 + 4) +
                          5.0 * T4(i) * b.row(i * 6 + 5));
            B1.row(3) = B1.row(2);

            // negative acceleration（负加速度）
            B1.row(4) = -(2.0 * b.row(i * 6 + 2) + 6.0 * T1(i) * b.row(i * 6 + 3) +
                          12.0 * T2(i) * b.row(i * 6 + 4) + 20.0 * T3(i) * b.row(i * 6 + 5));

            // negative jerk（负急动度）
            B1.row(5) = -(6.0 * b.row(i * 6 + 3) + 24.0 * T1(i) * b.row(i * 6 + 4) +
                          60.0 * T2(i) * b.row(i * 6 + 5));

            // negative snap（负snap）
            B1.row(0) = -(24.0 * b.row(i * 6 + 4) + 120.0 * T1(i) * b.row(i * 6 + 5));

            // negative crackle（负crackle）
            B1.row(1) = -120.0 * b.row(i * 6 + 5);

            gradByTimes(i) = B1.cwiseProduct(adjGrad.block<6, 3>(6 * i + 3, 0)).sum();
        }

        // 最后一段的时间梯度
        // negative velocity（负速度）
        B2.row(0) = -(b.row(6 * N - 5) + 2.0 * T1(N - 1) * b.row(6 * N - 4) +
                      3.0 * T2(N - 1) * b.row(6 * N - 3) + 4.0 * T3(N - 1) * b.row(6 * N - 2) +
                      5.0 * T4(N - 1) * b.row(6 * N - 1));

        // negative acceleration（负加速度）
        B2.row(1) = -(2.0 * b.row(6 * N - 4) + 6.0 * T1(N - 1) * b.row(6 * N - 3) +
                      12.0 * T2(N - 1) * b.row(6 * N - 2) + 20.0 * T3(N - 1) * b.row(6 * N - 1));

        // negative jerk（负急动度）
        B2.row(2) = -(6.0 * b.row(6 * N - 3) + 24.0 * T1(N - 1) * b.row(6 * N - 2) +
                      60.0 * T2(N - 1) * b.row(6 * N - 1));

        gradByTimes(N - 1) = B2.cwiseProduct(adjGrad.block<3, 3>(6 * N - 3, 0)).sum();

        gradByTimes += partialGradByTimes;  // 加上直接时间依赖的梯度
    }
};

}  // namespace minco

#endif
