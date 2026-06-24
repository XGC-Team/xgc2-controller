/*
    MIT License

    Copyright (c) 2020 Zhepei Wang (wangzhepei@live.com)

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

// 多项式求根器头文件
// 该文件实现了多种多项式求根算法，包括解析解法和数值解法
#ifndef ROOT_FINDER_HPP
#define ROOT_FINDER_HPP

#define _USE_MATH_DEFINES
#include <Eigen/Eigen>
#include <cfloat>
#include <cmath>
#include <set>

// 求根器参数命名空间
namespace RootFinderParam {
constexpr size_t highestOrder = 64;  // 支持的最高多项式阶数
}

// 求根器私有函数命名空间
// 包含了求根算法的内部实现函数
namespace RootFinderPriv {

/**
 * @brief 计算多项式除法的余数 u(x) mod v(x)
 * @param u 被除多项式系数数组
 * @param v 除多项式系数数组（首项系数必须为 1.0 或 -1.0）
 * @param r 存储余数的数组
 * @param lu 被除多项式的长度
 * @param lv 除多项式的长度
 * @return 余数多项式的长度
 *
 * 注：v 的首项系数（即 v[0]）必须为 1.0 或 -1.0
 *     u, v, r 的长度分别为 lu, lv, lu
 */
inline int polyMod(double* u, double* v, double* r, int lu, int lv) {
    int orderu = lu - 1;  // 被除多项式的阶数
    int orderv = lv - 1;  // 除多项式的阶数

    // 将 u 复制到 r 中作为初始值
    memcpy(r, u, lu * sizeof(double));

    // 根据 v 的首项系数符号进行不同的处理
    if (v[0] < 0.0) {
        // 如果首项系数为负，对奇数次幂的项取反
        for (int i = orderv + 1; i <= orderu; i += 2) {
            r[i] = -r[i];
        }
        // 执行多项式除法（长除法）
        for (int i = 0; i <= orderu - orderv; i++) {
            for (int j = i + 1; j <= orderv + i; j++) {
                r[j] = -r[j] - r[i] * v[j - i];
            }
        }
    } else {
        // 首项系数为正时的标准多项式除法
        for (int i = 0; i <= orderu - orderv; i++) {
            for (int j = i + 1; j <= orderv + i; j++) {
                r[j] = r[j] - r[i] * v[j - i];
            }
        }
    }

    // 去除余数多项式中的高阶零系数，确定实际阶数
    int k = orderv - 1;
    while (k >= 0 && fabs(r[orderu - k]) < DBL_EPSILON) {
        r[orderu - k] = 0.0;
        k--;
    }

    // 返回余数多项式的实际长度
    return (k <= 0) ? 1 : (k + 1);
}

/**
 * @brief 计算多项式在点 x 处的值
 * @param p 多项式系数数组（长度为 len）
 * @param len 系数数组的长度
 * @param x 求值点
 * @return 多项式 p(x) 在点 x 处的值
 *
 * 注意：此处不应使用 Horner 方法！！！
 * 尽管 Horner 方法效率高，但数值稳定性差。
 * 这些误差对于求根算法尤其有害。
 * 当在零点附近求值时，必然会发生灾难性抵消（相近数相减）。
 * 因此，Horner 方法可能会降低某些求根算法的速度。
 */
inline double polyEval(double* p, int len, double x) {
    double retVal = 0.0;

    if (len > 0) {
        if (fabs(x) < DBL_EPSILON) {
            // 当 x ≈ 0 时，多项式值就是常数项
            retVal = p[len - 1];
        } else if (x == 1.0) {
            // 当 x = 1 时，多项式值就是所有系数之和
            for (int i = len - 1; i >= 0; i--) {
                retVal += p[i];
            }
        } else {
            // 一般情况：直接幂次计算（更稳定但效率较低）
            double xn = 1.0;

            for (int i = len - 1; i >= 0; i--) {
                retVal += p[i] * xn;
                xn *= x;  // 累乘计算 x 的幂次
            }
        }
    }

    return retVal;
}

/**
 * @brief 求解三次方程 a*x^3 + b*x^2 + c*x + d = 0 的所有实根
 * @param a 三次项系数
 * @param b 二次项系数
 * @param c 一次项系数
 * @param d 常数项
 * @return 包含所有实根的集合（自动去重）
 */
inline std::set<double> solveCub(double a, double b, double c, double d) {
    std::set<double> roots;

    // cos(120°) 和 sin(120°) 的常量值
    constexpr double cos120 = -0.50;
    constexpr double sin120 = 0.866025403784438646764;

    // 如果常数项为 0，说明 x=0 是一个根
    if (fabs(d) < DBL_EPSILON) {
        // 第一个解是 x = 0
        roots.insert(0.0);

        // 降阶为二次方程：a*x^2 + b*x + c = 0
        d = c;
        c = b;
        b = a;
        a = 0.0;
    }

    // 如果三次项系数为 0，降阶处理
    if (fabs(a) < DBL_EPSILON) {
        if (fabs(b) < DBL_EPSILON) {
            // 一次方程：c*x + d = 0
            if (fabs(c) > DBL_EPSILON)
                roots.insert(-d / c);
        } else {
            // 二次方程：b*x^2 + c*x + d = 0
            double discriminant = c * c - 4.0 * b * d;  // 判别式 Δ = b^2 - 4ac
            if (discriminant >= 0) {
                double inv2b = 1.0 / (2.0 * b);
                double y = sqrt(discriminant);
                // 两个实根：x = (-b ± √Δ) / (2a)
                roots.insert((-c + y) * inv2b);
                roots.insert((-c - y) * inv2b);
            }
        }
    } else {
        // 三次方程：使用 Cardano 公式求解
        double inva = 1.0 / a;
        double invaa = inva * inva;
        double bb = b * b;
        double bover3a = b * (1.0 / 3.0) * inva;  // b / (3a)
        // 化为标准形式 x^3 + px + q = 0
        double p = (3.0 * a * c - bb) * (1.0 / 3.0) * invaa;
        double halfq =
            (2.0 * bb * b - 9.0 * a * b * c + 27.0 * a * a * d) * (0.5 / 27.0) * invaa * inva;
        // 判别式：Δ = (q/2)^2 + (p/3)^3
        double yy = p * p * p / 27.0 + halfq * halfq;

        if (yy > DBL_EPSILON) {
            // 判别式为正：一个实根，两个共轭复根
            double y = sqrt(yy);
            double uuu = -halfq + y;
            double vvv = -halfq - y;
            double www = fabs(uuu) > fabs(vvv) ? uuu : vvv;
            // 三次根
            double w = (www < 0) ? -pow(fabs(www), 1.0 / 3.0) : pow(www, 1.0 / 3.0);
            roots.insert(w - p / (3.0 * w) - bover3a);
        } else if (yy < -DBL_EPSILON) {
            // 判别式为负：三个不同的实根（使用三角法求解）
            double x = -halfq;
            double y = sqrt(-yy);
            double theta;
            double r;
            double ux;
            double uyi;
            // 转换为极坐标形式
            if (fabs(x) > DBL_EPSILON) {
                theta = (x > 0.0) ? atan(y / x) : (atan(y / x) + M_PI);
                r = sqrt(x * x - yy);
            } else {
                // 垂直线情况
                theta = M_PI / 2.0;
                r = y;
            }
            // 计算三次根
            theta /= 3.0;
            r = pow(r, 1.0 / 3.0);
            // 转换为复数坐标
            ux = cos(theta) * r;
            uyi = sin(theta) * r;
            // 第一个解
            roots.insert(ux + ux - bover3a);
            // 第二个解：旋转 +120 度
            roots.insert(2.0 * (ux * cos120 - uyi * sin120) - bover3a);
            // 第三个解：旋转 -120 度
            roots.insert(2.0 * (ux * cos120 + uyi * sin120) - bover3a);
        } else {
            // 判别式为零：两个实根（其中一个是重根）
            double www = -halfq;
            double w = (www < 0.0) ? -pow(fabs(www), 1.0 / 3.0) : pow(www, 1.0 / 3.0);
            // 第一个解（单根）
            roots.insert(w + w - bover3a);
            // 第二个解（二重根），旋转 +120 度
            roots.insert(2.0 * w * cos120 - bover3a);
        }
    }
    return roots;
}

/**
 * @brief 求解四次方程对应的预解方程（三次方程）
 * @param x 输出数组，存储求解结果（长度必须为 3）
 * @param a 二次项系数
 * @param b 一次项系数
 * @param c 常数项
 * @return 实根的数量
 *
 * 预解方程形式：y^3 + a*y^2 + b*y + c = 0
 * 用于 Ferrari 方法求解四次方程
 */
inline int solveResolvent(double* x, double a, double b, double c) {
    double a2 = a * a;
    double q = (a2 - 3.0 * b) / 9.0;
    double r = (a * (2.0 * a2 - 9.0 * b) + 27.0 * c) / 54.0;
    double r2 = r * r;
    double q3 = q * q * q;
    double A, B;
    // 判别式 r^2 < q^3：三个实根
    if (r2 < q3) {
        double t = r / sqrt(q3);
        // 将 t 限制在 [-1, 1] 范围内（处理数值误差）
        if (t < -1.0) {
            t = -1.0;
        }
        if (t > 1.0) {
            t = 1.0;
        }
        t = acos(t);
        a /= 3.0;
        q = -2.0 * sqrt(q);
        // 三个实根（使用三角恒等式）
        x[0] = q * cos(t / 3.0) - a;
        x[1] = q * cos((t + M_PI * 2.0) / 3.0) - a;
        x[2] = q * cos((t - M_PI * 2.0) / 3.0) - a;
        return 3;
    } else {
        // 判别式 r^2 >= q^3：一个或两个实根
        A = -pow(fabs(r) + sqrt(r2 - q3), 1.0 / 3.0);
        if (r < 0.0) {
            A = -A;
        }
        B = (0.0 == A ? 0.0 : q / A);

        a /= 3.0;
        x[0] = (A + B) - a;                // 第一个实根
        x[1] = -0.5 * (A + B) - a;         // 另一个可能的实根（实部）
        x[2] = 0.5 * sqrt(3.0) * (A - B);  // 虚部
        // 如果虚部接近 0，说明有两个实根
        if (fabs(x[2]) < DBL_EPSILON) {
            x[2] = x[1];
            return 2;
        }

        return 1;  // 只有一个实根
    }
}

/**
 * @brief 求解首一四次方程的所有实根
 * @param a 三次项系数
 * @param b 二次项系数
 * @param c 一次项系数
 * @param d 常数项
 * @return 包含所有实根的集合
 *
 * 方程形式：x^4 + a*x^3 + b*x^2 + c*x + d = 0
 * 使用 Ferrari 方法求解
 */
inline std::set<double> solveQuartMonic(double a, double b, double c, double d) {
    std::set<double> roots;

    // 预解方程的系数
    double a3 = -b;
    double b3 = a * c - 4.0 * d;
    double c3 = -a * a * d - c * c + 4.0 * b * d;

    // 求解预解方程：y^3 - b*y^2 + (ac - 4*d)*y - a^2*d - c^2 + 4*b*d = 0
    double x3[3];
    int iZeroes = solveResolvent(x3, a3, b3, c3);

    double q1, q2, p1, p2, D, sqrtD, y;

    y = x3[0];
    // 选择绝对值最大的 y 值（提高数值稳定性）
    if (iZeroes != 1) {
        if (fabs(x3[1]) > fabs(y)) {
            y = x3[1];
        }
        if (fabs(x3[2]) > fabs(y)) {
            y = x3[2];
        }
    }

    // Ferrari 分解：h1 + h2 = y 且 h1*h2 = d  <==>  h^2 - y*h + d = 0 (其中 h 对应 q)

    D = y * y - 4.0 * d;
    if (fabs(D) < DBL_EPSILON)  // D == 0
    {
        q1 = q2 = y * 0.5;
        // g1 + g2 = a 且 g1 + g2 = b - y  <==>  g^2 - a*g + b - y = 0 (其中 p 对应 g)
        D = a * a - 4.0 * (b - y);
        if (fabs(D) < DBL_EPSILON)  // D == 0
        {
            p1 = p2 = a * 0.5;
        } else {
            sqrtD = sqrt(D);
            p1 = (a + sqrtD) * 0.5;
            p2 = (a - sqrtD) * 0.5;
        }
    } else {
        sqrtD = sqrt(D);
        q1 = (y + sqrtD) * 0.5;
        q2 = (y - sqrtD) * 0.5;
        // g1 + g2 = a 且 g1*h2 + g2*h1 = c (其中 g 对应 p) 使用 Cramer 法则
        p1 = (a * q1 - c) / (q1 - q2);
        p2 = (c - a * q2) / (q1 - q2);
    }

    // 求解第一个二次方程：x^2 + p1*x + q1 = 0
    D = p1 * p1 - 4.0 * q1;
    if (fabs(D) < DBL_EPSILON) {
        // 判别式为 0：一个二重根
        roots.insert(-p1 * 0.5);
    } else if (D > 0.0) {
        // 判别式为正：两个不同的实根
        sqrtD = sqrt(D);
        roots.insert((-p1 + sqrtD) * 0.5);
        roots.insert((-p1 - sqrtD) * 0.5);
    }

    // 求解第二个二次方程：x^2 + p2*x + q2 = 0
    D = p2 * p2 - 4.0 * q2;
    if (fabs(D) < DBL_EPSILON) {
        // 判别式为 0：一个二重根
        roots.insert(-p2 * 0.5);
    } else if (D > 0.0) {
        // 判别式为正：两个不同的实根
        sqrtD = sqrt(D);
        roots.insert((-p2 + sqrtD) * 0.5);
        roots.insert((-p2 - sqrtD) * 0.5);
    }

    return roots;
}

/**
 * @brief 求解一般四次方程的所有实根
 * @param a 四次项系数
 * @param b 三次项系数
 * @param c 二次项系数
 * @param d 一次项系数
 * @param e 常数项
 * @return 包含所有实根的集合
 *
 * 方程形式：a*x^4 + b*x^3 + c*x^2 + d*x + e = 0
 * 注意：所有系数都可以为零
 */
inline std::set<double> solveQuart(double a, double b, double c, double d, double e) {
    if (fabs(a) < DBL_EPSILON) {
        // 四次项系数为 0，降阶为三次方程
        return solveCub(b, c, d, e);
    } else {
        // 化为首一四次方程求解
        return solveQuartMonic(b / a, c / a, d / a, e / a);
    }
}

/**
 * @brief 通过特征值方法求解多项式在指定区间内的实根
 * @param coeffs 多项式系数向量
 * @param lbound 区间下界
 * @param ubound 区间上界
 * @param tol 容差（虚部小于此值的复根被视为实根）
 * @return 区间 (lbound, ubound) 内的实根集合
 *
 * 方法：计算多项式伴随矩阵的特征值
 * 虚部模小于 tol 的复根被认为是实根
 */
inline std::set<double> eigenSolveRealRoots(const Eigen::VectorXd& coeffs, double lbound,
                                            double ubound, double tol) {
    std::set<double> rts;

    int order = (int)coeffs.size() - 1;
    // 构造首一多项式系数
    Eigen::VectorXd monicCoeffs(order + 1);
    monicCoeffs << 1.0, coeffs.tail(order) / coeffs(0);

    // 构造伴随矩阵（Companion Matrix）
    Eigen::MatrixXd companionMat(order, order);
    companionMat.setZero();
    companionMat(0, order - 1) = -monicCoeffs(order);
    for (int i = 1; i < order; i++) {
        companionMat(i, i - 1) = 1.0;
        companionMat(i, order - 1) = -monicCoeffs(order - i);
    }

    // 计算伴随矩阵的特征值（即多项式的根）
    Eigen::VectorXcd eivals = companionMat.eigenvalues();
    double real;
    int eivalsNum = eivals.size();
    for (int i = 0; i < eivalsNum; i++) {
        real = eivals(i).real();
        // 筛选虚部足够小且在指定区间内的根
        if (eivals(i).imag() < tol && real > lbound && real < ubound)
            rts.insert(real);
    }

    return rts;
}

/**
 * @brief 计算 Sturm 序列在点 x 处的符号变化数
 * @param x 求值点
 * @param sturmSeqs Sturm 序列数组的指针数组
 * @param szSeq 每个 Sturm 序列的大小数组
 * @param len Sturm 序列的数量
 * @return 符号变化的次数
 *
 * 第 i 个序列的大小为 szSeq[i]，存储在 sturmSeqs[i][] 中，0 <= i < len
 * Sturm 定理：区间端点的符号变化数之差等于区间内实根的个数
 */
inline double numSignVar(double x, double** sturmSeqs, int* szSeq, int len) {
    double y, lasty;
    int signVar = 0;
    lasty = polyEval(sturmSeqs[0], szSeq[0], x);
    for (int i = 1; i < len; i++) {
        y = polyEval(sturmSeqs[i], szSeq[i], x);
        // 如果相邻两项符号不同，或某项为 0，符号变化数加 1
        if (lasty == 0.0 || lasty * y < 0.0) {
            ++signVar;
        }
        lasty = y;
    }

    return signVar;
};

/**
 * @brief 计算多项式的导数系数
 * @param coeffs 原多项式系数数组
 * @param dcoeffs 导数多项式系数数组（输出）
 * @param len 原多项式系数数组的长度
 */
inline void polyDeri(double* coeffs, double* dcoeffs, int len) {
    int horder = len - 1;  // 多项式的阶数
    for (int i = 0; i < horder; i++) {
        // 对 x^(horder-i) 求导得到 (horder-i) * x^(horder-i-1)
        dcoeffs[i] = (horder - i) * coeffs[i];
    }
    return;
}

/**
 * @brief 安全牛顿法求根
 * @tparam F 函数类型
 * @tparam DF 导数函数类型
 * @param func 待求根的函数
 * @param dfunc 函数的导数
 * @param l 区间下界
 * @param h 区间上界
 * @param tol 容差
 * @param maxIts 最大迭代次数
 * @return 区间内的根
 *
 * 要求：f(l) * f(h) <= 0（区间两端函数值异号）
 * 结合了牛顿法和二分法，保证收敛性
 */
template <typename F, typename DF>
inline double safeNewton(const F& func, const DF& dfunc, const double& l, const double& h,
                         const double& tol, const int& maxIts) {
    double xh, xl;
    double fl = func(l);
    double fh = func(h);
    // 边界检查：如果边界点恰好是根，直接返回
    if (fl == 0.0) {
        return l;
    }
    if (fh == 0.0) {
        return h;
    }
    // 确定 xl 和 xh，使得 f(xl) < 0, f(xh) > 0
    if (fl < 0.0) {
        xl = l;
        xh = h;
    } else {
        xh = l;
        xl = h;
    }

    double rts = 0.5 * (xl + xh);  // 初始猜测值为区间中点
    double dxold = fabs(xh - xl);  // 上一次的步长
    double dx = dxold;
    double f = func(rts);
    double df = dfunc(rts);
    double temp;
    for (int j = 0; j < maxIts; j++) {
        // 如果牛顿法会超出边界或收敛太慢，则使用二分法
        if ((((rts - xh) * df - f) * ((rts - xl) * df - f) > 0.0) ||
            (fabs(2.0 * f) > fabs(dxold * df))) {
            dxold = dx;
            dx = 0.5 * (xh - xl);  // 二分法
            rts = xl + dx;
            if (xl == rts) {
                break;  // 达到机器精度，停止迭代
            }
        } else {
            // 使用牛顿法
            dxold = dx;
            dx = f / df;
            temp = rts;
            rts -= dx;  // 牛顿迭代：x_{n+1} = x_n - f(x_n)/f'(x_n)
            if (temp == rts) {
                break;  // 达到机器精度，停止迭代
            }
        }

        // 如果步长足够小，收敛
        if (fabs(dx) < tol) {
            break;
        }

        // 更新函数值和导数值
        f = func(rts);
        df = dfunc(rts);
        // 缩小区间
        if (f < 0.0) {
            xl = rts;
        } else {
            xh = rts;
        }
    }

    return rts;
}

/**
 * @brief 收缩区间以求解多项式在指定区间内的单个零点
 * @param coeffs 多项式系数数组
 * @param numCoeffs 系数数组的长度
 * @param lbound 区间下界
 * @param ubound 区间上界
 * @param tol 容差
 * @return 区间 [lbound, ubound] 内的零点
 *
 * 要求：coeffs(lbound) * coeffs(ubound) < 0（区间两端函数值异号）
 *       lbound < ubound
 */
inline double shrinkInterval(double* coeffs, int numCoeffs, double lbound, double ubound,
                             double tol) {
    // 计算多项式的导数系数
    double* dcoeffs = new double[numCoeffs - 1];
    polyDeri(coeffs, dcoeffs, numCoeffs);
    // 定义函数和导数的 lambda 表达式
    auto func = [&coeffs, &numCoeffs](double x) { return polyEval(coeffs, numCoeffs, x); };
    auto dfunc = [&dcoeffs, &numCoeffs](double x) { return polyEval(dcoeffs, numCoeffs - 1, x); };
    constexpr int maxDblIts = 128;  // 最大迭代次数
    // 使用安全牛顿法求根
    double rts = safeNewton(func, dfunc, lbound, ubound, tol, maxDblIts);
    delete[] dcoeffs;
    return rts;
}

/**
 * @brief 递归隔离区间 (l, r) 内的所有实根
 * @param l 区间左端点
 * @param r 区间右端点
 * @param fl 左端点处多项式的值 fl := sturmSeqs[0](l)
 * @param fr 右端点处多项式的值 fr := sturmSeqs[0](r)
 * @param lnv 左端点处的符号变化数
 * @param rnv 右端点处的符号变化数
 * @param tol 容差
 * @param sturmSeqs Sturm 序列数组
 * @param szSeq 每个 Sturm 序列的大小
 * @param len Sturm 序列的数量
 * @param rts 存储根的集合（输出）
 *
 * 要求：fl != 0, fr != 0, l < r, lnv != rnv
 *       lnv = numSignVar(l), rnv = numSignVar(r)
 *       sturmSeqs[0](x) 在 (l, r) 内至少有一个根
 */
inline void recurIsolate(double l, double r, double fl, double fr, int lnv, int rnv, double tol,
                         double** sturmSeqs, int* szSeq, int len, std::set<double>& rts) {
    int nrts = lnv - rnv;  // 根据 Sturm 定理，区间内根的个数
    double fm;
    double m;

    if (nrts == 0) {
        // 无根，直接返回
        return;
    } else if (nrts == 1) {
        // 只有一个根
        if (fl * fr < 0) {
            // 端点函数值异号，直接收缩区间求根
            rts.insert(shrinkInterval(sturmSeqs[0], szSeq[0], l, r, tol));
            return;
        } else {
            // 端点函数值同号（可能是重根），使用二分法
            int maxDblIts = 128;  // 最大迭代次数

            for (int i = 0; i < maxDblIts; i++) {
                // 计算偶数重根：使用 Sturm 序列的第二项（导数）
                if (fl * fr < 0) {
                    rts.insert(shrinkInterval(sturmSeqs[1], szSeq[1], l, r, tol));
                    return;
                }

                m = (l + r) / 2.0;  // 取区间中点
                fm = polyEval(sturmSeqs[0], szSeq[0], m);

                // 如果中点处多项式值为 0 或区间足够小，找到根
                if (fm == 0 || fabs(r - l) < tol) {
                    rts.insert(m);
                    return;
                } else {
                    // 判断根在哪一侧，缩小区间
                    if (lnv == numSignVar(m, sturmSeqs, szSeq, len)) {
                        l = m;
                        fl = fm;
                    } else {
                        r = m;
                        fr = fm;
                    }
                }
            }

            rts.insert(m);
            return;
        }
    } else if (nrts > 1) {
        // 区间内存在多个根
        int maxDblIts = 128;  // 最大迭代次数

        int mnv;
        int bias = 0;         // 偏移量，用于处理中点恰好为根的情况
        bool biased = false;  // 是否需要偏移标志
        for (int i = 0; i < maxDblIts; i++) {
            bias = biased ? bias : 0;
            if (!biased) {
                // 正常情况：取区间中点
                m = (l + r) / 2.0;
            } else {
                // 中点恰好是根的情况：略微偏移以避开根
                m = (r - l) / pow(2.0, bias + 1.0) + l;
                biased = false;
            }
            mnv = numSignVar(m, sturmSeqs, szSeq, len);

            // 如果区间足够小，直接返回中点作为根
            if (fabs(r - l) < tol) {
                rts.insert(m);
                return;
            } else {
                fm = polyEval(sturmSeqs[0], szSeq[0], m);
                if (fm == 0) {
                    // 中点恰好是根，需要偏移以分隔其他根
                    bias++;
                    biased = true;
                } else if (lnv != mnv && rnv != mnv) {
                    // 左右两侧都有根，递归处理两个子区间
                    recurIsolate(l, m, fl, fm, lnv, mnv, tol, sturmSeqs, szSeq, len, rts);
                    recurIsolate(m, r, fm, fr, mnv, rnv, tol, sturmSeqs, szSeq, len, rts);
                    return;
                } else if (lnv == mnv) {
                    // 根在右侧，缩小到右半区间
                    l = m;
                    fl = fm;
                } else {
                    // 根在左侧，缩小到左半区间
                    r = m;
                    fr = fm;
                }
            }
        }

        rts.insert(m);
        return;
    }
};

/**
 * @brief 使用 Sturm 定理隔离并计算多项式在指定区间内的所有实根
 * @param coeffs 多项式系数向量
 * @param lbound 搜索区间下界
 * @param ubound 搜索区间上界
 * @param tol 容差
 * @return 区间 (lbound, ubound) 内的所有实根集合
 *
 * 要求：首项系数非零
 *       coeffs(lbound) != 0, coeffs(ubound) != 0, lbound < ubound
 *
 * 算法步骤：
 * 1. 计算多项式根的上界（Cauchy 界和 Kojima 界）
 * 2. 构造 Sturm 序列
 * 3. 使用递归方法隔离所有根
 */
inline std::set<double> isolateRealRoots(const Eigen::VectorXd& coeffs, double lbound,
                                         double ubound, double tol) {
    std::set<double> rts;

    // 计算首一多项式系数（最高次项系数归一化为 1）
    int order = (int)coeffs.size() - 1;
    Eigen::VectorXd monicCoeffs(order + 1);
    monicCoeffs << 1.0, coeffs.tail(order) / coeffs(0);

    // 计算 Cauchy 上界：多项式所有根的模的上界
    // Cauchy 界：rho_c = 1 + max|a_i|，其中 a_i 为首一多项式的非最高次项系数
    double rho_c = 1 + monicCoeffs.tail(order).cwiseAbs().maxCoeff();

    // 计算 Kojima 上界：通常比 Cauchy 界更紧
    // Kojima 界基于相邻非零系数的比值
    Eigen::VectorXd nonzeroCoeffs(order + 1);
    nonzeroCoeffs.setZero();
    int nonzeros = 0;
    double tempEle;
    // 收集所有非零系数
    for (int i = 0; i < order + 1; i++) {
        tempEle = monicCoeffs(i);
        if (fabs(tempEle) >= DBL_EPSILON) {
            nonzeroCoeffs(nonzeros++) = tempEle;
        }
    }
    nonzeroCoeffs = nonzeroCoeffs.head(nonzeros).eval();
    // 计算相邻非零系数的比值的绝对值
    Eigen::VectorXd kojimaVec =
        nonzeroCoeffs.tail(nonzeros - 1).cwiseQuotient(nonzeroCoeffs.head(nonzeros - 1)).cwiseAbs();
    kojimaVec.tail(1) /= 2.0;  // 最后一项特殊处理
    double rho_k = 2.0 * kojimaVec.maxCoeff();

    // 选择更紧的界，然后放宽 1.0 以得到开区间
    double rho = std::min(rho_c, rho_k) + 1.0;

    // 收紧搜索界限：确保在根的上界范围内
    lbound = std::max(lbound, -rho);
    ubound = std::min(ubound, rho);

    // 构造 Sturm 序列
    int len = monicCoeffs.size();
    double sturmSeqs[(RootFinderParam::highestOrder + 1) * (RootFinderParam::highestOrder + 1)];
    int szSeq[RootFinderParam::highestOrder + 1] = {
        0};  // 显式初始化为零（gcc 在 -O3 下可能忽略此初始化）
    double* offsetSeq[RootFinderParam::highestOrder + 1];
    int num = 0;

    // 第一个序列是原多项式，第二个是其导数
    for (int i = 0; i < len; i++) {
        sturmSeqs[i] = monicCoeffs(i);
        sturmSeqs[i + 1 + len] = (order - i) * sturmSeqs[i] / order;
    }
    szSeq[0] = len;
    szSeq[1] = len - 1;
    offsetSeq[0] = sturmSeqs + len - szSeq[0];
    offsetSeq[1] = sturmSeqs + 2 * len - szSeq[1];

    num += 2;

    // 继续构造 Sturm 序列：每一项是前两项相除的负余数
    bool remainderConstant = false;
    int idx = 0;
    while (!remainderConstant) {
        // 计算多项式除法的余数
        szSeq[idx + 2] =
            polyMod(offsetSeq[idx], offsetSeq[idx + 1], &(sturmSeqs[(idx + 3) * len - szSeq[idx]]),
                    szSeq[idx], szSeq[idx + 1]);
        offsetSeq[idx + 2] = sturmSeqs + (idx + 3) * len - szSeq[idx + 2];

        // 如果余数是常数，序列构造完成
        remainderConstant = szSeq[idx + 2] == 1;
        // 归一化余数：除以首项系数的绝对值，并取负
        for (int i = 1; i < szSeq[idx + 2]; i++) {
            offsetSeq[idx + 2][i] /= -fabs(offsetSeq[idx + 2][0]);
        }
        offsetSeq[idx + 2][0] = offsetSeq[idx + 2][0] > 0.0 ? -1.0 : 1.0;
        num++;
        idx++;
    }

    // 递归隔离开区间内的所有不同根
    recurIsolate(lbound, ubound, polyEval(offsetSeq[0], szSeq[0], lbound),
                 polyEval(offsetSeq[0], szSeq[0], ubound),
                 numSignVar(lbound, offsetSeq, szSeq, len),
                 numSignVar(ubound, offsetSeq, szSeq, len), tol, offsetSeq, szSeq, len, rts);

    return rts;
}

}  // namespace RootFinderPriv

// RootFinder 命名空间：多项式求根器的公共接口
namespace RootFinder {

/**
 * @brief 计算两个多项式的卷积（即多项式乘法）
 * @param lCoef 左多项式系数向量
 * @param rCoef 右多项式系数向量
 * @return 卷积结果多项式的系数向量
 *
 * 功能：计算 lCoef(x) * rCoef(x) 的结果
 * 例如：(a0*x^2 + a1*x + a2) * (b0*x + b1) = 结果多项式
 * 结果多项式的阶数 = lCoef 的阶数 + rCoef 的阶数
 */
inline Eigen::VectorXd polyConv(const Eigen::VectorXd& lCoef, const Eigen::VectorXd& rCoef) {
    // 结果多项式的长度 = 两个多项式长度之和 - 1
    Eigen::VectorXd result(lCoef.size() + rCoef.size() - 1);
    result.setZero();
    // 卷积计算：result[i] = sum(lCoef[j] * rCoef[i-j])
    for (int i = 0; i < result.size(); i++) {
        for (int j = 0; j <= i; j++) {
            result(i) +=
                (j < lCoef.size() && (i - j) < rCoef.size()) ? (lCoef(j) * rCoef(i - j)) : 0;
        }
    }

    return result;
}

// // This function needs FFTW 3 and only performs better when the scale is large
// inline Eigen::VectorXd polyConvFFT(const Eigen::VectorXd &lCoef, const Eigen::VectorXd &rCoef)
// // Calculate the convolution of lCoef(x) and rCoef(x) using FFT
// // This function is fast when orders of both poly are larger than 100
// {
//     int paddedLen = lCoef.size() + rCoef.size() - 1;
//     int complexLen = paddedLen / 2 + 1;
//     Eigen::VectorXd result(paddedLen);
//     double *rBuffer = fftw_alloc_real(paddedLen);
//     // Construct FFT plan and buffers
//     fftw_complex *cForwardBuffer = fftw_alloc_complex(complexLen);
//     fftw_complex *cBackwardBuffer = fftw_alloc_complex(complexLen);
//     fftw_plan forwardPlan = fftw_plan_dft_r2c_1d(paddedLen, rBuffer, cForwardBuffer,
//                                                  FFTW_ESTIMATE | FFTW_DESTROY_INPUT);
//     fftw_plan backwardPlan = fftw_plan_dft_c2r_1d(paddedLen, cBackwardBuffer, rBuffer,
//                                                   FFTW_ESTIMATE | FFTW_DESTROY_INPUT);
//     // Pad lCoef by zeros
//     int len = lCoef.size();
//     for (int i = 0; i < len; i++)
//     {
//         rBuffer[i] = lCoef(i);
//     }
//     for (int i = len; i < paddedLen; i++)
//     {
//         rBuffer[i] = 0.0;
//     }
//     // Compute fft(pad(lCoef(x)) and back it up
//     fftw_execute(forwardPlan);
//     memcpy(cBackwardBuffer, cForwardBuffer, sizeof(fftw_complex) * complexLen);
//     // Pad rCoef by zeros
//     len = rCoef.size();
//     for (int i = 0; i < len; i++)
//     {
//         rBuffer[i] = rCoef(i);
//     }
//     for (int i = len; i < paddedLen; i++)
//     {
//         rBuffer[i] = 0.0;
//     }
//     // Compute fft(pad(rCoef(x))
//     fftw_execute(forwardPlan);
//     // Compute fft(pad(lCoef(x)).fft(pad(rCoef(x))
//     double real, imag;
//     for (int i = 0; i < complexLen; i++)
//     {
//         real = cBackwardBuffer[i][0];
//         imag = cBackwardBuffer[i][1];
//         cBackwardBuffer[i][0] = real * cForwardBuffer[i][0] -
//                                 imag * cForwardBuffer[i][1];
//         cBackwardBuffer[i][1] = imag * cForwardBuffer[i][0] +
//                                 real * cForwardBuffer[i][1];
//     }
//     // Compute ifft(fft(pad(lCoef(x)).fft(pad(rCoef(x)))
//     fftw_execute(backwardPlan);
//     // Recover the original intensity
//     double intensity = 1.0 / paddedLen;
//     for (int i = 0; i < paddedLen; i++)
//     {
//         result(i) = rBuffer[i] * intensity;
//     }
//     // Destruct FFT plan and buffers
//     fftw_destroy_plan(forwardPlan);
//     fftw_destroy_plan(backwardPlan);
//     fftw_free(rBuffer);
//     fftw_free(cForwardBuffer);
//     fftw_free(cBackwardBuffer);
//     return result;
// }

/**
 * @brief 计算多项式的自卷积（即多项式平方）
 * @param coef 多项式系数向量
 * @return 自卷积结果多项式的系数向量
 *
 * 功能：计算 coef(x) * coef(x) = coef(x)^2
 * 相比于普通卷积，自卷积可以利用对称性进行优化，减少计算量
 * 算法利用了对称性：coef(j) * coef(i-j) 出现两次（当 j != i-j 时）
 */
inline Eigen::VectorXd polySqr(const Eigen::VectorXd& coef) {
    int coefSize = coef.size();
    int resultSize = coefSize * 2 - 1;  // 结果多项式的长度
    int lbound, rbound;
    Eigen::VectorXd result(resultSize);
    double temp;
    for (int i = 0; i < resultSize; i++) {
        temp = 0;
        // 确定求和范围
        lbound = i - coefSize + 1;
        lbound = lbound > 0 ? lbound : 0;
        rbound = coefSize < (i + 1) ? coefSize : (i + 1);
        rbound += lbound;
        // 利用对称性优化计算
        if (rbound & 1)  // 比 rbound % 2 == 1 更快的位运算判断奇偶
        {
            rbound >>= 1;  // 比 rbound /= 2 更快的位运算除以2
            // 中间项只出现一次
            temp += coef(rbound) * coef(rbound);
        } else {
            rbound >>= 1;  // 比 rbound /= 2 更快的位运算除以2
        }

        // 其他项都出现两次（利用对称性）
        for (int j = lbound; j < rbound; j++) {
            temp += 2.0 * coef(j) * coef(i - j);
        }
        result(i) = temp;
    }

    return result;
}

/**
 * @brief 计算多项式在点 x 处的值
 * @param coeffs 多项式系数向量
 * @param x 求值点
 * @param numericalStability 是否使用数值稳定的方法（默认为 true）
 * @return 多项式 coeffs(x) 在点 x 处的值
 *
 * 方法：
 * - numericalStability = true: 使用直接幂次计算（数值稳定但较慢）
 * - numericalStability = false: 使用 Horner 方法（快速但数值稳定性较差）
 *
 * 注意：Horner 方法虽然效率高，但在零点附近会有灾难性抵消误差
 *       当 coeffs(x) 接近 0 时应使用数值稳定的方法
 */
inline double polyVal(const Eigen::VectorXd& coeffs, double x, bool numericalStability = true) {
    double retVal = 0.0;
    int order = (int)coeffs.size() - 1;

    if (order >= 0) {
        if (fabs(x) < DBL_EPSILON) {
            // x ≈ 0 时，多项式值就是常数项
            retVal = coeffs(order);
        } else if (x == 1.0) {
            // x = 1 时，多项式值就是所有系数之和
            retVal = coeffs.sum();
        } else {
            if (numericalStability) {
                // 数值稳定方法：直接计算每一项的幂次再求和
                // 避免了灾难性抵消，但计算量较大
                double xn = 1.0;

                for (int i = order; i >= 0; i--) {
                    retVal += coeffs(i) * xn;
                    xn *= x;  // 累乘计算 x 的幂次
                }
            } else {
                // Horner 方法：p(x) = (...((a0*x + a1)*x + a2)*x + ... + an)
                // 计算效率高但数值稳定性差
                int len = coeffs.size();

                for (int i = 0; i < len; i++) {
                    retVal = retVal * x + coeffs(i);
                }
            }
        }
    }

    return retVal;
}

/**
 * @brief 计算多项式在指定区间内的不同实根个数
 * @param coeffs 多项式系数向量
 * @param l 区间左端点
 * @param r 区间右端点
 * @return 区间 (l, r) 内不同实根的个数
 *
 * 使用 Sturm 定理计算根的数量
 * 要求：边界值 coeffs(l) 和 coeffs(r) 必须非零
 *
 * Sturm 定理：区间 (l, r) 内的不同实根个数 = 左端点的符号变化数 - 右端点的符号变化数
 */
inline int countRoots(const Eigen::VectorXd& coeffs, double l, double r) {
    int nRoots = 0;

    // 去除前导零系数，确定有效系数个数
    int originalSize = coeffs.size();
    int valid = originalSize;
    for (int i = 0; i < originalSize; i++) {
        if (fabs(coeffs(i)) < DBL_EPSILON) {
            valid--;
        } else {
            break;
        }
    }

    // 检查常数项是否为零
    if (valid > 0 && fabs(coeffs(originalSize - 1)) > DBL_EPSILON) {
        // 构造首一多项式（最高次项系数归一化为 1）
        Eigen::VectorXd monicCoeffs(valid);
        monicCoeffs << 1.0,
            coeffs.segment(originalSize - valid + 1, valid - 1) / coeffs(originalSize - valid);

        // 构造 Sturm 序列
        int len = monicCoeffs.size();
        int order = len - 1;
        double sturmSeqs[(RootFinderParam::highestOrder + 1) * (RootFinderParam::highestOrder + 1)];
        int szSeq[RootFinderParam::highestOrder + 1] = {
            0};  // 显式初始化为零（gcc 在 -O3 下可能忽略此初始化）
        int num = 0;

        // 第一个序列是原多项式，第二个是其导数
        for (int i = 0; i < len; i++) {
            sturmSeqs[i] = monicCoeffs(i);
            sturmSeqs[i + 1 + len] = (order - i) * sturmSeqs[i] / order;
        }
        szSeq[0] = len;
        szSeq[1] = len - 1;
        num += 2;

        // 继续构造 Sturm 序列
        bool remainderConstant = false;
        int idx = 0;
        while (!remainderConstant) {
            // 计算多项式除法的余数
            szSeq[idx + 2] = RootFinderPriv::polyMod(&(sturmSeqs[(idx + 1) * len - szSeq[idx]]),
                                                     &(sturmSeqs[(idx + 2) * len - szSeq[idx + 1]]),
                                                     &(sturmSeqs[(idx + 3) * len - szSeq[idx]]),
                                                     szSeq[idx], szSeq[idx + 1]);
            remainderConstant = szSeq[idx + 2] == 1;
            // 归一化余数
            for (int i = 1; i < szSeq[idx + 2]; i++) {
                sturmSeqs[(idx + 3) * len - szSeq[idx + 2] + i] /=
                    -fabs(sturmSeqs[(idx + 3) * len - szSeq[idx + 2]]);
            }
            sturmSeqs[(idx + 3) * len - szSeq[idx + 2]] /=
                -fabs(sturmSeqs[(idx + 3) * len - szSeq[idx + 2]]);
            num++;
            idx++;
        }

        // 计算两个边界处的符号变化数
        double yl, lastyl, yr, lastyr;
        lastyl = RootFinderPriv::polyEval(&(sturmSeqs[len - szSeq[0]]), szSeq[0], l);
        lastyr = RootFinderPriv::polyEval(&(sturmSeqs[len - szSeq[0]]), szSeq[0], r);
        for (int i = 1; i < num; i++) {
            yl = RootFinderPriv::polyEval(&(sturmSeqs[(i + 1) * len - szSeq[i]]), szSeq[i], l);
            yr = RootFinderPriv::polyEval(&(sturmSeqs[(i + 1) * len - szSeq[i]]), szSeq[i], r);
            // 统计符号变化：相邻项符号不同或为零时计数
            if (lastyl == 0.0 || lastyl * yl < 0.0) {
                ++nRoots;  // 左端点符号变化数增加
            }
            if (lastyr == 0.0 || lastyr * yr < 0.0) {
                --nRoots;  // 右端点符号变化数减少（因为要计算差值）
            }
            lastyl = yl;
            lastyr = yr;
        }
    }

    return nRoots;
}

/**
 * @brief 求解多项式在指定区间内的所有实根（主接口函数）
 * @param coeffs 多项式系数向量
 * @param lbound 搜索区间下界
 * @param ubound 搜索区间上界
 * @param tol 容差
 * @param isolation 是否使用隔离方法（默认为 true）
 * @return 区间 (lbound, ubound) 内的所有实根集合
 *
 * 算法策略：
 * 1. 对于 reduced_order < 5 的多项式，使用解析解（封闭形式解）
 * 2. 对于高阶多项式：
 *    - isolation = true: 使用 Sturm 定理和几何性质对每个根进行隔离
 *                        然后使用安全牛顿法高效收缩区间
 *    - isolation = false: 计算多项式伴随矩阵的特征值
 *
 * 要求：
 * - 首项系数必须非零
 * - coeffs(lbound) != 0, coeffs(ubound) != 0
 * - lbound < ubound
 */
inline std::set<double> solvePolynomial(const Eigen::VectorXd& coeffs, double lbound, double ubound,
                                        double tol, bool isolation = true) {
    std::set<double> rts;

    // 去除前导零系数（最高次项的零系数）
    int valid = coeffs.size();
    for (int i = 0; i < coeffs.size(); i++) {
        if (fabs(coeffs(i)) < DBL_EPSILON) {
            valid--;
        } else {
            break;
        }
    }

    // 去除尾部零系数（常数项及低次项的零系数）
    int offset = 0;  // 尾部零系数个数（对应 x=0 根的重数）
    int nonzeros = valid;
    if (valid > 0) {
        for (int i = 0; i < valid; i++) {
            if (fabs(coeffs(coeffs.size() - i - 1)) < DBL_EPSILON) {
                nonzeros--;
                offset++;
            } else {
                break;
            }
        }
    }

    // 特殊情况处理
    if (nonzeros == 0) {
        // 所有系数为零：任意实数都是根
        rts.insert(INFINITY);
        rts.insert(-INFINITY);
    } else if (nonzeros == 1 && offset == 0) {
        // 只有常数项非零：无根
        rts.clear();
    } else {
        // 构造去除前导零和尾部零的系数向量
        Eigen::VectorXd ncoeffs(std::max(5, nonzeros));
        ncoeffs.setZero();
        ncoeffs.tail(nonzeros) << coeffs.segment(coeffs.size() - valid, nonzeros);

        if (nonzeros <= 5) {
            // 低阶多项式（≤ 4 次）：使用解析解（三次、四次方程公式）
            rts = RootFinderPriv::solveQuart(ncoeffs(0), ncoeffs(1), ncoeffs(2), ncoeffs(3),
                                             ncoeffs(4));
        } else {
            // 高阶多项式：使用数值方法
            if (isolation) {
                // 隔离法：使用 Sturm 定理隔离根，然后用安全牛顿法求精确值
                rts = RootFinderPriv::isolateRealRoots(ncoeffs, lbound, ubound, tol);
            } else {
                // 特征值法：计算伴随矩阵的特征值
                rts = RootFinderPriv::eigenSolveRealRoots(ncoeffs, lbound, ubound, tol);
            }
        }

        // 如果有尾部零系数，说明 x=0 是根
        if (offset > 0) {
            rts.insert(0.0);
        }
    }

    // 过滤掉不在指定区间 (lbound, ubound) 内的根
    for (auto it = rts.begin(); it != rts.end();) {
        if (*it > lbound && *it < ubound) {
            it++;  // 保留在区间内的根
        } else {
            it = rts.erase(it);  // 删除区间外的根
        }
    }

    return rts;
}

}  // namespace RootFinder

#endif