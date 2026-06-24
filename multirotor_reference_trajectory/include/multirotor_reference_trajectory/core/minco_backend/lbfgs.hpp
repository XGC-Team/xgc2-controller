// L-BFGS优化算法头文件
// L-BFGS (Limited-memory Broyden-Fletcher-Goldfarb-Shanno) 是一种用于无约束优化的拟牛顿方法
// 该实现支持光滑和非光滑函数的优化，适用于大规模优化问题
#ifndef LBFGS_HPP
#define LBFGS_HPP

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>

// lbfgs命名空间：包含L-BFGS优化算法的所有相关定义和实现
namespace lbfgs {
// ----------------------- 数据类型部分 -----------------------

/**
 * L-BFGS优化参数结构体
 * 包含控制L-BFGS优化行为的所有参数
 */
struct lbfgs_parameter_t {
    /**
     * 用于近似逆Hessian矩阵的修正数量（即历史记忆大小）
     * L-BFGS算法会存储前m次迭代的计算结果来近似当前迭代的逆Hessian矩阵
     * 该参数控制有限内存（修正）的大小
     * 默认值为8，不建议使用小于3的值，过大的值会导致计算时间过长
     */
    int mem_size = 8;

    /**
     * 梯度收敛测试的epsilon值，注意：在非光滑情况下不要使用此参数！
     * 对于非光滑函数，应将其设置为0.0并使用基于past-delta的测试
     * 该参数决定了求解的精度。当满足以下条件时，最小化过程终止：
     *      ||g(x)||_inf / max(1, ||x||_inf) < g_epsilon
     * 其中||.||_inf是无穷范数
     * 默认值为1.0e-5
     * 实践中应大于1.0e-6，因为L-BFGS不直接减少一阶残差，
     * 当||g||很小时，函数值可能被machine_prec破坏
     */
    double g_epsilon = 1.0e-5;

    /**
     * 基于delta的收敛测试的距离（迭代次数间隔）
     * 该参数决定用于计算目标函数下降率的迭代距离
     * 如果该参数为0，则不执行基于delta的收敛测试
     * 默认值为3
     */
    int past = 3;

    /**
     * 收敛测试的delta值
     * 该参数决定目标函数的最小下降率
     * 当满足以下条件时，算法停止迭代：
     *      |f' - f| / max(1, |f|) < delta
     * 其中f'是past次迭代之前的目标函数值，f是当前迭代的目标函数值
     * 默认值为1.0e-6
     */
    double delta = 1.0e-6;

    /**
     * 最大迭代次数
     * 当迭代次数超过此参数时，lbfgs_optimize()函数将以
     * LBFGSERR_MAXIMUMITERATION状态码终止最小化过程
     * 将此参数设置为0会持续优化直到收敛或出错
     * 默认值为0（即不限制迭代次数）
     */
    int max_iterations = 0;

    /**
     * 线搜索的最大尝试次数
     * 该参数控制每次迭代中线搜索程序的函数和梯度评估次数
     * 默认值为64
     */
    int max_linesearch = 64;

    /**
     * 线搜索的最小步长
     * 默认值为1.0e-20
     * 除非指数对于所使用的机器过大，或者问题的尺度极其不合理
     * （在这种情况下应增加指数），否则无需修改此值
     */
    double min_step = 1.0e-20;

    /**
     * 线搜索的最大步长
     * 默认值为1.0e+20
     * 除非指数对于所使用的机器过大，或者问题的尺度极其不合理
     * （在这种情况下应增加指数），否则无需修改此值
     */
    double max_step = 1.0e+20;

    /**
     * 控制线搜索精度的参数（函数下降系数，用于Armijo条件）
     * 默认值为1.0e-4
     * 该参数应大于0且小于1.0
     */
    double f_dec_coeff = 1.0e-4;

    /**
     * 控制线搜索精度的参数（曲率系数，用于Wolfe条件）
     * 默认值为0.9
     * 如果相对于迭代成本，函数和梯度评估的代价较小
     * （在求解大规模问题时有时会出现这种情况），
     * 将此参数设置为较小的值可能会更有利，典型的小值是0.1
     * 该参数应大于f_dec_coeff参数且小于1.0
     */
    double s_curv_coeff = 0.9;

    /**
     * 用于确保非凸函数全局收敛的参数（谨慎更新因子）
     * 默认值为1.0e-6
     * 该参数执行L-BFGS的所谓"谨慎更新"，特别是当收敛性不足时
     * 该参数必须为正值，但在实践中可能小于1.0e-3
     */
    double cautious_factor = 1.0e-6;

    /**
     * 浮点值的机器精度
     * 默认值为1.0e-16
     * 该参数必须是由客户端程序设置的正值，用于估计机器精度
     */
    double machine_prec = 1.0e-16;
};

/**
 * lbfgs_optimize()函数的返回值枚举
 * 粗略地说，负值表示错误
 */
enum {
    /** L-BFGS达到收敛 */
    LBFGS_CONVERGENCE = 0,
    /** L-BFGS满足停止准则 */
    LBFGS_STOP,
    /** 迭代已被监控回调函数取消 */
    LBFGS_CANCELED,

    /** 未知错误 */
    LBFGSERR_UNKNOWNERROR = -1024,
    /** 指定的变量数量无效 */
    LBFGSERR_INVALID_N,
    /** 指定的参数lbfgs_parameter_t::mem_size无效 */
    LBFGSERR_INVALID_MEMSIZE,
    /** 指定的参数lbfgs_parameter_t::g_epsilon无效 */
    LBFGSERR_INVALID_GEPSILON,
    /** 指定的参数lbfgs_parameter_t::past无效 */
    LBFGSERR_INVALID_TESTPERIOD,
    /** 指定的参数lbfgs_parameter_t::delta无效 */
    LBFGSERR_INVALID_DELTA,
    /** 指定的参数lbfgs_parameter_t::min_step无效 */
    LBFGSERR_INVALID_MINSTEP,
    /** 指定的参数lbfgs_parameter_t::max_step无效 */
    LBFGSERR_INVALID_MAXSTEP,
    /** 指定的参数lbfgs_parameter_t::f_dec_coeff无效 */
    LBFGSERR_INVALID_FDECCOEFF,
    /** 指定的参数lbfgs_parameter_t::s_curv_coeff无效 */
    LBFGSERR_INVALID_SCURVCOEFF,
    /** 指定的参数lbfgs_parameter_t::machine_prec无效 */
    LBFGSERR_INVALID_MACHINEPREC,
    /** 指定的参数lbfgs_parameter_t::max_linesearch无效 */
    LBFGSERR_INVALID_MAXLINESEARCH,
    /** 函数值变为NaN或Inf */
    LBFGSERR_INVALID_FUNCVAL,
    /** 线搜索步长小于lbfgs_parameter_t::min_step */
    LBFGSERR_MINIMUMSTEP,
    /** 线搜索步长大于lbfgs_parameter_t::max_step */
    LBFGSERR_MAXIMUMSTEP,
    /** 线搜索达到最大次数，假设不满足或精度无法达到 */
    LBFGSERR_MAXIMUMLINESEARCH,
    /** 算法达到最大迭代次数 */
    LBFGSERR_MAXIMUMITERATION,
    /** 相对搜索区间宽度至少为lbfgs_parameter_t::machine_prec */
    LBFGSERR_WIDTHTOOSMALL,
    /** 发生逻辑错误（负线搜索步长） */
    LBFGSERR_INVALIDPARAMETERS,
    /** 当前搜索方向增加了目标函数值 */
    LBFGSERR_INCREASEGRADIENT,
};

/**
 * 提供目标函数和梯度评估的回调接口
 *
 * lbfgs_optimize()函数在需要时调用此函数来获取目标函数及其梯度的值
 * 客户端程序必须实现此函数，以在给定当前变量值的情况下评估目标函数及其梯度的值
 *
 * @param  instance    客户端发送给lbfgs_optimize()函数的用户数据
 * @param  x           当前变量值
 * @param  g           梯度向量。回调函数必须计算当前变量的梯度值
 * @retval double      当前变量的目标函数值
 */
typedef double (*lbfgs_evaluate_t)(void* instance, const Eigen::VectorXd& x, Eigen::VectorXd& g);

/**
 * 提供当前线搜索开始时步长上界的回调接口
 *
 * lbfgs_optimize()函数调用此函数以获取搜索步长的上界值
 * 输入包括线搜索前的变量初始值和当前步长向量（可以是下降方向）
 * 客户端程序可以实现此函数以实现更高效的线搜索
 * 不应考虑大于此上界的任何步长，例如它具有非常大甚至为inf的函数值
 * 注意：在提供的上界处的函数值应该是有限的！
 * 如果不使用此功能，只需将其设置为nullptr
 *
 * @param  instance    客户端发送给lbfgs_optimize()函数的用户数据
 * @param  xp          当前线搜索之前的变量值
 * @param  d           步长向量，可以是下降方向
 * @retval double      当前线搜索中步长的上界，使得(stpbound * d)是最大合理步长
 */
typedef double (*lbfgs_stepbound_t)(void* instance, const Eigen::VectorXd& xp,
                                    const Eigen::VectorXd& d);

/**
 * 监控最小化过程进度的回调接口
 *
 * lbfgs_optimize()函数在每次迭代时调用此函数
 * 通过实现此函数，客户端程序可以存储或显示最小化过程的当前进度
 * 如果不使用此功能，只需将其设置为nullptr
 *
 * @param  instance    客户端发送给lbfgs_optimize()函数的用户数据
 * @param  x           当前变量值
 * @param  g           当前变量的梯度值
 * @param  fx          当前目标函数值
 * @param  step        此次迭代使用的线搜索步长
 * @param  k           迭代计数
 * @param  ls          此次迭代调用的评估次数
 * @retval int         返回0继续最小化过程，返回非零值将取消最小化过程
 */
typedef int (*lbfgs_progress_t)(void* instance, const Eigen::VectorXd& x, const Eigen::VectorXd& g,
                                const double fx, const double step, const int k, const int ls);

/**
 * 回调数据结构体
 * 用于封装所有回调函数指针和用户数据
 */
struct callback_data_t {
    void* instance = nullptr;                    // 用户数据指针
    lbfgs_evaluate_t proc_evaluate = nullptr;    // 目标函数和梯度评估回调
    lbfgs_stepbound_t proc_stepbound = nullptr;  // 步长上界回调
    lbfgs_progress_t proc_progress = nullptr;    // 进度监控回调
};

// ----------------------- L-BFGS算法实现部分 -----------------------

/**
 * 适用于光滑或非光滑函数的线搜索方法（Lewis-Overton线搜索）
 * 该函数执行线搜索以找到同时满足Armijo条件和弱Wolfe条件的点
 * 它与回溯线搜索一样鲁棒，但进一步适用于连续和分段光滑函数，
 * 在这些函数中强Wolfe条件通常不成立
 *
 * @see Adrian S. Lewis and Michael L. Overton. Nonsmooth optimization
 *      via quasi-Newton methods. Mathematical Programming, Vol 141,
 *      No 1, pp. 135-163, 2013.
 *
 * @param  x        输出：线搜索后的新变量值
 * @param  f        输出：新点处的目标函数值
 * @param  g        输出：新点处的梯度值
 * @param  stp      输入/输出：步长，输入初始步长，输出最终步长
 * @param  s        搜索方向（通常是下降方向）
 * @param  xp       线搜索前的变量值
 * @param  gp       线搜索前的梯度值
 * @param  stpmin   最小步长
 * @param  stpmax   最大步长
 * @param  cd       回调数据
 * @param  param    L-BFGS参数
 * @retval int      成功返回评估次数，失败返回负值错误码
 */
inline int line_search_lewisoverton(Eigen::VectorXd& x, double& f, Eigen::VectorXd& g, double& stp,
                                    const Eigen::VectorXd& s, const Eigen::VectorXd& xp,
                                    const Eigen::VectorXd& gp, const double stpmin,
                                    const double stpmax, const callback_data_t& cd,
                                    const lbfgs_parameter_t& param) {
    // 评估次数计数器
    int count = 0;
    // brackt: 标记是否找到了包含满足条件点的区间
    // touched: 标记是否已经尝试过最大步长
    bool brackt = false, touched = false;
    // finit: 初始目标函数值
    // dginit: 初始梯度在搜索方向上的投影
    // dgtest: Armijo条件的阈值
    // dstest: 弱Wolfe条件的阈值
    double finit, dginit, dgtest, dstest;
    // mu, nu: 搜索区间的下界和上界
    double mu = 0.0, nu = stpmax;

    /* 检查输入参数是否有错误 */
    if (!(stp > 0.0)) {
        return LBFGSERR_INVALIDPARAMETERS;
    }

    /* 计算初始梯度在搜索方向上的投影 */
    dginit = gp.dot(s);

    /* 确保s指向下降方向（梯度投影应为负） */
    if (0.0 < dginit) {
        return LBFGSERR_INCREASEGRADIENT;
    }

    /* 保存代价函数的初始值 */
    finit = f;
    // 计算Armijo条件的阈值（充分下降条件）
    dgtest = param.f_dec_coeff * dginit;
    // 计算弱Wolfe条件的阈值（曲率条件）
    dstest = param.s_curv_coeff * dginit;

    while (true) {
        // 计算新的试探点：x = xp + stp * s
        x = xp + stp * s;

        /* 在新点处评估目标函数和梯度值 */
        f = cd.proc_evaluate(cd.instance, x, g);
        ++count;

        /* 测试函数值是否有效（不是无穷大或NaN） */
        if (std::isinf(f) || std::isnan(f)) {
            return LBFGSERR_INVALID_FUNCVAL;
        }
        /* 检查Armijo条件（充分下降条件）：f(x + αd) ≤ f(x) + c1·α·∇f(x)^T·d */
        if (f > finit + stp * dgtest) {
            // 如果不满足Armijo条件，说明步长过大，更新上界
            nu = stp;
            brackt = true;
        } else {
            /* 检查弱Wolfe条件（曲率条件）：∇f(x + αd)^T·d ≥ c2·∇f(x)^T·d */
            if (g.dot(s) < dstest) {
                // 如果不满足弱Wolfe条件，说明步长过小，更新下界
                mu = stp;
            } else {
                // 同时满足Armijo条件和弱Wolfe条件，线搜索成功
                return count;
            }
        }
        // 检查是否达到最大线搜索次数
        if (param.max_linesearch <= count) {
            /* 达到最大迭代次数 */
            return LBFGSERR_MAXIMUMLINESEARCH;
        }
        // 检查搜索区间是否过小
        if (brackt && (nu - mu) < param.machine_prec * nu) {
            /* 相对区间宽度小于机器精度 */
            return LBFGSERR_WIDTHTOOSMALL;
        }

        // 更新下一次试探的步长
        if (brackt) {
            // 如果已经确定了包含解的区间，使用二分法
            stp = 0.5 * (mu + nu);
        } else {
            // 如果还没有找到上界，步长加倍
            stp *= 2.0;
        }

        // 检查步长是否小于最小值
        if (stp < stpmin) {
            /* 步长达到最小值 */
            return LBFGSERR_MINIMUMSTEP;
        }
        // 检查步长是否大于最大值
        if (stp > stpmax) {
            if (touched) {
                /* 已经尝试过最大步长，仍不满足条件 */
                return LBFGSERR_MAXIMUMSTEP;
            } else {
                /* 尝试使用最大步长 */
                touched = true;
                stp = stpmax;
            }
        }
    }
}

/**
 * 启动L-BFGS优化
 *
 * 基本假设：
 * 1. f(x)是C2连续函数或C0连续但分段C2连续的函数
 * 2. f(x)有下界
 * 3. f(x)的水平集有界
 * 4. g(x)是梯度或次梯度
 * 5. 在初始猜测x0处梯度存在
 *
 * 用户必须实现一个与::lbfgs_evaluate_t兼容的函数（评估回调）并将函数指针传递给lbfgs_optimize()
 * 类似地，用户可以实现与::lbfgs_stepbound_t兼容的函数来提供步长的外部上界，
 * 以及::lbfgs_progress_t（进度回调）来获取当前进度（如变量、函数值和梯度等）
 * 并在必要时取消迭代过程。步长上界和进度回调的实现是可选的：
 * 如果不需要进度通知，用户可以传递nullptr
 *
 * @param  x               决策变量向量
 *                         注意：在调用前必须设置初始猜测x0！
 *                         客户端程序可以通过此向量接收决策变量，
 *                         在最小化过程中会在这些点查询代价函数及其梯度
 * @param  f               接收变量最终代价函数值的引用
 * @param  proc_evaluate   提供函数f(x)和梯度g(x)评估的回调函数
 *                         给定当前变量x的值。客户端程序必须实现一个
 *                         与lbfgs_evaluate_t兼容的回调函数并传递函数指针
 * @param  proc_stepbound  提供要搜索的步长上界值的回调函数
 *                         提供线搜索前变量的初始值和当前步长向量（可以是负梯度）
 *                         客户端程序可以实现此函数以实现更高效的线搜索
 *                         如果不使用，只需设置为nullptr
 * @param  proc_progress   接收最小化过程进度的回调函数
 *                         （迭代次数、当前代价函数值等）
 *                         如果不需要进度报告，此参数可以设置为nullptr
 * @param  instance        客户端程序的用户数据指针
 *                         回调函数将接收此参数的值
 * @param  param           L-BFGS优化的参数
 * @retval int             状态码。如果最小化过程无错误终止，
 *                         此函数返回非负整数。负整数表示错误
 */
inline int lbfgs_optimize(Eigen::VectorXd& x, double& f, lbfgs_evaluate_t proc_evaluate,
                          lbfgs_stepbound_t proc_stepbound, lbfgs_progress_t proc_progress,
                          void* instance, const lbfgs_parameter_t& param) {
    // ret: 返回状态码
    // i, j: 循环索引
    // k: 迭代次数
    // ls: 线搜索评估次数
    // end: 有限内存中最新存储位置的索引
    // bound: 当前使用的修正数量
    int ret, i, j, k, ls, end, bound;
    // step: 当前步长
    // step_min, step_max: 步长的下界和上界
    // fx: 当前目标函数值
    // ys: y^T·s的值（用于逆Hessian近似）
    // yy: y^T·y的值（用于缩放）
    double step, step_min, step_max, fx, ys, yy;
    // gnorm_inf: 梯度的无穷范数
    // xnorm_inf: 变量的无穷范数
    // beta: L-BFGS递推公式中的中间变量
    // rate: 目标函数下降率
    // cau: 谨慎更新的阈值
    double gnorm_inf, xnorm_inf, beta, rate, cau;

    // n: 决策变量的维度
    const int n = x.size();
    // m: 有限内存的大小（修正数量）
    const int m = param.mem_size;

    /* 检查输入参数是否有错误 */
    if (n <= 0) {
        return LBFGSERR_INVALID_N;
    }
    if (m <= 0) {
        return LBFGSERR_INVALID_MEMSIZE;
    }
    if (param.g_epsilon < 0.0) {
        return LBFGSERR_INVALID_GEPSILON;
    }
    if (param.past < 0) {
        return LBFGSERR_INVALID_TESTPERIOD;
    }
    if (param.delta < 0.0) {
        return LBFGSERR_INVALID_DELTA;
    }
    if (param.min_step < 0.0) {
        return LBFGSERR_INVALID_MINSTEP;
    }
    if (param.max_step < param.min_step) {
        return LBFGSERR_INVALID_MAXSTEP;
    }
    if (!(param.f_dec_coeff > 0.0 && param.f_dec_coeff < 1.0)) {
        return LBFGSERR_INVALID_FDECCOEFF;
    }
    if (!(param.s_curv_coeff < 1.0 && param.s_curv_coeff > param.f_dec_coeff)) {
        return LBFGSERR_INVALID_SCURVCOEFF;
    }
    if (!(param.machine_prec > 0.0)) {
        return LBFGSERR_INVALID_MACHINEPREC;
    }
    if (param.max_linesearch <= 0) {
        return LBFGSERR_INVALID_MAXLINESEARCH;
    }

    /* 准备中间变量 */
    Eigen::VectorXd xp(n);                        // 上一次迭代的变量值
    Eigen::VectorXd g(n);                         // 当前梯度
    Eigen::VectorXd gp(n);                        // 上一次迭代的梯度
    Eigen::VectorXd d(n);                         // 搜索方向
    Eigen::VectorXd pf(std::max(1, param.past));  // 历史目标函数值（用于delta测试）

    /* 初始化有限内存 */
    Eigen::VectorXd lm_alpha = Eigen::VectorXd::Zero(m);  // L-BFGS递推公式中的alpha值
    Eigen::MatrixXd lm_s = Eigen::MatrixXd::Zero(n, m);   // 变量差：s_k = x_{k+1} - x_k
    Eigen::MatrixXd lm_y = Eigen::MatrixXd::Zero(n, m);   // 梯度差：y_k = g_{k+1} - g_k
    Eigen::VectorXd lm_ys = Eigen::VectorXd::Zero(m);  // y^T·s的值（用于逆Hessian近似）

    /* 构造回调数据 */
    callback_data_t cd;
    cd.instance = instance;
    cd.proc_evaluate = proc_evaluate;
    cd.proc_stepbound = proc_stepbound;
    cd.proc_progress = proc_progress;

    /* 在初始点评估目标函数值和梯度 */
    fx = cd.proc_evaluate(cd.instance, x, g);

    /* 存储初始的代价函数值 */
    pf(0) = fx;

    /*
    计算搜索方向；
    假设初始Hessian矩阵H_0为单位矩阵，因此初始方向为负梯度
    */
    d = -g;

    /*
    确保初始变量不是驻点（梯度不为零）
    */
    gnorm_inf = g.cwiseAbs().maxCoeff();  // 计算梯度的无穷范数
    xnorm_inf = x.cwiseAbs().maxCoeff();  // 计算变量的无穷范数

    if (gnorm_inf / std::max(1.0, xnorm_inf) < param.g_epsilon) {
        /* 初始猜测已经是驻点 */
        ret = LBFGS_CONVERGENCE;
    } else {
        /*
        计算初始步长：
        使用1/||d||作为初始步长，保证第一步不会太大
        */
        step = 1.0 / d.norm();

        k = 1;      // 迭代计数器
        end = 0;    // 有限内存循环缓冲区的当前位置
        bound = 0;  // 当前存储的修正数量

        while (true) {
            /* 保存当前位置和梯度向量 */
            xp = x;
            gp = g;

            /* 如果可以动态提供步长上界，则应用它 */
            step_min = param.min_step;
            step_max = param.max_step;
            if (cd.proc_stepbound) {
                // 调用用户提供的步长上界回调函数
                step_max = cd.proc_stepbound(cd.instance, xp, d);
                // 确保不超过参数中设置的最大步长
                step_max = step_max < param.max_step ? step_max : param.max_step;
                // 如果当前步长超过上界，则调整为上界的一半
                step = step < step_max ? step : 0.5 * step_max;
            }

            /* 执行线搜索以找到最优步长 */
            ls = line_search_lewisoverton(x, fx, g, step, d, xp, gp, step_min, step_max, cd, param);

            if (ls < 0) {
                /* 线搜索失败，恢复到上一个点 */
                x = xp;
                g = gp;
                ret = ls;
                break;
            }

            /* 报告优化进度 */
            if (cd.proc_progress) {
                // 调用进度回调函数，如果返回非零值则取消优化
                if (cd.proc_progress(cd.instance, x, g, fx, step, k, ls)) {
                    ret = LBFGS_CANCELED;
                    break;
                }
            }

            /*
            收敛性测试
            使用以下准则判断是否收敛：
            ||g(x)||_inf / max(1, ||x||_inf) < g_epsilon
            即：归一化的梯度无穷范数小于阈值
            */
            gnorm_inf = g.cwiseAbs().maxCoeff();  // 计算梯度的无穷范数
            xnorm_inf = x.cwiseAbs().maxCoeff();  // 计算变量的无穷范数
            if (gnorm_inf / std::max(1.0, xnorm_inf) < param.g_epsilon) {
                /* 达到收敛条件 */
                ret = LBFGS_CONVERGENCE;
                break;
            }

            /*
            基于函数值下降的停止准则测试
            使用以下准则判断是否停止：
            |f(past_x) - f(x)| / max(1, |f(x)|) < delta
            即：相对函数值变化小于阈值
            */
            if (0 < param.past) {
                /* 只有当迭代次数k >= past时才测试停止准则 */
                if (param.past <= k) {
                    /* 计算相对函数值下降率 */
                    rate = std::fabs(pf(k % param.past) - fx) / std::max(1.0, std::fabs(fx));

                    if (rate < param.delta) {
                        // 函数值变化太小，满足停止准则
                        ret = LBFGS_STOP;
                        break;
                    }
                }

                /* 存储当前的代价函数值用于后续比较 */
                pf(k % param.past) = fx;
            }

            // 检查是否达到最大迭代次数
            if (param.max_iterations != 0 && param.max_iterations <= k) {
                /* 达到最大迭代次数 */
                ret = LBFGSERR_MAXIMUMITERATION;
                break;
            }

            /* 增加迭代计数 */
            ++k;

            /*
            更新向量s和y：
            s_{k+1} = x_{k+1} - x_{k} = step * d_{k}  （变量的变化量）
            y_{k+1} = g_{k+1} - g_{k}  （梯度的变化量）
            这两个向量用于构造逆Hessian矩阵的近似
            */
            lm_s.col(end) = x - xp;  // 变量差
            lm_y.col(end) = g - gp;  // 梯度差

            /*
            计算标量ys和yy：
            ys = y^T · s = 1 / ρ （ρ是BFGS更新公式中的参数）
            yy = y^T · y （梯度差的平方范数）
            注意：yy用于缩放Hessian矩阵H_0（Cholesky因子）
            */
            ys = lm_y.col(end).dot(lm_s.col(end));  // 计算y^T·s
            yy = lm_y.col(end).squaredNorm();       // 计算y^T·y
            lm_ys(end) = ys;                        // 存储ys值

            /* 计算负梯度（初始搜索方向） */
            d = -g;

            /*
            谨慎更新策略（Cautious Update）
            只有满足以下条件时才执行更新：
            (y^T · s) / ||s_{k+1}||^2 > ε * ||g_{k}||^α
            其中ε是谨慎因子，推荐α = 1

            这不是为了强制近似Hessian矩阵的正定性（弱Wolfe条件已经确保ys > 0），
            而是为了确保全局收敛性，如下文所述：
            Dong-Hui Li and Masao Fukushima. On the global convergence of
            the BFGS method for nonconvex unconstrained optimization problems.
            SIAM Journal on Optimization, Vol 11, No 4, pp. 1054-1064, 2011.
            */
            cau = lm_s.col(end).squaredNorm() * gp.norm() * param.cautious_factor;

            if (ys > cau) {
                /*
                使用递推公式计算dir = -(H · g)，即L-BFGS搜索方向
                这在以下文献的第779页中有描述：
                Jorge Nocedal.
                Updating Quasi-Newton Matrices with Limited Storage.
                Mathematics of Computation, Vol. 35, No. 151,
                pp. 773--782, 1980.

                L-BFGS两循环递推算法：
                第一个循环：从最新到最旧的修正，计算α值并更新q
                第二个循环：从最旧到最新的修正，计算β值并更新r
                */
                ++bound;
                bound = m < bound ? m : bound;  // 修正数量不超过m
                end = (end + 1) % m;            // 更新循环缓冲区索引

                // 第一个循环：向后遍历，计算α并更新方向
                j = end;
                for (i = 0; i < bound; ++i) {
                    j = (j + m - 1) % m;  // 向前移动索引（循环）
                    /* 计算α_j = ρ_j * s_j^T · q_{k+1} */
                    lm_alpha(j) = lm_s.col(j).dot(d) / lm_ys(j);
                    /* 更新q_i = q_{i+1} - α_i * y_i */
                    d += (-lm_alpha(j)) * lm_y.col(j);
                }

                // 使用Hessian初始近似H_0 = (ys/yy) * I进行缩放
                d *= ys / yy;

                // 第二个循环：向前遍历，计算β并更新方向
                for (i = 0; i < bound; ++i) {
                    /* 计算β_j = ρ_j * y_j^T · γ_i */
                    beta = lm_y.col(j).dot(d) / lm_ys(j);
                    /* 更新γ_{i+1} = γ_i + (α_j - β_j) * s_j */
                    d += (lm_alpha(j) - beta) * lm_s.col(j);
                    j = (j + 1) % m;  // 向后移动索引（循环）
                }
            }

            /* 搜索方向d已准备好，首先尝试步长为1 */
            step = 1.0;
        }
    }

    /* 返回代价函数的最终值 */
    f = fx;

    return ret;
}

/**
 * 获取lbfgs_optimize()返回码的字符串描述
 *
 * @param err  lbfgs_optimize()返回的状态码
 * @return     对应的错误描述字符串
 */
inline const char* lbfgs_strerror(const int err) {
    // 根据错误码返回相应的描述信息
    switch (err) {
        case LBFGS_CONVERGENCE:
            // 成功：达到收敛（梯度满足g_epsilon阈值）
            return "Success: reached convergence (g_epsilon).";

        case LBFGS_STOP:
            // 成功：满足停止准则（历史函数值下降小于delta）
            return "Success: met stopping criteria (past f decrease less than delta).";

        case LBFGS_CANCELED:
            // 迭代被监控回调函数取消
            return "The iteration has been canceled by the monitor callback.";

        case LBFGSERR_UNKNOWNERROR:
            // 未知错误
            return "Unknown error.";

        case LBFGSERR_INVALID_N:
            // 无效的变量数量
            return "Invalid number of variables specified.";

        case LBFGSERR_INVALID_MEMSIZE:
            // 无效的内存大小参数
            return "Invalid parameter lbfgs_parameter_t::mem_size specified.";

        case LBFGSERR_INVALID_GEPSILON:
            // 无效的梯度epsilon参数
            return "Invalid parameter lbfgs_parameter_t::g_epsilon specified.";

        case LBFGSERR_INVALID_TESTPERIOD:
            // 无效的past参数
            return "Invalid parameter lbfgs_parameter_t::past specified.";

        case LBFGSERR_INVALID_DELTA:
            // 无效的delta参数
            return "Invalid parameter lbfgs_parameter_t::delta specified.";

        case LBFGSERR_INVALID_MINSTEP:
            // 无效的最小步长参数
            return "Invalid parameter lbfgs_parameter_t::min_step specified.";

        case LBFGSERR_INVALID_MAXSTEP:
            // 无效的最大步长参数
            return "Invalid parameter lbfgs_parameter_t::max_step specified.";

        case LBFGSERR_INVALID_FDECCOEFF:
            // 无效的函数下降系数参数
            return "Invalid parameter lbfgs_parameter_t::f_dec_coeff specified.";

        case LBFGSERR_INVALID_SCURVCOEFF:
            // 无效的曲率系数参数
            return "Invalid parameter lbfgs_parameter_t::s_curv_coeff specified.";

        case LBFGSERR_INVALID_MACHINEPREC:
            // 无效的机器精度参数
            return "Invalid parameter lbfgs_parameter_t::machine_prec specified.";

        case LBFGSERR_INVALID_MAXLINESEARCH:
            // 无效的最大线搜索次数参数
            return "Invalid parameter lbfgs_parameter_t::max_linesearch specified.";

        case LBFGSERR_INVALID_FUNCVAL:
            // 函数值变为NaN或Inf
            return "The function value became NaN or Inf.";

        case LBFGSERR_MINIMUMSTEP:
            // 线搜索步长小于最小值
            return "The line-search step became smaller than lbfgs_parameter_t::min_step.";

        case LBFGSERR_MAXIMUMSTEP:
            // 线搜索步长大于最大值
            return "The line-search step became larger than lbfgs_parameter_t::max_step.";

        case LBFGSERR_MAXIMUMLINESEARCH:
            // 线搜索达到最大次数，假设不满足或精度无法达到
            return "Line search reaches the maximum try number, assumptions not satisfied or "
                   "precision not achievable.";

        case LBFGSERR_MAXIMUMITERATION:
            // 算法达到最大迭代次数
            return "The algorithm routine reaches the maximum number of iterations.";

        case LBFGSERR_WIDTHTOOSMALL:
            // 相对搜索区间宽度小于机器精度
            return "Relative search interval width is at least lbfgs_parameter_t::machine_prec.";

        case LBFGSERR_INVALIDPARAMETERS:
            // 发生逻辑错误（负的线搜索步长）
            return "A logic error (negative line-search step) occurred.";

        case LBFGSERR_INCREASEGRADIENT:
            // 当前搜索方向增加了代价函数值
            return "The current search direction increases the cost function value.";

        default:
            // 未知的错误码
            return "(unknown)";
    }
}

}  // namespace lbfgs

#endif
