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

#ifndef FLATNESS_HPP
#define FLATNESS_HPP

#include <Eigen/Eigen>
#include <cmath>

// 微分平坦性命名空间：用于四旋翼飞行器的微分平坦映射
namespace flatness {
// 微分平坦映射类：实现四旋翼的微分平坦性变换
// 详细说明见：the bundled flatness mapping reference
// 微分平坦性是指系统的状态和输入可以用平坦输出及其导数来表示
// 对于四旋翼，位置及其导数（速度、加速度、jerk）是平坦输出
class FlatnessMap {
   public:
    // 重置微分平坦映射的参数
    // 参数说明：
    //   vehicle_mass: 飞行器质量 (kg)
    //   gravitational_acceleration: 重力加速度 (m/s^2)
    //   horitonral_drag_coeff: 水平方向阻力系数
    //   vertical_drag_coeff: 垂直方向阻力系数
    //   parasitic_drag_coeff: 寄生阻力系数
    //   speed_smooth_factor: 速度平滑因子，避免除零
    inline void reset(const double& vehicle_mass, const double& gravitational_acceleration,
                      const double& horitonral_drag_coeff, const double& vertical_drag_coeff,
                      const double& parasitic_drag_coeff, const double& speed_smooth_factor) {
        mass = vehicle_mass;                // 飞行器质量
        grav = gravitational_acceleration;  // 重力加速度
        dh = horitonral_drag_coeff;         // 水平阻力系数
        dv = vertical_drag_coeff;           // 垂直阻力系数
        cp = parasitic_drag_coeff;          // 寄生阻力系数
        veps = speed_smooth_factor;         // 速度平滑因子

        return;
    }

    // 正向映射：从平坦输出（位置及其导数）到状态和输入
    // 输入参数：
    //   vel: 速度向量 (m/s)
    //   acc: 加速度向量 (m/s^2)
    //   jer: jerk向量（加速度的导数）(m/s^3)
    //   psi: 偏航角 (rad)
    //   dpsi: 偏航角速度 (rad/s)
    // 输出参数：
    //   thr: 推力 (N)
    //   quat: 四元数姿态 [w, x, y, z]
    //   omg: 角速度向量 (rad/s)
    inline void forward(const Eigen::Vector3d& vel, const Eigen::Vector3d& acc,
                        const Eigen::Vector3d& jer, const double& psi, const double& dpsi,
                        double& thr, Eigen::Vector4d& quat, Eigen::Vector3d& omg) {
        // 声明中间变量
        double w0, w1, w2, dw0, dw1, dw2;

        // 提取速度分量
        v0 = vel(0);
        v1 = vel(1);
        v2 = vel(2);
        // 提取加速度分量
        a0 = acc(0);
        a1 = acc(1);
        a2 = acc(2);
        // 计算速度的平滑模长，添加veps避免除零
        cp_term = sqrt(v0 * v0 + v1 * v1 + v2 * v2 + veps);
        // 计算阻力相关项：1 + cp * |v|
        w_term = 1.0 + cp * cp_term;
        // 计算考虑寄生阻力的速度项
        w0 = w_term * v0;
        w1 = w_term * v1;
        w2 = w_term * v2;
        // 计算单位质量的水平阻力系数
        dh_over_m = dh / mass;
        // 计算未归一化的推力方向向量（考虑加速度、阻力和重力）
        zu0 = a0 + dh_over_m * w0;
        zu1 = a1 + dh_over_m * w1;
        zu2 = a2 + dh_over_m * w2 + grav;
        // 计算推力方向向量各分量的平方
        zu_sqr0 = zu0 * zu0;
        zu_sqr1 = zu1 * zu1;
        zu_sqr2 = zu2 * zu2;
        // 计算推力方向向量分量的交叉乘积
        zu01 = zu0 * zu1;
        zu12 = zu1 * zu2;
        zu02 = zu0 * zu2;
        // 计算推力方向向量的模的平方
        zu_sqr_norm = zu_sqr0 + zu_sqr1 + zu_sqr2;
        // 计算推力方向向量的模
        zu_norm = sqrt(zu_sqr_norm);
        // 归一化推力方向向量，得到机体z轴方向
        z0 = zu0 / zu_norm;
        z1 = zu1 / zu_norm;
        z2 = zu2 / zu_norm;
        // 计算归一化梯度的分母项
        ng_den = zu_sqr_norm * zu_norm;
        // 计算归一化梯度矩阵的元素（用于后续求导）
        ng00 = (zu_sqr1 + zu_sqr2) / ng_den;
        ng01 = -zu01 / ng_den;
        ng02 = -zu02 / ng_den;
        ng11 = (zu_sqr0 + zu_sqr2) / ng_den;
        ng12 = -zu12 / ng_den;
        ng22 = (zu_sqr0 + zu_sqr1) / ng_den;
        // 计算速度和加速度的点积
        v_dot_a = v0 * a0 + v1 * a1 + v2 * a2;
        // 计算w项对时间的导数系数
        dw_term = cp * v_dot_a / cp_term;
        // 计算w项对时间的导数各分量
        dw0 = w_term * a0 + dw_term * v0;
        dw1 = w_term * a1 + dw_term * v1;
        dw2 = w_term * a2 + dw_term * v2;
        // 计算z方向（推力方向）导数的中间项
        dz_term0 = jer(0) + dh_over_m * dw0;
        dz_term1 = jer(1) + dh_over_m * dw1;
        dz_term2 = jer(2) + dh_over_m * dw2;
        // 通过归一化梯度矩阵计算机体z轴方向的导数
        dz0 = ng00 * dz_term0 + ng01 * dz_term1 + ng02 * dz_term2;
        dz1 = ng01 * dz_term0 + ng11 * dz_term1 + ng12 * dz_term2;
        dz2 = ng02 * dz_term0 + ng12 * dz_term1 + ng22 * dz_term2;
        // 计算考虑垂直阻力的力向量各分量
        f_term0 = mass * a0 + dv * w0;
        f_term1 = mass * a1 + dv * w1;
        f_term2 = mass * (a2 + grav) + dv * w2;
        // 计算推力大小：力向量在机体z轴方向的投影
        thr = z0 * f_term0 + z1 * f_term1 + z2 * f_term2;
        // 计算倾斜四元数的辅助变量（不含偏航角）
        tilt_den = sqrt(2.0 * (1.0 + z2));
        tilt0 = 0.5 * tilt_den;
        tilt1 = -z1 / tilt_den;
        tilt2 = z0 / tilt_den;
        // 计算半偏航角的三角函数值
        c_half_psi = cos(0.5 * psi);
        s_half_psi = sin(0.5 * psi);
        // 组合倾斜和偏航，计算最终的姿态四元数 [w, x, y, z]
        quat(0) = tilt0 * c_half_psi;
        quat(1) = tilt1 * c_half_psi + tilt2 * s_half_psi;
        quat(2) = tilt2 * c_half_psi - tilt1 * s_half_psi;
        quat(3) = tilt0 * s_half_psi;
        // 计算偏航角的三角函数值
        c_psi = cos(psi);
        s_psi = sin(psi);
        // 计算角速度的辅助变量
        omg_den = z2 + 1.0;
        omg_term = dz2 / omg_den;
        // 计算机体坐标系下的角速度x分量（roll rate）
        omg(0) = dz0 * s_psi - dz1 * c_psi - (z0 * s_psi - z1 * c_psi) * omg_term;
        // 计算机体坐标系下的角速度y分量（pitch rate）
        omg(1) = dz0 * c_psi + dz1 * s_psi - (z0 * c_psi + z1 * s_psi) * omg_term;
        // 计算机体坐标系下的角速度z分量（yaw rate）
        omg(2) = (z1 * dz0 - z0 * dz1) / omg_den + dpsi;

        return;
    }

    // 反向传播：计算梯度从输出到输入的传播（用于优化）
    // 这是自动微分的反向模式，用于计算损失函数对平坦输出的梯度
    // 输入参数（上游梯度）：
    //   pos_grad: 位置的梯度
    //   vel_grad: 速度的梯度
    //   thr_grad: 推力的梯度
    //   quat_grad: 四元数的梯度
    //   omg_grad: 角速度的梯度
    // 输出参数（下游梯度）：
    //   pos_total_grad: 位置的总梯度
    //   vel_total_grad: 速度的总梯度
    //   acc_total_grad: 加速度的总梯度
    //   jer_total_grad: jerk的总梯度
    //   psi_total_grad: 偏航角的总梯度
    //   dpsi_total_grad: 偏航角速度的总梯度
    inline void backward(const Eigen::Vector3d& pos_grad, const Eigen::Vector3d& vel_grad,
                         const double& thr_grad, const Eigen::Vector4d& quat_grad,
                         const Eigen::Vector3d& omg_grad, Eigen::Vector3d& pos_total_grad,
                         Eigen::Vector3d& vel_total_grad, Eigen::Vector3d& acc_total_grad,
                         Eigen::Vector3d& jer_total_grad, double& psi_total_grad,
                         double& dpsi_total_grad) const {
        // 声明所有中间变量的梯度（后缀'b'表示backward/梯度）
        double w0b, w1b, w2b, dw0b, dw1b, dw2b;
        double z0b, z1b, z2b, dz0b, dz1b, dz2b;
        double v_sqr_normb, cp_termb, w_termb;
        double zu_sqr_normb, zu_normb, zu0b, zu1b, zu2b;
        double zu_sqr0b, zu_sqr1b, zu_sqr2b, zu01b, zu12b, zu02b;
        double ng00b, ng01b, ng02b, ng11b, ng12b, ng22b, ng_denb;
        double dz_term0b, dz_term1b, dz_term2b, f_term0b, f_term1b, f_term2b;
        double tilt_denb, tilt0b, tilt1b, tilt2b, head0b, head3b;
        double cpsib, spsib, omg_denb, omg_termb;
        double tempb, tilt_den_sqr;

        // 反向传播四元数梯度到倾斜四元数分量
        tilt0b = s_half_psi * (quat_grad(3)) + c_half_psi * (quat_grad(0));
        head3b = tilt0 * (quat_grad(3)) + tilt2 * (quat_grad(1)) - tilt1 * (quat_grad(2));
        tilt2b = c_half_psi * (quat_grad(2)) + s_half_psi * (quat_grad(1));
        head0b = tilt2 * (quat_grad(2)) + tilt1 * (quat_grad(1)) + tilt0 * (quat_grad(0));
        tilt1b = c_half_psi * (quat_grad(1)) - s_half_psi * (quat_grad(2));
        tilt_den_sqr = tilt_den * tilt_den;
        tilt_denb = (z1 * tilt1b - z0 * tilt2b) / tilt_den_sqr + 0.5 * tilt0b;
        // 反向传播角速度梯度
        omg_termb = -((z0 * c_psi + z1 * s_psi) * (omg_grad(1))) -
                    (z0 * s_psi - z1 * c_psi) * (omg_grad(0));
        tempb = omg_grad(2) / omg_den;
        dpsi_total_grad = omg_grad(2);  // 偏航角速度的梯度
        z1b = dz0 * tempb;
        dz0b = z1 * tempb + c_psi * (omg_grad(1)) + s_psi * (omg_grad(0));
        z0b = -(dz1 * tempb);
        dz1b = s_psi * (omg_grad(1)) - z0 * tempb - c_psi * (omg_grad(0));
        omg_denb =
            -((z1 * dz0 - z0 * dz1) * tempb / omg_den) - dz2 * omg_termb / (omg_den * omg_den);
        // 继续反向传播，累积梯度到偏航角相关变量
        tempb = -(omg_term * (omg_grad(1)));
        cpsib = dz0 * (omg_grad(1)) + z0 * tempb;
        spsib = dz1 * (omg_grad(1)) + z1 * tempb;
        z0b += c_psi * tempb;
        z1b += s_psi * tempb;
        tempb = -(omg_term * (omg_grad(0)));
        spsib += dz0 * (omg_grad(0)) + z0 * tempb;
        cpsib += -dz1 * (omg_grad(0)) - z1 * tempb;
        z0b += s_psi * tempb + tilt2b / tilt_den + f_term0 * (thr_grad);
        z1b += -c_psi * tempb - tilt1b / tilt_den + f_term1 * (thr_grad);
        dz2b = omg_termb / omg_den;
        z2b = omg_denb + tilt_denb / tilt_den + f_term2 * (thr_grad);
        // 计算偏航角的总梯度
        psi_total_grad =
            c_psi * spsib + 0.5 * c_half_psi * head3b - s_psi * cpsib - 0.5 * s_half_psi * head0b;
        // 反向传播推力梯度到力项
        f_term0b = z0 * (thr_grad);
        f_term1b = z1 * (thr_grad);
        f_term2b = z2 * (thr_grad);
        // 反向传播到归一化梯度矩阵和z方向导数项
        ng02b = dz_term0 * dz2b + dz_term2 * dz0b;
        dz_term0b = ng02 * dz2b + ng01 * dz1b + ng00 * dz0b;
        ng12b = dz_term1 * dz2b + dz_term2 * dz1b;
        dz_term1b = ng12 * dz2b + ng11 * dz1b + ng01 * dz0b;
        ng22b = dz_term2 * dz2b;
        dz_term2b = ng22 * dz2b + ng12 * dz1b + ng02 * dz0b;
        ng01b = dz_term0 * dz1b + dz_term1 * dz0b;
        ng11b = dz_term1 * dz1b;
        ng00b = dz_term0 * dz0b;
        // 反向传播到jerk（加加速度）
        jer_total_grad(2) = dz_term2b;
        dw2b = dh_over_m * dz_term2b;
        jer_total_grad(1) = dz_term1b;
        dw1b = dh_over_m * dz_term1b;
        jer_total_grad(0) = dz_term0b;
        dw0b = dh_over_m * dz_term0b;
        // 反向传播到加速度
        tempb = cp * (v2 * dw2b + v1 * dw1b + v0 * dw0b) / cp_term;
        acc_total_grad(2) = mass * f_term2b + w_term * dw2b + v2 * tempb;
        acc_total_grad(1) = mass * f_term1b + w_term * dw1b + v1 * tempb;
        acc_total_grad(0) = mass * f_term0b + w_term * dw0b + v0 * tempb;
        // 反向传播到速度
        vel_total_grad(2) = dw_term * dw2b + a2 * tempb;
        vel_total_grad(1) = dw_term * dw1b + a1 * tempb;
        vel_total_grad(0) = dw_term * dw0b + a0 * tempb;
        cp_termb = -(v_dot_a * tempb / cp_term);
        // 反向传播归一化梯度矩阵的梯度
        tempb = ng22b / ng_den;
        zu_sqr0b = tempb;
        zu_sqr1b = tempb;
        ng_denb = -((zu_sqr0 + zu_sqr1) * tempb / ng_den);
        zu12b = -(ng12b / ng_den);
        tempb = ng11b / ng_den;
        ng_denb += zu12 * ng12b / (ng_den * ng_den) - (zu_sqr0 + zu_sqr2) * tempb / ng_den;
        zu_sqr0b += tempb;
        zu_sqr2b = tempb;
        zu02b = -(ng02b / ng_den);
        zu01b = -(ng01b / ng_den);
        tempb = ng00b / ng_den;
        ng_denb += zu02 * ng02b / (ng_den * ng_den) + zu01 * ng01b / (ng_den * ng_den) -
                   (zu_sqr1 + zu_sqr2) * tempb / ng_den;
        // 反向传播推力方向向量的模和模的平方
        zu_normb = zu_sqr_norm * ng_denb - (zu2 * z2b + zu1 * z1b + zu0 * z0b) / zu_sqr_norm;
        zu_sqr_normb = zu_norm * ng_denb + zu_normb / (2.0 * zu_norm);
        tempb += zu_sqr_normb;
        zu_sqr1b += tempb;
        zu_sqr2b += tempb;
        // 反向传播未归一化推力方向向量
        zu2b = z2b / zu_norm + zu0 * zu02b + zu1 * zu12b + 2 * zu2 * zu_sqr2b;
        w2b = dv * f_term2b + dh_over_m * zu2b;
        zu1b = z1b / zu_norm + zu2 * zu12b + zu0 * zu01b + 2 * zu1 * zu_sqr1b;
        w1b = dv * f_term1b + dh_over_m * zu1b;
        zu_sqr0b += zu_sqr_normb;
        zu0b = z0b / zu_norm + zu2 * zu02b + zu1 * zu01b + 2 * zu0 * zu_sqr0b;
        w0b = dv * f_term0b + dh_over_m * zu0b;
        // 反向传播阻力相关项
        w_termb = a2 * dw2b + a1 * dw1b + a0 * dw0b + v2 * w2b + v1 * w1b + v0 * w0b;
        // 累积加速度的梯度
        acc_total_grad(2) += zu2b;
        acc_total_grad(1) += zu1b;
        acc_total_grad(0) += zu0b;
        // 反向传播速度平滑项
        cp_termb += cp * w_termb;
        v_sqr_normb = cp_termb / (2.0 * cp_term);
        // 累积速度的总梯度（包括上游梯度）
        vel_total_grad(2) += w_term * w2b + 2 * v2 * v_sqr_normb + vel_grad(2);
        vel_total_grad(1) += w_term * w1b + 2 * v1 * v_sqr_normb + vel_grad(1);
        vel_total_grad(0) += w_term * w0b + 2 * v0 * v_sqr_normb + vel_grad(0);
        // 位置的总梯度直接来自上游梯度（位置不参与正向计算）
        pos_total_grad(2) = pos_grad(2);
        pos_total_grad(1) = pos_grad(1);
        pos_total_grad(0) = pos_grad(0);

        return;
    }

   private:
    // === 飞行器物理参数 ===
    double mass;  // 飞行器质量 (kg)
    double grav;  // 重力加速度 (m/s^2)
    double dh;    // 水平阻力系数
    double dv;    // 垂直阻力系数
    double cp;    // 寄生阻力系数
    double veps;  // 速度平滑因子（避免除零）

    // === 运动状态分量 ===
    double v0, v1, v2;  // 速度分量 (m/s)
    double a0, a1, a2;  // 加速度分量 (m/s^2)
    double v_dot_a;     // 速度和加速度的点积

    // === 推力方向相关变量 ===
    double z0, z1, z2;     // 归一化的机体z轴方向（推力方向）
    double dz0, dz1, dz2;  // 推力方向的时间导数

    // === 阻力和平滑项 ===
    double cp_term;    // 速度的平滑模长：sqrt(v^2 + veps)
    double w_term;     // 阻力项：1 + cp * |v|
    double dh_over_m;  // 单位质量的水平阻力系数

    // === 推力方向向量（未归一化）===
    double zu_sqr_norm;                // 推力方向向量的模的平方
    double zu_norm;                    // 推力方向向量的模
    double zu0, zu1, zu2;              // 推力方向向量各分量
    double zu_sqr0, zu_sqr1, zu_sqr2;  // 推力方向向量各分量的平方
    double zu01, zu12, zu02;           // 推力方向向量分量的交叉项

    // === 归一化梯度矩阵（用于求导）===
    double ng00, ng01, ng02;  // 归一化梯度矩阵第一行
    double ng11, ng12;        // 归一化梯度矩阵第二、三行
    double ng22;              // 归一化梯度矩阵对角元素
    double ng_den;            // 归一化梯度的分母

    // === 中间计算项 ===
    double dw_term;                       // w项的时间导数系数
    double dz_term0, dz_term1, dz_term2;  // z方向导数的中间项
    double f_term0, f_term1, f_term2;     // 力向量各分量

    // === 姿态四元数相关变量 ===
    double tilt_den;                // 倾斜四元数计算的分母
    double tilt0, tilt1, tilt2;     // 倾斜四元数分量（不含偏航）
    double c_half_psi, s_half_psi;  // 半偏航角的余弦和正弦
    double c_psi, s_psi;            // 偏航角的余弦和正弦

    // === 角速度相关变量 ===
    double omg_den;   // 角速度计算的分母
    double omg_term;  // 角速度计算的中间项
};
}  // namespace flatness

#endif
