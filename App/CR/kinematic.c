#include <stdint.h>
#include <math.h>

#include "kinematic.h"

#define PI 3.14159265358979323846

/**********************************************************
***	编写作者：blinest

***	qq：1071378062
**********************************************************/
// 基础版本的常曲率模型
void calculate_L(float R, float theta[], float phi[], float deltaL[]) {
	deltaL[0] = - R * theta[0] * cos(phi[0]);
	deltaL[2] = - R * theta[0] * cos(phi[0] + 2.0 / 3.0 * PI);
	deltaL[4] = - R * theta[0] * cos(phi[0] + 4.0 / 3.0 * PI);
	deltaL[1] = - R * theta[0] * cos(phi[0] + PI / 3.0) + R * theta[1] * cos(phi[1] + PI / 3.0);
	deltaL[3] = - R * theta[0] * cos(phi[0] + PI) - R * theta[1] * cos(phi[1] + PI);
	deltaL[5] = - R * theta[0] * cos(phi[0] + 5.0 / 3.0 * PI) - R * theta[1] * cos(phi[1] + 5.0 / 3.0 * PI);
}
/**
 * @brief 运动学逆解：由 deltaL 计算 theta 和 phi
 * @param R       结构参数（半径）
 * @param deltaL  输入，长度6的数组：d0, d1, d2, d3, d4, d5
 * @param theta   输出，长度2的数组：theta[0], theta[1]
 * @param phi     输出，长度2的数组：phi[0], phi[1] （弧度）
 */
void inverse_kinematics(float R, const float deltaL[], float theta[], float phi[]) {
    // ---------- 第一步：求解 theta0, phi0 ----------
    float d0 = deltaL[0];
    float d2 = deltaL[2];
    float d4 = deltaL[4];

    float alpha0 = d0;
    float beta0  = (d4 - d2) / sqrtf(3.0f);
    float A0     = sqrtf(alpha0 * alpha0 + beta0 * beta0);

    if (A0 < 1e-6f) {          // 奇异情况，长度几乎为零
        theta[0] = 0.0f;
        phi[0]   = 0.0f;       // 角度可任意，设0
    } else {
        theta[0] = A0 / R;
        phi[0]   = atan2f(beta0, alpha0);
    }

    // ---------- 第二步：扣除 theta0 贡献，求解 theta1, phi1 ----------
    // 为避免 phi[0] 在奇异情况下未定义，直接从 alpha0/beta0 算出 cos 和 sin
    float cos_p0, sin_p0;
    if (A0 < 1e-6f) {
        cos_p0 = 1.0f;  sin_p0 = 0.0f;  // A0=0 贡献为0，此值不影响
    } else {
        cos_p0 = alpha0 / A0;
        sin_p0 = beta0  / A0;
    }

    // 计算 c1 = d1 - A0*cos(phi0 + π/3) 等
    // 利用三角展开：cos(φ+π/3)=cosφ*cos(π/3)-sinφ*sin(π/3)，但直接计算更清晰
    float cos_phi0_plus_PI3  = cos_p0 * 0.5f - sin_p0 * 0.8660254037844386f; // cos(π/3)=0.5, sin(π/3)=√3/2
    float cos_phi0_plus_PI   = -cos_p0;                                      // cos(φ+π) = -cosφ
    float cos_phi0_plus_5PI3 = cos_p0 * 0.5f + sin_p0 * 0.8660254037844386f; // cos(φ+5π/3)=cos(φ-π/3)=cosφ/2+sinφ*√3/2

    float c1 = deltaL[1] - A0 * cos_phi0_plus_PI3;
    float c3 = deltaL[3] - A0 * cos_phi0_plus_PI;
    float c5 = deltaL[5] - A0 * cos_phi0_plus_5PI3;

    float alpha1 = c1;
    float beta1  = (c5 - c3) / sqrtf(3.0f);
    float A1     = sqrtf(alpha1 * alpha1 + beta1 * beta1);

    if (A1 < 1e-6f) {
        theta[1] = 0.0f;
        phi[1]   = 0.0f;
    } else {
        theta[1] = A1 / R;
        float phi1_prime = atan2f(beta1, alpha1);
        phi[1] = phi1_prime - M_PI / 3.0f;
        // 可选：将角度归化到 (-π, π] 或 [0, 2π)
        // 这里保留 atan2 的原始范围 (-π, π] 再减去 π/3，仍在合理范围内
    }
}