#ifdef USE_SQUARE_ADVANCED
/*!
 * @file OpenFIRE_Square_Advanced.h
 * @brief Light Gun library for 4 LED setup
 * @n CPP file for Samco Light Gun 4 LED setup
 *
 * @copyright alessandro-satanassi, https://github.com/alessandro-satanassi, 2025
 * @copyright GNU Lesser General Public License
 *
 * @author [Alessandro Satanassi](alessandro@cittini.it)
 * @version V1.0
 * @date 2025
 * 
 * I thank you for producing the first original code:
 * 
 * @copyright Samco, https://github.com/samuelballantyne, 2024
 * @copyright GNU Lesser General Public License
 *
 * @author [Sam Ballantyne](samuelballantyne@hotmail.com)
 * @version V1.0
 * @date 2024
 */

#ifndef _OpenFIRE_Square_Advanced_h_
#define _OpenFIRE_Square_Advanced_h_

#include <stdint.h>
#include "OpenFIREConst.h"


class OpenFIRE_Square {
public:
    //================================================================
    // ENUM PUBBLICO E API
    //================================================================

    // Metodo principale di elaborazione
    void begin(const int* px, const int* py, unsigned int seen);
    
    // Metodi 'Getter' per l'accesso allo stato finale
    int X(int index) const { return FinalX[index]; }
    int Y(int index) const { return FinalY[index]; }
    unsigned int testSee(int index) const { return see[index]; }
    int testMedianX() const { return medianX; }
    int testMedianY() const { return medianY; }
    float H() const { return height; }
    float W() const { return width; }
    float Ang() const { return angle; }
    unsigned int seen() const { return seenFlags; }

private:
    //================================================================
    // COSTANTI DI TUNING FILTRO KALMAN (Pos/Vel) – version 18
    //================================================================
    static constexpr float base_kf_X_MIN              = 0.0f;
    static constexpr float base_kf_X_MAX              = static_cast<float>(MouseMaxX * 3);
    static constexpr float base_kf_Y_MIN              = 0.0f;
    static constexpr float base_kf_Y_MAX              = static_cast<float>(MouseMaxY * 3);

    static constexpr float base_kf_X_CENTER           = (base_kf_X_MAX + base_kf_X_MIN) / 2.0f;
    static constexpr float base_kf_Y_CENTER           = (base_kf_Y_MAX + base_kf_Y_MIN) / 2.0f;
    static constexpr float base_kf_HALF_WIDTH         = (base_kf_X_MAX - base_kf_X_MIN) / 2.0f;
    static constexpr float base_kf_HALF_HEIGHT        = (base_kf_Y_MAX - base_kf_Y_MIN) / 2.0f;

    static constexpr float base_kf_INITIAL_P_POS_VALUE = 100.0f;
    static constexpr float base_kf_INITIAL_P_VEL_VALUE = 10.0f;

    static constexpr float base_kf_MIN_COVARIANCE_VALUE = 1e-6f;
    static constexpr float base_kf_MAX_P_VALUE          = 1e6f;

    static constexpr float base_kf_Q_MIN_PROCESS        = 0.2f;
    static constexpr float base_kf_Q_MAX_PROCESS        = 8.0f;
    static constexpr float base_kf_ACCEL_Q_THRESH_START = 5.0f;
    static constexpr float base_kf_ACCEL_Q_THRESH_END   = 50.0f;

    static constexpr float base_kf_R_MIN                = 0.05f;
    static constexpr float base_kf_R_MAX                = 1000.0f;
    static constexpr float base_kf_ACCEL_R_THRESH_START = 10.0f;
    static constexpr float base_kf_ACCEL_R_THRESH_END   = 100.0f;

    static constexpr float base_kf_R_X_EDGE_SMOOTH_START = 0.4f;
    static constexpr float base_kf_R_X_EDGE_SMOOTH_END   = 1.0f;
    static constexpr float base_kf_R_Y_EDGE_SMOOTH_START = 0.4f;
    static constexpr float base_kf_R_Y_EDGE_SMOOTH_END   = 1.0f;

    static constexpr float base_kf_R_AT_X_EDGE          = 2000.0f;
    static constexpr float base_kf_R_AT_Y_EDGE          = 2000.0f;
    static constexpr float base_kf_R_AT_X_EDGE_FOR_Y    = 500.0f;
    static constexpr float base_kf_R_AT_Y_EDGE_FOR_X    = 500.0f;

    static constexpr float base_kf_R_CROSS_AXIS_INFLUENCE_X = 0.2f;
    static constexpr float base_kf_R_CROSS_AXIS_INFLUENCE_Y = 0.2f;

    //================================================================
    // STATO INTERNO FILTRO KALMAN (Pos/Vel)
    //================================================================
    static float base_kf_x_state[4][2];
    static float base_kf_y_state[4][2];

    static float base_kf_p_x_00[4], base_kf_p_x_01[4];
    static float base_kf_p_x_10[4], base_kf_p_x_11[4];
    static float base_kf_p_y_00[4], base_kf_p_y_01[4];
    static float base_kf_p_y_10[4], base_kf_p_y_11[4];

    static float base_kf_last_measured_x[4];
    static float base_kf_last_measured_y[4];
    static float base_kf_last_vx_raw[4];
    static float base_kf_last_vy_raw[4];

    static bool base_kf_is_initialized_all_points;

    //================================================================
    // STATO DEL TRACKER
    //================================================================
    int  FinalX[4] = {400 * CamToMouseMult, 623 * CamToMouseMult, 400 * CamToMouseMult, 623 * CamToMouseMult};
    int  FinalY[4] = {200 * CamToMouseMult, 200 * CamToMouseMult, 568 * CamToMouseMult, 568 * CamToMouseMult};
    int  medianX = MouseMaxX / 2;
    int  medianY = MouseMaxY / 2;
    unsigned int see[4];
    float height;
    float width;
    float angle;

    unsigned int start = 0;
    unsigned int seenFlags = 0;

    //================================================================
    // METODI PRIVATI
    //================================================================
    void Kalman_filter_base();
};

#endif // _OpenFIRE_Square_Advanced_h_
#endif //USE_SQUARE_ADVANCED