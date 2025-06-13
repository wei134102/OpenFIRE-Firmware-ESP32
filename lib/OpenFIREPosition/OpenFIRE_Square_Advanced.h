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
};

#endif // _OpenFIRE_Square_Advanced_h_
#endif //USE_SQUARE_ADVANCED