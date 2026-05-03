#ifdef USE_PERSPECTIVE_ADVANCED
/*!
 * @file OpenFIRE_Perspective_Advanced.h
 * @brief Light Gun library for 4 LED setup
 * @n CPP file for Samco Light Gun 4 LED setup
 *
 * @copyright alessandro-satanassi, https://github.com/alessandro-satanassi, 2026
 * @copyright GNU Lesser General Public License
 *
 * @author [Alessandro Satanassi](alessandro@cittini.it)
 * @version V2.0
 * @date 2026
 * 
 * I thank you for producing the first original code:
 * 
 * @copyright Samco, https://github.com/samuelballantyne, 2024
 * @copyright GNU Lesser General Public License
 *
 * @author [Sam Ballantyne](samuelballantyne@hotmail.com)
 * @version V1.0
 * @date 2024
 * Derived from Wiimote Whiteboard library:
 * Copyright 2021 88hcsif
 * Copyright (c) 2008 Stephane Duchesneau
 * by Stephane Duchesneau <stephane.duchesneau@gmail.com>
 * Ported from Johnny Lee's C# WiiWhiteboard project (Warper.cs file)
 */


#ifndef OpenFIRE_Perspective_Advanced_h
#define OpenFIRE_Perspective_Advanced_h

#include "OpenFIREConst.h" 

// ==============================================================================
// PARAMETRI FISICI DELLA LIGHTGUN
// ==============================================================================
// Valori di calibrazione di default per l'ottica. k1 compensa la distorsione a barilotto 
// tipica delle lenti grandangolari economiche delle telecamere IR.
#define DEFAULT_LENS_K1 0.006f       

// Regola l'altezza della mira (asse Y) in base alla distanza del giocatore dalla TV.
// Serve a compensare la differenza fisica di altezza tra la canna della pistola 
// e il sensore ottico posizionato al suo interno (o sopra).
#define DEFAULT_PARALLAX_FACTOR 0.0f  

// Costanti matematiche assolute (Calcolate dal compilatore, costo zero su ESP32).
// Normalizzano le coordinate in un range tra 0 e 1 prima dei calcoli matriciali.
// È vitale per prevenire l'overflow aritmetico e la perdita di precisione catastrofica 
// durante le moltiplicazioni incrociate nel calcolo della matrice prospettica.
static constexpr float NORM_SCALE = 10000.0f;  
static constexpr float INV_NORM_SCALE = 1.0f / 10000.0f; 

static constexpr float CX = (float)MouseResX * 0.5f;
static constexpr float CY = (float)MouseResY * 0.5f;
static constexpr float INV_CX = 1.0f / CX;
static constexpr float INV_CY = 1.0f / CY;
// ==============================================================================

class OpenFIRE_Perspective {

private:
  bool init = false;
  
  // Matrici lineari (1D) da 9 elementi invece che [3][3]. 
  // Migliora drasticamente il caching della CPU e permette cicli di accesso 
  // diretti in memoria contigua durante le pesanti moltiplicazioni matriciali.
  float srcmatrix[9];
  float dstmatrix[9];
  float warpmatrix[9];

  float srcX = CX;
  float srcY = CY;

  float k1 = DEFAULT_LENS_K1;              
  float parallaxFactor = DEFAULT_PARALLAX_FACTOR;  
  
  // Tracciamento dell'area per il parallasse. L'area calcolata viene usata come proxy 
  // affidabile e leggero (al posto della trigonometria) per stimare i cambiamenti 
  // di distanza (asse Z) del giocatore rispetto allo schermo.
  float baseArea = 0.0f;        
  float smoothedArea = 0.0f; 

  int dstX = 0;
  int dstY = 0;

  inline void applyLensCorrection(float &x, float &y);
  inline float calculateQuadArea(float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3);

public:
  void warp(int x0, int y0, int x1, int y1, int x2, int y2, int x3, int y3, float dx0, float dy0, float dx1, float dy1, float dx2, float dy2, float dx3, float dy3);
  void source(float adjustedX, float adjustedY);
  
  void setLensCorrection(float coefficientK1);
  void setDynamicParallax(float hardwareOffset); 
  
  void deinit(bool set);
  int getX();
  int getY();
};

#endif

#endif // USE_PERSPECTIVE_ADVANCED