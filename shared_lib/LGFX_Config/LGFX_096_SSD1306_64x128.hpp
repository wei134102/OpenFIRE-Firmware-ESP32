/*!
 * @file LGFX_096_SSD1306_64x128.hpp
 * @brief Library for LGFX_096_SSD1306_64x128
 * @n CPP Library for LGFX_096_SSD1306_64x128
 *
 * @copyright alessandro-satanassi, https://github.com/alessandro-satanassi, 2026
 * @copyright GNU Lesser General Public License
 *
 * @author [Alessandro Satanassi](alessandro@cittini.it)
 * @version V1.0
 * @date 2026
 */

#pragma once

// ===================================================================================
// CONTRACT API: ASTRAZIONE COLORI MONOCROMATICI
// ===================================================================================
// Mappiamo i concetti binari (acceso/spento) sui valori colore a 16-bit di LovyanGFX.
// Questo garantisce che il codice di alto livello (es. OpenFire) possa disegnare 
// interfacce senza doversi preoccupare se l'hardware sottostante è un OLED o un TFT.
// adattarlo al codice già scritto di OpenFire definizione dei colori
#define BLACK TFT_BLACK   ///< Draw 'off' pixels
#define WHITE TFT_WHITE   ///< Draw 'on' pixels

// ===================================================================================
// DEFINIZIONE DEVICE: APPROCCIO A RUNTIME (I2C)
// ===================================================================================
// A differenza dei TFT ad alte prestazioni, configuriamo l'OLED con un costruttore 
// dinamico. Questo permette di scansionare e riassegnare i pin I2C a runtime, 
// garantendo massima flessibilità per display secondari "plug & play".
class LGFX_SSD1306 : public lgfx::LGFX_Device
{
  lgfx::Panel_SSD1306 _panel_instance; // Pannello SSD1306
  lgfx::Bus_I2C _bus_instance;         // Bus I2C

public:
  LGFX_SSD1306(uint8_t i2c_port, int16_t sda, int16_t scl, uint8_t i2c_addr, uint16_t width, uint16_t height) //IN AUTOMATICO DAL PANNELLO 128X64
  { 
    // ===================================================================================
    // HARDWARE LAYER: BUS I2C
    // ===================================================================================
    {   // configurazione del bus I2C
        auto cfg = _bus_instance.config();
        cfg.i2c_port = i2c_port;  // Porta I2C (0 o 1)
        cfg.pin_sda = sda;        // Pin SDA dinamico
        cfg.pin_scl = scl;        // Pin SCL dinamico
        cfg.i2c_addr = i2c_addr;  // Indirizzo del display (es. 0x3C o 0x3D)
        
        // Fissiamo il bus a 400kHz (Fast Mode). È il limite superiore garantito dalle 
        // specifiche I2C standard per stabilità in presenza di cavi/interferenze,
        // sufficiente per il volume di dati di un display 128x64 monocromatico.
        //cfg.freq_write = 400000; //800000;  // Imposta la frequenza a 800kHz (800.000) // defoult se non si imposta è 400khz
        cfg.freq_write  = 400000;
        cfg.freq_read   = 400000;
        _bus_instance.config(cfg);
        _panel_instance.setBus(&_bus_instance);
    }

    // ===================================================================================
    // PRESENTATION LAYER: PANNELLO OLED
    // ===================================================================================
    {   // configurazione del pannello
        auto cfg = _panel_instance.config();
        cfg.panel_width = width;
        cfg.panel_height = height;
        //cfg.offset_x = 0; // se necessario
        //cfg.offset_y = 0; // se necessario
        //cfg.invert = true;
        //cfg.rgb_order = false;
        //cfg.offset_rotation = 2;
       
       _panel_instance.setRotation(2); // 2 = 180 GRADI
       _panel_instance.setBrightness(255);

       //cfg.pin_rst = -1; // pin reset opzionale (-1 = disable)
       //cfg.pin_bl = 69; //pin backlight (se presente)  ????? esiste ???
       //cfg.pin_cs = -1; // pin clable select (se presente) (-1 = disable)
       // cfg.pin_busy = -1; // (-1 = disable)


       _panel_instance.config(cfg);
    }

    setPanel(&_panel_instance); // Associa il pannello alla classe
  }
};