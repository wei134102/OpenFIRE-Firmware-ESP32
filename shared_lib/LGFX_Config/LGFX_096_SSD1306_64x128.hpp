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

// adattarlo al codice già scritto di OpenFire definizione dei colori
#define BLACK TFT_BLACK   ///< Draw 'off' pixels
#define WHITE TFT_WHITE   ///< Draw 'on' pixels

class LGFX_SSD1306 : public lgfx::LGFX_Device
{
  lgfx::Panel_SSD1306 _panel_instance; // Pannello SSD1306
  lgfx::Bus_I2C _bus_instance;         // Bus I2C
  uint16_t _height;                     // 保存高度用于后续配置

public:
  LGFX_SSD1306(uint8_t i2c_port, int16_t sda, int16_t scl, uint8_t i2c_addr, uint16_t width, uint16_t height) //支持128x64(0.96寸)和128x32(0.91寸)
  { 
    _height = height;  // 保存高度
    
    {   // configurazione del bus I2C
        auto cfg = _bus_instance.config();
        cfg.i2c_port = i2c_port;  // Porta I2C (0 o 1)
        cfg.pin_sda = sda;        // Pin SDA dinamico
        cfg.pin_scl = scl;        // Pin SCL dinamico
        cfg.i2c_addr = i2c_addr;  // Indirizzo del display (es. 0x3C o 0x3D)
        //cfg.freq_write = 400000; //800000;  // Imposta la frequenza a 800kHz (800.000) // defoult se non si imposta è 400khz
        cfg.freq_write  = 400000;
        cfg.freq_read   = 400000;
        _bus_instance.config(cfg);
        _panel_instance.setBus(&_bus_instance);
    }

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
  
  // 配置SSD1306驱动参数的方法，应在init()之后调用
  // 根据屏幕高度配置SSD1306驱动参数
  // 0.96寸(128x64): 驱动路数0x3F(64 duty), com pin配置0x12
  // 0.91寸(128x32): 驱动路数0x1F(32 duty), com pin配置0x02
  void configureSSD1306Params() {
    if (_height == 32) {
      // 0.91寸OLED配置
      // 通过I2C总线发送命令：0xA8 0x1F (设置驱动路数为32)
      // 然后发送：0xDA 0x02 (设置com pin配置)
      uint8_t cmd1[] = {0xA8, 0x1F};
      uint8_t cmd2[] = {0xDA, 0x02};
      _bus_instance.writeBytes(cmd1, 2, false, true);
      delay(1);
      _bus_instance.writeBytes(cmd2, 2, false, true);
    }
    // 0.96寸使用默认配置，LovyanGFX应该已经自动配置了
  }
};
