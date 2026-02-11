# 0.91寸屏幕显示优化方案

## 问题分析

当前0.91寸屏幕(128x32)显示生命和弹药时只显示上半部分的原因：

1. **显示区域划分错误**:
   - 当前代码将生命值显示放在Y=16-31区域（下半部分）
   - 但弹药显示仍在Y=0-15区域（上半部分）
   - 导致生命值信息被截断或完全看不到

2. **字体尺寸过大**:
   - `NUMBER_GLYPH_WIDTH=21, NUMBER_GLYPH_HEIGHT=36` 对32像素高度来说太大
   - 心形图标尺寸也不适合小屏幕

3. **布局冲突**:
   - 上半部分显示弹药，下半部分显示生命值
   - 但实际显示时可能重叠或超出屏幕边界

## 优化方案

### 1. 修改宏定义（在OpenFIREdisplay.h中）

```cpp
// 在现有定义后添加0.91寸优化定义
#ifdef OLED_091_INCH
    // 0.91寸屏幕优化布局
    #define AMMO_DISPLAY_AREA_Y 0        // 弹药显示在顶部
    #define AMMO_DISPLAY_HEIGHT 16       // 弹药区域高度
    #define LIFE_DISPLAY_AREA_Y 16       // 生命值显示在底部
    #define LIFE_DISPLAY_HEIGHT 16        // 生命值区域高度
    
    // 缩小字体定义
    #define COMPACT_NUMBER_WIDTH 12      // 缩小数字宽度
    #define COMPACT_NUMBER_HEIGHT 16     // 缩小数字高度
    #define COMPACT_HEART_WIDTH 8       // 缩小心形宽度
    #define COMPACT_HEART_HEIGHT 8      // 缩小心形高度
    
    // 优化的显示位置
    #define AMMO_SINGLE_POS_X 30        // 单人弹药X位置
    #define AMMO_DUAL_POS_X 60         // 双人弹药X位置
    #define LIFE_BAR_POS_X 30           // 生命条X位置
    #define HEART_POS_X 80              // 心形X位置
#endif
```

### 2. 修改PrintAmmo函数

```cpp
void ExtDisplay::PrintAmmo(const uint &ammo)
{
    if(display != nullptr) {
        currentAmmo = ammo;
        
        #ifdef OLED_091_INCH
            // 0.91寸优化：使用顶部16像素高度
            display->fillRect(0, AMMO_DISPLAY_AREA_Y, SCREEN_WIDTH, AMMO_DISPLAY_HEIGHT, BLACK);
            
            uint ammoLeft = ammo / 10;
            uint ammoRight = ammo - (ammoLeft * 10);
            
            // 使用缩小的数字字体
            display->setTextSize(1);
            display->setTextColor(WHITE, BLACK);
            display->setCursor(AMMO_SINGLE_POS_X, AMMO_DISPLAY_AREA_Y + 2);
            display->print(ammo);
            
            // 添加"A"标识
            display->setCursor(AMMO_SINGLE_POS_X + 20, AMMO_DISPLAY_AREA_Y + 2);
            display->print("A");
            
        #else
            // 原有逻辑（0.96寸屏幕）
            uint ammoLeft = ammo / 10;
            uint ammoRight = ammo - (ammoLeft * 10);
            // ... 原有代码
        #endif
        
        display->display();
    }
}
```

### 3. 修改PrintLife函数

```cpp
void ExtDisplay::PrintLife(const uint &life)
{
    if(display != nullptr) {
        currentLife = life;
        lifeEmpty = life ? false : true;
        
        #ifdef OLED_091_INCH
            // 0.91寸优化：使用底部16像素高度
            display->fillRect(0, LIFE_DISPLAY_AREA_Y, SCREEN_WIDTH, LIFE_DISPLAY_HEIGHT, BLACK);
            
            if(lifeBar) {
                // 缩小的生命条
                uint8_t barWidth = (life > 5) ? 60 : (life * 12);
                display->fillRect(LIFE_BAR_POS_X, LIFE_DISPLAY_AREA_Y + 6, barWidth, 4, WHITE);
                
                // 显示数字
                display->setTextSize(1);
                display->setTextColor(WHITE, BLACK);
                display->setCursor(LIFE_BAR_POS_X + barWidth + 2, LIFE_DISPLAY_AREA_Y + 2);
                display->print(life);
            } else {
                // 缩小心形图标
                uint8_t heartCount = (life > 4) ? 4 : life;
                for(uint8_t i = 0; i < heartCount; i++) {
                    // 简单的心形绘制（使用字符或小位图）
                    display->setCursor(HEART_POS_X + i * 10, LIFE_DISPLAY_AREA_Y + 4);
                    display->print("<3"); // 使用心形字符
                }
                
                // 显示数字
                if(life > 0) {
                    display->setCursor(HEART_POS_X + heartCount * 10 + 5, LIFE_DISPLAY_AREA_Y + 4);
                    display->print(life);
                }
            }
            
        #else
            // 原有逻辑（0.96寸屏幕）
            // ... 保持原有代码不变
        #endif
        
        display->display();
    }
}
```

### 4. 修改Screen_Normal模式的显示逻辑

```cpp
case Screen_Normal:
    #ifdef OLED_091_INCH
        // 0.91寸优化：上下分区显示
        // 上半部分：弹药信息
        PrintAmmo(currentAmmo);
        // 下半部分：生命值信息
        PrintLife(currentLife);
    #else
        // 原有逻辑
        // ... 保持原有代码
    #endif
    break;
```

## 实施步骤

1. **修改OpenFIREdisplay.h**:
   - 在现有宏定义后添加0.91寸优化宏
   - 不需要新的函数声明，使用现有函数

2. **修改OpenFIREdisplay.cpp**:
   - 在PrintAmmo函数中添加`#ifdef OLED_091_INCH`分支
   - 在PrintLife函数中添加`#ifdef OLED_091_INCH`分支
   - 修改Screen_Normal case的显示逻辑

3. **编译测试**:
   - 在platformio.ini中添加`-D OLED_091_INCH`编译标志
   - 测试0.91寸屏幕显示效果

## 优势

1. **向后兼容**: 不影响0.96寸屏幕的现有功能
2. **条件编译**: 通过宏定义控制，代码简洁
3. **空间优化**: 充分利用32像素高度
4. **清晰可见**: 确保生命和弹药信息都完整显示

这个方案通过宏定义实现了条件编译，既解决了0.91寸屏幕的显示问题，又保持了代码的整洁性和兼容性。
