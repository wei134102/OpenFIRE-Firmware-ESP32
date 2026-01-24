#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
修复OpenFIREdisplay.cpp中ScreenPause_LowButtonToggle的0.91寸屏幕显示问题
"""

def fix_low_button_toggle_display():
    # Read the file
    with open('src/OpenFIREdisplay.cpp', 'r', encoding='utf-8') as f:
        content = f.read()

    # Define the old and new content
    old_content = '''            #ifdef OLED_091_INCH
            // 0.91寸屏幕：只显示当前选中的项目
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 20);
            display->println(" Mode Change ");
            #else'''
    
    new_content = '''            #ifdef OLED_091_INCH
            // 0.91寸屏幕：只显示当前选中的项目
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 20);
            if(OF_Prefs::toggles[OF_Const::lowButtonsMode]) {
              display->println(" Low Button: ON ");
            } else {
              display->println(" Low Button: OFF ");
            }
            #else'''

    # Perform the replacement
    if old_content in content:
        new_file_content = content.replace(old_content, new_content)
        with open('src/OpenFIREdisplay.cpp', 'w', encoding='utf-8') as f:
            f.write(new_file_content)
        print("Successfully fixed ScreenPause_LowButtonToggle display issue!")
        return True
    else:
        print("Pattern not found in file.")
        return False

if __name__ == "__main__":
    fix_low_button_toggle_display()