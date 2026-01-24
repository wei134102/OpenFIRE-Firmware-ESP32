#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
修复OpenFIREdisplay.cpp中ScreenPause_ModeChange的0.91寸屏幕显示问题
"""

def fix_modechange_display():
    # Read the file
    with open('src/OpenFIREdisplay.cpp', 'r', encoding='utf-8') as f:
        content = f.read()

    # Define the old content (incorrect Low Button display in ModeChange)
    old_content = '''            #ifdef OLED_091_INCH
            // 0.91寸屏幕：只显示当前选中的项目
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 20);
            if(OF_Prefs::toggles[OF_Const::lowButtonsMode]) {
              display->println(" Low Button: ON ");
            } else {
              display->println(" Low Button: OFF ");
            }
            #else'''

    # Define the correct content (should display Mode Change in ModeChange)
    new_content = '''            #ifdef OLED_091_INCH
            // 0.91寸屏幕：只显示当前选中的项目
            display->setTextColor(BLACK, WHITE);
            display->setCursor(0, 20);
            display->println(" Mode Change ");
            #else'''

    # Perform the replacement
    if old_content in content:
        new_file_content = content.replace(old_content, new_content)
        with open('src/OpenFIREdisplay.cpp', 'w', encoding='utf-8') as f:
            f.write(new_file_content)
        print("Successfully fixed ScreenPause_ModeChange display issue!")
        return True
    else:
        print("Pattern not found in file.")
        # Let's try to find what's actually there
        import re
        # Find the ModeChange case
        modechange_pattern = r'case ScreenPause_ModeChange:.*?#endif.*?break;'
        matches = re.findall(modechange_pattern, content, re.DOTALL)
        if matches:
            print("Found ModeChange section, but exact pattern didn't match")
            for match in matches:
                if 'OLED_091_INCH' in match:
                    print("ModeChange section with OLED_091_INCH found:")
                    print(match[:200] + "...")
        return False

if __name__ == "__main__":
    fix_modechange_display()