#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
修复OpenFIREdisplay.cpp中Screen_Docked的0.91寸屏幕显示问题
将"Open FIER"改为"Open FIRE"
"""

def fix_docked_display():
    # Read the file
    with open('src/OpenFIREdisplay.cpp', 'r', encoding='utf-8') as f:
        content = f.read()

    # Define the old and new content - fix the typo "FIER" to "FIRE"
    old_content = '            display->print("Open FIER");'
    new_content = '            display->print("Open FIRE");'

    # Perform the replacement
    if old_content in content:
        new_file_content = content.replace(old_content, new_content)
        with open('src/OpenFIREdisplay.cpp', 'w', encoding='utf-8') as f:
            f.write(new_file_content)
        print("Successfully fixed Screen_Docked display - corrected 'FIER' to 'FIRE'!")
        return True
    else:
        print("Pattern not found in file.")
        return False

if __name__ == "__main__":
    fix_docked_display()