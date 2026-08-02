# -*- coding: utf-8 -*-
"""Generate OpenFIRE Gun + Dongle user manuals (Chinese OLED_MENU_ZH / English)."""
from __future__ import annotations

import argparse
from pathlib import Path

from docx import Document
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml.ns import qn
from docx.shared import Cm, Pt

DOCS_DIR = Path(__file__).resolve().parent
OUT_ZH = DOCS_DIR / "OpenFIRE光枪与Dongle使用说明书.docx"
OUT_EN = DOCS_DIR / "OpenFIRE_Gun_Dongle_Manual_EN.docx"


def set_font(run, name: str = "Calibri", size: int = 11, east_asia: str | None = None):
    run.font.name = name
    run.font.size = Pt(size)
    if east_asia:
        run._element.rPr.rFonts.set(qn("w:eastAsia"), east_asia)


def add_title(doc, text, level=0, font="Calibri", east_asia=None):
    if level == 0:
        p = doc.add_paragraph()
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        r = p.add_run(text)
        set_font(r, font, 22, east_asia)
        r.bold = True
    else:
        doc.add_heading(text, level=level)


def add_p(doc, text, bold=False, font="Calibri", east_asia=None):
    p = doc.add_paragraph()
    r = p.add_run(text)
    set_font(r, font, 11, east_asia)
    r.bold = bold
    return p


def add_bullets(doc, items, font="Calibri", east_asia=None):
    for item in items:
        p = doc.add_paragraph(item, style="List Bullet")
        for r in p.runs:
            set_font(r, font, 11, east_asia)


def add_table(doc, headers, rows, font="Calibri", east_asia=None):
    t = doc.add_table(rows=1 + len(rows), cols=len(headers))
    t.style = "Table Grid"
    hdr = t.rows[0].cells
    for i, h in enumerate(headers):
        hdr[i].text = h
        for p in hdr[i].paragraphs:
            for r in p.runs:
                set_font(r, font, 10, east_asia)
                r.bold = True
    for ri, row in enumerate(rows):
        cells = t.rows[ri + 1].cells
        for ci, val in enumerate(row):
            cells[ci].text = str(val)
            for p in cells[ci].paragraphs:
                for r in p.runs:
                    set_font(r, font, 10, east_asia)
    doc.add_paragraph()


def _init_doc():
    doc = Document()
    sec = doc.sections[0]
    sec.top_margin = Cm(2.5)
    sec.bottom_margin = Cm(2.5)
    sec.left_margin = Cm(2.8)
    sec.right_margin = Cm(2.8)
    return doc


def build_zh():
    font = "微软雅黑"
    doc = _init_doc()

    add_title(doc, "OpenFIRE 光枪与无线 Dongle\n详细使用说明书", east_asia=font)
    add_p(doc, "适用固件：OpenFIRE-Firmware-ESP32（ESP32-S3 光枪 + ESP-NOW Dongle，中文 OLED 菜单版）", east_asia=font)
    add_p(doc, "配套配置软件：OpenFIRE-App-cn（可选）", east_asia=font)
    add_p(doc, "文档版本：1.2  |  编写日期：2026年6月", east_asia=font)
    add_p(doc, "说明：本版说明书与固件 OLED_MENU_ZH 中文界面一一对应；表中「OLED 显示」列为屏幕上实际文字。", east_asia=font)
    doc.add_paragraph()

    add_title(doc, "一、产品概述", 1)
    add_p(doc,
          "OpenFIRE 是一套基于四红外点（4IR）定位的光枪系统。ESP32-S3 版本支持两种与 PC 连接方式："
          "① USB 有线直连；② 通过 ESP-NOW 无线 Dongle 连接。无线模式下，PC 识别到的设备与有线连接相同，"
          "游戏与力反馈软件无需额外适配。", east_asia=font)
    add_p(doc, "本说明书涵盖：光枪（GUN）日常操作、中文 OLED 暂停菜单、校准、多玩家编号；"
          "无线接收器（Dongle）与无线踏板的配对、屏幕状态与故障排除。", east_asia=font)

    add_title(doc, "1.1 包装与配件（参考）", 2)
    add_bullets(doc, [
        "光枪本体（含 IR 摄像头、按键、OLED、可选震动/电磁阀等）",
        "USB 数据线（用于有线连接或充电/刷机）",
        "无线 Dongle（ESP32-S3，插 PC USB，SSD1306/TFT 型号显示中文配对状态）",
        "IR 发射器条（需安装在显示器上下或四边，依方形/菱形布局而定）",
        "（可选）无线踏板模块，用于 Time Crisis 类游戏的掩护操作",
    ], east_asia=font)

    add_title(doc, "1.2 安全提示", 2)
    add_bullets(doc, [
        "首次开机及摇杆自动校准时，约 2 秒内请勿触碰模拟摇杆。",
        "电磁阀（Solenoid）工作时会有机械冲击与热量，请勿将枪口对准人员或动物。",
        "长时间使用请关注 OLED 温度显示；超温保护触发时会限制力反馈输出。",
        "刷机、改线请在断电状态下进行，并核对板型引脚文档（docs/BOARDS_PIN.md）。",
    ], east_asia=font)

    add_title(doc, "二、首次开机与连接", 1)
    add_title(doc, "2.1 启动顺序（必读）", 2)
    add_p(doc, "光枪上电后按以下顺序自动执行（OLED 中文提示对照）：", east_asia=font)
    add_table(doc,
              ["阶段", "OLED 显示（中文）", "说明"],
              [
                  ["摇杆自动校准", "校准就绪", "约 2 秒，请勿动摇杆"],
                  ["USB 检测", "USB模式 → 等待USB", "判断有线或无线"],
                  ["连接成功", "连接就绪 / USB已连接", "有线或无线握手完成"],
                  ["首次屏幕校准", "欢迎! / 扣扳机 / 扣扳机开始校准", "未校准时按扳机进入五点校准"],
              ], east_asia=font)

    add_title(doc, "2.2 有线 USB 连接", 2)
    add_bullets(doc, [
        "用 USB 线连接光枪与 PC。",
        "等待 Windows 识别 HID 设备（设备名通常为 FIRECon P1～P4，取决于枪号设置）。",
        "OLED 顶部可能显示 USB已连接；左下角显示 USB 图标。",
        "可用 OpenFIRE-App-cn 连接 COM 口进行高级配置（需进入 Docked 模式，见第十章）。",
    ], east_asia=font)

    add_title(doc, "2.3 无线 Dongle 连接（推荐顺序）", 2)
    add_bullets(doc, [
        "步骤 1：将 Dongle 插入 PC USB，等待其完成启动（OLED 显示 WiFi扫描 / 优选信道 / 请稍候，约 10～15 秒）。",
        "步骤 2：光枪不要插 PC USB，然后打开光枪电源。",
        "步骤 3：Dongle 显示 信道xx配对 / 请开光枪 / ESP-NOW监听 时，光枪应开始广播配对。",
        "光枪 OLED 顶部可能显示 信道xx等待。",
        "步骤 4：配对成功后 Dongle 显示 射频已连接 → 玩家 / 信道 / 枪 MAC；PC 出现与光枪配置一致的设备。",
        "步骤 5：若有无线踏板，Dongle 可能短暂显示 等待踏板；光枪搜索踏板时显示 踏台搜索xx秒。",
        "步骤 6：OLED 左下角显示 Wi-Fi 图标表示无线模式。",
        "再次开机时，光枪会优先连接上次配对的 Dongle；若失败则重新全信道搜索。",
    ], east_asia=font)

    add_title(doc, "2.4 玩具模式（无 PC 试玩）", 2)
    add_p(doc,
          "无线未连接且 USB 未挂载时，上电约 3 秒内按住扳机可进入玩具模式："
          "OLED 显示 玩具模式?Xs 倒计时，就绪后显示 玩具模式就绪。"
          "可在无 PC 情况下本地试玩电磁阀等功能（具体取决于硬件配置）。", east_asia=font)

    add_title(doc, "三、OLED 状态栏说明（运行界面）", 1)
    add_table(doc,
              ["区域", "OLED 显示示例", "含义"],
              [
                  ["顶部左侧", "1 A ST 42C", "玩家编号 1～4、配置槽 A～D、布局 ST/SC、温度"],
                  ["顶部右侧", "关 / 计时5分", "游戏计时：关=未启用；数字=剩余秒/分"],
                  ["顶部整行", "配置: ProfileA P1", "暂停/切换配置时显示"],
                  ["左下", "USB / Wi-Fi 图标", "有线或无线 Dongle"],
                  ["中下", "RF / LOW / AF", "震动FFB、少键模式、连发（缩写）"],
                  ["右下", "鼠标 / 手柄 / MiSTer 图标", "当前 USB 输出模式"],
              ], east_asia=font)
    add_p(doc, "布局缩写：ST = 菱形（Diamond），SC = 方形（Square）。"
          "操作反馈时顶部栏会短暂显示中文，例如：模式:手柄、连发:开、少键:关、震动FFB:开、已发Esc、游戏时间到 等。", east_asia=font)

    add_title(doc, "四、暂停菜单（中文 OLED）", 1)
    add_p(doc,
          "暂停菜单用于切换配置、校准、改 USB 模式、设置枪号、保存设置等。"
          "系统有两种交互方式，由 simplePause 开关决定（默认一般为 Direct Pause 热键模式）。", east_asia=font)

    add_title(doc, "4.1 进入与退出暂停", 2)
    add_table(doc,
              ["操作", "按键组合", "OLED 相关提示"],
              [
                  ["进入暂停", "Reload + Select（换弹 + 选择）", "使用 + 当前配置名"],
                  ["退出暂停", "Reload + Home（换弹 + 主页）", "恢复运行界面"],
                  ["长按进入暂停（需开启 Hold to Pause）", "Trigger + A 按住约 2.5 秒", "—"],
                  ["Simple 模式长按退出", "A + B 按住约一半 Hold to Pause 时间", "—"],
              ], east_asia=font)

    add_title(doc, "4.2 运行模式快捷键（无需进暂停）", 2)
    add_table(doc,
              ["功能", "按键组合", "OLED 提示"],
              [
                  ["发送 Esc 到 PC", "Reload + Start", "已发Esc"],
                  ["发送 Esc 到 PC（快捷）", "Trigger + A + C（扳机 + A + C 同时松开）", "已发Esc"],
                  ["鼠标模式 ↔ 手柄模式", "A + B + Trigger", "模式:键鼠 / 模式:手柄"],
              ], east_asia=font)
    add_p(doc, "注：C 键在固件中对应 Gun C（Reload 按键位）。MiSTer 模式需在暂停菜单「模式切换」中循环选择。", east_asia=font)

    add_title(doc, "4.3 Direct Pause（默认热键模式）", 2)
    add_p(doc, "进入暂停后，OLED 四行显示 A> / B> / 始> / 选> 与四个配置名；多数功能通过组合键完成。", east_asia=font)
    add_table(doc,
              ["功能", "按键组合"],
              [
                  ["屏幕校准", "暂停内按 Trigger（扳机）"],
                  ["保存全部设置到 Flash", "Start + Select"],
                  ["IR 灵敏度 +", "B + Up"],
                  ["IR 灵敏度 −", "B + Down"],
                  ["运行模式 Normal", "Start + A"],
                  ["运行模式 Average / Average2", "Start + B"],
                  ["发送 Esc", "Reload + Start 或 Trigger+A+C"],
                  ["鼠标 ↔ 手柄", "A + B + Trigger"],
                  ["切换配置 A/B/Start/Select", "暂停界面按对应面键"],
                  ["校准中跳过中心步", "校准过程中按 A"],
              ], east_asia=font)

    add_title(doc, "4.4 Simple Pause（列表菜单模式）", 2)
    add_p(doc, "若已在 OpenFIRE App 中开启 Simple Pause：A=上一项，B=下一项，Trigger=确认执行。"
          "下列「OLED 显示」与固件中文菜单完全一致。", east_asia=font)
    add_table(doc,
              ["OLED 显示（中文）", "功能说明"],
              [
                  ["中心校准", "以当前位置为摇杆新中心（需配置模拟摇杆引脚）"],
                  ["摇杆校准", "约 4 秒摇杆画圈 + 回中（OLED 提示：摇杆:旋转 → 松开回中 → 摇杆:完成）"],
                  ["屏幕校准", "进入屏幕五点瞄准校准"],
                  ["配置选择", "选择 Profile A/B/Start/Select（子界面显示 A>/B>/始>/选>）"],
                  ["模式切换", "循环：键鼠 → 手柄 → MiSTer（当前:键鼠 / 当前:手柄 / 当前:MiSTer）"],
                  ["布局切换", "方形 ↔ 菱形（布局:方形 / 布局:菱形）"],
                  ["枪号 / 枪号P1-P4", "设置玩家 P1～P4，同步 USB PID 与设备名"],
                  ["摇杆模式", "摇杆输出：摇杆:手柄 / 摇杆:十字 / 摇杆:按键"],
                  ["死区", "摇杆死区 0～30%（死区:N%）"],
                  ["轴模式", "有符号 / 无符号轴"],
                  ["互换摇杆", "红外定位轴与物理摇杆左右互换"],
                  ["游戏计时", "0 / 5 / 10 / 15 / 20 分钟（计时:关 / 计时N分），到期显示 游戏时间到"],
                  ["保存设置", "保存配置与全局设置到 Flash（保存中... / 保存 成功）"],
                  ["震动开关", "电机震动总开关（震动:开 / 震动:关）"],
                  ["电磁开关", "电磁阀总开关（电磁:开 / 电磁:关）"],
                  ["连发开关", "连发模式（连发:开 / 连发:关）"],
                  ["少键模式", "低按键映射模式（少键:开 / 少键:关）"],
                  ["震动FFB开关", "是否用震动代替电磁响应串口力反馈（震动FFB:开 / 关 / 无）"],
                  ["反转X轴 / 反转Y轴", "模拟摇杆轴向反转"],
                  ["发送Esc", "向 PC 发送一次 Esc 键"],
              ], east_asia=font)
    add_p(doc, "注：无对应硬件的菜单项会自动跳过。枪号子菜单：A/B 选择 P1～P4，Trigger 保存，Reload+Home 取消。", east_asia=font)

    add_title(doc, "五、枪号与多光枪设置（P1～P4）", 1)
    add_bullets(doc, [
        "每只光枪可设为 P1、P2、P3、P4，对应 USB Product ID 1～4，设备名如 FIRECon P1。",
        "OLED 显示 枪号P1～P4 或顶部 1～4；Simple 菜单选 枪号；或 OpenFIRE-App-cn → Settings → USB Identity。",
        "修改后需 保存设置，并重新插拔 USB 后 Windows 才会识别新编号。",
        "多枪同屏：每只枪设不同枪号，各接不同 USB 口或使用多个 Dongle。",
    ], east_asia=font)

    add_title(doc, "六、USB 输出模式", 1)
    add_table(doc,
              ["模式", "OLED 显示", "典型用途"],
              [
                  ["键鼠（默认）", "当前:键鼠 / 模式:键鼠", "MAME、Demul、TeknoParrot 等"],
                  ["手柄", "当前:手柄 / 模式:手柄", "需要手柄协议的游戏"],
                  ["MiSTer", "当前:MiSTer / 模式:MiSTer", "MiSTer FPGA 平台"],
              ], east_asia=font)

    add_title(doc, "七、校准配置文件（Profile）", 1)
    add_p(doc, "系统内置 4 套 Profile，默认名称：Profile A、Profile B、Profile Start、Profile Select。", east_asia=font)
    add_p(doc, "OLED 运行栏显示 A～D 简写；配置选择界面显示 名称(方) / 名称(菱)。", east_asia=font)
    add_bullets(doc, [
        "Direct 暂停：按 OLED 上 A>/B>/始>/选> 对应键快速切换。",
        "Simple 暂停：「配置选择」子菜单。",
        "OpenFIRE App：Profiles 标签页编辑后 Confirm 保存。",
        "修改后务必「保存设置」或 Start+Select 写入 Flash，否则断电丢失。",
    ], east_asia=font)

    add_title(doc, "八、校准指南", 1)
    add_title(doc, "8.1 屏幕瞄准校准（必做）", 2)
    add_p(doc, "用于将 IR 坐标映射到显示器区域，首次使用或换显示器后必须执行。", east_asia=font)
    add_bullets(doc, [
        "触发：首次开机按扳机；暂停菜单「屏幕校准」；Direct 暂停按扳机；OpenFIRE App 校准窗口。",
        "OLED 校准界面顶部显示 校准: + 配置名。",
        "步骤：Top → Bottom → Left → Right → Center → Verify。",
        "取消：Reload+Home 恢复原校准数据。",
    ], east_asia=font)

    add_title(doc, "8.2 模拟摇杆校准", 2)
    add_table(doc,
              ["类型", "OLED 菜单/提示", "操作"],
              [
                  ["上电自动中心校准", "校准就绪", "每次开机约 2 秒，勿触碰摇杆"],
                  ["中心校准", "中心校准 → 中心已校", "将当前位置设为新中心"],
                  ["摇杆校准", "摇杆校准 → 摇杆:旋转/完成", "约 4 秒内画满圈后回中"],
              ], east_asia=font)

    add_title(doc, "8.3 IR 发射器安装", 2)
    add_bullets(doc, [
        "方形布局（SC）：发射器安装在显示器上下。",
        "菱形布局（ST）：发射器安装在显示器四边中点。",
        "OpenFIRE App：Open IR Emitter Alignment Assistant 可查看对齐示意图。",
    ], east_asia=font)

    add_title(doc, "九、无线 Dongle 详细说明", 1)
    add_title(doc, "9.1 Dongle 是什么", 2)
    add_p(doc,
          "Dongle 是插在 PC 上的 ESP32-S3 接收器，通过 ESP-NOW 2.4GHz 与光枪通信。"
          "配对完成后向 PC 呈现与光枪相同的 USB 身份，并转发 HID 与串口力反馈数据。", east_asia=font)

    add_title(doc, "9.2 屏幕状态对照（中文 OLED）", 2)
    add_table(doc,
              ["OLED 显示（中文）", "含义", "您应做什么"],
              [
                  ["OpenFIRE接收器", "启动 Logo", "等待"],
                  ["RF嗅探ch / WiFi扫描", "自动选择干扰最小信道", "勿拔 USB，约 10～15 秒"],
                  ["优选信道 / 请稍候", "信道评估中", "等待"],
                  ["信道xx配对 / 请开光枪 / ESP-NOW监听", "等待光枪配对", "打开光枪（勿插 PC USB）"],
                  ["射频已连接 / 等待数据流", "射频已连，等待 HID 流", "等待数秒"],
                  ["等待踏板", "等待无线踏板握手", "有踏板则打开；无踏板等待"],
                  ["玩家:N / 信道:N / 枪:MAC", "配对成功", "可在 PC 测试"],
              ], east_asia=font)

    add_title(doc, "9.3 无线踏板", 2)
    add_bullets(doc, [
        "踏板本体无 OLED，配对时由光枪 OLED 顶部显示 踏台搜索xx秒。",
        "Dongle 配对光枪后可能显示 等待踏板，直至踏板无线握手完成。",
    ], east_asia=font)

    add_title(doc, "十、OpenFIRE-App-cn 配置软件（可选）", 1)
    add_bullets(doc, [
        "用途：引脚映射、开关选项、Profile 参数、USB P1～P4、IR 校准、按键测试、力反馈测试。",
        "连接：选择 COM 口，App 发送 Dock 命令进入 Docked 模式后同步数据。",
        "保存：修改后点击 Confirm。",
    ], east_asia=font)

    add_title(doc, "十一、力反馈与 MAMEHOOKER", 1)
    add_bullets(doc, [
        "编译启用 MAMEHOOKER 时，光枪/Dongle 提供 9600 波特率串口通道。",
        "「震动FFB开关」控制是否用震动电机代替电磁阀响应串口力反馈（与「震动开关」不同）。",
        "「电磁开关」控制电磁阀硬件；「震动开关」控制电机震动硬件。",
    ], east_asia=font)

    add_title(doc, "十二、常见问题与故障排除", 1)
    add_table(doc,
              ["现象", "可能原因", "处理办法"],
              [
                  ["Dongle 一直 请开光枪/搜索中", "光枪未开、仍插 USB、距离过远", "先 Dongle 后开枪；拔枪 USB；靠近 PC"],
                  ["PC 设备名不变/仍是 P1", "枪号未保存或未重插 USB", "保存设置；重新插拔 USB"],
                  ["无线连上后断线", "2.4G 干扰", "缩短距离；减少同频段设备"],
                  ["瞄准偏移", "未校准或换显示器", "重做屏幕校准；检查 ST/SC 与发射器安装"],
                  ["力反馈无效", "COM 口错误或 FFB 关闭", "选 Dongle COM；开启震动FFB开关"],
              ], east_asia=font)

    add_title(doc, "附录 A：推荐日常流程速查", 1)
    add_p(doc, "【无线游玩】Dongle 接 PC → 光枪开机（不插 USB）→ 等 Wi-Fi 图标 → 进入游戏。", bold=True, east_asia=font)
    add_p(doc, "【改配置/校准】Reload+Select 进暂停 → 操作 → 保存设置 → Reload+Home 退出。", bold=True, east_asia=font)
    add_p(doc, "【发 Esc】Reload+Start 或 Trigger+A+C。", bold=True, east_asia=font)

    add_title(doc, "附录 B：相关资源", 1)
    add_bullets(doc, [
        "固件仓库：OpenFIRE-Firmware-ESP32",
        "配置 App：OpenFIRE-App-cn",
        "板型引脚：docs/BOARDS_PIN.md",
    ], east_asia=font)

    doc.save(OUT_ZH)
    print("Saved:", OUT_ZH)


def build_en():
    doc = _init_doc()

    add_title(doc, "OpenFIRE Light Gun & Wireless Dongle\nUser Manual")
    add_p(doc, "Firmware: OpenFIRE-Firmware-ESP32 (ESP32-S3 light gun + ESP-NOW Dongle, English OLED UI)")
    add_p(doc, "Optional configuration app: OpenFIRE Desktop App (Team OpenFIRE / OpenFIRE-App-cn)")
    add_p(doc, "Document version: 1.0  |  Date: June 2026")
    add_p(doc, "Note: OLED text in this manual matches the default English firmware strings "
          "(build without -D OLED_MENU_ZH).")
    doc.add_paragraph()

    add_title(doc, "1. Product Overview", 1)
    add_p(doc,
          "OpenFIRE is a four-point infrared (4IR) light gun system. The ESP32-S3 port supports two PC connection "
          "methods: (1) USB wired direct connection; (2) ESP-NOW wireless via a Dongle. In wireless mode the PC sees "
          "the same HID device as wired—no extra game or force-feedback adapters are required.")
    add_p(doc, "This manual covers: light gun (GUN) daily use, OLED pause menu, calibration, player IDs (P1–P4); "
          "wireless receiver (Dongle) and wireless pedal pairing, on-screen status, and troubleshooting.")

    add_title(doc, "1.1 Package contents (reference)", 2)
    add_bullets(doc, [
        "Light gun (IR camera, buttons, OLED, optional rumble/solenoid, etc.)",
        "USB cable (wired play, charging, or firmware flashing)",
        "Wireless Dongle (ESP32-S3, plugs into PC USB; SSD1306/TFT models show pairing status on OLED)",
        "IR emitter bar(s) (mount on monitor top/bottom for Square layout, or on all four sides for Diamond)",
        "(Optional) Wireless pedal module for Time Crisis–style cover mechanics",
    ])

    add_title(doc, "1.2 Safety", 2)
    add_bullets(doc, [
        "On boot and during auto stick center calibration (~2 s), do not touch the analog stick.",
        "Solenoid recoil produces mechanical impact and heat—never point the gun at people or animals.",
        "Monitor the temperature readout on the OLED during long sessions; overtemperature protection may limit FFB.",
        "Flash firmware or rewire only when powered off. See docs/BOARDS_PIN.md for board pin maps.",
    ])

    add_title(doc, "2. First Boot & Connection", 1)
    add_title(doc, "2.1 Boot sequence", 2)
    add_p(doc, "After power-on the gun runs this sequence automatically:")
    add_table(doc,
              ["Stage", "OLED text (English)", "Meaning"],
              [
                  ["Stick auto-cal", " CALIBRATION READY ", "~2 s—do not move stick"],
                  ["USB detection", " go in usb mode  →  wait for usb mount ", "Wired vs wireless path"],
                  ["Link ready", "  !! LINK READY !!  /  USB mounted ", "USB or wireless link up"],
                  ["First-time aim cal", "Welcome! / Pull trigger / start calibration!", "Pull trigger to start 5-point cal"],
              ])

    add_title(doc, "2.2 Wired USB", 2)
    add_bullets(doc, [
        "Connect the gun to the PC with USB.",
        "Wait for Windows to enumerate the HID device (name typically FIRECon P1–P4 depending on Gun ID).",
        "Top bar may show  USB mounted ; lower-left shows USB icon.",
        "Use the OpenFIRE Desktop App on the COM port for advanced settings (Docked mode—see Section 10).",
    ])

    add_title(doc, "2.3 Wireless Dongle (recommended order)", 2)
    add_bullets(doc, [
        "Step 1: Plug the Dongle into the PC USB port. Wait for startup (OLED: Scanning WiFi / best channel eval / Please wait, ~10–15 s).",
        "Step 2: Do NOT plug the gun into PC USB. Power on the gun.",
        "Step 3: When the Dongle shows Ch:xx Pair / Turn on lightgun / ESP-NOW listen, the gun should start pairing broadcast.",
        "Gun OLED top bar may show Ch:xx Waiting link.",
        "Step 4: After success the Dongle shows RF link OK → Player / Channel / Gun MAC; PC shows the gun’s configured device name.",
        "Step 5: With a wireless pedal, the Dongle may show AWAIT PEDAL; the gun shows Search Pedal [xxs] while searching.",
        "Step 6: Wi-Fi icon at lower-left indicates wireless mode.",
        "On subsequent boots the gun prefers the last paired Dongle; if that fails it rescans all channels.",
    ])

    add_title(doc, "2.4 Boy Mode (offline play without PC)", 2)
    add_p(doc,
          "If wireless is down and USB is not mounted, hold the trigger within ~3 s of power-on to enter Boy Mode: "
          "OLED shows Boy? BOOT/TRIG Xs countdown, then  BOY MODE READY . "
          "Allows local solenoid/rumble testing without a PC (hardware dependent).")

    add_title(doc, "3. OLED Status Bar (Gameplay Screen)", 1)
    add_table(doc,
              ["Area", "OLED example", "Meaning"],
              [
                  ["Top left", "P1: P_A Diam 42C", "Player ID, profile slot (P_A…P_D), layout Diam/Squa, temperature"],
                  ["Top right", "OFF / 300", "Play Timer: OFF = disabled; number = remaining seconds"],
                  ["Full top row", "Prof: ProfileA P1", "Shown when pausing or switching profiles"],
                  ["Lower left", "USB / Wi-Fi icon", "Wired or wireless Dongle"],
                  ["Lower center", "RF / LOW / AF", "Rumble FFB, Low Button mode, Autofire (abbreviations)"],
                  ["Lower right", "Mouse / Gamepad / MiSTer icon", "Current USB output mode"],
              ])
    add_p(doc, "Transient top-bar messages include: Mode Changed: Gamepad, Autofire: ON, Low Button: OFF, "
          "Rumble FFB: ON, Sent Escape Key!, Play time over, etc.")

    add_title(doc, "4. Pause Menu (English OLED)", 1)
    add_p(doc,
          "The pause menu switches profiles, runs calibration, changes USB mode, sets Gun ID, saves settings, etc. "
          "Two interaction styles exist, controlled by the simplePause option (default is usually Direct Pause hotkeys).")

    add_title(doc, "4.1 Enter / exit pause", 2)
    add_table(doc,
              ["Action", "Button combo", "OLED hint"],
              [
                  ["Enter pause", "Reload + Select", "Using  + current profile name"],
                  ["Exit pause", "Reload + Home", "Return to gameplay screen"],
                  ["Hold to pause (if Hold to Pause enabled)", "Trigger + A ~2.5 s", "—"],
                  ["Simple Pause hold exit", "A + B ~half Hold to Pause time", "—"],
              ])

    add_title(doc, "4.2 Runtime shortcuts (no pause menu)", 2)
    add_table(doc,
              ["Function", "Button combo", "OLED message"],
              [
                  ["Send Esc to PC", "Reload + Start", "Sent Escape Key!"],
                  ["Send Esc (quick)", "Trigger + A + C (release all together)", "Sent Escape Key!"],
                  ["Mouse/KB ↔ Gamepad", "A + B + Trigger", "Mode Changed: Mouse/KB / Gamepad"],
              ])
    add_p(doc, "Note: Gun C maps to the Reload button bit. MiSTer mode is selected via  Mode Change  in Simple Pause.")

    add_title(doc, "4.3 Direct Pause (default hotkey mode)", 2)
    add_p(doc, "In pause, four lines show  A > / B > / Str> / Sel>  with profile names; most actions use combos:")
    add_table(doc,
              ["Function", "Button combo"],
              [
                  ["Screen calibration", "Trigger while paused"],
                  ["Save all settings to Flash", "Start + Select"],
                  ["IR sensitivity +", "B + Up"],
                  ["IR sensitivity −", "B + Down"],
                  ["Run mode Normal", "Start + A"],
                  ["Run mode Average / Average2", "Start + B"],
                  ["Send Esc", "Reload + Start or Trigger+A+C"],
                  ["Mouse/KB ↔ Gamepad", "A + B + Trigger"],
                  ["Switch profile A/B/Start/Select", "Press matching face button on pause screen"],
                  ["Skip center step during cal", "Press A during calibration"],
              ])

    add_title(doc, "4.4 Simple Pause (scrolling list mode)", 2)
    add_p(doc, "If Simple Pause is enabled in the OpenFIRE App: A = previous item, B = next item, Trigger = confirm. "
          "Menu labels match firmware exactly:")
    add_table(doc,
              ["OLED label (English)", "Function"],
              [
                  [" Center Calibrate ", "Set current stick position as new center (requires analog pins)"],
                  [" Range Calibrate ", "~4 s stick rotation + release to center (RangeCal: rotate → Release to center → OK)"],
                  [" Calibrate ", "Enter 5-point screen aim calibration"],
                  [" Profile Select ", "Pick Profile A/B/Start/Select (sub-screen: A>/B>/Str>/Sel>)"],
                  [" Mode Change ", "Cycle Mouse/KB → Gamepad → MiSTer ( Current: … )"],
                  [" Layout Toggle ", "Square ↔ Diamond ( Layout: Square / Diamond )"],
                  [" Gun ID / Gun ID (P1-P4) ", "Set player P1–P4; updates USB PID and device name"],
                  [" Stick Output Mode ", "Stick: Gamepad / D-Pad / Keyboard Arrows"],
                  [" Deadzone ", "Stick deadzone 0–30% ( Deadzone: N% )"],
                  [" Axis Mode ", "Signed / Unsigned axis"],
                  [" Swap Sticks ", "Swap IR aim axis with physical stick L/R"],
                  [" Play Timer ", "0 / 5 / 10 / 15 / 20 min ( Play Timer: OFF / Play Timer: N min ); expiry: Play time over"],
                  [" Save Gun Settings ", "Write profiles & globals to Flash ( Saving... / Save successful )"],
                  [" Rumble Toggle ", "Master rumble motor ( Toggling Rumble ON/OFF )"],
                  [" Solenoid Toggle ", "Master solenoid ( Toggling Solenoid ON/OFF )"],
                  [" Autofire Toggle ", "Autofire ( Autofire: ON/OFF )"],
                  [" Low Button Toggle ", "Low-button mapping ( Low Button: ON/OFF )"],
                  [" Rumble FFB Toggle ", "Use rumble instead of solenoid for serial FFB ( Rumble FFB: ON/OFF/N/A )"],
                  [" Analog Invert X / Y ", "Invert stick axis"],
                  [" Send Escape Keypress", "Send one Esc key to PC"],
              ])
    add_p(doc, "Items without hardware are skipped automatically. Gun ID sub-menu: A/B pick P1–P4, Trigger saves, Reload+Home cancels.")

    add_title(doc, "5. Gun ID & Multiple Guns (P1–P4)", 1)
    add_bullets(doc, [
        "Each gun can be P1, P2, P3, or P4 (USB Product ID 1–4, device name e.g. FIRECon P1).",
        "Set via OLED  Gun ID: Pn , Simple Pause  Gun ID , or App → Settings → USB Identity.",
        "After changing Gun ID you must  Save Gun Settings  and replug USB for Windows to see the new ID.",
        "Multi-gun: assign unique IDs; use separate USB ports or one Dongle per gun.",
    ])

    add_title(doc, "6. USB Output Modes", 1)
    add_table(doc,
              ["Mode", "OLED", "Typical use"],
              [
                  ["Mouse/KB (default)", " Current: Mouse/KB ", "MAME, Demul, TeknoParrot, etc."],
                  ["Gamepad", " Current: Gamepad ", "Games expecting gamepad HID"],
                  ["MiSTer", " Current: MiSTer ", "MiSTer FPGA platform"],
              ])
    add_p(doc, "Switch via Simple Pause  Mode Change , runtime A+B+Trigger (Mouse/KB ↔ Gamepad), or Desktop App.")

    add_title(doc, "7. Calibration Profiles", 1)
    add_p(doc, "Four built-in profiles: Profile A, Profile B, Profile Start, Profile Select.")
    add_p(doc, "Idle bar shows P_A…P_D shorthand; profile picker may show  Name (Square)  or  Name (Diamond) .")
    add_bullets(doc, [
        "Direct Pause: press the button matching A>/B>/Str>/Sel> on the pause screen.",
        "Simple Pause:  Profile Select  submenu.",
        "Desktop App: Profiles tab → Confirm to save.",
        "Always  Save Gun Settings  or Start+Select before power-off.",
    ])

    add_title(doc, "8. Calibration Guide", 1)
    add_title(doc, "8.1 Screen aim calibration (required)", 2)
    add_bullets(doc, [
        "Maps IR coordinates to your monitor. Required on first use or after changing displays.",
        "Start: pull trigger at first boot; pause  Calibrate ; Direct Pause trigger; App cal window.",
        "Top bar during cal:  Cali:  + profile name.",
        "Steps: Top → Bottom → Left → Right → Center → Verify. Reload+Home cancels and restores old data.",
    ])

    add_title(doc, "8.2 Analog stick calibration", 2)
    add_table(doc,
              ["Type", "OLED menu / message", "Action"],
              [
                  ["Boot auto center", " CALIBRATION READY ", "~2 s at every boot—don’t touch stick"],
                  [" Center Calibrate ", "Center calibrated", "Set current position as center"],
                  [" Range Calibrate ", "RangeCal: rotate / OK", "Rotate stick ~4 s, release to center"],
              ])
    add_p(doc, "Stick range calibration is done on the gun OLED ( Range Calibrate ); the Desktop App has no range-cal UI yet.")

    add_title(doc, "8.3 IR emitter mounting", 2)
    add_bullets(doc, [
        "Square layout (Squa): emitters on top and bottom of the monitor.",
        "Diamond layout (Diam): emitters at the mid-point of all four sides.",
        "Use the App’s Open IR Emitter Alignment Assistant for a layout diagram.",
    ])

    add_title(doc, "9. Wireless Dongle", 1)
    add_title(doc, "9.1 What the Dongle does", 2)
    add_p(doc,
          "The Dongle is an ESP32-S3 receiver on the PC. It talks to the gun over ESP-NOW (2.4 GHz) and presents "
          "the same USB identity and forwards HID plus serial force-feedback data.")

    add_title(doc, "9.2 OLED status (English)", 2)
    add_table(doc,
              ["OLED text", "Meaning", "What to do"],
              [
                  ["OpenFIRE DONGLE", "Boot logo", "Wait"],
                  ["RF sniff ch / Scanning WiFi", "Picking least-congested channel", "Leave USB plugged ~10–15 s"],
                  ["Best channel eval / Please wait", "Channel survey", "Wait"],
                  ["Ch:xx Pair / Turn on lightgun / ESP-NOW listen", "Waiting for gun", "Power gun (no PC USB on gun)"],
                  ["RF link OK / Wait gun stream", "RF up, waiting HID stream", "Wait a few seconds"],
                  ["AWAIT PEDAL", "Waiting for wireless pedal", "Power pedal if used; else wait"],
                  ["Player: N / Channel: N / Gun: MAC", "Paired", "Test on PC"],
              ])
    add_p(doc, "TFT Dongles (e.g. LilyGO) may show Searching / best channel / Scanning WiFi / Connecting / AWAIT PEDAL / Player:N.")

    add_title(doc, "9.3 Wireless pedal", 2)
    add_bullets(doc, [
        "The pedal has no OLED. While searching, the gun top bar shows Search Pedal [xxs].",
        "After the gun pairs, the Dongle may show AWAIT PEDAL until the pedal handshake completes.",
        "Pedal gameplay matches a wired pedal; status is LED-only on the pedal unit.",
    ])

    add_title(doc, "9.4 PC USB & COM port", 2)
    add_bullets(doc, [
        "Before pairing completes the PC may see a placeholder device; afterward the gun’s configured name appears.",
        "Force feedback (Mamehooker, etc.): select the Dongle COM port at 9600 baud.",
        "Change VID/PID/mode on the gun, then re-pair so the Dongle refreshes its USB descriptor.",
    ])

    add_title(doc, "9.5 Multi-device tips", 2)
    add_bullets(doc, [
        "Recommended: one gun + one Dongle. Multiplayer: unique Gun IDs and separate Dongles.",
        "Avoid multiple Dongles in pairing mode competing for the same gun.",
        "If a gun sticks to an old Dongle MAC, clear binding or wait for timeout and full-channel rescan.",
    ])

    add_title(doc, "10. OpenFIRE Desktop App (optional)", 1)
    add_bullets(doc, [
        "Pin mapping, toggles, profile parameters, USB P1–P4, IR calibration, button test, FFB test.",
        "Connect via COM port; App sends Dock command to enter Docked mode and sync data.",
        "Click Confirm after edits. Gun ID: Settings → USB Identity.",
        "Use a firmware build compatible with your App branch (OpenFIRE-App or OpenFIRE-App-cn).",
    ])

    add_title(doc, "11. Force Feedback & MAMEHOOKER", 1)
    add_bullets(doc, [
        "With MAMEHOOKER enabled at compile time, gun/Dongle expose a 9600 baud serial channel.",
        "PC FFB software → serial → (wireless: via Dongle) → gun drives solenoid/rumble.",
        "OLED may show Mamehook ammo/life when configured.",
        " Rumble FFB Toggle  chooses rumble motor vs solenoid for serial FFB (separate from  Rumble Toggle ).",
        " Solenoid Toggle  = solenoid hardware;  Rumble Toggle  = rumble motor hardware.",
    ])

    add_title(doc, "12. Troubleshooting", 1)
    add_table(doc,
              ["Symptom", "Likely cause", "Fix"],
              [
                  ["Dongle stuck on Turn on lightgun / Searching", "Gun off, gun on PC USB, out of range", "Dongle first, then gun; unplug gun USB; move closer"],
                  ["Gun keeps Search Pedal", "Pedal off or out of range", "Power pedal; move near Dongle"],
                  ["PC name stays P1", "Gun ID not saved or USB not replugged", "Save Gun Settings; replug USB"],
                  ["App cannot set Gun ID", "App/firmware protocol mismatch", "Update to matching firmware/App pair"],
                  ["Wireless drops after connect", "2.4 GHz congestion", "Reduce distance; fewer Wi-Fi devices"],
                  ["OLED blank", "Wrong I2C address/pins for board", "Verify PlatformIO board env; replug USB"],
                  ["Aim offset", "Uncalibrated or new monitor", "Re-run screen cal; check Diam/Squa vs emitter mount"],
                  ["FFB dead", "Wrong COM or FFB disabled", "Dongle COM @ 9600; enable Rumble FFB Toggle; check wiring"],
                  ["Locked after Play time over", "Play Timer expired", "Wait for lockout to end or set Play Timer: OFF"],
                  ["Stick drift on boot", "Stick touched during cal", "Don’t touch stick first 2 s; run Center/Range Calibrate"],
              ])

    add_title(doc, "Appendix A: Quick daily workflow", 1)
    add_p(doc, "[Wired] USB to PC → confirm USB icon → play.", bold=True)
    add_p(doc, "[Wireless] Dongle to PC → gun on (no USB to gun) → Wi-Fi icon → play.", bold=True)
    add_p(doc, "[Settings/cal] Reload+Select → pause → change → Save Gun Settings or Start+Select → Reload+Home.", bold=True)
    add_p(doc, "[Esc] Reload+Start or Trigger+A+C.", bold=True)
    add_p(doc, "[Multiplayer] Unique Gun ID per gun → save → replug USB / one Dongle each.", bold=True)

    add_title(doc, "Appendix B: Chinese vs English firmware (menu cross-reference)", 1)
    add_table(doc,
              ["Chinese (OLED_MENU_ZH)", "English (default)"],
              [
                  ["中心校准", "Center Calibrate"],
                  ["摇杆校准", "Range Calibrate"],
                  ["屏幕校准", "Calibrate"],
                  ["配置选择", "Profile Select"],
                  ["少键模式", "Low Button Toggle"],
                  ["震动FFB开关", "Rumble FFB Toggle"],
                  ["游戏计时", "Play Timer"],
                  ["玩具模式", "Boy Mode"],
                  ["枪号", "Gun ID"],
                  ["布局 ST / SC", "Diam / Squa"],
              ])

    add_title(doc, "Appendix C: Supported boards (ESP32-S3)", 1)
    add_bullets(doc, [
        "ESP32-S3 WROOM-1 DevKitC-1 (N16R8 / N8R2)",
        "Waveshare ESP32-S3-Pico / ESP32-S3-Zero (N8R8 / N4R2)",
        "ESP32-S3 Super Mini (4 MB) and ESP32-S3 N8R8 (8 MB Flash + 8 MB PSRAM)",
        "See docs/BOARDS_PIN.md and lightgun/platformio.ini environments for build targets.",
    ])

    add_title(doc, "Appendix D: Resources & license", 1)
    add_bullets(doc, [
        "Firmware: OpenFIRE-Firmware-ESP32 (github.com/wei134102/OpenFIRE-Firmware-ESP32)",
        "Upstream: Team OpenFIRE / alessandro-satanassi OpenFIRE-Firmware-ESP32",
        "Board pinouts: docs/BOARDS_PIN.md",
        "OpenFIRE is open source; ESP32 port, ESP-NOW wireless, and OLED i18n by community maintainers.",
        "Features depend on your flashed firmware version and hardware configuration.",
    ])

    doc.save(OUT_EN)
    print("Saved:", OUT_EN)


def main():
    parser = argparse.ArgumentParser(description="Generate OpenFIRE user manual DOCX files.")
    parser.add_argument("--lang", choices=("zh", "en", "all"), default="all",
                        help="Which manual to generate (default: all)")
    args = parser.parse_args()
    if args.lang in ("zh", "all"):
        build_zh()
    if args.lang in ("en", "all"):
        build_en()


if __name__ == "__main__":
    main()
