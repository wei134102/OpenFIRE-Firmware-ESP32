# -*- coding: utf-8 -*-
"""Generate OpenFIRE Gun + Dongle user manual (Chinese)."""
from docx import Document
from docx.shared import Pt, Cm
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml.ns import qn

OUT = r"d:\code\OpenFIRE-Firmware-ESP32\docs\OpenFIRE光枪与Dongle使用说明书.docx"


def set_cn_font(run, name="微软雅黑", size=11):
    run.font.name = name
    run.font.size = Pt(size)
    run._element.rPr.rFonts.set(qn("w:eastAsia"), name)


def add_title(doc, text, level=0):
    if level == 0:
        p = doc.add_paragraph()
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        r = p.add_run(text)
        set_cn_font(r, size=22)
        r.bold = True
    else:
        doc.add_heading(text, level=level)


def add_p(doc, text, bold=False):
    p = doc.add_paragraph()
    r = p.add_run(text)
    set_cn_font(r)
    r.bold = bold
    return p


def add_bullets(doc, items):
    for item in items:
        p = doc.add_paragraph(item, style="List Bullet")
        for r in p.runs:
            set_cn_font(r)


def add_table(doc, headers, rows):
    t = doc.add_table(rows=1 + len(rows), cols=len(headers))
    t.style = "Table Grid"
    hdr = t.rows[0].cells
    for i, h in enumerate(headers):
        hdr[i].text = h
        for p in hdr[i].paragraphs:
            for r in p.runs:
                set_cn_font(r, size=10)
                r.bold = True
    for ri, row in enumerate(rows):
        cells = t.rows[ri + 1].cells
        for ci, val in enumerate(row):
            cells[ci].text = str(val)
            for p in cells[ci].paragraphs:
                for r in p.runs:
                    set_cn_font(r, size=10)
    doc.add_paragraph()


def build():
    doc = Document()
    sec = doc.sections[0]
    sec.top_margin = Cm(2.5)
    sec.bottom_margin = Cm(2.5)
    sec.left_margin = Cm(2.8)
    sec.right_margin = Cm(2.8)

    add_title(doc, "OpenFIRE 光枪与无线 Dongle\n详细使用说明书")
    add_p(doc, "适用固件：OpenFIRE-Firmware-ESP32（ESP32-S3 光枪 + ESP-NOW Dongle）")
    add_p(doc, "配套配置软件：OpenFIRE-App-cn（可选）")
    add_p(doc, "文档版本：1.0  |  编写日期：2026年5月")
    doc.add_paragraph()

    # --- 1 ---
    add_title(doc, "一、产品概述", 1)
    add_p(doc,
          "OpenFIRE 是一套基于四红外点（4IR）定位的光枪系统。ESP32-S3 版本支持两种与 PC 连接方式："
          "① USB 有线直连；② 通过 ESP-NOW 无线 Dongle 连接。无线模式下，PC 识别到的设备与有线连接相同，"
          "游戏与力反馈软件无需额外适配。")
    add_p(doc, "本说明书涵盖：光枪（GUN）日常操作、OLED 暂停菜单、校准、多玩家编号；"
          "以及无线接收器（Dongle）的配对、屏幕状态与故障排除。", bold=False)

    add_title(doc, "1.1 包装与配件（参考）", 2)
    add_bullets(doc, [
        "光枪本体（含 IR 摄像头、按键、可选 OLED、震动/电磁阀等）",
        "USB 数据线（用于有线连接或充电/刷机）",
        "无线 Dongle（ESP32-S3，插 PC USB，带 TFT/OLED 屏幕的型号可显示配对状态）",
        "IR 发射器条（需安装在显示器上下或四边，依 Square/Diamond 布局而定）",
        "（可选）无线踏板模块，用于 Time Crisis 类游戏的掩护操作",
    ])

    add_title(doc, "1.2 安全提示", 2)
    add_bullets(doc, [
        "首次开机及摇杆自动校准时，约 2 秒内请勿触碰模拟摇杆。",
        "电磁阀（Solenoid）工作时会有机械冲击与热量，请勿将枪口对准人员或动物。",
        "长时间使用请关注 OLED 温度显示；超温保护触发时会限制力反馈输出。",
        "刷机、改线请在断电状态下进行，并核对板型引脚文档（docs/BOARDS.md）。",
    ])

    # --- 2 ---
    add_title(doc, "二、首次开机与连接", 1)

    add_title(doc, "2.1 启动顺序（必读）", 2)
    add_p(doc, "光枪上电后按以下顺序自动执行：")
    add_bullets(doc, [
        "【约 2 秒】模拟摇杆自动中心校准 — OLED 可能显示 CALIBRATION… — 请勿动摇杆。",
        "【约 1 秒】连接方式选择：若 USB 已插入 PC 且被识别 → 优先有线 USB；否则进入无线模式搜索 Dongle。",
        "无线搜索期间若插入 USB，会立即切换为有线模式。",
        "若当前 Profile 从未做过屏幕校准，OLED 会提示 Welcome / Pull trigger，按扳机进入五点校准。",
    ])

    add_title(doc, "2.2 有线 USB 连接", 2)
    add_bullets(doc, [
        "用 USB 线连接光枪与 PC。",
        "等待 Windows 识别 HID 设备（设备名通常为 FIRECon P1～P4 等，取决于 Gun ID 设置）。",
        "OLED 左下角显示 USB 图标表示当前为有线模式。",
        "可用 OpenFIRE-App-cn 连接 COM 口进行高级配置（需进入 Docked 模式，见第十章）。",
    ])

    add_title(doc, "2.3 无线 Dongle 连接（推荐顺序）", 2)
    add_bullets(doc, [
        "步骤 1：将 Dongle 插入 PC USB，等待其完成启动（部分型号会先扫描 Wi-Fi 信道约 10～15 秒）。",
        "步骤 2：光枪不要插 PC USB（或确保 USB 未被 PC 枚举），然后打开光枪电源。",
        "步骤 3：Dongle 屏幕显示 Pair / Turn on lightgun / SEARCHING 时，光枪应开始广播配对。",
        "步骤 4：配对成功后，Dongle 显示 Player、Channel、Gun MAC 等信息；PC 出现与光枪配置一致的设备。",
        "步骤 5：OLED 左下角显示 Wi-Fi 图标表示无线模式。",
        "再次开机时，光枪会优先连接上次配对的 Dongle；若失败则重新全信道搜索。",
    ])

    add_title(doc, "2.4 Boy Mode（无 PC 试玩）", 2)
    add_p(doc,
          "无线未连接且 USB 未挂载时，上电约 3 秒内按住扳机可进入 Boy Mode："
          "可在无 PC 情况下本地试玩电磁阀等功能（具体取决于硬件配置）。")

    # --- 3 OLED ---
    add_title(doc, "三、OLED 状态栏说明（运行界面）", 1)
    add_table(doc,
              ["区域", "显示内容", "含义"],
              [
                  ["左上", "P1: ProA Squa 42°C", "玩家编号 P1～P4、当前 Profile 简写、IR 布局 Square/Diamond、温度"],
                  ["右上", "Play Timer", "游玩计时：OFF 或剩余分钟，到期后锁定输入"],
                  ["左下", "USB / Wi-Fi 图标", "当前连接方式：有线或无线 Dongle"],
                  ["中下", "RF / LOW / AF", "力反馈震动、低按钮模式、连发模式等状态"],
                  ["右下", "鼠标 / 手柄 / MiSTer 图标", "当前 USB 输出模式"],
              ])

    # --- 4 Pause ---
    add_title(doc, "四、暂停菜单", 1)
    add_p(doc,
          "暂停菜单用于切换 Profile、校准、改 USB 模式、设置 Gun ID、保存配置等。"
          "系统有两种交互方式，由 simplePause 开关决定（默认一般为 Direct Pause 热键模式）。")

    add_title(doc, "4.1 进入与退出暂停", 2)
    add_table(doc,
              ["操作", "按键组合"],
              [
                  ["进入暂停", "Reload + Select（换弹 + 选择）"],
                  ["退出暂停", "Reload + Home（换弹 + 主页）"],
                  ["长按进入暂停（需开启 Hold to Pause）", "Trigger + A 按住约 2.5 秒"],
                  ["Simple 模式长按退出", "A + B 按住约一半 Hold to Pause 时间"],
              ])

    add_title(doc, "4.2 Direct Pause（默认热键模式）", 2)
    add_p(doc, "进入暂停后，OLED 四行对应四个 Profile 快捷键；多数功能通过组合键完成，无需滚动菜单。")
    add_table(doc,
              ["功能", "按键组合"],
              [
                  ["屏幕校准", "暂停内按 Trigger（扳机）"],
                  ["保存全部设置到 Flash", "Start + Select"],
                  ["IR 灵敏度 +", "B + Up"],
                  ["IR 灵敏度 −", "B + Down"],
                  ["运行模式 Normal", "Start + A"],
                  ["运行模式 Average / Average2", "Start + B（在两种平均模式间切换）"],
                  ["发送 Esc 键到 PC", "Reload + Start"],
                  ["鼠标模式 ↔ 手柄模式", "A + B + Trigger（不含 MiSTer 切换）"],
                  ["切换 Profile A/B/Start/Select", "暂停界面按对应面键"],
                  ["校准中跳过中心步", "校准过程中按 A"],
              ])

    add_title(doc, "4.3 Simple Pause（列表菜单模式）", 2)
    add_p(doc, "若已在 OpenFIRE App 中开启 Simple Pause：A=上一项，B=下一项，Trigger=确认执行。")
    add_table(doc,
              ["菜单项（OLED 英文）", "功能说明"],
              [
                  ["Center Calibrate", "以当前位置为摇杆新中心（需配置模拟摇杆引脚）"],
                  ["Range Calibrate", "约 4 秒画圈 + 回中，校准摇杆活动范围"],
                  ["Calibrate", "进入屏幕五点瞄准校准"],
                  ["Profile Select", "选择 Profile A/B/Start/Select"],
                  ["Mode Change", "循环：鼠标键盘 → 手柄 → MiSTer"],
                  ["Layout Toggle", "Square（方形四点）↔ Diamond（菱形四点）"],
                  ["Gun ID (P1-P4)", "设置玩家编号，同步 USB PID 与设备名"],
                  ["Stick: Gamepad / D-Pad / Keys", "模拟摇杆输出方式"],
                  ["Deadzone: N%", "摇杆死区 0～30%"],
                  ["Axis: Signed / Unsigned", "有符号 / 无符号轴（Joypad-OS 兼容）"],
                  ["Swap Sticks: ON/OFF", "红外定位与物理摇杆左右互换"],
                  ["Play Timer", "0 / 5 / 10 / 15 / 20 分钟，到期锁定输入"],
                  ["Save Gun Settings", "保存 Profile 与全局设置到 Flash"],
                  ["Rumble / Solenoid / Autofire Toggle", "震动、电磁阀、连发开关（视硬件而定）"],
              ])
    add_p(doc, "注：无对应硬件的菜单项会自动跳过。Gun ID 子菜单：A/B 选择 P1～P4，Trigger 保存，Reload+Home 取消。")

    # --- 5 Gun ID ---
    add_title(doc, "五、Gun ID 与多光枪设置（P1～P4）", 1)
    add_bullets(doc, [
        "每只光枪可设为 P1、P2、P3、P4，对应 USB Product ID 1～4，设备名如 FIRECon P1。",
        "OLED 暂停菜单：Simple 模式下选 Gun ID；或 OpenFIRE-App-cn → Settings → USB Identity 选 P1～P4。",
        "修改 Gun ID 后需保存设置，并重新插拔 USB（或有线重连）后 Windows 才会识别新编号。",
        "多枪同屏游戏：每只枪设不同 Player 编号，各接不同 USB 口或使用多个 Dongle。",
    ])

    # --- 6 USB modes ---
    add_title(doc, "六、USB 输出模式", 1)
    add_table(doc,
              ["模式", "说明", "典型用途"],
              [
                  ["Mouse/Keyboard（默认）", "绝对坐标鼠标 + 键盘按键映射", "MAME、Demul、TeknoParrot 等"],
                  ["Gamepad", "标准手柄 + 模拟摇杆", "需要手柄协议的游戏或前端"],
                  ["MiSTer Optimized", "针对 MiSTer FPGA 的优化映射", "MiSTer 平台"],
              ])
    add_p(doc, "切换方式：Simple 菜单 Mode Change；Direct 暂停 A+B+Trigger（鼠标↔手柄）；OpenFIRE App 设置。")

    # --- 7 Profiles ---
    add_title(doc, "七、校准配置文件（Profile）", 1)
    add_p(doc, "系统内置 4 套 Profile，默认名称：Profile A、Profile B、Profile Start、Profile Select。")
    add_p(doc, "每套 Profile 独立保存：屏幕边距校准、IR 灵敏度、运行模式（Normal/Average）、Square/Diamond 布局、NeoPixel 颜色、显示名称等。")
    add_bullets(doc, [
        "Direct 暂停：按 OLED 上对应键快速切换。",
        "Simple 暂停：Profile Select 子菜单选择。",
        "OpenFIRE App：Profiles 标签页编辑后 Confirm 保存。",
        "修改后务必 Save Gun Settings 或 Start+Select 写入 Flash，否则断电丢失。",
    ])

    # --- 8 Calibration ---
    add_title(doc, "八、校准指南", 1)

    add_title(doc, "8.1 屏幕瞄准校准（必做）", 2)
    add_p(doc, "用于将 IR 坐标映射到显示器区域，首次使用或换显示器后必须执行。")
    add_p(doc, "触发方式：首次开机按扳机；暂停菜单 Calibrate；Direct 暂停按扳机；OpenFIRE App 校准窗口。")
    add_p(doc, "步骤（每步对准屏幕指示点后再按扳机）：")
    add_bullets(doc, [
        "Top（顶部）→ Bottom（底部）→ Left（左侧）→ Right（右侧）",
        "Center（中心）— 保存四边 offset",
        "Verify（验证）— 试射满意后按扳机完成；不满意可按 A+B 重来",
        "取消：Reload+Home 恢复原校准数据",
    ])
    add_p(doc, "OpenFIRE App 校准：连接 COM 口 → Calibration Profiles → Calibrate Profile X → 全屏按提示操作 → 主界面 Confirm 保存。")

    add_title(doc, "8.2 模拟摇杆校准", 2)
    add_table(doc,
              ["类型", "时机", "操作"],
              [
                  ["上电自动中心校准", "每次开机约 2 秒", "勿触碰摇杆"],
                  ["Center Calibrate", "暂停菜单", "将当前位置设为新中心"],
                  ["Range Calibrate", "暂停菜单", "约 4 秒内画满圈后回中"],
              ])
    add_p(doc, "OpenFIRE App 目前无摇杆画圈校准界面，请在光枪 OLED 暂停菜单中操作。")

    add_title(doc, "8.3 IR 发射器安装", 2)
    add_bullets(doc, [
        "Square 布局：发射器安装在显示器上下（或对应上下 IR 点位置）。",
        "Diamond 布局：发射器安装在显示器四边中点。",
        "OpenFIRE App 菜单：Open IR Emitter Alignment Assistant 可查看对齐示意图。",
    ])

    # --- 9 Dongle ---
    add_title(doc, "九、无线 Dongle 详细说明", 1)

    add_title(doc, "9.1 Dongle 是什么", 2)
    add_p(doc,
          "Dongle 是插在 PC 上的 ESP32-S3 接收器，通过 ESP-NOW 2.4GHz 与光枪通信。"
          "配对完成后，Dongle 向 PC 呈现与光枪相同的 USB 身份（VID/PID/设备名），"
          "并转发 HID 输入与串口力反馈数据。PC 无需单独安装蓝牙配对。")

    add_title(doc, "9.2 屏幕状态对照", 2)
    add_table(doc,
              ["屏幕文字", "含义", "您应做什么"],
              [
                  ["OpenFIRE DONGLE / Logo", "启动中", "等待"],
                  ["RF sniff / WiFi scan / Please wait", "自动选择干扰最小的 Wi-Fi 信道", "勿拔 USB，约 10～15 秒"],
                  ["Ch:xx Pair / Turn on lightgun / SEARCHING", "等待光枪配对", "打开光枪（勿插 PC USB）"],
                  ["RF link OK / Wait gun stream", "射频已连，等待数据流", "等待数秒"],
                  ["Attesa PEDAL", "等待无线踏板握手（部分固件）", "有踏板则打开；无踏板等待或重启"],
                  ["Player / Channel / Gun MAC", "配对成功", "可在 PC 测试"],
              ])

    add_title(doc, "9.3 PC 端 USB 与 COM 口", 2)
    add_bullets(doc, [
        "配对成功前，PC 可能只看到占位设备（如 FIRECon / VID F143）；完成后变为光枪实际配置的名称。",
        "力反馈（Mamehooker、Hook of the Reaper 等）：选择 Dongle 对应的 COM 口，波特率 9600。",
        "改 VID/PID/模式请在光枪端（App 或暂停菜单）设置，然后重启配对使 Dongle 更新。",
    ])

    add_title(doc, "9.4 多台设备建议", 2)
    add_bullets(doc, [
        "建议固定「1 枪 + 1 Dongle」组合；多人时使用不同 Gun ID 与不同 Dongle。",
        "避免多个 Dongle 同时处于 Pair 状态抢连同一光枪。",
        "更换 Dongle 后，光枪若仍连旧 MAC，需清除绑定或等待超时后全信道重搜。",
    ])

    # --- 10 App ---
    add_title(doc, "十、OpenFIRE-App-cn 配置软件（可选）", 1)
    add_bullets(doc, [
        "用途：引脚映射、开关选项、Profile 参数、USB P1～P4、IR 校准、按键测试、力反馈测试。",
        "连接：选择 COM 口，App 发送 Dock 命令进入 Docked 模式后同步数据。",
        "保存：修改后点击 Confirm，等待进度条完成。Gun ID 在 Settings → USB Identity 选 P1～P4。",
        "固件需与 App 同属 OpenFIRE ESP32 分支（版本号含对应 fork 标记），否则 App 会拒绝连接。",
    ])

    # --- 11 MAME ---
    add_title(doc, "十一、力反馈与 MAMEHOOKER", 1)
    add_bullets(doc, [
        "编译启用 MAMEHOOKER 时，光枪/Dongle 提供 9600 波特率串口通道。",
        "PC 端软件发送力反馈指令 → 串口 →（无线时经 Dongle）→ 光枪驱动电磁阀/震动。",
        "OLED 可显示 Mamehook 弹药、生命等信息（取决于游戏与软件配置）。",
        "Simple 菜单 Rumble FFB 开关控制是否响应串口力反馈。",
    ])

    # --- 12 Troubleshooting ---
    add_title(doc, "十二、常见问题与故障排除", 1)
    add_table(doc,
              ["现象", "可能原因", "处理办法"],
              [
                  ["Dongle 一直 Pair/SEARCHING", "光枪未开、仍插 USB、距离过远", "先 Dongle 后开枪；拔枪 USB；靠近 PC"],
                  ["PC 设备名不变/仍是 P1", "Gun ID 未保存或未重插 USB", "保存设置；重新插拔 USB；App 确认 P1～P4 已 Commit"],
                  ["App 无法设置 Gun ID", "固件与 App 协议键名不一致", "升级至已修复 GunId 同步的最新固件"],
                  ["无线连上后断线", "2.4G 干扰", "缩短距离；减少同频段设备；使用自动选频 Dongle 固件"],
                  ["OLED 全黑", "I2C 地址/引脚与板型不符", "核对编译板型；重插 USB"],
                  ["瞄准偏移", "未校准或换显示器", "重做屏幕校准；检查 Square/Diamond 与发射器安装"],
                  ["力反馈无效", "COM 口错误或模式关闭", "选 Dongle COM；开启 Rumble FFB；检查电磁阀/震动接线"],
                  ["Play Timer 到期无法操作", "计时锁定设计", "等待计时结束或于暂停菜单将 Play Timer 设为 OFF"],
                  ["上电摇杆漂移", "校准时碰了摇杆", "重启并在 2 秒内勿触碰；或 Range/Center 校准"],
              ])

    # --- 13 Appendix ---
    add_title(doc, "附录 A：推荐日常流程速查", 1)
    add_p(doc, "【有线游玩】USB 接 PC → 确认 USB 图标 → 进入游戏。", bold=True)
    add_p(doc, "【无线游玩】Dongle 接 PC → 光枪开机（不插 USB）→ 等 Wi-Fi 图标 → 进入游戏。", bold=True)
    add_p(doc, "【改 Profile/校准】Reload+Select 进暂停 → 操作 → Save 或 Start+Select → Reload+Home 退出。", bold=True)
    add_p(doc, "【多玩家】每只枪设不同 Gun ID → 保存 → 重插 USB / 各用 Dongle。", bold=True)

    add_title(doc, "附录 B：相关资源", 1)
    add_bullets(doc, [
        "固件仓库：OpenFIRE-Firmware-ESP32",
        "配置 App：OpenFIRE-App-cn",
        "硬件参考：PICON-AS 无线光枪套件说明",
        "原版项目：Team OpenFIRE",
        "板型引脚：lightgun/docs/BOARDS.md",
    ])

    add_title(doc, "附录 C：版权声明", 1)
    add_p(doc,
          "OpenFIRE 光枪系统基于 Team OpenFIRE 开源项目。ESP32 移植与 ESP-NOW 无线功能由 "
          "OpenFIRE-Firmware-ESP32 维护者实现。本说明书仅供用户参考，功能以实际刷入的固件版本与硬件配置为准。")

    doc.save(OUT)
    print("Saved:", OUT)


if __name__ == "__main__":
    build()
