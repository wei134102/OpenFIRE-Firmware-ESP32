# unsignedAxis 暂存说明（待后续实现）

## 当前状态

- `Gamepad16_.unsignedAxis` 字段已保留（用于兼容 `main.cpp` 的读写），避免编译报错。
- 但 `TinyUSB_Devices.cpp` 中与 `unsignedAxis` 相关的映射分支逻辑当前未启用。
- 因此现阶段属于“有配置项、无实际行为切换”的暂存状态。

## 受影响位置

- `src/main.cpp`
  - 菜单中仍会写入 `Gamepad16.unsignedAxis`。
- `lib/TinyUSB_Devices/TinyUSB_Devices.h`
  - 已包含 `bool unsignedAxis = false;` 兼容字段。
- `lib/TinyUSB_Devices/TinyUSB_Devices.cpp`
  - `moveCam()` / `moveStick()` / `press()` / `release()` 目前未按 `unsignedAxis` 分支处理。

## 后续实现建议（TODO）

1. 在 `Gamepad16_::moveCam()` 中恢复 signed/unsigned 双分支映射。
2. 在 `Gamepad16_::moveStick()` 中恢复 signed/unsigned 双分支映射。
3. 若需要 LT/RT 轴化行为，在 `press()` / `release()` 中恢复 unsigned 模式下的触发器逻辑。
4. 验证 `AnalogInvertX/Y` 与 `AnalogSwapSticks` 在 unsigned 模式下的联动结果。
5. 进行实机回归：鼠标模式、键鼠混合、gamepad 模式、ESP-NOW 统一包模式。

## 备注

- 当前优先目标是保持编译通过与现有流程稳定。
- 后续恢复 `unsignedAxis` 逻辑时，建议单独提交，便于回滚和对比测试。
