# OLED / LCD 使用文档

> 适用板卡：**DJI_Board_TypeA**（STM32F427）  
> OLED 型号：SSD1306 兼容，128×64 像素，I²C 接口  
> 库路径：`shared/libraries/oled.h / oled.cc`

---

## 1. 硬件接线

| OLED 引脚 | 板卡引脚 | 说明 |
|-----------|---------|------|
| VCC       | 3.3 V   |      |
| GND       | GND     |      |
| SCL       | I2C2_SCL | TypeA 使用 `hi2c2` |
| SDA       | I2C2_SDA |      |

I²C 地址：`0x3C`（默认，部分模块为 `0x3D`，看 SA0 引脚）

---

## 2. 初始化

```cpp
#include "i2c.h"
#include "oled.h"

static display::OLED* OLED = nullptr;

void RM_RTOS_Init(void) {
  OLED = new display::OLED(&hi2c2, 0x3C);
}
```

> **注意**：必须 `#include "i2c.h"` 才能让 `hi2c2` 可见，否则 C++ 编译器会报未定义错误。

---

## 3. 常用 API

### 3.1 清屏 / 填充

```cpp
OLED->OperateGram(display::PEN_CLEAR);     // 全屏清零（黑）
OLED->OperateGram(display::PEN_WRITE);     // 全屏置1（白）
OLED->OperateGram(display::PEN_INVERSION); // 全屏像素取反
```

### 3.2 刷新到屏幕

所有绘制操作只写到内存 gram，**必须调用 `RefreshGram()` 才会真正输出到屏幕**。

```cpp
OLED->RefreshGram();
```

### 3.3 写文字

```cpp
OLED->Printf(row, col, fmt, ...);
```

| 参数 | 说明 |
|------|------|
| `row` | 文本行，0~4（共 5 行，每行高 12 px，小字体） |
| `col` | 文本列（字符单位），建议从 `2` 开始以避免左侧像素被遮挡 |
| `fmt` | printf 格式字符串 |

示例：

```cpp
OLED->Printf(0, 2, "Hello World");
OLED->Printf(1, 2, "Val: %d", some_int);
OLED->RefreshGram();
```

> **左边距建议**：屏幕物理左侧约有 1 个字符宽（6 px）可能被外壳遮挡，  
> 建议 `col` 从 `2` 开始（留 12 px 边距）确保文字完整可见。

### 3.4 画点 / 画线

```cpp
OLED->DrawPoint(x, y, pen);                     // 单点，x∈[0,127] y∈[0,63]
OLED->DrawLine(x1, y1, x2, y2, pen);            // 直线
```

`pen` 取值：

| 值 | 效果 |
|----|------|
| `display::PEN_CLEAR` | 置 0（黑） |
| `display::PEN_WRITE` | 置 1（白） |
| `display::PEN_INVERSION` | 像素取反（高亮） |

### 3.5 内置动画

```cpp
OLED->ShowRMLOGO();        // 显示 RoboMaster logo
OLED->ShowIlliniRMLOGO();  // 显示 Illini RM logo
OLED->DrawCat();           // 彩虹猫动画（需在循环中每帧调用）
```

---

## 4. 高亮反选（菜单选中效果）

通过对选中行所有像素逐行取反，可实现白底黑字的高亮效果：

```cpp
// 对第 text_row 行（0-based，每行12px）做像素取反
static void InvertTextRow(uint8_t text_row) {
  const uint8_t y_start = text_row * 12;
  const uint8_t y_end   = y_start + 11;
  for (uint8_t y = y_start; y <= y_end; ++y) {
    OLED->DrawLine(0, y, 127, y, display::PEN_INVERSION);
  }
}
```

调用顺序：先 `Printf` 写文字（白字黑底），再 `InvertTextRow` 取反，最后 `RefreshGram`。

---

## 5. 按键输入（GPIO 上拉输入）

### 5.1 硬件配置

四个按键接 **PC0~PC3**，另一端接 GND，使用内部上拉：

| 按键 | GPIO | 功能（示例）|
|------|------|------------|
| K1   | PC0  | 光标上移   |
| K2   | PC1  | 光标下移   |
| K3   | PC2  | 确认（进入子页面）|
| K4   | PC3  | 取消（返回上一页）|

### 5.2 初始化代码

```cpp
__HAL_RCC_GPIOC_CLK_ENABLE();
GPIO_InitTypeDef GPIO_InitStruct;
GPIO_InitStruct.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull  = GPIO_PULLUP;
GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
```

> **注意**：不要用 `= {0}` 零初始化 `GPIO_InitTypeDef`，编译器开了  
> `-Werror=missing-field-initializers`，必须把每个字段显式赋值。

### 5.3 带消抖的边沿检测

```cpp
static bool ReadKey(uint16_t pin) {
  return HAL_GPIO_ReadPin(GPIOC, pin) == GPIO_PIN_RESET; // 上拉，按下为低电平
}

// 在任务循环里：
bool prev_k1 = false;
uint32_t last_k1 = 0;

while (true) {
  bool k1 = ReadKey(GPIO_PIN_0);
  uint32_t now = HAL_GetTick();

  if (k1 && !prev_k1 && (now - last_k1 >= 200)) { // 上升沿 + 200ms 消抖
    last_k1 = now;
    // 处理 K1 按下事件
  }
  prev_k1 = k1;
  osDelay(10);
}
```

---

## 6. 完整菜单示例框架

`vehicles/Dart/LCD_Test.cc` 中实现了一个两条目的交互菜单，逻辑如下：

```
启动
  └─ 显示 RM LOGO（2s）
  └─ 显示 Illini RM LOGO（2s）
  └─ 进入 STATE_MENU
        K1 → cursor 上移，高亮选中行
        K2 → cursor 下移，高亮选中行
        K3 → 进入 STATE_DETAIL，显示对应详情页
  └─ STATE_DETAIL
        K4 → 返回 STATE_MENU
```

渲染层（DrawMenu / DrawDetail）每次都先 `OperateGram(PEN_CLEAR)` 清屏再重绘，  
通过 `need_redraw` 标志避免无谓刷新。

---

## 7. 注意事项

1. **`hi2c2` vs `hi2c1`**：TypeA 板 OLED 接在 I2C2，TypeC 等其他板请查对应 `.ioc` 文件。
2. **每次改动绘制内容后都需要 `RefreshGram()`**，否则屏幕不更新。
3. **`DrawCat()` 会自带清屏**，如需在动画帧上叠加文字，需在每帧 `DrawCat()` 后再次调用 `Printf` + `RefreshGram`。
4. **屏幕坐标原点**在左上角，X 轴向右（0~127），Y 轴向下（0~63）。
