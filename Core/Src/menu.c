#include "menu.h"
#include "foc.h"

// 1. 定义参数 (5个调参 + 1个模式切换)
float Target_Speed = 0.0f;
float Speed_Kp     = 0.15f;
float Speed_Ki     = 0.01f;
float Current_Kp   = 0.05f;
float Current_Ki   = 0.005f;

// 2. 菜单状态机
typedef enum {
    MODE_VIEW = 0,    // 监控模式
    MODE_MENU_SEL,    // 菜单选择模式
    MODE_PARAM_EDIT   // 参数修改模式
} MenuMode_t;

MenuMode_t current_mode = MODE_VIEW;
int8_t menu_index = 0;

/* USER CODE BEGIN 4 */
void Menu_Analyse(void)
{
    // ---  第一部分：按钮事件与状态切换 ---
    if (EC11.Btn_Flag == 1)
    {
        EC11.Btn_Flag = 0;
        OLED_Clear();

        switch (current_mode)
        {
            case MODE_VIEW:
                current_mode = MODE_MENU_SEL;
                EC11.Count = menu_index;
                break;

            case MODE_MENU_SEL:
                current_mode = MODE_PARAM_EDIT;
                EC11.Count = 0;
                break;

            case MODE_PARAM_EDIT:
                // 参数修改完成，保存并返回菜单选择
                // 如果编辑的是模式开关, 菜单修改后需要即时生效
                current_mode = MODE_MENU_SEL;
                EC11.Count = menu_index;
                break;
        }
    }

    // ---  第二部分：旋钮事件与屏幕刷新 ---
    switch (current_mode)
    {
        case MODE_VIEW:
            OLED_Printf(0, 0, "=== FOC Monitor ===");
            OLED_Printf(0, 2, "Mode: %s", (foc_mode == FOC_SENSORED) ? "Sensored" : "Sensorless");
            OLED_Printf(0, 3, "Tar Spd: %.2f r/s", Target_Speed);
            OLED_Printf(0, 6, "[Press to Tuning]");
            break;

        case MODE_MENU_SEL:
            // menu_index: 0~5 (6个菜单项)
            if (EC11.Count < 0)  EC11.Count = 0;
            if (EC11.Count > 5)  EC11.Count = 5;
            menu_index = EC11.Count;

            OLED_Printf(0, 0, "=== Select Param ===");
            OLED_Printf(0, 1, "%s0.Mode: %s",
                (menu_index == 0) ? " >" : "  ",
                (foc_mode == FOC_SENSORED) ? "Sensored" : "Sensorless");
            OLED_Printf(0, 2, "%s1.Tar Spd: %.2f", (menu_index == 1) ? " >" : "  ", Target_Speed);
            OLED_Printf(0, 3, "%s2.Spd_Kp : %.3f", (menu_index == 2) ? " >" : "  ", Speed_Kp);
            OLED_Printf(0, 4, "%s3.Spd_Ki : %.3f", (menu_index == 3) ? " >" : "  ", Speed_Ki);
            OLED_Printf(0, 5, "%s4.Cur_Kp : %.3f", (menu_index == 4) ? " >" : "  ", Current_Kp);
            OLED_Printf(0, 6, "%s5.Cur_Ki : %.3f", (menu_index == 5) ? " >" : "  ", Current_Ki);
            OLED_Printf(0, 7, "[Press to Edit]");
            break;

        case MODE_PARAM_EDIT:
            OLED_Printf(0, 0, "=== Editing... ===");

            switch (menu_index)
            {
                case 0: // 切换有感/无感模式
                    if (EC11.Count != 0) {
                        foc_mode = (foc_mode == FOC_SENSORED) ? FOC_SENSORLESS : FOC_SENSORED;
                        if (foc_mode == FOC_SENSORLESS) {
                            Sensorless_Enter();
                        } else {
                            Sensorless_Exit();
                        }
                    }
                    OLED_Printf(0, 3, "Mode -> %s",
                        (foc_mode == FOC_SENSORED) ? "Sensored" : "Sensorless");
                    break;
                case 1: // 修改 Target_Speed
                    Target_Speed += (float)EC11.Count * 0.1f;
                    OLED_Printf(0, 3, "Tar Spd -> %.2f", Target_Speed);
                    break;
                case 2: // 修改 Speed_Kp
                    Speed_Kp += (float)EC11.Count * 0.01f;
                    if(Speed_Kp < 0) Speed_Kp = 0;
                    OLED_Printf(0, 3, "Speed_Kp -> %.3f", Speed_Kp);
                    break;
                case 3: // 修改 Speed_Ki
                    Speed_Ki += (float)EC11.Count * 0.001f;
                    if(Speed_Ki < 0) Speed_Ki = 0;
                    OLED_Printf(0, 3, "Speed_Ki -> %.3f", Speed_Ki);
                    break;
                case 4: // 修改 Current_Kp
                    Current_Kp += (float)EC11.Count * 0.005f;
                    if(Current_Kp < 0) Current_Kp = 0;
                    OLED_Printf(0, 3, "Current_Kp -> %.3f", Current_Kp);
                    break;
                case 5: // 修改 Current_Ki
                    Current_Ki += (float)EC11.Count * 0.0005f;
                    if(Current_Ki < 0) Current_Ki = 0;
                    OLED_Printf(0, 3, "Current_Ki -> %.3f", Current_Ki);
                    break;
            }
            EC11.Count = 0;
            OLED_Printf(0, 6, "[Press to Save]");
            break;
    }
}
