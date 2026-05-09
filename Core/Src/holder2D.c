#include "holder2D.h"
#include "main.h"

/* 硬件配置初始化（大端模式 & 轮模式设置） */
void setup_holder(void) {
    setEnd(0);    // 设置舵机通信为大端模式（高位字节在前）
    WheelMode(1); // 启用轮模式（持续旋转，需配合角度控制使用）
}

/**​
 * @brief 设置舵机绝对角度
 * @param ID 舵机ID（如ID1=俯仰舵机，ID6=水平舵机）
 * @param angle 目标角度（单位：度，范围0~360）
 * @note 角度转换为12位编码（0~4096），直接控制舵机到指定位置
 */
void set_servo_angle_holder(uint8_t ID, float angle) {
	setup_holder();
    // 角度转舵机位置编码（360°对应4096步）
    int position = (angle / 360.0) * 4096; 
    // 写入目标位置，速度=2400（单位需查手册），时间=50ms（动作时间）
    WritePosEx(ID, position, 2400, 50); 
}
void set_servo_angle_negative_holder(uint8_t ID, float angle) {
	setup_holder();
    // 角度转舵机位置编码（360°对应4096步）
    int position = (angle / 360.0) * 4096; 
    // 写入目标位置，速度=2400（单位需查手册），时间=50ms（动作时间）
    WritePosEx(ID, 2048-position, 2400, 50); 
}
/**​
 * @brief 设置舵机相对角度偏移
 * @param ID 舵机ID
 * @param angle 相对偏移角度（单位：度，可正负）
 * @note 基于当前位置叠加偏移量，自动限制在0~4096范围内
 */
void set_servo_angle_offset_holder(uint8_t ID, float angle) {
	setup_holder();
    // 计算偏移量（角度→编码）
    int offset = (angle / 360.0) * 4096; 
    // 读取当前位置（需确保舵机反馈正常）
    int pre_position = ReadPos(ID); 
    // 计算目标位置并限制范围
    int target_position = pre_position + offset;
    target_position = (target_position > 4096) ? 4096 : 
                     (target_position < 0) ? 0 : target_position;
    // 平滑移动至新位置（参数同绝对角度函数）
    WritePosEx(ID, target_position, 2400, 50);
}
/**
 * @brief 根据目标坐标(x, y)控制云台俯仰角和水平角,ID需在函数定义中修改
 * @param x 目标X坐标（米）
 * @param y 目标Y坐标（米）
 */
void set_xy_position_holder(float x, float y) {
    

    // 2. 计算角度（弧度转角度）
    float angle_n = atan(x) * (180.0f / 3.1415926);  // 水平角（ID6）
    float angle_m = atan(y) * (180.0f / 3.1415926);  // 俯仰角（ID8）

    // 3. 控制舵机（示例调用）
    set_servo_angle_negative_holder(1, 180+angle_n);  // 水平舵机ID6
    set_servo_angle_negative_holder(1, 180+angle_m);  // 俯仰舵机ID8
 
}

/*​
 * @brief 控制云台绘制指定圆心和半径的圆形轨迹
 * @param CENTER_X 圆心X坐标（单位：米）
 * @param CENTER_Y 圆心Y坐标（单位：米）
 * @param r 圆半径（单位：毫米，要求 >=10mm）
 * @param n 圆周上的点数（要求 >0）
 *
 * @note  角度精度由set_xy_position内部处理（0.4度步进）
 *       
 */
void draw_circle_holder(float CENTER_X, float CENTER_Y,float r,int n) {
 if (n <= 0||r<10) return;
    for (int i = 0; i < n; i++) {
        float theta = 2 * 3.1415926 * i / n;
        float x = CENTER_X + r * 0.001 * cosf(theta);
        float y = CENTER_Y + r* 0.001 * sinf(theta);
        
       set_xy_position_holder(x,y );
            HAL_Delay(30);
        
    }
}

