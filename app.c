/**
 ******************************************************************************
 ** ファイル名 : app.c
 **
 ** 概要 : 二輪差動型ライントレースロボットのTOPPERS/HRP3用Cサンプルプログラム
 **
 ** 注記 : sample_c4 (sample_c3にBluetooth通信リモートスタート機能を追加)
 ******************************************************************************
 **/

#include "ev3api.h"
#include "app.h"
#include "etroboc_ext.h"

#if defined(BUILD_MODULE)
#include "module_cfg.h"
#else
#include "kernel_cfg.h"
#endif

#define DEBUG

#if defined(DEBUG)
#define _debug(x) (x)
#else
#define _debug(x)
#endif

#if defined(MAKE_BT_DISABLE)
static const int _bt_enabled = 0;
#else
static const int _bt_enabled = 1;
#endif

/**
 * シミュレータかどうかの定数を定義します
 */
#if defined(MAKE_SIM)
static const int _SIM = 1;
#elif defined(MAKE_EV3)
static const int _SIM = 0;
#else
static const int _SIM = 0;
#endif

/**
 * 左コース/右コース向けの設定を定義します
 * デフォルトは左コース(ラインの右エッジをトレース)です
 */
#if defined(MAKE_RIGHT)
static const int _LEFT = 0;
static const motor_port_t
    arm_motor = EV3_PORT_A,
    left_motor = EV3_PORT_C,
    right_motor = EV3_PORT_B;
#define _EDGE 1
#else
static const int _LEFT = 1;
static const motor_port_t
    arm_motor = EV3_PORT_A,
    left_motor = EV3_PORT_B,
    right_motor = EV3_PORT_C;
#define _EDGE 1
#endif

#define DELAY_TIME 4*1000U
#define PID_DELAY_TIME 0*1000U

#define DELTA_T 0.0040
//chokusinn
// #define KP1 1.74
// #define KI1 0.0001
// #define KD1 0.02495

#define KP0 2.0
#define KI0 0.0
#define KD0 0.0

//chokusinn
#define KP1 1.6
#define KI1 0.0
#define KD1 0.1

#define KP2 3.0
#define KI2 0.0157
#define KD2 1.0
//ooenn
#define KP3 3.1
#define KI3 0.007
#define KD3 0.14
//magari(soto)(最初のカーブ)
#define KP4_1 2.1
#define KI4_1 0.01
#define KD4_1 0.16
//magari(soto)(2番目のカーブ)
#define KP4_2 1.9
#define KI4_2 0.008
#define KD4_2 0.16

//magari(uti)
#define KP5 3.0
#define KI5 0.002
#define KD5 0.3
//1/4rad
#define KP6 1.9
#define KI6 0.015
#define KD6 0.1
//shou
#define KP7 3.3
#define KI7 0.0027
#define KD7 0.23

#define KP8 4.5
#define KI8 0.0001
#define KD8 0.065


float diff[2] = {0, 0};
float integral = 0;

signed char forward; /* 前後進命令 */
signed char turn;    /* 旋回命令 */
// signed char pwm_L, pwm_R; /* 左右モーターPWM出力 */

static int refrect = 0;
static int ao_count = 0;
static int aka_count = 0;
static int g_count = 0;
static int y_count = 0;
static int b_count = 0;
static int ab_count = 0;
static int w_count = 0;
static int right_wheel[10];
static int left_wheel[10]; 
static int arm_wheel[2];
static int kuro = 0;
static int siro = 0;
static int flag = 0;
static int speed = 0;
static int angle = 0;
static int _wheelCountTemp;
static int _refrect;

/**
 * センサー、モーターの接続を定義します
 */
static const sensor_port_t
    touch_sensor = EV3_PORT_1,
    color_sensor = EV3_PORT_2,
    sonar_sensor = EV3_PORT_3,
    gyro_sensor = EV3_PORT_4;

// static const motor_port_t
//     arm_motor = EV3_PORT_A,
//     left_motor = EV3_PORT_C,
//     right_motor = EV3_PORT_B;

static int bt_cmd = 0;  /* Bluetoothコマンド 1:リモートスタート */
static FILE *bt = NULL; /* Bluetoothファイルハンドル */

/* 下記のマクロは個体/環境に合わせて変更する必要があります */
/* sample_c1マクロ */
#define LIGHT_WHITE 29                        /* 白色の光センサ値 */
#define LIGHT_BLACK 4                         /* 黒色の光センサ値 */
#define LR_MOTOR_RATIO 1                      /*左右モーターのパワー比(Lpower:Rpower)*/

static int CENTER = (LIGHT_WHITE + LIGHT_BLACK) / 2; /*ライン中央の光センサ値*/
/* sample_c2マクロ */
#define SONAR_ALERT_DISTANCE 30 /* 超音波センサによる障害物検知距離[cm] */
/* sample_c4マクロ */
//#define DEVICE_NAME     "ET0"  /* Bluetooth名 sdcard:\ev3rt\etc\rc.conf.ini LocalNameで設定 */
//#define PASS_KEY        "1234" /* パスキー    sdcard:\ev3rt\etc\rc.conf.ini PinCodeで設定 */
#define CMD_START '1' /* リモートスタートコマンド */

/* LCDフォントサイズ */
#define CALIB_FONT (EV3_FONT_SMALL)
#define CALIB_FONT_WIDTH (6 /*TODO: magic number*/)
#define CALIB_FONT_HEIGHT (8 /*TODO: magic number*/)

#define P_GAIN 1.5F    /* 完全停止用モーター制御比例係数 */
#define PWM_ABS_MAX 60 /* 完全停止用モーター制御PWM絶対最大値 */

/* 関数プロトタイプ宣言 */
static void drive(int r_power, int l_power);
static int sonar_alert(int sonar_alert_distance);
static void _syslog(int level, char *text);
static void _log(char *text);
static int abs(int value);
static float mathLimit(float value, float min, float max);
static int pid(int edge,int sensorVal,int forward,float pGain,float iGain,float dGain,int _center);
static int getRgb(void);
static void calibration(void);
static int waitPutButton(void);
static int waitSonar(void);

static void turnDriveAngleAction(int _forward_right, int _forward_left, int _angle);
static void turnAngleAction(int _forward, int _angle, int _turn);
static void turnAngleActionFaster(int _forward, int _angle, int _turn, int _max, int _delay_per_DELAY_TIME);
static void turnAngleActionSlower(int _forward, int _angle, int _turn, int _min, int _delay_per_DELAY_TIME);
static void turnColorAction(int _forward, int _color, int _restraintAngle, int _turn);
static void turnColorActionSlower(int _forward, int _color, int _restraintAngle, int _turn, int _min, int _delay_per_DELAY_TIME);
static void turnAngleAction_toStraighten(int _forward, int _angle, int _turn, int _min, int _delay_per_DELAY_TIME);
static void turnAngleActionSlower_toStraighten(int _forward, int _angle, int _turn, int _min_turn, int _delay_per_DELAY_TIME_turn, int _min_forward, int _delay_per_DELAY_TIME_forward);
static void turnColorAction_toStraighten(int _forward, int _turn, int _color, int _restraintAngle, int _min, int _delay_per_DELAY_TIME);
static void turnColorActionSlower_toStraighten(int _forward, int _turn, int _color, int _restraintAngle, int _min_turn, int _delay_per_DELAY_TIME_turn, int _min_forward, int _delay_per_DELAY_TIME_forward);

static void forwardAngleAction(int _forward, int _angle);
static void forwardAngleActionFaster(int _forward, int _angle, int _max, int _delay_per_DELAY_TIME);
static void forwardAngleActionSlower(int _forward, int _angle, int _min, int _delay_per_DELAY_TIME);
static void forwardColorAction(int _forward, int _color, int _restraintAngle);
static void forwardColorActionSlower(int _forward, int _color, int _restraintAngle, int _min, int _delay_per_DELAY_TIME);

static void pidLineTraceAction(int _edge, int _forward, float p, float i, float d);
static void pidLineTraceAngleAction(int _edge, int _forward, int _angle, float p, float i, float d);
static void pidLineTraceAngleActionFaster(int _edge, int _forward, int _angle, int _max, int _delay_per_DELAY_TIME, float p, float i, float d);
static void pidLineTraceAngleActionSlower(int _edge, int _forward, int _angle, int _min, int _delay_per_DELAY_TIME, float p, float i, float d);
static void pidLineTraceColorAction(int _edge, int _forward, int _color, int _restraintAngle, float p, float i, float d);
static void pidLineTraceColorActionSlower(int _edge, int _forward, int _color, int _restraintAngle, int _min, int _delay_per_DELAY_TIME, float p, float i, float d);


// static void tail_control(signed int angle);
// static void backlash_cancel(signed char lpwm, signed char rpwm, int32_t *lenc, int32_t *renc);


static void startCalibration(void)
{
    for (int i = 0; i < 3; i++)
    {
        if (i == 0)
            _log("Put Black");
        else if (i == 1)
            _log("Put White");
        waitPutButton();
        waitSonar();
        if (i == 0)
        {
            kuro = ev3_color_sensor_get_reflect(color_sensor);
            printf("[[[%d]]]",kuro);
        }
        else if (i == 1)
        {
            siro = ev3_color_sensor_get_reflect(color_sensor);
            printf("[[[%d]]]",siro);
        }
    }
    CENTER = (kuro + siro) / 2;
}

//*****************************************************************************
// 関数名 : m_control
// 引数 : goal (モーター目標角度[度])
// 返り値 : 無し
// 概要 : 走行体完全停止用モーターの角度制御
//*****************************************************************************
static void m_control(signed int goal)
{
    float pwm = (float)(goal - ev3_motor_get_counts(EV3_PORT_A)) * P_GAIN; /* 比例制御 */
    /* PWM出力飽和処理 */
    if (pwm > PWM_ABS_MAX)
    {
        pwm = PWM_ABS_MAX;
    }
    else if (pwm < -PWM_ABS_MAX)
    {
        pwm = -PWM_ABS_MAX;
    }
    if (pwm == 0)
    {
        ev3_motor_stop(arm_motor, true);
    }
    else
    {
        ev3_motor_set_power(arm_motor, (signed char)pwm);
    }
}



int onOffTrace(int forward, int _edge)
{
    int sensorVal=ev3_color_sensor_get_reflect(color_sensor);
    int turnVal=mathLimit((CENTER - sensorVal) * 8, -110, 110);
    ev3_motor_steer(left_motor,right_motor,(int)forward,(int)turnVal*_edge);
    return turnVal*_edge;
}




//*****************************************************************************
//関数名　:　trace
//引数　:　なし
//返り値　:　なし
//概要　:　ライントレースをする
//*****************************************************************************

void lineTrace_1()
{
    ev3_motor_set_power(arm_motor, 50);
    ev3_motor_stop(arm_motor, true);
    pidLineTraceAngleActionFaster(-1,10,3450,70,10,KP1,KI1,KD1);
    pidLineTraceAngleAction(-1,70,500,KP4_1,KI4_1,KD4_1);
    pidLineTraceAngleAction(-1,70,1800,KP1,KI1,KD1);
    pidLineTraceAngleAction(-1,70,600,KP4_2,KI4_2,KD4_2);
    pidLineTraceAngleAction(-1,70,200,KP1,KI1,KD1);
    turnDriveAngleAction(50,10,60);
    pidLineTraceAngleAction(1,70,800,KP3,KI3,KD3);
}

void lineTrace_2()
{   
    // pidLineTraceColorAction(-1,50,3,0,KP3,KI3,KD3);
    // pidLineTraceColorAction(1,50,2,0,KP3,KI3,KD3);
    turnAngleAction(50,40,30);
    pidLineTraceAngleAction(-1,70,1800,KP6,KI6,KD6);
    pidLineTraceAngleAction(-1,50,300,KP6,KI6,KD6);
    ev3_speaker_play_tone(240, 100);
}

void lineTrace_3()//doubleloop_small
{
    // turnAngleAction(50,80,-30);
    pidLineTraceColorAction(-1,70,3,0,KP7,KI7,KD7);
}

void lineTrace_4() /*doubleloop_last*/
{
    pidLineTraceColorAction(-1,40,2,0,KP7,KI7,KD7);
    turnDriveAngleAction(40,10,50);
    pidLineTraceAngleActionSlower(1,50,1450,20,60,KP3,KI3,KD3);
    turnColorActionSlower_toStraighten(50,-3,2,800,0,40,25,5);
}

void mission_1()
{
    pidLineTraceColorActionSlower(-1,30,5,0,5,10,KP1,KI1,KD1);
    // turnDriveAngleAction(50,10,90);
    turnColorActionSlower_toStraighten(30,-20,2,10,0,30,20,20);
}
// void lineTrace_5()
// {

//     pidLineTraceColorAction(-1,45,2,0,KP7,KI7,KD7);
//     pidLineTraceAngleAction(-1,50,1600,KP3,KI3,KD3);
// }

// void lineTrace_6()
// {
    
//     pidLineTraceColorAction(-1,40,2,0,KP1,KI1,KD1);
//     pidLineTraceAngleAction(-1,50,2500,KP7,KI7,KD7);

// }

// void lineTrace_7()
// {
//     pidLineTraceAngleAction(-1,50,2500,KP7,KI7,KD7);
//     ev3_speaker_play_tone(240, 100);
//     pidLineTraceColorAction(-1,40,5,0,KP7,KI7,KD7);

// }

/* メインタスク */
void main_task(intptr_t unused)
{
    /* LCD画面表示 */
    ev3_lcd_fill_rect(0, 0, EV3_LCD_WIDTH, EV3_LCD_HEIGHT, EV3_LCD_WHITE);

    _log("HackEV sample_c4");
    if (_LEFT)
        _log("Left course:");
    else
        _log("Right course:");

    /* センサー入力ポートの設定 */
    ev3_sensor_config(sonar_sensor, ULTRASONIC_SENSOR);
    ev3_sensor_config(color_sensor, COLOR_SENSOR);
    ev3_color_sensor_get_reflect(color_sensor); /* 反射率モード */
    ev3_sensor_config(touch_sensor, TOUCH_SENSOR);

    /* モーター出力ポートの設定 */
    ev3_motor_config(arm_motor, MEDIUM_MOTOR);
    ev3_motor_config(left_motor, LARGE_MOTOR);
    ev3_motor_config(right_motor, LARGE_MOTOR);

    if (_bt_enabled)
    {
        /* Open Bluetooth file */
        bt = ev3_serial_open_file(EV3_SERIAL_BT);
        assert(bt != NULL);

        /* Bluetooth通信タスクの起動 */
        act_tsk(BT_TASK);
    }

    ev3_led_set_color(LED_ORANGE); /* 初期化完了通知 */

    _log("Go to the start, ready?");
    if (_SIM)
        _log("Hit SPACE bar to start");
    else
        _log("Tap Touch Sensor to start");

    if (_bt_enabled)
    {
        fprintf(bt, "Bluetooth Remote Start: Ready.\n", EV3_SERIAL_BT);
        fprintf(bt, "send '1' to start\n", EV3_SERIAL_BT);
    }

    // for (int i = 0; i < 2; i++)
    // {
    //     if (i == 0)
    //         _log("Put Black");
    //     else if (i == 1)
    //         _log("Put White");
    //     waitPutButton();
    //     if (i == 0)
    //     {
    //         kuro = ev3_color_sensor_get_reflect(color_sensor);
    //         printf("[[[%d]]]",kuro);
    //     }
    //     else if (i == 1)
    //     {
    //         siro = ev3_color_sensor_get_reflect(color_sensor);
    //         printf("[[[%d]]]",siro);
    //     }
    //     ev3_speaker_play_tone(440, 100);
    // }
    // CENTER = (kuro + siro) / 2;

    // /* スタート待機 */
    // while (1)
    // {
    //     // tail_control(TAIL_ANGLE_STAND_UP); /* 完全停止用角度に制御 */
    //     // if (bt_cmd == 1)
    //     // {
    //     //     break; /* リモートスタート */
    //     // }
    //     waitPutButton();
    //     ev3_speaker_play_tone(640, 100);
    //     tslp_tsk(10 * 1000U); /* 10msecウェイト */
    //     break;
    // }
    startCalibration();
    // calibration();
    // printf("white:%d,black:%d\nnegative:%d,positive:%d\n",whitePoint,blackPoint,negative,positive);
    /* スタート待機 */
    // tail_control(TAIL_ANGLE_STAND_UP); /* 完全停止用角度に制御 */
    // if (bt_cmd == 1)
    // {
    //     break; /* リモートスタート */
    // }
    
    // ev3_speaker_play_tone(640, 100);
    tslp_tsk(10 * 1000U); /* 10msecウェイト */

    /* 走行モーターエンコーダーリセット */
    ev3_motor_reset_counts(arm_motor);
    ev3_motor_reset_counts(left_motor);
    ev3_motor_reset_counts(right_motor);

    ev3_led_set_color(LED_GREEN); /* スタート通知 */

    /**
     * Main loop
     */

    // int i[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    // if(flag == 0)
    // {
    //     ev3_motor_rotate(arm_motor,60,12,true);
    //     flag = 1;
    //     ao_count = 100;
    //     b_count = 100;
    // }
    // else
    // {
    //     ev3_color_sensor_get_rgb_raw(color_sensor, &rgb_val);
    //     printf("   %d:%d:%d   ",rgb_val.r,rgb_val.g,rgb_val.b);
    //     _log("debug");
    // }

    
    // testRun(-1,1200, 60);
    // lineTrace_0(-1,30,500);
    lineTrace_1();//start
    lineTrace_2();//1/4rad
    lineTrace_3();//doubleloop_small
    lineTrace_4();//doubleloop_last
    // lineTrace_5();
    // lineTrace_6();
    // lineTrace_7();
    ev3_speaker_play_tone(140, 100);
    //missionzone
    // mission_1();
    //     else if (ao_count == 1)
    //     {
    //         if (get_rgb() == 3 && right_wheel[0]-right_wheel[1]>200)
    //         {
    //             ao_count = 3;
    //             right_wheel[1] = right_wheel[0];
    //             left_wheel[1] = left_wheel[0];
    //             ev3_speaker_play_tone(440, 100);
    //         }
    //         // forward = turn = 0;
    //         // if (sonar_alert() == 1 ) /* 障害物検知 */
    //         // {
    //         //     forward = turn = 0; /* 障害物を検知したら停止 */
    //         // }
    //         // else
    //         // {
    //         lineTrace_2();//1/4rad
    //         //}
    //     }
    //     else if (ao_count == 2)
    //     {
    //         if (get_rgb() == 3 && right_wheel[0]-right_wheel[1]>200)
    //         {
    //             ao_count = 4;
                
    //             ev3_speaker_play_tone(440, 100);
    //             right_wheel[1] = right_wheel[0];
    //             left_wheel[1] = left_wheel[0];                        
    //         }
    //         // if (sonar_alert() == 1 /*|| right_wheel[0] > 10000*/) /* 障害物検知 */
    //         // {
    //         //     forward = turn = 0; /* 障害物を検知したら停止 */
    //         // }
    //         // else
    //         // {
    //         lineTrace_3();//doubleloop_small_no-ao-count_p
    //         // }

    //     }
    //     else if (ao_count == 3)
    //     {
    //         if (get_rgb()== 3 && left_wheel[0]-left_wheel[1]>400)
    //         {
    //             ao_count = 4;
    //             ev3_speaker_play_tone(440, 100);
    //             right_wheel[1] = right_wheel[0];
                
    //         }
    //         // if (sonar_alert() == 1 /*|| right_wheel[0] > 10000*/) /* 障害物検知 */
    //         // {
    //         //     forward = turn = 0; /* 障害物を検知したら停止 */
    //         //     // ev3_motor_rotate(left_motor,15,20,true);
    //         // }
    //         // else
    //         // {
    //         lineTrace_4();//doubleloop_small
    //         // }
    //     }
    //     else if(ao_count == 4)
    //     {
    //         if (get_rgb()==3 && right_wheel[0] - right_wheel[1]>1000)
    //         {
    //             ao_count = 5;
    //             ev3_speaker_play_tone(440,100);
    //             right_wheel[1] = right_wheel[0];
    //         }
    //         if (sonar_alert() == 1 /*|| right_wheel[0] > 10000*/) /* 障害物検知 */
    //         {
    //             forward = turn = 0; /* 障害物を検知したら停止 */
    //         }
    //         else
    //         {
    //             lineTrace_5();/*doubleloop_last*/
    //         }
    //     }
    //     else if(ao_count == 5)
    //     {
    //         if (get_rgb() == 5)
    //         {
    //             b_count = 1;
    //             ao_count = 0;
    //             ev3_speaker_play_tone(440,100);
    //             ev3_motor_reset_counts(right_motor);
    //             while (ev3_motor_get_counts(right_motor)<150)
    //             {
    //                 ev3_motor_set_power(right_motor,20);
    //                 ev3_motor_set_power(left_motor,-7);
    //             }
                
    //             ev3_speaker_play_tone(440,100);
    //             right_wheel[1] = right_wheel[0];
    //         }
    //         if(sonar_alert() == 1)
    //         {
    //             forward = turn = 0;
    //         }
    //         else
    //         {
    //             // forward = 30;
    //             // turn = pid(refrect,1,-1);
    //         }
    //     }
    // }
    // //R1
    // else if (b_count == 1)
    // {
    //     ev3_motor_set_power(arm_motor,90);
    //     ev3_motor_stop(arm_motor, true);                
    //     if (ao_count == 0)
    //     {
    //         if (get_rgb() == 3)
    //         {
    //             ao_count = 1;
    //             ev3_speaker_play_tone(440,100);
    //             ev3_motor_reset_counts(left_motor);
    //             while (ev3_motor_get_counts(left_motor) < 200)
    //             {
    //                 ev3_motor_set_power(right_motor,0);
    //                 ev3_motor_set_power(left_motor,20);
    //             }
    //             right_wheel[1] = right_wheel[0];
    //         }
    //         if (sonar_alert() == 1)
    //         {
    //             forward = turn = 0;
    //         }
    //         else
    //         {
    //             forward = 20;
    //             turn = 0;
    //         }
    //     }
    //     else if(ao_count == 1)
    //     {
    //         if (get_rgb() == 3)
    //         {
    //             ao_count = 2;
    //             ev3_speaker_play_tone(440,100);
    //             ev3_motor_reset_counts(right_motor);
    //             while (ev3_motor_get_counts(right_motor)<120)
    //             {
    //                 ev3_motor_set_power(left_motor,15);
    //                 ev3_motor_set_power(right_motor,17);
    //             }
    //             right_wheel[1] = right_wheel[0];
    //         }
    //         if (sonar_alert() == 1)
    //         {
    //             forward = turn = 0;
    //         }
    //         else 
    //         {
    //             forward =28;
    //             turn = pid(refrect,8,1);
    //         }
    //     }
    //     else if(ao_count == 2)
    //     {
    //         if(get_rgb() == 5)
    //         {
    //             ao_count = 3;
    //             aka_count = 1;
    //             ev3_speaker_play_tone(440,100);
    //             ev3_motor_reset_counts(right_motor);
    //             while (ev3_motor_get_counts(right_motor)<120)
    //             {
    //                 ev3_motor_set_power(left_motor,15);
    //                 ev3_motor_set_power(right_motor,17);
    //             }
    //             right_wheel[1] = right_wheel[0];
    //         }
    //         if(sonar_alert() == 1)
    //         {
    //             forward = turn = 0;
    //         }
    //         else
    //         {
    //             forward = 28;
    //             turn = pid(refrect,8,1);
    //         }
    //     }
    //     else if(aka_count == 1)
    //     {
    //         if(get_rgb() == 5)
    //         {
    //             aka_count = 2;
    //             ev3_speaker_play_tone(440,100);
    //             ev3_motor_stop(left_motor,true);
    //             ev3_motor_stop(right_motor,true);
    //             _log("debug");
    //             ev3_motor_reset_counts(right_motor);
    //             while (ev3_motor_get_counts(right_motor)<15)
    //             {
    //                 ev3_motor_set_power(right_motor,5);
    //                 ev3_motor_set_power(left_motor,5);
    //             }
    //             ev3_motor_stop(right_motor,true);
    //             ev3_motor_stop(left_motor,true);
    //             ev3_motor_rotate(arm_motor,60,12,true);
    //             _log("debug");
    //             right_wheel[1] = right_wheel[0];
    //         }
    //         if(sonar_alert() ==1)
    //         {
    //             forward = turn = 0;
    //         }
    //         else
    //         {
    //             forward = 25;
    //             turn =pid(refrect,8,1);
    //         }
    //     }
    //     else if(aka_count == 2)
    //     {
    //         ev3_color_sensor_get_rgb_raw(color_sensor,&rgb_val);
    //         printf("   %d:%d:%d   ",rgb_val.r,rgb_val.g,rgb_val.b);
    //         if(get_rgb() == 7)
    //         {
    //             _log("red_BLOCK");
    //             b_count = 2;
    //             ab_count = 1;
    //             ao_count = 0;
    //             aka_count = 0;
    //             ev3_speaker_play_tone(440,100);
    //             ev3_motor_stop(right_motor,true);
    //             ev3_motor_stop(left_motor,true);
    //             ev3_motor_rotate(arm_motor,-40,5,true);
    //             ev3_motor_reset_counts(right_motor);
    //             while (ev3_motor_get_counts(right_motor)<260)
    //             {
    //                 ev3_motor_set_power(left_motor,-2);
    //                 ev3_motor_set_power(right_motor,16);
    //             }
    //             ev3_motor_rotate(arm_motor,-20,5,true);
    //             ev3_speaker_play_tone(440,100);
    //             right_wheel[1] = right_wheel[0];
    //         }
    //         else if(get_rgb() == 8)
    //         {
    //             _log("EMPTY");
    //             b_count = 2;
    //             ao_count = 0;
    //             aka_count = 0;
    //             ev3_speaker_play_tone(280,100);
    //             ev3_motor_stop(right_motor,true);
    //             ev3_motor_stop(left_motor,true);
    //             ev3_motor_rotate(arm_motor,-60,5,true);
    //             ev3_motor_reset_counts(right_motor);
    //             while (ev3_motor_get_counts(right_motor)<140)
    //             {
    //                 ev3_motor_set_power(left_motor,-10);
    //                 ev3_motor_set_power(right_motor,20);
    //             }
    //             ev3_motor_reset_counts(right_motor);
    //             while (ev3_motor_get_counts(right_motor)<100)
    //             {
    //                 ev3_motor_set_power(left_motor,20);
    //                 ev3_motor_set_power(right_motor,20);
    //             }
    //             ev3_speaker_play_tone(440,100);
    //         }
    //         else if(get_rgb() == 9)
    //         {
    //             _log("blue_BLOCK");
    //             ev3_speaker_play_tone(360,100);
    //            
    // reset_counts(right_motor);
    //             while (ev3_motor_get_counts(right_motor)<140)
    //             {
    //                 ev3_motor_set_power(left_motor,-10);
    //                 ev3_motor_set_power(right_motor,20);
    //             }
    //             ev3_motor_reset_counts(right_motor);
    //             while (ev3_motor_get_counts(right_motor)<100)
    //             {
    //                 ev3_motor_set_power(left_motor,20);
    //                 ev3_motor_set_power(right_motor,20);
    //             }
    //             ev3_speaker_play_tone(440,100);
    //             right_wheel[1] = right_wheel[0];
    //             b_count = 2;
    //             ao_count = 0;
    //             aka_count = 0;
    //             w_count = 1;
    //         }
    //         if(sonar_alert() == 1)
    //         {
    //             forward = turn = 0;
    //         }
    //         else
    //         {
    //             forward = turn = 0;
    //         }
    //     }
    // }
    // //R2
    // else if (b_count == 2)
    // {
    //     ev3_motor_set_power(arm_motor,90);
    //     ev3_motor_stop(arm_motor, true);
    //     if (aka_count == 0)
    //     {
    //         if (get_rgb() == 5)
    //         {
    //             aka_count = 1;
    //             ev3_speaker_play_tone(440,100);
    //             ev3_motor_reset_counts(right_motor);
    //             while (ev3_motor_get_counts(right_motor)<200)
    //             {
    //                 ev3_motor_set_power(left_motor,-3);
    //                 ev3_motor_set_power(right_motor,15);
    //             }
    //             right_wheel[1] = right_wheel[0];
    //         }
    //         if (sonar_alert() == 1)
    //         {
    //             forward = turn = 0;
    //         }
    //         else
    //         {
    //             forward = 25;
    //             turn = pid(refrect,8,1);
    //         }
    //     }
    //     else if(aka_count == 1)
    //     {
    //         if (get_rgb() == 5)
    //         {
    //             aka_count = 2;
    //             ev3_speaker_play_tone(440,100);
    //             if(ab_count == 1)
    //             {
    //                 ev3_motor_stop(right_motor,true);
    //                 ev3_motor_stop(left_motor,true);
    //                 ev3_motor_rotate(arm_motor,25,5,true);   
    //             }
    //             ev3_motor_reset_counts(right_motor);
    //             while (ev3_motor_get_counts(right_motor)<120)
    //             {
    //                 ev3_motor_set_power(left_motor,18);
    //                 ev3_motor_set_power(right_motor,20);
    //             }
    //             if(ab_count == 1)
    //             {
    //                 ev3_motor_stop(right_motor,true);
    //                 ev3_motor_stop(left_motor,true);
    //                 ev3_motor_rotate(arm_motor,-20,5,true);
    //             }
    //             right_wheel[1] = right_wheel[0];
    //         }
    //         if (sonar_alert() == 1)
    //         {
    //             forward = turn = 0;
    //         }
    //         else 
    //         {
    //             forward =28;
    //             turn = pid(refrect,8,1);
    //         }
    //     }
    //     else if(aka_count == 2)
    //     {
    //         if(get_rgb() == 3)
    //         {
    //             ao_count = 1;
    //             aka_count = 3;
    //             ev3_speaker_play_tone(440,100);
    //             if(ab_count == 1)
    //             {
    //                 ev3_motor_stop(right_motor,true);
    //                 ev3_motor_stop(left_motor,true);
    //                 ev3_motor_rotate(arm_motor,25,5,true);
    //             }
    //             ev3_motor_reset_counts(right_motor);
    //             while (ev3_motor_get_counts(right_motor)<120)
    //             {
    //                 ev3_motor_set_power(left_motor,18);
    //                 ev3_motor_set_power(right_motor,20);
    //             }
    //             if(ab_count == 1)
    //             {
    //                 ev3_motor_stop(right_motor,true);
    //                 ev3_motor_stop(left_motor,true);
    //                 ev3_motor_rotate(arm_motor,-20,5,true);
    //             }
    //             right_wheel[1] = right_wheel[0];
    //         }
    //         if(sonar_alert() == 1)
    //         {
    //             forward = turn = 0;
    //         }
    //         else
    //         {
    //             forward = 28;
    //             turn = pid(refrect,8,1);
    //         }
    //     }
    //     else if(ao_count == 1)
    //     {
    //         if(get_rgb() == 3)
    //         {
    //             ao_count = 2;
    //             ev3_speaker_play_tone(440,100);
    //             ev3_motor_stop(right_motor,true);
    //             ev3_motor_stop(left_motor,true);
    //             if(ab_count != 1)
    //             {
    //                 ev3_motor_reset_counts(right_motor);
    //                 while (ev3_motor_get_counts(right_motor)<15)
    //                 {
    //                     ev3_motor_set_power(right_motor,5);
    //                     ev3_motor_set_power(left_motor,5);
    //                 }
    //                 ev3_motor_stop(right_motor,true);
    //                 ev3_motor_stop(left_motor,true);
    //                 ev3_motor_rotate(arm_motor,65,12,true);
    //             }
    //             right_wheel[1] = right_wheel[0];
    //         }
    //         if(sonar_alert() ==1)
    //         {
    //             forward = turn = 0;
    //         }
    //         else
    //         {
    //             forward = 28;
    //             turn =pid(refrect,8,1);
    //         }
    //     }
    //     else if(ao_count ==2)
    //     {
    //         if(ab_count == 1)
    //         {
    //             b_count = 3;
    //             ao_count = 0;
    //             ev3_motor_rotate(arm_motor,20,15,true);
    //             ev3_motor_reset_counts(left_motor);
    //             while (ev3_motor_get_counts(left_motor)<160)
    //             {
    //                 ev3_motor_set_power(left_motor,20);
    //                 ev3_motor_set_power(right_motor,-5);
    //             }
    //             ev3_motor_rotate(arm_motor,-20,15,true);
    //             ev3_speaker_play_tone(440,100);
    //         }
    //         else if(get_rgb() == 7)
    //         {
    //             _log("red_BLOCK");
    //             b_count = 3;
    //             ab_count = 1;
    //             ao_count = 0;
    //             aka_count = 0;
    //             ev3_speaker_play_tone(440,100);
    //             ev3_motor_stop(right_motor,true);
    //             ev3_motor_stop(left_motor,true);
    //             ev3_motor_rotate(arm_motor,-40,5,true);
    //             ev3_motor_reset_counts(left_motor);
    //             while (ev3_motor_get_counts(left_motor)<260)
    //             {
    //                 ev3_motor_set_power(left_motor,16);
    //                 ev3_motor_set_power(right_motor,-2);
    //             }
    //             ev3_motor_rotate(arm_motor,-20,5,true);
    //             ev3_speaker_play_tone(440,100);
    //             right_wheel[1] = right_wheel[0];
    //         }
    //         else if(get_rgb() == 8)
    //         {
    //             _log("EMPTY");
    //             b_count = 3;
    //             ao_count = 0;
    //             aka_count = 0;
    //             ev3_speaker_play_tone(280,100);
    //             ev3_motor_stop(right_motor,true);
    //             ev3_motor_stop(left_motor,true);
    //             ev3_motor_rotate(arm_motor,-65,5,true);
    //             ev3_motor_reset_counts(left_motor);
    //             while (ev3_motor_get_counts(left_motor)<140)
    //             {
    //                 ev3_motor_set_power(left_motor,20);
    //                 ev3_motor_set_power(right_motor,-10);
    //             }
    //             ev3_motor_reset_counts(right_motor);
    //             while (ev3_motor_get_counts(right_motor)<100)
    //             {
    //                 ev3_motor_set_power(left_motor,20);
    //                 ev3_motor_set_power(right_motor,20);
    //             }
    //             ev3_speaker_play_tone(440,100);
    //         }
    //         else if(get_rgb() == 9)
    //         {
    //             _log("blue_BLOCK");
    //             b_count = 3;
    //             ao_count = 0;
    //             aka_count = 0;
    //             w_count+=1 ;
    //             ev3_speaker_play_tone(360,100);
    //             ev3_motor_stop(right_motor,true);
    //             ev3_motor_stop(left_motor,true);
    //             ev3_motor_rotate(arm_motor,-60,5,true);
    //             ev3_motor_reset_counts(left_motor);
    //             while (ev3_motor_get_counts(left_motor)<140)
    //             {
    //                 ev3_motor_set_power(left_motor,20);
    //                 ev3_motor_set_power(right_motor,-10);
    //             }
    //             ev3_motor_reset_counts(right_motor);
    //             while (ev3_motor_get_counts(right_motor)<100)
    //             {
    //                 ev3_motor_set_power(left_motor,20);
    //                 ev3_motor_set_power(right_motor,20);
    //             }
    //             ev3_speaker_play_tone(440,100);
    //             right_wheel[1] = right_wheel[0];
    //         }
    //         if(sonar_alert() == 1)
    //         {
    //             forward = turn = 0;
    //         }
    //         else
    //         {
    //             forward = turn = 0;
    //         }
    //     }
    // }
    // //B4
    // else if(b_count == 3)
    // {
    //     ev3_motor_set_power(arm_motor,90);
    //     ev3_motor_stop(arm_motor, true);
    //     if(g_count == 0)
    //     {
    //         if(get_rgb() ==4)
    //         {
    //             g_count = 1;
    //             ev3_speaker_play_tone(440,100);
    //             ev3_motor_reset_counts(left_motor);
    //             while (ev3_motor_get_counts(left_motor) <= 195)
    //             {
    //                 ev3_motor_set_power(left_motor,15);
    //                 ev3_motor_set_power(right_motor,-5);
    //             }
    //         }
    //         if(sonar_alert() == 1)
    //         {
    //             forward = turn = 0;
    //         }
    //         else
    //         {
    //             forward = 25;
    //             turn = pid(refrect,8,-1);
    //         }
    //     }
    //     else if(g_count == 1)
    //     {
    //         if(get_rgb() == 4)
    //         {
    //             g_count = 2;
    //             ev3_speaker_play_tone(440,100);
    //             if(ab_count == 1)
    //             {
    //                 ev3_motor_stop(right_motor,true);
    //                 ev3_motor_stop(left_motor,true);
    //                 ev3_motor_rotate(arm_motor,25,5,true);
    //             }
    //             ev3_motor_reset_counts(right_motor);
    //             while (ev3_motor_get_counts(right_motor)<120)
    //             {
    //                 ev3_motor_set_power(left_motor,20);
    //                 ev3_motor_set_power(right_motor,18);
    //             }
    //             if(ab_count == 1)
    //             {
    //                 ev3_motor_stop(right_motor,true);
    //                 ev3_motor_stop(left_motor,true);
    //                 ev3_motor_rotate(arm_motor,-20,5,true);
    //             }
    //         }
    //         if(sonar_alert() == 1)
    //         {
    //             forward = turn = 0;
    //         }
    //         else
    //         {
    //             forward = 28;
    //             turn = pid(refrect,8,-1);
    //         }
    //     }
    //     else if(g_count == 2)
    //     {
    //         if(get_rgb() == 6)
    //         {
    //             g_count = 3;
    //             y_count = 1;
    //             ev3_speaker_play_tone(440,100);
    //             if(ab_count == 1)
    //             {
    //                 ev3_motor_stop(right_motor,true);
    //                 ev3_motor_stop(left_motor,true);
    //                 ev3_motor_rotate(arm_motor,25,5,true);
    //             }
    //             ev3_motor_reset_counts(right_motor);
    //             while (ev3_motor_get_counts(right_motor)<120)
    //             {
    //                 ev3_motor_set_power(left_motor,20);
    //                 ev3_motor_set_power(right_motor,18);
    //             }
    //             if(ab_count == 1)
    //             {
    //                 ev3_motor_stop(right_motor,true);
    //                 ev3_motor_stop(left_motor,true);
    //                 ev3_motor_rotate(arm_motor,-20,5,true);
    //             }
    //         }
    //         if(sonar_alert() == 1)
    //         {
    //             forward = turn = 0;
    //         }
    //         else
    //         {
    //             forward = 28;
    //             turn = pid(refrect,8,-1);
    //         }
    //     }
    //     else if(y_count == 1)
    //     {
    //         if(get_rgb() == 6)
    //         {
    //             y_count = 2;
    //             ev3_speaker_play_tone(440,100);
    //             ev3_motor_stop(right_motor,true);
    //             ev3_motor_stop(left_motor,true);
    //             if(ab_count != 1)
    //             {
    //                 ev3_motor_reset_counts(right_motor);
    //                 while (ev3_motor_get_counts(right_motor)<15)
    //                 {
    //                     ev3_motor_set_power(right_motor,5);
    //                     ev3_motor_set_power(left_motor,5);
    //                 }
    //                 ev3_motor_stop(right_motor,true);
    //                 ev3_motor_stop(left_motor,true);
    //                 ev3_motor_rotate(arm_motor,65,12,true);
    //             }
    //             right_wheel[1] = right_wheel[0];
    //         }
    //         if(sonar_alert() == 1)
    //         {
    //             forward = turn = 0;
    //         }
    //         else
    //         {
    //             forward = 28;
    //             turn = pid(refrect,8,-1);
    //         }
    //     }
    //     else if(y_count == 2)
    //     {
    //         if(ab_count == 1)
    //         {
    //             b_count = 4;
    //             y_count = 0;
    //             g_count = 0;
    //             ao_count = 0;
    //             ev3_motor_rotate(arm_motor,20,15,true);
    //             ev3_motor_reset_counts(right_motor);
    //             while (ev3_motor_get_counts(right_motor)< 160)
    //             {
    //                 ev3_motor_set_power(right_motor,20);
    //                 ev3_motor_set_power(left_motor,-5);
    //             }
                
    //             ev3_motor_rotate(arm_motor,-20,15,true);
    //             ev3_speaker_play_tone(440,100);
    //         }
    //         else if(get_rgb() == 7)
    //         {
    //             _log("red_BLOCK");
    //             b_count = 4;
    //             ab_count = 1;
    //             g_count = 0;
    //             y_count = 0;
    //             ev3_speaker_play_tone(440,100);
    //             ev3_motor_stop(right_motor,true);
    //             ev3_motor_stop(left_motor,true);
    //             ev3_motor_rotate(arm_motor,-40,5,true);
    //             ev3_motor_reset_counts(right_motor);
    //             while (ev3_motor_get_counts(right_motor)<260)
    //             {
    //                 ev3_motor_set_power(left_motor,-2);
    //                 ev3_motor_set_power(right_motor,16);
    //             }
    //             ev3_motor_rotate(arm_motor,-20,5,true);
    //             ev3_speaker_play_tone(440,100);
    //             right_wheel[1] = right_wheel[0];
    //         }
    //         else if(get_rgb() == 8)
    //         {
    //             _log("EMPTY");
    //             b_count = 4;
    //             g_count = 0;
    //             y_count = 0;
    //             ev3_speaker_play_tone(280,100);
    //             ev3_motor_stop(right_motor,true);
    //             ev3_motor_stop(left_motor,true);
    //             ev3_motor_rotate(arm_motor,-65,5,true);
    //             ev3_motor_reset_counts(right_motor);
    //             while (ev3_motor_get_counts(right_motor)<130)
    //             {
    //                 ev3_motor_set_power(left_motor,-10);
    //                 ev3_motor_set_power(right_motor,20);
    //             }
    //             ev3_motor_reset_counts(right_motor);
    //             while (ev3_motor_get_counts(right_motor)<100)
    //             {
    //                 ev3_motor_set_power(left_motor,20);
    //                 ev3_motor_set_power(right_motor,20);
    //             }
    //             ev3_speaker_play_tone(440,100);
    //         }
    //         else if(get_rgb() == 9)
    //         {
    //             _log("blue_BLOCK");
    //             b_count = 4;
    //             g_count = 0;
    //             y_count = 0;
    //             w_count++;
    //             ev3_speaker_play_tone(360,100);
    //             ev3_motor_stop(right_motor,true);
    //             ev3_motor_stop(left_motor,true);
    //             ev3_motor_rotate(arm_motor,-60,5,true);
    //             ev3_motor_reset_counts(right_motor);
    //             while (ev3_motor_get_counts(right_motor)<130)
    //             {
    //                 ev3_motor_set_power(left_motor,-10);
    //                 ev3_motor_set_power(right_motor,20);
    //             }
    //             ev3_motor_reset_counts(right_motor);
    //             while (ev3_motor_get_counts(right_motor)<100)
    //             {
    //                 ev3_motor_set_power(left_motor,20);
    //                 ev3_motor_set_power(right_motor,20);
    //             }
    //             ev3_speaker_play_tone(440,100);
    //             right_wheel[1] = right_wheel[0];
    //         }
    //         if(sonar_alert() == 1)
    //         {
    //             forward = turn = 0;
    //         }
    //         else
    //         {
    //             forward = turn = 0;
    //         }
    //     }

    // }
    // //Y2
    // else if(b_count == 4)
    // {
    //     ev3_motor_set_power(arm_motor,90);
    //     ev3_motor_stop(arm_motor, true);
    //     if(y_count == 0)
    //     {
    //         if(get_rgb() == 6)
    //         {
    //             y_count = 1;
    //             ev3_speaker_play_tone(440,100);
    //             ev3_motor_reset_counts(right_motor);
    //             while (ev3_motor_get_counts(right_motor)<200)
    //             {
    //                 ev3_motor_set_power(left_motor,-3);
    //                 ev3_motor_set_power(right_motor,15);
    //             }
    //         }
    //         if(sonar_alert() == 1)
    //         {
    //             forward = turn = 0;
    //         }
    //         else
    //         {
    //             forward = 25;
    //             turn = pid(refrect,8,1);
    //         }
    //     }
    //     else if(y_count == 1)
    //     {
    //         if(get_rgb() == 6)
    //         {
    //             y_count = 2;
    //             ev3_speaker_play_tone(440,100);
    //             if(ab_count == 1)
    //             {
    //                 ev3_motor_stop(right_motor,true);
    //                 ev3_motor_stop(left_motor,true);
    //                 ev3_motor_rotate(arm_motor,25,5,true);
    //             }
    //             ev3_motor_reset_counts(right_motor);
    //             while (ev3_motor_get_counts(right_motor)<120)
    //             {
    //                 ev3_motor_set_power(left_motor,18);
    //                 ev3_motor_set_power(right_motor,20);
    //             }
    //             if(ab_count == 1)
    //             {
    //                 ev3_motor_stop(right_motor,true);
    //                 ev3_motor_stop(left_motor,true);
    //                 ev3_motor_rotate(arm_motor,-20,5,true);
    //             }
    //         }
    //         if(sonar_alert() == 1)
    //         {
    //             forward = turn = 0;
    //         }
    //         else
    //         {
    //             forward = 30;
    //             turn = pid(refrect,8,1);
    //         }
    //     }
    //     else if(y_count == 2)
    //     {
    //         if(get_rgb() == 4)
    //         {
    //             g_count = 1;
    //             y_count = 3;
    //             ev3_speaker_play_tone(440,100);
    //             if(ab_count == 1)
    //             {
    //                 ev3_motor_stop(right_motor,true);
    //                 ev3_motor_stop(left_motor,true);
    //                 ev3_motor_rotate(arm_motor,25,5,true);
    //             }
    //             ev3_motor_reset_counts(right_motor);
    //             while (ev3_motor_get_counts(right_motor)<120)
    //             {
    //                 ev3_motor_set_power(left_motor,18);
    //                 ev3_motor_set_power(right_motor,20);
    //             }
    //             if(ab_count == 1)
    //             {
    //                 ev3_motor_stop(right_motor,true);
    //                 ev3_motor_stop(left_motor,true);
    //                 ev3_motor_rotate(arm_motor,-20,5,true);
    //             }
    //         }
    //         if(sonar_alert() == 1)
    //         {
    //             forward = turn = 0;
    //         }
    //         else
    //         {
    //             forward = 30;
    //             turn = pid(refrect,8,1);
    //         }
    //     }
    //     else if(g_count == 1)
    //     {
    //         if(get_rgb() == 4)
    //         {
    //             g_count = 2;
    //             ev3_speaker_play_tone(440,100);
    //             ev3_motor_stop(right_motor,true);
    //             ev3_motor_stop(left_motor,true);
    //             right_wheel[1] = right_wheel[0];
    //         }
    //         if(sonar_alert() == 1)
    //         {
    //             forward = turn = 0;
    //         }
    //         else
    //         {
    //             forward = 30;
    //             turn = pid(refrect,8,1);
    //         }
    //     }
    //     else if(g_count == 2)
    //     {
    //         if(sonar_alert() == 1)
    //         {
    //             forward = turn = 0;
    //         }
    //         else
    //         {
    //             ev3_motor_stop(right_motor,true);
    //             ev3_motor_stop(left_motor,true);
    //             ev3_motor_rotate(arm_motor,20,15,true);
    //             ev3_motor_rotate(left_motor,280,10,true);
    //             ev3_motor_rotate(arm_motor,-20,15,true);
    //             ev3_motor_stop(right_motor,true);
    //             ev3_motor_stop(left_motor,true);
    //             g_count = 3;
    //             b_count = 5;
    //         }
    //     }
    // }
    // //G4
    // else if(b_count == 5)
    // {
    //     if(get_rgb()==2)
    //     {
    //         ev3_speaker_play_tone(240, 100);
    //         ev3_motor_stop(right_motor,true);
    //         ev3_motor_stop(left_motor,true);
    //         ev3_motor_rotate(arm_motor,25,10,true);
    //         ev3_motor_reset_counts(left_motor);
    //         while(ev3_motor_get_counts(left_motor)<160)
    //         {
    //             ev3_motor_set_power(right_motor,-5);
    //             ev3_motor_set_power(left_motor,10);
    //         }
    //         ev3_motor_rotate(arm_motor,-20,10,true);
    //         b_count = 6;
    //         right_wheel[1] = right_wheel[0];
    //     }
    //     else
    //     {
    //         forward=25;
    //         turn = 0;
    //     }
    // }
    // //ガレージ
    // else if(b_count == 6)
    // {
    //     if(get_rgb() == 11 && right_wheel[0] - right_wheel[1] >1100)
    //     {
    //         b_count = 7;
    //         ev3_speaker_play_tone(440,100);
    //         _log("END");
    //         ev3_motor_stop(right_motor,true);
    //         ev3_motor_stop(left_motor,true);
    //         forward = turn = 0;
    //         break;
    //     }
    //     else
    //     {
    //         forward = 30;
    //         turn = pid(refrect,8,1);
    //     }
    // }
    // /*スラローム攻略開始*/
    // /*ここから調整していって*/


    /* 左右モータでロボットのステアリング操作を行う */
    // ev3_motor_steer(
    //     left_motor,
    //     right_motor,
    //     (int)forward,
    //     (int)turn);

    // tslp_tsk(DELAY_TIME); 


    ev3_motor_stop(right_motor, false);
    ev3_motor_stop(left_motor, false);

    if (_bt_enabled)
    {
        ter_tsk(BT_TASK);
        fclose(bt);
    }

    ext_tsk();
}

//**********************************************************************************************************************************************************
// 関数名 : sonar_alert
// 引数 : 無し
// 返り値 : 1(障害物あり)/0(障害物無し)
// 概要 : 超音波センサによる障害物検知
//**********************************************************************************************************************************************************
static int sonar_alert(int sonar_alert_distance)
{
    static unsigned int counter = 0;
    static int alert = 0;

    signed int distance;

    if (++counter == 40 / 4) /* 約40msec周期毎に障害物検知  */
    {
        /*
         * 超音波センサによる距離測定周期は、超音波の減衰特性に依存します。
         * NXTの場合は、40msec周期程度が経験上の最短測定周期です。
         * EV3の場合は、要確認
         */
        distance = ev3_ultrasonic_sensor_get_distance(sonar_sensor);
        if ((distance <= sonar_alert_distance) && (distance >= 0))
        {
            alert = 1; /* 障害物を検知 */
        }
        else
        {
            alert = 0; /* 障害物無し */
        }
        counter = 0;
    }
    return alert;
}

//**********************************************************************************************************************************************************
// 関数名 : bt_task
// 引数 : unused
// 返り値 : なし
// 概要 : Bluetooth通信によるリモートスタート。 Tera Termなどのターミナルソフトから、
//       ASCIIコードで1を送信すると、リモートスタートする。
//**********************************************************************************************************************************************************
void bt_task(intptr_t unused)
{
    while (1)
    {
        if (_bt_enabled)
        {
            uint8_t c = fgetc(bt); /* 受信 */
            switch (c)
            {
            case '1':
                bt_cmd = 1;
                break;
            default:
                break;
            }
            fputc(c, bt); /* エコーバック */
        }
    }
}

//**********************************************************************************************************************************************************
// 関数名 : _syslog
// 引数 :   int   level - SYSLOGレベル
//          char* text  - 出力文字列
// 返り値 : なし
// 概要 : SYSLOGレベルlebelのログメッセージtextを出力します。
//        SYSLOGレベルはRFC3164のレベル名をそのまま（ERRだけはERROR）
//        `LOG_WARNING`の様に定数で指定できます。
//**********************************************************************************************************************************************************
static void _syslog(int level, char *text)
{
    static int _log_line = 0;
    if (_SIM)
    {
        syslog(level, text);
    }
    else
    {
        ev3_lcd_draw_string(text, 0, CALIB_FONT_HEIGHT * _log_line++);
    }
}

//**********************************************************************************************************************************************************
// 関数名 : _log
// 引数 :   char* text  - 出力文字列
// 返り値 : なし
// 概要 : SYSLOGレベルNOTICEのログメッセージtextを出力します。
//**********************************************************************************************************************************************************
static void _log(char *text)
{
    _syslog(LOG_NOTICE, text);
}

//**********************************************************************************************************************************************************
//関数名　:　drive
//引数　:　右モータパワー, 左モータパワー
//返り値　:　なし
//概要　:　左右モータパワー調整可能
//**********************************************************************************************************************************************************
static void drive(int r_power, int l_power)
{
    ev3_motor_set_power(right_motor, r_power);
    ev3_motor_set_power(left_motor, l_power);
}

//**********************************************************************************************************************************************************
//関数名　:　abs
//引数　:　整数値
//返り値　:　引数として渡されたvalueの絶対値
//概要　:　絶対値表示
//**********************************************************************************************************************************************************
static int abs(int value)
{
    if (value>=0)
    {
        return value;
    }
    else if(value<0)
    {
        return -1*value;
    }
}

//**********************************************************************************************************************************************************
//関数名　:　mathLimit
//引数　:　浮動小数点値, 下限, 上限
//返り値　:　引数として渡されたvalueの制限された値
//概要　:　上限と下限でvalueの値を制限する
//**********************************************************************************************************************************************************
static float mathLimit(float value, float min, float max)
{
    if (value > max)
        value = max;
    if (value < min)
        value = min;
    return value;
}

//**********************************************************************************************************************************************************
//関数名　:　pid
//引数　:　左右トレース選択, 反射光の値, 速度, pGain, iGain, dGain, 反射光の目標値
//返り値　:　turn(曲がり比率)
//概要　:　pid制御走行を行う
//**********************************************************************************************************************************************************
static int pid(int edge,int sensorVal,int forward,float pGain,float iGain,float dGain,int _center)
{
    float p, i, d;
    
    diff[0] = diff[1];
    diff[1] = sensorVal - _center;
    integral += ((diff[1] + diff[0]) / 2.0) * DELTA_T;
    p = pGain * diff[1];
    i = iGain * integral;
    d = dGain * (diff[1] - diff[0]) / DELTA_T;

    int turnVal=mathLimit((p + i + d)*edge*_EDGE, -110, 110);
    ev3_motor_steer(left_motor,right_motor,(int)forward,(int)turnVal);

    return turnVal;
}

rgb_raw_t rgb_val;
//**********************************************************************************************************************************************************
//関数名　:　get_rgb
//引数　:　なし
//返り値　:　RGBの値から定義下カラーコードの値（int型）で返す
//概要　:　rbgか色コードを出力
//**********************************************************************************************************************************************************
static int getRgb(void)
{
    int color_et;
    int ret, bet, get;
    ev3_color_sensor_get_rgb_raw(color_sensor, &rgb_val);
    ret = rgb_val.r;
    get = rgb_val.g;
    bet = rgb_val.b;
    color_et = 0;
    // printf("\nr%d:g%d:b%d\n",rgb_val.r,rgb_val.g,rgb_val.b);
    // if (ret >= 120 && get >= 115 && bet >= 160 &&
    //     ret <= 140 && get <= 135 && bet <= 200)
    // {
    //     color_et = 1; /*白*/
    // }
    if (ret >= 5 && get >= 0 && bet >= 10 &&
        ret <= 50 && get <= 70 && bet <= 100)
    {                 // 20 40
        color_et = 2; /*黒*/
    }
    if (ret >= 5 && get >= 50 && bet >= 50 &&
        ret <= 45 && get <= 90 && bet <= 80 /*&&right_wheel[0]>7000*/)
    {
        color_et = 10; /*青+white*/
    }
    if (ret >= 10 && get >= 50 && bet >= 55 &&
        ret <= 45 && get <= 100 && bet <= 85)
    {
        color_et = 3; /*青*/
    }
    if (b_count != 0 && ret >= 10 && get >= 80 && bet >= 10 &&
        ret <= 40 && get <= 110 && bet <= 35)
    {
        color_et = 4; /*緑*/
    }
    if (ret >= 60 && get >= 10 && bet >= 0 &&
        ret <= 85 && get <= 40 && bet <= 35)
    {
        color_et = 5; /*赤*/
    }
    if (b_count != 0 && ret >= 60 && get >= 100 && bet >= 0 &&
        ret <= 90 && get <= 130 && bet <= 30)
    {
        color_et = 6; /*黄*/
    }
    if (b_count != 0 && ret >= 60 && get >= 10 && bet >= 0 &&
        ret <= 80 && get <= 30 && bet <= 10)
    {
        color_et = 7; // 赤ブロック
    }
    if (b_count != 0 && ret >= 0 && get >= 0 && bet >= 0 &&
        ret <= 9 && get <= 9 && bet <= 9)
    {
        color_et = 8; // 空
    }
    if (b_count != 0 && ret >= 10 && get >= 35 && bet >= 50 &&
        ret <= 20 && get <= 45 && bet <= 60)
    {
        color_et = 9; // 青
    }
    // if(ret >= 20 && get >= 65 && bet >= 50 &&
    //     ret <= 50 && get <= 95 && bet <= 80)
    // {
    //     color_et =11;
    // }


    return color_et;
}

//**********************************************************************************************************************************************************
// 関数名　:　calibration
// 引数　:　なし
// 返り値　:　なし
// 概要　:　色判定範囲指定用の測定用関数
//**********************************************************************************************************************************************************
static void calibration(void)
{
    while(1)
    {
        waitSonar();
        ev3_color_sensor_get_rgb_raw(color_sensor, &rgb_val);
        printf("\n   r:g:b=%d:%d:%d   \n",rgb_val.r,rgb_val.g,rgb_val.b);
        printf("\n");
        printf("%d\n",getRgb());
        _log("debug");
        tslp_tsk(40 * 1000U);
    }
}

//**********************************************************************************************************************************************************
// 関数名　:　waitPutButton
// 引数　: なし
// 返り値　:　1
// 概要　:　タッチセンサでの待機関数
//**********************************************************************************************************************************************************
static int waitPutButton(void)
{
    const int BUTTON_CHECK_NUMBER_OF_TIMES = 250;
    for(int _i=0; _i<BUTTON_CHECK_NUMBER_OF_TIMES; _i++)
    {
        while(ev3_touch_sensor_is_pressed(touch_sensor) == 0) // ボタンが押されるまでループ
            tslp_tsk(DELAY_TIME);
        tslp_tsk(DELAY_TIME);
    }
    ev3_speaker_play_tone(240, 100);
    for(int _i=0; _i<BUTTON_CHECK_NUMBER_OF_TIMES/2; _i++)
    {
        while(ev3_touch_sensor_is_pressed(touch_sensor) == 1) // 押したボタンが離されるまでループ
            tslp_tsk(DELAY_TIME);
        tslp_tsk(DELAY_TIME);
    }
    ev3_speaker_play_tone(440, 100);
    return 1;
}

//**********************************************************************************************************************************************************
// 関数名　:　waitSonar
// 引数　: なし
// 返り値　:　1
// 概要　:　超音波センサでの待機関数
//**********************************************************************************************************************************************************
static int waitSonar(void)
{
    while (sonar_alert(10))
        tslp_tsk(DELAY_TIME); 
    while (sonar_alert(10) == 0)
        tslp_tsk(DELAY_TIME); 
    ev3_speaker_play_tone(640, 100);
    return 1;
}
//**********************************************************************************************************************************************************
// 関数名　:　turnDriveAngleAction
// 引数　:　右ホイール速度, 左ホイール速度,走行角度
// 返り値　:　なし
// 概要　:　左右のモーターパワーを設定可能
//**********************************************************************************************************************************************************
static void turnDriveAngleAction(int _forward_right, int _forward_left, int _angle)
{
    _wheelCountTemp = (ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0;
    while ((ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0 < _wheelCountTemp + _angle)
    {
        ev3_motor_set_power(right_motor, _forward_right);
        ev3_motor_set_power(left_motor, _forward_left);
        tslp_tsk(DELAY_TIME); 
    }
}

//**********************************************************************************************************************************************************
// 関数名　:　turnAngleAction
// 引数　:　速度, 走行角度, 左右モーターパワー比
// 返り値　:　なし
// 概要　:　角度曲進する（モーターの個体差は考えない）
//**********************************************************************************************************************************************************
static void turnAngleAction(int _forward, int _angle, int _turn)
{
    _wheelCountTemp = (ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0;
    while ((ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0 < _wheelCountTemp + _angle)
    {
        ev3_motor_steer(left_motor,right_motor,(int)_forward,(int)_turn);
        tslp_tsk(DELAY_TIME); 
    }
}

//**********************************************************************************************************************************************************
// 関数名　:　turnAngleActionFaster
// 引数　:　速度, 走行角度, 左右モーターパワー比, 増加速度上限, 増加率
// 返り値　:　なし
// 概要　:　加速しつつ角度曲進する（モーターの個体差は考えない）
//**********************************************************************************************************************************************************
static void turnAngleActionFaster(int _forward, int _angle, int _turn, int _max, int _delay_per_DELAY_TIME)
{
    _wheelCountTemp = (ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0;
    static int _delay=0;
    while ((ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0 < _wheelCountTemp + _angle)
    {
        ev3_motor_steer(left_motor,right_motor,(int)_forward,(int)_turn);
        _delay++;
        if (_delay%_delay_per_DELAY_TIME==0)
        {
            if (_max > _forward)
            {
                _forward+=1;
            }
        }
        tslp_tsk(DELAY_TIME+PID_DELAY_TIME); 
    }
}

//**********************************************************************************************************************************************************
// 関数名　:　turnAngleActionSlower
// 引数　:　速度, 走行角度, 左右モーターパワー比, 減少速度下限, 減少率
// 返り値　:　なし
// 概要　:　減速しつつ角度曲進する（モーターの個体差は考えない）
//**********************************************************************************************************************************************************
static void turnAngleActionSlower(int _forward, int _angle, int _turn, int _min, int _delay_per_DELAY_TIME)
{
    _wheelCountTemp = (ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0;
    static int _delay=0;
    while ((ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0 < _wheelCountTemp + _angle)
    {
        ev3_motor_steer(left_motor,right_motor,(int)_forward,(int)_turn);
        _delay++;
        if (_delay%_delay_per_DELAY_TIME==0)
        {
            if (_min < _forward)
            {
                _forward-=1;
            }
        }
        tslp_tsk(DELAY_TIME+PID_DELAY_TIME); 
    }
}

//**********************************************************************************************************************************************************
// 関数名　:　turnColorAction
// 引数　:　速度, 指定色, 読み取り停止角度距離
// 返り値　:　なし
// 概要　:　指定色検知まで角度曲進する（モーターの個体差は考えない）
//**********************************************************************************************************************************************************
static void turnColorAction(int _forward, int _turn, int _color, int _restraintAngle)
{
    _wheelCountTemp = (ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0;
    while (getRgb() != _color || (ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0 < _wheelCountTemp + _restraintAngle)
    {
        ev3_motor_steer(left_motor,right_motor,(int)_forward,(int)_turn);
        tslp_tsk(DELAY_TIME); 
    }
    ev3_speaker_play_tone(440, 100);   
}

//**********************************************************************************************************************************************************
// 関数名　:　turnColorActionSlower
// 引数　:　速度, 指定色, 読み取り停止角度距離, 減少速度下限, 減少率
// 返り値　:　なし
// 概要　:　減速しつつ指定色検知まで角度曲進する（モーターの個体差は考えない）
//**********************************************************************************************************************************************************
static void turnColorActionSlower(int _forward, int _turn, int _color, int _restraintAngle, int _min, int _delay_per_DELAY_TIME)
{
    _wheelCountTemp = (ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0;
    static int _delay=0;
    turnAngleAction(_forward, _restraintAngle, _turn);
    while (getRgb() != _color)
    {
        ev3_motor_steer(left_motor,right_motor,(int)_forward,(int)_turn);
        _delay++;
        if (_delay%_delay_per_DELAY_TIME==0)
        {
            if (_min < _forward)
            {
                _forward-=1;
            }
        }
        tslp_tsk(DELAY_TIME); 
    }
    ev3_speaker_play_tone(440, 100);   
}

//**********************************************************************************************************************************************************
// 関数名　:　turnAngleAction_toStraighten
// 引数　:　速度, 走行角度, 左右モーターパワー比, 減少角度下限, 減少率
// 返り値　:　なし
// 概要　:　角度曲進の際に曲進角を徐々に緩める
//**********************************************************************************************************************************************************
static void turnAngleAction_toStraighten(int _forward, int _angle, int _turn, int _min, int _delay_per_DELAY_TIME)
{
    _wheelCountTemp = (ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0;
    static int _delay=0;
    static int sign;
    if (_turn<0)
    {
        sign=-1;
    }
    else if(_turn>0)
    {
        sign=1;
    }
    else
    {
        sign=0;
    }
    while ((ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0 < _wheelCountTemp + _angle)
    {
        ev3_motor_steer(left_motor,right_motor,(int)_forward,(int)_turn);
        _delay++;
        if (_delay%_delay_per_DELAY_TIME==0)
        {
            if (abs(_min) < abs(_turn))
            {
                _turn-=sign;
            }
        }
        
        tslp_tsk(DELAY_TIME); 
    }
}

//**********************************************************************************************************************************************************
// 関数名　:　turnAngleActionSlower_toStraighten
// 引数　:　速度, 走行角度, 左右モーターパワー比, 減少角度下限, 角度値減少率, 減少速度下限, 速度値減少率
// 返り値　:　なし
// 概要　:　減速しつつ角度曲進する（モーターの個体差は考えない）
//**********************************************************************************************************************************************************
static void turnAngleActionSlower_toStraighten(int _forward, int _angle, int _turn, int _min_turn, int _delay_per_DELAY_TIME_turn, int _min_forward, int _delay_per_DELAY_TIME_forward)
{
    _wheelCountTemp = (ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0;
    static int _delay=0;
    static int sign;
    if (_turn<0)
    {
        sign=-1;
    }
    else if(_turn>0)
    {
        sign=1;
    }
    else
    {
        sign=0;
    }
    while ((ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0 < _wheelCountTemp + _angle)
    {
        ev3_motor_steer(left_motor,right_motor,(int)_forward,(int)_turn);
        _delay++;
        if (_delay%_delay_per_DELAY_TIME_turn==0)
        {
            if (abs(_min_turn) < abs(_turn))
            {
                _turn-=sign;
            }
        }
        if (_delay%_delay_per_DELAY_TIME_forward==0)
        {
            if (_min_forward < _forward)
            {
                _forward-=1;
            }
        }
        tslp_tsk(DELAY_TIME); 
    }
}

//**********************************************************************************************************************************************************
// 関数名　:　turnColorAction_toStraighten
// 引数　:　速度, 左右モーターパワー比, 指定色, 読み取り停止角度距離, 減少角度下限, 角度値減少率
// 返り値　:　なし
// 概要　:　減速しつつ角度曲進する（モーターの個体差は考えない）
//**********************************************************************************************************************************************************
static void turnColorAction_toStraighten(int _forward, int _turn, int _color, int _restraintAngle, int _min, int _delay_per_DELAY_TIME)
{
    _wheelCountTemp = (ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0;
    static int _delay=0;
    static int sign;
    if (_turn<0)
    {
        sign=-1;
    }
    else if(_turn>0)
    {
        sign=1;
    }
    else
    {
        sign=0;
    }
    turnAngleAction(_forward, _restraintAngle, _turn);
    while (getRgb() != _color || (ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0 < _wheelCountTemp + _restraintAngle)
    {
        ev3_motor_steer(left_motor,right_motor,(int)_forward,(int)_turn);
        _delay++;
        if (_delay%_delay_per_DELAY_TIME==0)
        {
            if (abs(_min) < abs(_turn))
            {
                _turn-=sign;
            }
        }
        tslp_tsk(DELAY_TIME); 
    }
}

//**********************************************************************************************************************************************************
// 関数名　:　turnColorActionSlower_toStraighten
// 引数　:　速度, 左右モーターパワー比, 指定色, 読み取り停止角度距離, 減少角度下限, 角度値減少率, 減少速度下限, 速度値減少率
// 返り値　:　なし
// 概要　:　減速しつつ角度曲進する（モーターの個体差は考えない）
//**********************************************************************************************************************************************************
static void turnColorActionSlower_toStraighten(int _forward, int _turn, int _color, int _restraintAngle, int _min_turn, int _delay_per_DELAY_TIME_turn, int _min_forward, int _delay_per_DELAY_TIME_forward)
{
    _wheelCountTemp = (ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0;
    static int _delay=0;
    static int sign=0;
    if (_turn<0)
    {
        sign=-1;
    }
    else if(_turn>0)
    {
        sign=1;
    }
    turnAngleAction(_forward, _restraintAngle, _turn);
    while (getRgb() != _color)
    {
        ev3_motor_steer(left_motor,right_motor,(int)_forward,(int)_turn);
        _delay++;
        if (_delay%_delay_per_DELAY_TIME_turn==0)
        {
            if (abs(_min_turn) < abs(_turn))
            {
                _turn-=sign;
                printf("%d\n",_turn);
            }
        }
        if (_delay%_delay_per_DELAY_TIME_forward==0)
        {
            if (_min_forward < _forward)
            {
                _forward-=1;
            }
        }
        tslp_tsk(DELAY_TIME); 
    }
}

//**********************************************************************************************************************************************************
// 関数名　:　forwardAngleAction
// 引数　:　速度, 走行角度
// 返り値　:　なし
// 概要　:　角度直進する（モーターの個体差は考えない）
//**********************************************************************************************************************************************************
static void forwardAngleAction(int _forward, int _angle)
{
    _wheelCountTemp = (ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0;
    while ((ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0 < _wheelCountTemp + _angle)
    {
        ev3_motor_set_power(right_motor, _forward/LR_MOTOR_RATIO);
        ev3_motor_set_power(left_motor, _forward*LR_MOTOR_RATIO);
        tslp_tsk(DELAY_TIME); 
    }
}

//**********************************************************************************************************************************************************
// 関数名　:　forwardAngleActionFaster
// 引数　:　速度, 走行角度, 増加速度上限, 増加率
// 返り値　:　なし
// 概要　:　加速しつつ角度直進する（モーターの個体差は考えない）
//**********************************************************************************************************************************************************
static void forwardAngleActionFaster(int _forward, int _angle, int _max, int _delay_per_DELAY_TIME)
{
    _wheelCountTemp = (ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0;
    static int _delay=0;
    while ((ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0 < _wheelCountTemp + _angle)
    {
        ev3_motor_set_power(right_motor, _forward/LR_MOTOR_RATIO);
        ev3_motor_set_power(left_motor, _forward*LR_MOTOR_RATIO);
        _delay++;
        if (_delay%_delay_per_DELAY_TIME==0)
        {
            if (_max > _forward)
            {
                _forward+=1;
            }
        }
        tslp_tsk(DELAY_TIME); 
    }
}
//**********************************************************************************************************************************************************
// 関数名　:　forwardAngleActionSlower
// 引数　:　速度, 移動角度, 減少速度下限, 減少率
// 返り値　:　なし
// 概要　:　減速しつつ角度直進する（モーターの個体差は考えない）
//**********************************************************************************************************************************************************
static void forwardAngleActionSlower(int _forward, int _angle, int _min, int _delay_per_DELAY_TIME)
{
    _wheelCountTemp = (ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0;
    static int _delay=0;
    while ((ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0 < _wheelCountTemp + _angle)
    {
        ev3_motor_set_power(right_motor, _forward/LR_MOTOR_RATIO);
        ev3_motor_set_power(left_motor, _forward*LR_MOTOR_RATIO);
        _delay++;
        if (_delay%_delay_per_DELAY_TIME==0)
        {
            if (_min < _forward)
            {
                _forward-=1;
            }
        }
        tslp_tsk(DELAY_TIME);
    }
}

//**********************************************************************************************************************************************************
// 関数名　:　forwardColorAction
// 引数　:　速度, 指定色, 読み取り停止角度距離
// 返り値　:　なし
// 概要　:　指定色検知まで直進する（モーターの個体差は考えない）
//**********************************************************************************************************************************************************
static void forwardColorAction(int _forward, int _color, int _restraintAngle)
{
    _wheelCountTemp = (ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0;
    forwardAngleAction(_forward, _restraintAngle);
    while (getRgb() != _color)
    {
        ev3_motor_set_power(right_motor, _forward/LR_MOTOR_RATIO);
        ev3_motor_set_power(left_motor, _forward*LR_MOTOR_RATIO);
        tslp_tsk(DELAY_TIME);
    }
    ev3_speaker_play_tone(440, 100);
}

//**********************************************************************************************************************************************************
// 関数名　:　forwardColorActionSlower
// 引数　:　速度, 指定色, 読み取り停止角度距離, 減少速度下限, 減少率
// 返り値　:　なし
// 概要　:　減速しつつ指定色検知まで直進する（モーターの個体差は考えない）
//**********************************************************************************************************************************************************
static void forwardColorActionSlower(int _forward, int _color, int _restraintAngle, int _min, int _delay_per_DELAY_TIME)
{
    _wheelCountTemp = (ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0;
    static int _delay=0;
    forwardAngleAction(_forward, _restraintAngle);
    while (getRgb() != _color)
    {
        ev3_motor_set_power(right_motor, _forward/LR_MOTOR_RATIO);
        ev3_motor_set_power(left_motor, _forward*LR_MOTOR_RATIO);
        _delay++;
        if (_delay%_delay_per_DELAY_TIME==0)
        {
            if (_min < _forward)
            {
                _forward-=1;
            }
        }
        tslp_tsk(DELAY_TIME);
    }
    ev3_speaker_play_tone(440, 100);
}
//**********************************************************************************************************************************************************
// 関数名　:　pidLineTraceAction
// 引数　:　左右トレース選択, 速度, pGain, iGain, dGain
// 返り値　:　なし
// 概要　:　PID制御走行を行う
//**********************************************************************************************************************************************************
static void pidLineTraceAction(int _edge, int _forward, float p, float i, float d)
{
    _refrect = ev3_color_sensor_get_reflect(color_sensor);
    pid( _edge, _refrect, _forward, p, i, d, CENTER);
    tslp_tsk(DELAY_TIME+PID_DELAY_TIME);
}

//**********************************************************************************************************************************************************
// 関数名　:　pidLineTraceAngleAction
// 引数　:　左右トレース選択, 速度, 走行角度, pGain, iGain, dGain
// 返り値　:　なし
// 概要　:　PID制御で角度走行を行う
//**********************************************************************************************************************************************************
static void pidLineTraceAngleAction(int _edge, int _forward, int _angle, float p, float i, float d)
{
    _wheelCountTemp = (ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0;
    while ((ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0 < _wheelCountTemp + _angle)
    {
        _refrect = ev3_color_sensor_get_reflect(color_sensor);
        pid( _edge, _refrect, _forward, p, i, d, CENTER);
        tslp_tsk(DELAY_TIME+PID_DELAY_TIME);
    }
}

//**********************************************************************************************************************************************************
// 関数名　:　pidLineTraceAngleActionFaster
// 引数　:　左右トレース選択, 速度, 走行角度, 増加速度上限, 増加率, pGain, iGain, dGain
// 返り値　:　なし
// 概要　:　加速しつつPID制御で角度走行を行う
//**********************************************************************************************************************************************************
static void pidLineTraceAngleActionFaster(int _edge, int _forward, int _angle, int _max, int _delay_per_DELAY_TIME, float p, float i, float d)
{
    _wheelCountTemp = (ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0;
    static int _delay=0;
    while ((ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0 < _wheelCountTemp + _angle)
    {
        _refrect = ev3_color_sensor_get_reflect(color_sensor);
        pid( _edge, _refrect, _forward, p, i, d, CENTER);
        _delay++;
        if (_delay%_delay_per_DELAY_TIME==0)
        {
            if (_max > _forward)
            {
                _forward+=1;
            }
        }
        tslp_tsk(DELAY_TIME+PID_DELAY_TIME);
    }
}

//**********************************************************************************************************************************************************
// 関数名　:　pidLineTraceAngleActionSlower
// 引数　:　左右トレース選択, 速度, 走行角度, 減少速度下限, 減少率, pGain, iGain, dGain
// 返り値　:　なし
// 概要　:　減速しつつPID制御で角度走行を行う
//**********************************************************************************************************************************************************
static void pidLineTraceAngleActionSlower(int _edge, int _forward, int _angle, int _min, int _delay_per_DELAY_TIME, float p, float i, float d)
{
    _wheelCountTemp = (ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0;
    static int _delay=0;
    while ((ev3_motor_get_counts(right_motor) + ev3_motor_get_counts(left_motor)) / 2.0 < _wheelCountTemp + _angle)
    {
        _refrect = ev3_color_sensor_get_reflect(color_sensor);
        pid( _edge, _refrect, _forward, p, i, d, CENTER);
        _delay++;
        if (_delay%_delay_per_DELAY_TIME==0)
        {
            if (_min < _forward)
            {
                _forward-=1;
            }
        }
        tslp_tsk(DELAY_TIME);
    }
}

//**********************************************************************************************************************************************************
// 関数名　:　pidLineTraceColorAction
// 引数　:　左右トレース選択, 速度, 指定色, 読み取り停止角度距離, pGain, iGain, dGain
// 返り値　:　なし
// 概要　:　指定色検知までPID制御走行を行う
//**********************************************************************************************************************************************************
static void pidLineTraceColorAction(int _edge, int _forward, int _color, int _restraintAngle, float p, float i, float d)
{
    pidLineTraceAngleAction(_edge, _forward, _restraintAngle, p, i, d);
    CENTER-=7;
    while (getRgb() != _color)
    {
        pidLineTraceAngleAction(_edge, _forward, 20, p, i, d);
    }
    CENTER+=7;
    ev3_speaker_play_tone(440, 100);
}

//**********************************************************************************************************************************************************
// 関数名　:　pidLineTraceColorActionSlower
// 引数　:　左右トレース選択, 速度, 指定色, 読み取り停止角度距離, 減少速度下限, 減少率, pGain, iGain, dGain
// 返り値　:　なし
// 概要　:　減速しつつ指定色検知までPID制御走行を行う
//**********************************************************************************************************************************************************
static void pidLineTraceColorActionSlower(int _edge, int _forward, int _color, int _restraintAngle, int _min, int _delay_per_DELAY_TIME, float p, float i, float d)
{
    pidLineTraceAngleAction(_edge, _forward, _restraintAngle, p, i, d);
    static int _delay=0;
    CENTER-=7;
    while (getRgb() != _color)
    {
        pidLineTraceAngleAction(_edge, _forward, 20, p, i, d);
        _delay++;
        if (_delay%_delay_per_DELAY_TIME==0)
        {
            if (_min < _forward)
            {
                _forward-=1;
            }
        }
    }
    CENTER+=7;
    ev3_speaker_play_tone(440, 100);
}