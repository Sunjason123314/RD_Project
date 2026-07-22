#ifndef DLP_H
#define DLP_H

//标准库
#include "dlpc_common.h"
#include "dlpc347x.h"
#include "dlpc347x_internal_patterns.h"
#include "cypress_i2c.h"
#include "stdio.h"
#include "math.h"
#include "time.h"
#include <map>
#include <string>
#include <vector>
#include <functional>
//#pragma comment(lib,"G:/projects/Qt/qt_camera/build-qt_camera-Desktop_Qt_5_15_2_MSVC2019_64bit-Debug/dlpc347x.lib")
//#pragma comment(lib,"G:/projects/Qt/qt_camera/qt_camera/DLP/third_party/cyusbserial/cyusbserial.lib")
#include <QImage>
#include <opencv2/core.hpp>


#define SUCCESS                           0
#define FAIL                              1

#define MAX_WIDTH                         DLP4710_WIDTH
#define MAX_HEIGHT                        DLP4710_HEIGHT

#define NUM_PATTERN_SETS                  4
#define NUM_PATTERN_ORDER_TABLE_ENTRIES   6
#define NUM_ONE_BIT_HORIZONTAL_PATTERNS   4
#define NUM_EIGHT_BIT_HORIZONTAL_PATTERNS 6
#define NUM_ONE_BIT_VERTICAL_PATTERNS     4
#define NUM_EIGHT_BIT_VERTICAL_PATTERNS   6
#define TOTAL_HORIZONTAL_PATTERNS         (NUM_ONE_BIT_HORIZONTAL_PATTERNS + NUM_EIGHT_BIT_HORIZONTAL_PATTERNS)
#define TOTAL_VERTICAL_PATTERNS           (NUM_ONE_BIT_VERTICAL_PATTERNS + NUM_EIGHT_BIT_VERTICAL_PATTERNS)

// 本项目的 Cypress USB-I2C 桥单次 I2C 写有长度上限，1024+1 字节的 CyI2cWrite 会失败
// （日志 "CYPRESS_I2C_WriteI2C failed, length=1025"）。TI DLP 论坛与多份资料指出 flash
// 写数据需分块，256 字节是安全且推荐的块大小（小于 256 反而会有问题）。官方上位机能烧录
// 也是因其内部采用较小的分块传输。故将写块从 1024 降到 256。
#define FLASH_WRITE_BLOCK_SIZE            256
#define FLASH_READ_BLOCK_SIZE             256

#define MAX_WRITE_CMD_PAYLOAD             (FLASH_WRITE_BLOCK_SIZE + 8)
#define MAX_READ_CMD_PAYLOAD              (FLASH_READ_BLOCK_SIZE  + 8)

static uint8_t                                   s_HorizontalPatternData[TOTAL_HORIZONTAL_PATTERNS][MAX_HEIGHT];
static uint8_t                                   s_VerticalPatternData[TOTAL_VERTICAL_PATTERNS][MAX_WIDTH];
static DLPC347X_INT_PAT_PatternData_s            s_Patterns[TOTAL_HORIZONTAL_PATTERNS + TOTAL_VERTICAL_PATTERNS];//一个图像的数据
static DLPC347X_INT_PAT_PatternSet_s             s_PatternSets[NUM_PATTERN_SETS];//多个set的数据
static DLPC347X_INT_PAT_PatternOrderTableEntry_s s_PatternOrderTable[NUM_PATTERN_ORDER_TABLE_ENTRIES];

static uint8_t                                   s_WriteBuffer[MAX_WRITE_CMD_PAYLOAD];
static uint8_t                                   s_ReadBuffer[MAX_READ_CMD_PAYLOAD];

static bool                                      s_StartProgramming;
static uint8_t                                   s_FlashProgramBuffer[FLASH_WRITE_BLOCK_SIZE];
static uint16_t                                  s_FlashProgramBufferPtr;

static FILE*                                     s_FilePointer;

using namespace std;
class DLP
{
public:
    DLP();
    ~DLP();

    // 投影仪操作模式控制
    DLPC347X_OperatingMode_e Current_OperatingMode;
    map<string,DLPC347X_OperatingMode_e> OperatingModeList{
        {"EXTERNAL_VIDEO_PORT",DLPC347X_OM_EXTERNAL_VIDEO_PORT},
        {"TEST_PATTERN_GENERATOR",DLPC347X_OM_TEST_PATTERN_GENERATOR},
        {"SPLASH_SCREEN",DLPC347X_OM_SPLASH_SCREEN},
        {"SENS_EXTERNAL_PATTERN",DLPC347X_OM_SENS_EXTERNAL_PATTERN},
        {"SENS_INTERNAL_PATTERN",DLPC347X_OM_SENS_INTERNAL_PATTERN},
        {"SENS_SPACE_CODED_PATTERN",DLPC347X_OM_SENS_SPACE_CODED_PATTERN},
        {"STANDBY",DLPC347X_OM_STANDBY}
    };
    bool SelectOperatingMode(DLPC347X_OperatingMode_e OperatingMode);

    //图案投影控制
//    DLPC347X_PatternControl_e Current_PatternControl;
    map<string,DLPC347X_PatternControl_e> PatternControlList{
        {"DLPC347X_PC_START",DLPC347X_PC_START},
        {"DLPC347X_PC_STOP",DLPC347X_PC_STOP},
        {"DLPC347X_PC_PAUSE",DLPC347X_PC_PAUSE},
        {"DLPC347X_PC_STEP",DLPC347X_PC_STEP},
        {"DLPC347X_PC_RESUME",DLPC347X_PC_RESUME},
        {"DLPC347X_PC_RESET",DLPC347X_PC_RESET}
    };
    bool InternalPatternControl(DLPC347X_PatternControl_e PatternControl,int times);

    bool DisplayGridLines();

    uint16_t MaxRedLedCurrent;
    uint16_t MaxGreenLedCurrent;
    uint16_t MaxBlueLedCurrent;
    uint16_t RedLedCurrent,GreenLedCurrent,BlueLedCurrent;
    bool SetBrightness(double brightness);

    // 读取当前蓝光电流强度。优先真读硬件；本板卡读命令可能持续 NAK，读失败时回退到
    // 上次写入的缓存值(BlueLedCurrent)。outFromCache 返回该值是否来自缓存(true=缓存)。
    // 返回值恒为可用的电流值，供 UI 显示。
    uint16_t ReadCurrentBlueCurrent(bool* outFromCache = nullptr);


    //图像处理相关

    // 烧录进度回调：percent 为 0~100，stage 为当前阶段描述（如擦除/写入）。
    // 由上层(worker)设置，用于把烧录进度回传给 GUI 刷新进度条。可为空。
    using ProgressCallback = std::function<void(int percent, const std::string& stage)>;
    void SetProgressCallback(ProgressCallback cb) { m_progressCallback = std::move(cb); }

    // illuminationTimeUs/preDarkTimeUs/postDarkTimeUs 为每张图案的明亮时间、前黑暗时间、后黑暗时间(微秒)。
    // 传 0 或负数时沿用默认 40000us，保持与旧行为兼容。
    bool Input1DImageSet_8bit_V(vector<vector<cv::Mat>> imagesets,
                                int illuminationTimeUs = 40000,
                                int preDarkTimeUs = 40000,
                                int postDarkTimeUs = 40000);

    // 报告烧录进度：内部调用，回调为空时安全跳过。
    void reportProgress(int percent, const std::string& stage);

private:
    bool device_on=false;
    bool bus_access_requested=false;
    bool get_current_status();

    bool InitConnectionAndCommandLayer();

    ProgressCallback m_progressCallback; // 烧录进度回调，可为空。
};

#endif // DLP_H
