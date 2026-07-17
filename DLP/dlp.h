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

#define FLASH_WRITE_BLOCK_SIZE            1024
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


    //图像处理相关

    bool Input1DImageSet_8bit_V(vector<vector<cv::Mat>> imagesets);

private:
    bool device_on=false;
    bool bus_access_requested=false;
    bool get_current_status();

    bool InitConnectionAndCommandLayer();
};

#endif // DLP_H
