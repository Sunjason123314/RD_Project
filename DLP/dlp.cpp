#include "dlp.h"
#include <chrono>
#include <functional>
#include <iostream>
#include <stdexcept>
#include <thread>
using namespace std ;

namespace
{
uint32_t s_LastFlashProgramStatus = SUCCESS;

bool IsDlpcCommandSuccessful(uint32_t status, const char* command)
{
    if(status == SUCCESS)
        return true;

    cerr << command << " failed, status=" << status << endl;
    return false;
}

bool IsDeviceReady(bool deviceOn, const char* operation)
{
    if(deviceOn)
        return true;

    cerr << operation << " failed: DLP command layer is not available." << endl;
    return false;
}

constexpr uint32_t DLPC_I2C_TRANSACTION_RETRY_COUNT = 2;
constexpr uint32_t DLPC_I2C_TRANSACTION_RETRY_DELAY_MS = 50;
constexpr uint32_t DLPC_STATUS_READ_RETRY_COUNT = 6;
constexpr uint32_t DLPC_STATUS_READ_RETRY_DELAY_MS = 200;

// 初始化刚连接/刚释放 I2C 总线时，DLPC347x 偶尔会 NAK 第一次读命令。
// 对初始化状态读取做有限重试，避免一次瞬时 I2C 失败就使用默认 UI 状态。
template<typename CommandFunc>
bool ExecuteDlpcCommandWithRetry(CommandFunc commandFunc,
                                 const char* commandName,
                                 uint32_t retryCount,
                                 uint32_t retryDelayMs)
{
    uint32_t lastStatus = FAIL;

    for(uint32_t attempt = 1; attempt <= retryCount; ++attempt)
    {
        lastStatus = commandFunc();
        if(lastStatus == SUCCESS)
        {
            if(attempt > 1)
                cerr << commandName << " succeeded after " << (attempt - 1) << " retry attempt(s)." << endl;
            return true;
        }

        if(attempt < retryCount)
        {
            cerr << commandName << " read failed, status=" << lastStatus
                 << ", attempt=" << attempt << "/" << retryCount
                 << ", retrying..." << endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(retryDelayMs));
        }
    }

    cerr << commandName << " failed after " << retryCount
         << " attempts, last status=" << lastStatus << endl;
    return false;
}

void LogShortStatus(const DLPC347X_ShortStatus_s& status)
{
    cerr << "DLP short status: "
         << "SystemInitialized=" << static_cast<int>(status.SystemInitialized)
         << ", CommunicationError=" << static_cast<int>(status.CommunicationError)
         << ", SystemError=" << static_cast<int>(status.SystemError)
         << ", FlashEraseComplete=" << static_cast<int>(status.FlashEraseComplete)
         << ", FlashError=" << static_cast<int>(status.FlashError)
         << ", SensingSequenceError=" << static_cast<int>(status.SensingSequenceError)
         << ", Application=" << static_cast<int>(status.Application)
         << endl;
}

bool WaitForFlashEraseComplete(uint32_t timeoutSeconds,
                               const std::function<void(int)>& eraseProgress = {})
{
    const auto start = std::chrono::steady_clock::now();
    const auto deadline = start + std::chrono::seconds(timeoutSeconds);
    // 本项目板卡的 I2C 读命令(read data phase)会持续 NAK，无法读回 ShortStatus，
    // 但写命令(含 WriteFlashErase)正常。擦除 entire sens pattern 区域是写命令触发的
    // 异步操作，通常数秒内完成。因此当读状态始终不可用时，盲等固定时长后即视为擦除
    // 完成并继续烧录，避免因读不回状态而死等超时、进而让投影仪长时间无响应。
    // DLP4710(1080p 双 DMD)的图案 flash 区域较大，擦除整个 sens pattern 区域耗时较长，
    // 6 秒往往不够；擦除未完成时 flash 处于 busy，此时发写命令会失败。故盲等放大到 25 秒。
    const auto blindWaitDuration = std::chrono::seconds(25);
    DLPC347X_ShortStatus_s shortStatus = {};
    uint32_t readRetryCount = 0;
    bool anyReadSucceeded = false;

    do
    {
        const uint32_t status = DLPC347X_ReadShortStatus(&shortStatus);
        if(status != SUCCESS)
        {
            ++readRetryCount;
            cerr << "DLPC347X_ReadShortStatus failed while waiting for flash erase, status="
                 << status << ", retry=" << readRetryCount << endl;

            const auto now = std::chrono::steady_clock::now();

            // 盲等期间按已过时间比例上报擦除进度，让进度条在这段最耗时的等待里持续走动。
            if(eraseProgress)
            {
                const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
                const auto total = std::chrono::duration_cast<std::chrono::milliseconds>(blindWaitDuration).count();
                int pct = total > 0 ? static_cast<int>(elapsed * 100 / total) : 100;
                if(pct > 100) pct = 100;
                eraseProgress(pct);
            }

            // 读状态从未成功过(板卡不支持稳定读回)且已盲等足够久：判定擦除已完成。
            if(!anyReadSucceeded && (now - start) >= blindWaitDuration)
            {
                cerr << "Short-status read unavailable on this board; "
                        "assuming flash erase completed after blind wait." << endl;
                return true;
            }

            if(now >= deadline)
            {
                cerr << "Timed out waiting for DLP flash erase status to become readable." << endl;
                return false;
            }

            // Some DLPC347x firmware versions NAK/timeout short-status reads while flash erase is busy.
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        anyReadSucceeded = true;
        readRetryCount = 0;
        LogShortStatus(shortStatus);

        if(shortStatus.FlashError == DLPC347X_E_ERROR ||
           shortStatus.SystemError == DLPC347X_E_ERROR ||
           shortStatus.CommunicationError == DLPC347X_E_ERROR)
        {
            cerr << "DLP flash erase failed according to short status." << endl;
            return false;
        }

        if(shortStatus.FlashEraseComplete != DLPC347X_FE_NOT_COMPLETE)
            return true;

        if(std::chrono::steady_clock::now() >= deadline)
        {
            cerr << "Timed out waiting for DLP flash erase to complete." << endl;
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    } while(true);
}
}

/**
 * Implement the I2C write transaction here. The sample code here sends
 * data to the controller via the Cypress USB-Serial adapter.
 */
uint32_t WriteI2C(uint16_t WriteDataLength,
                  uint8_t* WriteData,
                  DLPC_COMMON_CommandProtocolData_s* ProtocolData)
{
    (void)ProtocolData;

    for(uint32_t attempt = 1; attempt <= DLPC_I2C_TRANSACTION_RETRY_COUNT; ++attempt)
    {
        if(CYPRESS_I2C_WriteI2C(WriteDataLength, WriteData))
            return SUCCESS;

        cerr << "CYPRESS_I2C_WriteI2C failed, length=" << WriteDataLength
             << ", attempt=" << attempt << "/" << DLPC_I2C_TRANSACTION_RETRY_COUNT;

        if(attempt < DLPC_I2C_TRANSACTION_RETRY_COUNT)
        {
            cerr << ", retrying..." << endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(DLPC_I2C_TRANSACTION_RETRY_DELAY_MS));
        }
        else
        {
            cerr << ", no more retries." << endl;
        }
    }

    return FAIL;
}

/**
 * Implement the I2C write/read transaction here. The sample code here
 * receives data from the controller via the Cypress USB-Serial adapter.
 */
uint32_t ReadI2C(uint16_t                           WriteDataLength,
                 uint8_t*                           WriteData,
                 uint16_t                           ReadDataLength,
                 uint8_t*                           ReadData,
                 DLPC_COMMON_CommandProtocolData_s* ProtocolData)
{
    (void)ProtocolData;

    for(uint32_t attempt = 1; attempt <= DLPC_I2C_TRANSACTION_RETRY_COUNT; ++attempt)
    {
        if(!CYPRESS_I2C_WriteI2C(WriteDataLength, WriteData))
        {
            cerr << "DLPC I2C read command write phase failed, writeLength=" << WriteDataLength
                 << ", readLength=" << ReadDataLength
                 << ", attempt=" << attempt << "/" << DLPC_I2C_TRANSACTION_RETRY_COUNT << endl;
        }
        else if(!CYPRESS_I2C_ReadI2C(ReadDataLength, ReadData))
        {
            cerr << "DLPC I2C read data phase failed, writeLength=" << WriteDataLength
                 << ", readLength=" << ReadDataLength
                 << ", attempt=" << attempt << "/" << DLPC_I2C_TRANSACTION_RETRY_COUNT << endl;
        }
        else
        {
            return SUCCESS;
        }

        if(attempt < DLPC_I2C_TRANSACTION_RETRY_COUNT)
            std::this_thread::sleep_for(std::chrono::milliseconds(DLPC_I2C_TRANSACTION_RETRY_DELAY_MS));
    }

    return FAIL;
}



void WaitForSeconds(uint32_t Seconds)
{
    uint32_t retTime = (uint32_t)(time(0)) + Seconds;	// Get finishing time.
    while (time(0) < retTime);					        // Loop until it arrives.
}
/**
 * A sample function that generates a 1-bit (binary) 1-D pattern
 * The function fills the byte array Data. Each byte in the in array corresponds
 * to a pixel. For a 1-bit pattern the value of each byte should be 1 or 0.
 */
void PopulateOneBitPatternData(uint16_t Length, uint8_t* Data, uint16_t NumBars)
{
    uint16_t PixelPos  = 0;
    uint16_t BarPos    = 0;
    uint16_t BarWidth  = Length / NumBars;
    uint8_t  PixelData = 0;

    for (; PixelPos < Length; PixelPos++)
    {
        Data[PixelPos] = PixelData;

        BarPos++;
        if (BarPos >= BarWidth)
        {
            BarPos = 0;
            PixelData = (PixelData == 0 ? 1 : 0);
        }
    }
}

/**
 * A sample function that generates an 8-bit (gray scale) 1-D pattern
 * The function fills the byte array Data. Each byte in the in array corresponds
 * to a pixel. For an 8-bit pattern the value of each byte can be 0 - 255.
 */
void PopulateEightBitPatternData(uint16_t Length, uint8_t* Data, uint16_t NumBars)
{
    uint16_t PixelPos     = 0;
    uint16_t BarPos       = 0;
    uint16_t BarWidth     = Length / (2 * NumBars);
    uint8_t  PixelData    = 0;
    int16_t  PixelDataInc = (int16_t)ceil(255.0 / BarWidth);

    for (; PixelPos < Length; PixelPos++)
    {
        Data[PixelPos] = PixelData;

        BarPos++;
        if (BarPos >= BarWidth)
        {
            BarPos    = 0;
            PixelDataInc = -PixelDataInc;
        }

        PixelData = (uint8_t)(PixelData + PixelDataInc);
    }
}

/**
 * Populates an array of DLPC347X_INT_PAT_PatternSet_s
 */
void PopulatePatternSetData(uint16_t DMDWidth, uint16_t DMDHeight)
{
    uint8_t                        HorzPatternIdx = 0;
    uint8_t                        VertPatternIdx = 0;
    uint8_t                        PatternIdx     = 0;
    uint8_t                        PatternSetIdx  = 0;
    uint8_t                        Index;
    uint16_t                       NumBars;
    DLPC347X_INT_PAT_PatternSet_s* PatternSet;

    /* Create a 1-bit (binary) Horizontal Pattern Set */
    PatternSet = &s_PatternSets[PatternSetIdx++];
    PatternSet->BitDepth     = DLPC347X_INT_PAT_BITDEPTH_ONE;
    PatternSet->Direction    = DLPC347X_INT_PAT_DIRECTION_HORIZONTAL;
    PatternSet->PatternCount = NUM_ONE_BIT_HORIZONTAL_PATTERNS;
    PatternSet->PatternArray = &s_Patterns[PatternIdx];
    for (Index = 0; Index < NUM_ONE_BIT_HORIZONTAL_PATTERNS; Index++)
    {
        NumBars = 2 * (Index + 1);
        PopulateOneBitPatternData(DMDHeight, s_HorizontalPatternData[HorzPatternIdx], NumBars);
        s_Patterns[PatternIdx].PixelArray      = s_HorizontalPatternData[HorzPatternIdx];
        s_Patterns[PatternIdx].PixelArrayCount = DMDHeight;
        PatternIdx++;
        HorzPatternIdx++;
    }

    /* Create a 1-bit (binary) Vertical Pattern Set */
    PatternSet = &s_PatternSets[PatternSetIdx++];
    PatternSet->BitDepth     = DLPC347X_INT_PAT_BITDEPTH_ONE;
    PatternSet->Direction    = DLPC347X_INT_PAT_DIRECTION_VERTICAL;
    PatternSet->PatternCount = NUM_ONE_BIT_VERTICAL_PATTERNS;
    PatternSet->PatternArray = &s_Patterns[PatternIdx];
    for (Index = 0; Index < NUM_ONE_BIT_VERTICAL_PATTERNS; Index++)
    {
        NumBars = 2 * (Index + 1);
        PopulateOneBitPatternData(DMDWidth, s_VerticalPatternData[VertPatternIdx], NumBars);
        s_Patterns[PatternIdx].PixelArray      = s_VerticalPatternData[VertPatternIdx];
        s_Patterns[PatternIdx].PixelArrayCount = DMDWidth;
        PatternIdx++;
        VertPatternIdx++;
    }

    /* Create an 8-bit (grayscale) Horizontal Pattern Set */
    PatternSet = &s_PatternSets[PatternSetIdx++];
    PatternSet->BitDepth     = DLPC347X_INT_PAT_BITDEPTH_EIGHT;
    PatternSet->Direction    = DLPC347X_INT_PAT_DIRECTION_HORIZONTAL;
    PatternSet->PatternCount = NUM_EIGHT_BIT_HORIZONTAL_PATTERNS;
    PatternSet->PatternArray = &s_Patterns[PatternIdx];
    for (Index = 0; Index < NUM_EIGHT_BIT_HORIZONTAL_PATTERNS; Index++)
    {
        NumBars = 2 * (Index + 1);
        PopulateEightBitPatternData(DMDHeight, s_HorizontalPatternData[HorzPatternIdx], NumBars);
        s_Patterns[PatternIdx].PixelArray      = s_HorizontalPatternData[HorzPatternIdx];
        s_Patterns[PatternIdx].PixelArrayCount = DMDHeight;
        PatternIdx++;
        HorzPatternIdx++;
    }

    /* Create an 8-bit (grayscale) Vertical Pattern Set */
    PatternSet = &s_PatternSets[PatternSetIdx++];
    PatternSet->BitDepth     = DLPC347X_INT_PAT_BITDEPTH_EIGHT;
    PatternSet->Direction    = DLPC347X_INT_PAT_DIRECTION_VERTICAL;
    PatternSet->PatternCount = NUM_EIGHT_BIT_VERTICAL_PATTERNS;
    PatternSet->PatternArray = &s_Patterns[PatternIdx];
    for (Index = 0; Index < NUM_EIGHT_BIT_VERTICAL_PATTERNS; Index++)
    {
        NumBars = 2 * (Index + 1);
        PopulateEightBitPatternData(DMDWidth, s_VerticalPatternData[VertPatternIdx], NumBars);
        s_Patterns[PatternIdx].PixelArray      = s_VerticalPatternData[VertPatternIdx];
        s_Patterns[PatternIdx].PixelArrayCount = DMDWidth;
        PatternIdx++;
        VertPatternIdx++;
    }
}

/**
 * Populates an array of DLPC347X_INT_PAT_PatternOrderTableEntry_s
 */
void PopulatePatternTableData()
{
    DLPC347X_INT_PAT_PatternOrderTableEntry_s* PatternOrderTableEntry;
    uint32_t                                   PatternOrderTableIdx = 0;
    uint32_t                                   PatternSetIdx        = 0;

    /* Pattern Table Entry 0 - uses Pattern Set 0 */
    PatternOrderTableEntry = &s_PatternOrderTable[PatternOrderTableIdx++];
    PatternOrderTableEntry->PatternSetIndex                        = PatternSetIdx;
    PatternOrderTableEntry->NumDisplayPatterns                     = s_PatternSets[PatternSetIdx++].PatternCount;
    PatternOrderTableEntry->IlluminationSelect                     = DLPC347X_INT_PAT_ILLUMINATION_BLUE ;
    PatternOrderTableEntry->InvertPatterns                         = false;
    PatternOrderTableEntry->IlluminationTimeInMicroseconds         = 40000;
    PatternOrderTableEntry->PreIlluminationDarkTimeInMicroseconds  = 40000;
    PatternOrderTableEntry->PostIlluminationDarkTimeInMicroseconds = 20000;

    /* Pattern Table Entry 1 - uses Pattern Set 1 */
    PatternOrderTableEntry = &s_PatternOrderTable[PatternOrderTableIdx++];
    PatternOrderTableEntry->PatternSetIndex                        = PatternSetIdx;
    PatternOrderTableEntry->NumDisplayPatterns                     = s_PatternSets[PatternSetIdx++].PatternCount;
    PatternOrderTableEntry->IlluminationSelect                     = DLPC347X_INT_PAT_ILLUMINATION_BLUE ;
    PatternOrderTableEntry->InvertPatterns                         = false;
    PatternOrderTableEntry->IlluminationTimeInMicroseconds         = 40000;
    PatternOrderTableEntry->PreIlluminationDarkTimeInMicroseconds  =40000;
    PatternOrderTableEntry->PostIlluminationDarkTimeInMicroseconds = 20000;

    /* Pattern Table Entry 2 - uses Pattern Set 2 */
    PatternOrderTableEntry = &s_PatternOrderTable[PatternOrderTableIdx++];
    PatternOrderTableEntry->PatternSetIndex                        = PatternSetIdx;
    PatternOrderTableEntry->NumDisplayPatterns                     = s_PatternSets[PatternSetIdx++].PatternCount;
    PatternOrderTableEntry->IlluminationSelect                     = DLPC347X_INT_PAT_ILLUMINATION_BLUE;
    PatternOrderTableEntry->InvertPatterns                         = false;
    PatternOrderTableEntry->IlluminationTimeInMicroseconds         = 40000;
    PatternOrderTableEntry->PreIlluminationDarkTimeInMicroseconds  = 40000;
    PatternOrderTableEntry->PostIlluminationDarkTimeInMicroseconds = 20000;

    /* Pattern Table Entry 3 - uses Pattern Set 3 */
    PatternOrderTableEntry = &s_PatternOrderTable[PatternOrderTableIdx++];
    PatternOrderTableEntry->PatternSetIndex                        = PatternSetIdx;
    PatternOrderTableEntry->NumDisplayPatterns                     = s_PatternSets[PatternSetIdx++].PatternCount;
    PatternOrderTableEntry->IlluminationSelect                     = DLPC347X_INT_PAT_ILLUMINATION_BLUE ;
    PatternOrderTableEntry->InvertPatterns                         = false;
    PatternOrderTableEntry->IlluminationTimeInMicroseconds         = 40000;
    PatternOrderTableEntry->PreIlluminationDarkTimeInMicroseconds  = 40000;
    PatternOrderTableEntry->PostIlluminationDarkTimeInMicroseconds = 20000;
}

void CopyDataToFlashProgramBuffer(uint8_t* Length, uint8_t** DataPtr)
{
    while ((*Length >= 1) &&
           (s_FlashProgramBufferPtr < sizeof(s_FlashProgramBuffer)))
    {
        s_FlashProgramBuffer[s_FlashProgramBufferPtr] = **DataPtr;
        s_FlashProgramBufferPtr++;
        (*DataPtr)++;
        (*Length)--;
    }
}

void ProgramFlashWithDataInBuffer(uint16_t Length)
{
    s_FlashProgramBufferPtr = 0;

    if(s_LastFlashProgramStatus != SUCCESS)
        return;

    uint32_t status = SUCCESS;
    if (s_StartProgramming)
    {
        s_StartProgramming = false;
        status = DLPC347X_WriteFlashStart(Length, s_FlashProgramBuffer);
    }
    else
    {
        status = DLPC347X_WriteFlashContinue(Length, s_FlashProgramBuffer);
    }

    if(status != SUCCESS)
    {
        cerr << "Program DLP flash block failed, status=" << status << endl;
        s_LastFlashProgramStatus = status;
    }
}

void WriteDataToFile(uint8_t Length, uint8_t* Data)
{
    fwrite(Data, 1, Length, s_FilePointer);
}

void GenerateAndWritePatternDataToFile(DLPC347X_INT_PAT_DMD_e DMD, char* FilePath)
{
    s_FilePointer = fopen(FilePath, "wb");

    /* Generate pattern data and write it to the flash.
     * The DLPC347X_INT_PAT_GeneratePatternDataBlock() function will call the
     * WriteDataToFile() function several times while it packs sections of the
     * pattern data.
     */
    DLPC347X_INT_PAT_GeneratePatternDataBlock(DMD,
                                              NUM_PATTERN_SETS,
                                              s_PatternSets,
                                              NUM_PATTERN_ORDER_TABLE_ENTRIES,
                                              s_PatternOrderTable,
                                              WriteDataToFile);

    fclose(s_FilePointer);
}

void BufferPatternDataAndProgramToFlash(uint8_t Length, uint8_t* Data)
{
    /* Copy data that can fit in the flash programming buffer */
    CopyDataToFlashProgramBuffer(&Length, &Data);

    /* Write data to flash if the buffer is full */
    if (s_FlashProgramBufferPtr >= sizeof(s_FlashProgramBuffer))
    {
        ProgramFlashWithDataInBuffer((uint16_t)sizeof(s_FlashProgramBuffer));
    }

    /* Copy remaining data (if any) to the flash programming buffer */
    CopyDataToFlashProgramBuffer(&Length, &Data);
}

void GenerateAndProgramPatternData(DLPC347X_INT_PAT_DMD_e DMD)
{
    s_StartProgramming = true;
    s_FlashProgramBufferPtr = 0;

    /* Let the controller know that we're going to program pattern data */
    DLPC347X_WriteFlashDataTypeSelect(DLPC347X_FDTS_ENTIRE_SENS_PATTERN_DATA);

    /* Erase the flash sectors that store pattern data */
    DLPC347X_WriteFlashErase();

    /* Read Short Status to make sure Erase is completed */
    if(!WaitForFlashEraseComplete(30))
        return;

    /* To program the flash, send blocks of data of up to 1024 bytes
     * to the controller at a time. Repeat the process until the entire
     * data is programmed to the flash.
     * Let the controller know the size of a data block that will be
     * transferred at a time.
     */
    DLPC347X_WriteFlashDataLength(sizeof(s_FlashProgramBuffer));

    /* Generate pattern data and program it to the flash.
     *
     * The DLPC347X_INT_PAT_GeneratePatternDataBlock() function calls the
     * BufferPatternDataAndProgramToFlash() function several times while it
     * generates pattern data.
     *
     * The BufferPatternDataAndProgramToFlash() function buffers data received,
     * programming the buffer content only when it is full. This is done in an
     * effort to make flash writes more efficient, overall greatly reducing the
     * time it takes to program the pattern data.
     *
     * After returning from the DLPC347X_INT_PAT_GeneratePatternBlock() function,
     * check if there is any data left in the buffer and program it. This needs
     * to be done since the BufferPatternDataAndProgramToFlash() function only
     * programs the buffer content if full.
     */
    DLPC347X_INT_PAT_GeneratePatternDataBlock(DMD,
                                              NUM_PATTERN_SETS,
                                              s_PatternSets,
                                              NUM_PATTERN_ORDER_TABLE_ENTRIES,
                                              s_PatternOrderTable,
                                              BufferPatternDataAndProgramToFlash);
    if (s_FlashProgramBufferPtr > 0)
    {
        /* Resend the block size since it could be less than
         * the previously specified size
         */
        DLPC347X_WriteFlashDataLength(s_FlashProgramBufferPtr);

        ProgramFlashWithDataInBuffer(s_FlashProgramBufferPtr);
    }
}

void LoadPatternOrderTableEntryfromFlash()
{
    DLPC347X_PatternOrderTableEntry_s PatternOrderTableEntry = {};

    /* Reload from Flash */
    DLPC347X_WritePatternOrderTableEntry(DLPC347X_WC_RELOAD_FROM_FLASH, &PatternOrderTableEntry);
}


void LoadPatternOrderTableEntry(uint8_t PatternSetIndex)
{
    DLPC347X_PatternOrderTableEntry_s PatternOrderTableEntry;

    /* Set PatternOrderTableEntry to select specific Pattern Set and configure settings */
    PatternOrderTableEntry.PatSetIndex = 1;
    PatternOrderTableEntry.NumberOfPatternsToDisplay = PatternSetIndex;
    PatternOrderTableEntry.RedIlluminator = DLPC347X_IE_DISABLE;
    PatternOrderTableEntry.GreenIlluminator = DLPC347X_IE_DISABLE;
    PatternOrderTableEntry.BlueIlluminator = DLPC347X_IE_ENABLE;
    PatternOrderTableEntry.PatternInvertLsword = 0;
    PatternOrderTableEntry.PatternInvertMsword = 0;
    PatternOrderTableEntry.IlluminationTime = 2000;
    PatternOrderTableEntry.PreIlluminationDarkTime = 250;
    PatternOrderTableEntry.PostIlluminationDarkTime = 60;
    DLPC347X_WritePatternOrderTableEntry(DLPC347X_WC_START, &PatternOrderTableEntry);
}

void WriteTestPatternGridLines()
{
    /* Write Input Image Size */
    DLPC347X_WriteInputImageSize(DLP4710_WIDTH, DLP4710_HEIGHT);

    /* Write Image Crop */
    DLPC347X_WriteImageCrop(0, 0, DLP4710_WIDTH, DLP4710_HEIGHT);

    /* Write Display Size */
    DLPC347X_WriteDisplaySize(0, 0, DLP4710_WIDTH, DLP4710_HEIGHT);

    /* Write Grid Lines */
    DLPC347X_GridLines_s GridLines;
    GridLines.Border = DLPC347X_BE_ENABLE;
    GridLines.BackgroundColor = DLPC347X_C_BLACK ;
    GridLines.ForegroundColor = DLPC347X_C_BLUE ;
    GridLines.HorizontalForegroundLineWidth = 0xF;
    GridLines.HorizontalBackgroundLineWidth = 0xF;
    GridLines.VerticalForegroundLineWidth = 0xF;
    GridLines.VerticalBackgroundLineWidth = 0xF;
    DLPC347X_WriteGridLines(&GridLines);
    DLPC347X_WriteOperatingModeSelect(DLPC347X_OM_TEST_PATTERN_GENERATOR);
    WaitForSeconds(5);
}

void WriteLookSelect(uint8_t LookNumber)
{
    /* Read Current Operating Mode Selected */
    DLPC347X_OperatingMode_e OperatingMode;
    DLPC347X_ReadOperatingModeSelect(&OperatingMode);

    /* Write RGB LED Current (based on Flash data) */
    DLPC347X_WriteRgbLedCurrent(0x03E8, 0x03E8, 0x03E8);

    /* Select Look */
    DLPC347X_WriteLookSelect(LookNumber);

    /* Submit Write Splash Screen Execute if in Splash Mode */
    if ((OperatingMode == DLPC347X_OM_SPLASH_SCREEN ) ||
        (OperatingMode == DLPC347X_OM_SENS_SPACE_CODED_PATTERN))
    {
        DLPC347X_WriteSplashScreenExecute();
    }
    WaitForSeconds(5);
}

void LoadPreBuildPatternData()
{
    /* 块大小统一使用 FLASH_WRITE_BLOCK_SIZE，避免硬编码 1024 超过 Cypress 桥单次写上限。 */
    uint8_t PatternDataArray[FLASH_WRITE_BLOCK_SIZE];
    const uint16_t blockSize = FLASH_WRITE_BLOCK_SIZE;

    /* Pattern File assumes to be in the \build\vs2017\dlpc347x folder */
    s_FilePointer = fopen("pattern_data_gui.bin", "rb");
    if (!s_FilePointer)
    {
        printf("Error opening the binary file!");
        return;
    }
    fseek(s_FilePointer, 0, SEEK_END);
    uint32_t PatternDataSize = ftell(s_FilePointer);
    fseek(s_FilePointer, 0, SEEK_SET);

    /* Select Flash Data Block and Erase the Block */
    DLPC347X_WriteFlashDataTypeSelect(DLPC347X_FDTS_ENTIRE_SENS_PATTERN_DATA);
    DLPC347X_WriteFlashErase();

    /* Read Short Status to make sure Erase is completed */
    if(!WaitForFlashEraseComplete(30))
    {
        fclose(s_FilePointer);
        return;
    }

    DLPC347X_WriteFlashDataLength(blockSize);
    fread(PatternDataArray, sizeof(PatternDataArray), 1, s_FilePointer);
    DLPC347X_WriteFlashStart(blockSize, PatternDataArray);

    int32_t BytesLeft = PatternDataSize - blockSize;
    do
    {
        fread(PatternDataArray, sizeof(PatternDataArray), 1, s_FilePointer);
        DLPC347X_WriteFlashContinue(blockSize, PatternDataArray);

        BytesLeft = BytesLeft - blockSize;
    } while (BytesLeft > 0);

    fclose(s_FilePointer);
}

DLP::DLP()
{
    Current_OperatingMode = DLPC347X_OM_SENS_INTERNAL_PATTERN;
    MaxRedLedCurrent = 0;
    MaxGreenLedCurrent = 0;
    MaxBlueLedCurrent = 1023;
    RedLedCurrent = 0;
    GreenLedCurrent = 0;
    BlueLedCurrent = 0;

    if(!InitConnectionAndCommandLayer())
        throw runtime_error("Failed to connect Cypress USB-I2C bridge for DLP projector.");

    // 有些投影仪/转接板不使用 TI EVM 的 GPIO 总线仲裁；仲裁失败不再直接判定为“连接失败”，
    // 否则会出现 USB-I2C 已连接但界面无法进入已连接状态的问题。后续具体读写仍会检查返回值。
    bus_access_requested = CYPRESS_I2C_RequestI2CBusAccess();
    if(!bus_access_requested)
        cerr << "Warning: failed to request DLP I2C bus access, continue without GPIO arbitration." << endl;

    // 刚打开 Cypress 句柄或刚做 GPIO 仲裁后，给 USB-I2C 桥和投影仪一点稳定时间。
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    device_on = true;

    // 当前板卡写命令可以正常工作，但部分读命令会在 I2C read data phase NAK/timeout。
    // 播放/烧录主要依赖写命令；连接阶段不再强制读状态，避免误报警。
    cerr << "DLP initialized with default UI status values; status read is skipped during connection." << endl;
}
DLP::~DLP()
{
    if(bus_access_requested)
        CYPRESS_I2C_RelinquishI2CBusAccess();

    device_on = false;
    bus_access_requested = false;
}

void maintest()
{
//    InitConnectionAndCommandLayer();

    /* TI DLP Pico EVMs use a GPIO handshake scheme for the controller I2C bus
     * arbitration. Call this method if using a TI EVM, remove otherwise
     */
//    CYPRESS_I2C_RequestI2CBusAccess();

    DLPC347X_WriteOperatingModeSelect(DLPC347X_OM_SENS_INTERNAL_PATTERN);
    DLPC347X_WriteInternalPatternControl(DLPC347X_PC_START, 0xFF);

    /* ***** Change to look 0 ***** */
    WriteLookSelect(0);

    /* ***** Write Test Pattern - GridLines ***** */
//    WriteTestPatternGridLines();

    /* ***** Test Internal Pattern Sensing ***** */
    bool LoadFromFile = false;	// Switch to load pattern from saved file or generated

    if (LoadFromFile)
    {
        /* Load pre-build Pattern Table and Sets */
        LoadPreBuildPatternData();
        LoadPatternOrderTableEntryfromFlash();
    }
    else
    {
        /* Prepare the data for pattern data block generation */
        PopulatePatternSetData(DLP4710_WIDTH, DLP4710_HEIGHT);
        PopulatePatternTableData();

        /* Stop pattern display */
        DLPC347X_WriteInternalPatternControl(DLPC347X_PC_STOP, 0);

        /* Generate and program the pattern data to the controller flash */
        GenerateAndProgramPatternData(DLPC347X_INT_PAT_DMD_DLP4710);

        /* Load Pattern Order Table Entry from Flash */
        LoadPatternOrderTableEntryfromFlash();

        /* Generate and write pattern data to a file */
        GenerateAndWritePatternDataToFile(DLPC347X_INT_PAT_DMD_DLP4710, "pattern_data.bin");
    }

    /* Display patterns */
    DLPC347X_WriteOperatingModeSelect(DLPC347X_OM_SENS_INTERNAL_PATTERN);
    DLPC347X_WriteInternalPatternControl(DLPC347X_PC_START, 0xFF);


}

///
/// \brief DLP::SelectOperatingMode????????????
/// \param OperatingMode
///
bool DLP::SelectOperatingMode(DLPC347X_OperatingMode_e OperatingMode)
{
    if(!IsDeviceReady(device_on, "SelectOperatingMode"))
        return false;

    if(!IsDlpcCommandSuccessful(DLPC347X_WriteOperatingModeSelect(OperatingMode), "DLPC347X_WriteOperatingModeSelect"))
        return false;

    Current_OperatingMode = OperatingMode;
    return true;
}

///
/// \brief DLP::InternalPatternControl????????????????
/// \param PatternControl
///
bool DLP::InternalPatternControl(DLPC347X_PatternControl_e PatternControl,int times)
{
    if(!IsDeviceReady(device_on, "InternalPatternControl"))
        return false;

    const uint8_t repeatCount = times <= 0 ? 0xFF : static_cast<uint8_t>(times - 1);
    return IsDlpcCommandSuccessful(DLPC347X_WriteInternalPatternControl(PatternControl, repeatCount),
                                   "DLPC347X_WriteInternalPatternControl");
}

bool DLP::DisplayGridLines()
{
    if(!IsDeviceReady(device_on, "DisplayGridLines"))
        return false;

    /* Write Input Image Size */
    if(!IsDlpcCommandSuccessful(DLPC347X_WriteInputImageSize(DLP4710_WIDTH, DLP4710_HEIGHT),
                                "DLPC347X_WriteInputImageSize"))
        return false;

    /* Write Image Crop */
    if(!IsDlpcCommandSuccessful(DLPC347X_WriteImageCrop(0, 0, DLP4710_WIDTH, DLP4710_HEIGHT),
                                "DLPC347X_WriteImageCrop"))
        return false;

    /* Write Display Size */
    if(!IsDlpcCommandSuccessful(DLPC347X_WriteDisplaySize(0, 0, DLP4710_WIDTH, DLP4710_HEIGHT),
                                "DLPC347X_WriteDisplaySize"))
        return false;

    DLPC347X_GridLines_s GridLines;
    GridLines.Border = DLPC347X_BE_ENABLE;
    GridLines.BackgroundColor = DLPC347X_C_BLACK;
    GridLines.ForegroundColor = DLPC347X_C_BLUE;
    GridLines.HorizontalForegroundLineWidth = 0xF;
    GridLines.HorizontalBackgroundLineWidth = 0xF;
    GridLines.VerticalForegroundLineWidth = 0xF;
    GridLines.VerticalBackgroundLineWidth = 0xF;

    if(!IsDlpcCommandSuccessful(DLPC347X_WriteGridLines(&GridLines), "DLPC347X_WriteGridLines"))
        return false;

    return SelectOperatingMode(DLPC347X_OM_TEST_PATTERN_GENERATOR);
}

bool DLP::SetBrightness(double brightness)
{
    if(!IsDeviceReady(device_on, "SetBrightness"))
        return false;

    if(brightness < 0)
        brightness = 0;
    if(MaxBlueLedCurrent > 0 && brightness > MaxBlueLedCurrent)
        brightness = MaxBlueLedCurrent;

    const uint16_t blueCurrent = static_cast<uint16_t>(brightness);
    if(!IsDlpcCommandSuccessful(DLPC347X_WriteRgbLedCurrent(0, 0, blueCurrent),
                                "DLPC347X_WriteRgbLedCurrent"))
        return false;

    // 当前板卡读回 RGB LED current 可能失败，但写入已经成功。
    // 本地缓存用于刷新 UI，避免亮度调节被读回失败误判为失败。
    RedLedCurrent = 0;
    GreenLedCurrent = 0;
    BlueLedCurrent = blueCurrent;
    return true;
}

uint16_t DLP::ReadCurrentBlueCurrent(bool* outFromCache)
{
    // 先尝试真读硬件电流。本板卡读命令可能 NAK，带重试。
    uint16_t r = 0, g = 0, b = 0;
    const bool readOk = device_on &&
        ExecuteDlpcCommandWithRetry([&]() {
                                        return DLPC347X_ReadRgbLedCurrent(&r, &g, &b);
                                    },
                                    "DLPC347X_ReadRgbLedCurrent",
                                    DLPC_STATUS_READ_RETRY_COUNT,
                                    DLPC_STATUS_READ_RETRY_DELAY_MS);

    if(readOk)
    {
        // 真读成功：更新缓存并返回真值。
        RedLedCurrent = r;
        GreenLedCurrent = g;
        BlueLedCurrent = b;
        if(outFromCache)
            *outFromCache = false;
        return BlueLedCurrent;
    }

    // 读失败：回退到上次写入的缓存值。
    cerr << "ReadRgbLedCurrent unavailable on this board; returning cached blue current="
         << BlueLedCurrent << endl;
    if(outFromCache)
        *outFromCache = true;
    return BlueLedCurrent;
}

void DLP::reportProgress(int percent, const std::string& stage)
{
    if(m_progressCallback)
        m_progressCallback(percent, stage);
}

bool DLP::Input1DImageSet_8bit_V(vector<vector<cv::Mat> > imagesets,
                                 int illuminationTimeUs,
                                 int preDarkTimeUs,
                                 int postDarkTimeUs)
{
    if(!IsDeviceReady(device_on, "Input1DImageSet_8bit_V"))
        return false;

    reportProgress(0, "准备数据");

    // 传入非正数时回退到默认 40000us，避免出现 0 明亮时间等非法配置。
    const uint32_t illumTime = illuminationTimeUs > 0 ? static_cast<uint32_t>(illuminationTimeUs) : 40000u;
    const uint32_t preDark   = preDarkTimeUs   >= 0 ? static_cast<uint32_t>(preDarkTimeUs)   : 40000u;
    const uint32_t postDark  = postDarkTimeUs  >= 0 ? static_cast<uint32_t>(postDarkTimeUs)  : 40000u;

//    WriteLookSelect(0);
    int total_size=0;
    for (int i = 0; i < imagesets.size(); ++i)
    {
        if(imagesets[i].size()>6)
        {
            cout<<"???????????????????6"<<endl;
            return false;
        }
        total_size+=imagesets[i].size();
    }
    if(total_size==0)
    {
        cout<<"?????????????0"<<endl;
        return false;
    }
    cout<<"??????????"<<total_size<<endl;

    //??????????
    uint8_t** imagedata = new uint8_t*[total_size];
    for (int i = 0; i < total_size; i++)
    {
        imagedata[i] = new uint8_t[DLP4710_WIDTH];
    }
//    uint8_t imagedata[11][DLP4710_WIDTH];

    DLPC347X_INT_PAT_PatternData_s* PatternDatas=new DLPC347X_INT_PAT_PatternData_s[total_size];
    int count=0;
    for (int i = 0; i < imagesets.size(); ++i)
    {
        for (int j = 0; j < imagesets[i].size(); ++j)
        {
            cout<<"??????"<<count<<endl;
            for (int k = 0; k < DLP4710_WIDTH; ++k)
            {
                imagedata[count][k]=imagesets[i][j].at<uchar>(0,k);
            }
            PatternDatas[count].PixelArray=imagedata[count];
            PatternDatas[count].PixelArrayCount=DLP4710_WIDTH;
            count++;
        }
    }

    //?????????
    DLPC347X_INT_PAT_PatternSet_s* PatternSets=new DLPC347X_INT_PAT_PatternSet_s[imagesets.size()];
//    DLPC347X_INT_PAT_PatternSet_s PatternSets[3];
    int position=0;
    for (int i = 0; i < imagesets.size(); ++i)
    {
        PatternSets[i].BitDepth=DLPC347X_INT_PAT_BITDEPTH_EIGHT;
        PatternSets[i].Direction=DLPC347X_INT_PAT_DIRECTION_VERTICAL;
        PatternSets[i].PatternCount=imagesets[i].size();
        PatternSets[i].PatternArray = &PatternDatas[position];
        position+=imagesets[i].size();
    }

    //????????
    DLPC347X_INT_PAT_PatternOrderTableEntry_s* PatternOrderTable=new DLPC347X_INT_PAT_PatternOrderTableEntry_s[imagesets.size()];
//    DLPC347X_INT_PAT_PatternOrderTableEntry_s PatternOrderTable[3];
    for (int i = 0; i < imagesets.size(); ++i)
    {
        PatternOrderTable[i].PatternSetIndex = i;
        PatternOrderTable[i].NumDisplayPatterns = PatternSets[i].PatternCount;
        PatternOrderTable[i].IlluminationSelect = DLPC347X_INT_PAT_ILLUMINATION_BLUE ;
        PatternOrderTable[i].InvertPatterns = false;
        PatternOrderTable[i].IlluminationTimeInMicroseconds = illumTime;
        PatternOrderTable[i].PreIlluminationDarkTimeInMicroseconds  = preDark;
        PatternOrderTable[i].PostIlluminationDarkTimeInMicroseconds = postDark;
    }

    if(!IsDlpcCommandSuccessful(DLPC347X_WriteInternalPatternControl(DLPC347X_PC_STOP, 0),
                                "DLPC347X_WriteInternalPatternControl(STOP)"))
        return false;

    //????????
    s_StartProgramming = true;
    s_FlashProgramBufferPtr = 0;
    s_LastFlashProgramStatus = SUCCESS;

    /* Let the controller know that we're going to program pattern data */
    if(!IsDlpcCommandSuccessful(DLPC347X_WriteFlashDataTypeSelect(DLPC347X_FDTS_ENTIRE_SENS_PATTERN_DATA),
                                "DLPC347X_WriteFlashDataTypeSelect"))
        return false;

    /* Erase the flash sectors that store pattern data */
    reportProgress(5, "擦除 Flash");
    if(!IsDlpcCommandSuccessful(DLPC347X_WriteFlashErase(), "DLPC347X_WriteFlashErase"))
        return false;

    /* Read Short Status to make sure Erase is completed */
    // 擦除是烧录最耗时的阶段(盲等 25 秒)，把它映射到进度条 5%~60% 区间持续走动。
    if(!WaitForFlashEraseComplete(30, [this](int erasePct) {
                                          reportProgress(5 + erasePct * 55 / 100, "擦除 Flash");
                                      }))
        return false;

    reportProgress(60, "写入图案数据");

    /* To program the flash, send blocks of data of up to 1024 bytes
     * to the controller at a time. Repeat the process until the entire
     * data is programmed to the flash.
     * Let the controller know the size of a data block that will be
     * transferred at a time.
     */
    if(!IsDlpcCommandSuccessful(DLPC347X_WriteFlashDataLength(sizeof(s_FlashProgramBuffer)),
                                "DLPC347X_WriteFlashDataLength"))
        return false;

    /* Generate pattern data and program it to the flash.
     *
     * The DLPC347X_INT_PAT_GeneratePatternDataBlock() function calls the
     * BufferPatternDataAndProgramToFlash() function several times while it
     * generates pattern data.
     *
     * The BufferPatternDataAndProgramToFlash() function buffers data received,
     * programming the buffer content only when it is full. This is done in an
     * effort to make flash writes more efficient, overall greatly reducing the
     * time it takes to program the pattern data.
     *
     * After returning from the DLPC347X_INT_PAT_GeneratePatternBlock() function,
     * check if there is any data left in the buffer and program it. This needs
     * to be done since the BufferPatternDataAndProgramToFlash() function only
     * programs the buffer content if full.
     */
    if(!IsDlpcCommandSuccessful(DLPC347X_INT_PAT_GeneratePatternDataBlock(DLPC347X_INT_PAT_DMD_DLP4710,
                                                                          imagesets.size(),
                                                                          PatternSets,
                                                                          imagesets.size(),
                                                                          PatternOrderTable,
                                                                          BufferPatternDataAndProgramToFlash),
                                "DLPC347X_INT_PAT_GeneratePatternDataBlock(program)"))
        return false;

    if(s_LastFlashProgramStatus != SUCCESS)
        return false;

    if (s_FlashProgramBufferPtr > 0)
    {
        /* Resend the block size since it could be less than
         * the previously specified size
         */
        if(!IsDlpcCommandSuccessful(DLPC347X_WriteFlashDataLength(s_FlashProgramBufferPtr),
                                    "DLPC347X_WriteFlashDataLength(last block)"))
            return false;

        ProgramFlashWithDataInBuffer(s_FlashProgramBufferPtr);
        if(s_LastFlashProgramStatus != SUCCESS)
            return false;
    }

    reportProgress(90, "加载图案表");
    DLPC347X_PatternOrderTableEntry_s ReloadPatternOrderTableEntry = {};
    if(!IsDlpcCommandSuccessful(DLPC347X_WritePatternOrderTableEntry(DLPC347X_WC_RELOAD_FROM_FLASH,
                                                                     &ReloadPatternOrderTableEntry),
                                "DLPC347X_WritePatternOrderTableEntry(RELOAD_FROM_FLASH)"))
        return false;


    s_FilePointer = fopen("pattern_data.bin", "wb");

    /* Generate pattern data and write it to the flash.
     * The DLPC347X_INT_PAT_GeneratePatternDataBlock() function will call the
     * WriteDataToFile() function several times while it packs sections of the
     * pattern data.
     */
    DLPC347X_INT_PAT_GeneratePatternDataBlock(DLPC347X_INT_PAT_DMD_DLP4710,
                                              imagesets.size(),
                                              PatternSets,
                                              imagesets.size(),
                                              PatternOrderTable,
                                              WriteDataToFile);

    fclose(s_FilePointer);

    //??????
    if(!SelectOperatingMode(DLPC347X_OM_SENS_INTERNAL_PATTERN))
        return false;
    if(!InternalPatternControl(DLPC347X_PC_START, 0))
        return false;

    reportProgress(100, "烧录完成");

//    for (int i = 0; i < total_size; i++)
//    {
//        delete[] imagedata[i];
//    }
//    delete[] imagedata;
//    delete[] PatternDatas;
//    delete[] PatternSets;
//    delete[] PatternOrderTable;

    return true;
}

///
/// \brief DLP::get_current_status?????????DLP??
///
bool DLP::get_current_status()
{
    if(!ExecuteDlpcCommandWithRetry([this]() {
                                        return DLPC347X_ReadOperatingModeSelect(&Current_OperatingMode);
                                    },
                                    "DLPC347X_ReadOperatingModeSelect",
                                    DLPC_STATUS_READ_RETRY_COUNT,
                                    DLPC_STATUS_READ_RETRY_DELAY_MS))
        return false;

    if(!ExecuteDlpcCommandWithRetry([this]() {
                                        return DLPC347X_ReadRgbLedMaxCurrent(&MaxRedLedCurrent,
                                                                             &MaxGreenLedCurrent,
                                                                             &MaxBlueLedCurrent);
                                    },
                                    "DLPC347X_ReadRgbLedMaxCurrent",
                                    DLPC_STATUS_READ_RETRY_COUNT,
                                    DLPC_STATUS_READ_RETRY_DELAY_MS))
        return false;

    if(!ExecuteDlpcCommandWithRetry([this]() {
                                        return DLPC347X_ReadRgbLedCurrent(&RedLedCurrent,
                                                                          &GreenLedCurrent,
                                                                          &BlueLedCurrent);
                                    },
                                    "DLPC347X_ReadRgbLedCurrent",
                                    DLPC_STATUS_READ_RETRY_COUNT,
                                    DLPC_STATUS_READ_RETRY_DELAY_MS))
        return false;

    return true;
}

/**
 * Initialize the command layer by setting up the read/write buffers and
 * callbacks.
 */
bool DLP::InitConnectionAndCommandLayer()
{
    DLPC_COMMON_InitCommandLibrary(s_WriteBuffer,
                                   sizeof(s_WriteBuffer),
                                   s_ReadBuffer,
                                   sizeof(s_ReadBuffer),
                                   WriteI2C,
                                   ReadI2C);

    const bool connected = CYPRESS_I2C_ConnectToCyI2C();
    cout << (connected ? "true" : "false") << endl;
    return connected;
}
