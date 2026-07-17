/*------------------------------------------------------------------------------
 * Copyright (c) 2019 Texas Instruments Incorporated - http://www.ti.com/
 *------------------------------------------------------------------------------
 *
 * NOTE: This file is auto generated from a command definition file.
 *       Please do not modify the file directly.                    
 *
 * Command Spec Version : 1.0
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the
 *   distribution.
 *
 *   Neither the name of Texas Instruments Incorporated nor the names of
 *   its contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * \file
 * \brief  DLPC347x Dual Commands
 */

#ifndef DLPC347X_DUAL_H
#define DLPC347X_DUAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "dlpc_common.h"
#include "stdbool.h"
#include "stdint.h"

typedef enum
{
    DLPC347X_DUAL_IM_DISABLED = 0x0,                                  /**< Idle Mode Disabled */
    DLPC347X_DUAL_IM_AUTO = 0x1,                                      /**< Auto Idle Mode Enabled */
    DLPC347X_DUAL_IM_MANUAL = 0x2,                                    /**< Manual Idle Mode Enabled */
} DLPC347X_DUAL_IdleMode_e;

typedef enum
{
    DLPC347X_DUAL_MVT_PATTERN22 = 0x0,                                /**< Two frames repeat, two frames repeat, etc. (2:2 pattern) */
    DLPC347X_DUAL_MVT_PATTERN32 = 0x1,                                /**< Three frames repeat, two frames repeat, etc. (3:2 pattern) */
} DLPC347X_DUAL_MotionVideoType_e;

typedef enum
{
    DLPC347X_DUAL_SIMV_STILL_IMAGE = 0x0,                             /**< Previous frame still */
    DLPC347X_DUAL_SIMV_MOTION_VIDEO = 0x1,                            /**< Previous frame video */
} DLPC347X_DUAL_StillImageMotionVideo_e;

typedef enum
{
    DLPC347X_DUAL_AOI_ACTIVE = 0x0,                                   /**< Previous frame active */
    DLPC347X_DUAL_AOI_IDLE = 0x1,                                     /**< Previous frame idle */
} DLPC347X_DUAL_ActiveOrIdle_e;

typedef enum
{
    DLPC347X_DUAL_OM_EXTERNAL_VIDEO_PORT = 0x0,                       /**< External Video Port */
    DLPC347X_DUAL_OM_TEST_PATTERN_GENERATOR = 0x1,                    /**< Test Pattern Generator */
    DLPC347X_DUAL_OM_SPLASH_SCREEN = 0x2,                             /**< Splash Screen */
    DLPC347X_DUAL_OM_SENS_EXTERNAL_PATTERN = 0x3,                     /**< External Pattern Streaming */
    DLPC347X_DUAL_OM_SENS_INTERNAL_PATTERN = 0x4,                     /**< Internal Pattern Streaming */
    DLPC347X_DUAL_OM_SENS_SPACE_CODED_PATTERN = 0x5,                  /**< Space Coded Pattern Streaming */
    DLPC347X_DUAL_OM_STANDBY = 0xFF,                                  /**< Standby */
} DLPC347X_DUAL_OperatingMode_e;

typedef enum
{
    DLPC347X_DUAL_CIM_CHROMA_INTERPOLATION = 0x0,                     /**< Chroma Interpolation */
    DLPC347X_DUAL_CIM_CHROMA_COPY = 0x1,                              /**< Chroma Copy */
} DLPC347X_DUAL_ChromaInterpolationMethod_e;

typedef enum
{
    DLPC347X_DUAL_CCS_CBCR = 0x0,                                     /**< CbCr */
    DLPC347X_DUAL_CCS_CRCB = 0x1,                                     /**< CrCb */
} DLPC347X_DUAL_ChromaChannelSwap_e;

typedef enum
{
    DLPC347X_DUAL_TPBE_DISABLE = 0x0,                                 /**< Disable */
    DLPC347X_DUAL_TPBE_ENABLE = 0x1,                                  /**< Enable */
} DLPC347X_DUAL_TestPatternBorderEnable_e;

typedef enum
{
    DLPC347X_DUAL_ICE_DISABLE = 0x0,                                  /**< Disable */
    DLPC347X_DUAL_ICE_ENABLE = 0x1,                                   /**< Enable */
} DLPC347X_DUAL_ImageCurtainEnable_e;

typedef enum
{
    DLPC347X_DUAL_CDI_DLPC3430 = 0x0,
    DLPC347X_DUAL_CDI_DLPC3433 = 0x1,
    DLPC347X_DUAL_CDI_DLPC3432 = 0x2,
    DLPC347X_DUAL_CDI_DLPC3434 = 0x3,
    DLPC347X_DUAL_CDI_DLPC3435 = 0x4,
    DLPC347X_DUAL_CDI_DLPC3438 = 0x5,
    DLPC347X_DUAL_CDI_DLPC3436 = 0x6,
    DLPC347X_DUAL_CDI_DLPC3437 = 0x7,
    DLPC347X_DUAL_CDI_DLPC3472 = 0x8,
    DLPC347X_DUAL_CDI_DLPC3439 = 0x9,
    DLPC347X_DUAL_CDI_DLPC3440 = 0xA,
    DLPC347X_DUAL_CDI_DLPC3478 = 0xB,
    DLPC347X_DUAL_CDI_DLPC3479 = 0xC,
    DLPC347X_DUAL_CDI_DLPC3470 = 0xF,
} DLPC347X_DUAL_ControllerDeviceId_e;

typedef enum
{
    DLPC347X_DUAL_EVF_DSI = 0x0,                                      /**< DSI, Bus Width: 1 - 4 */
    DLPC347X_DUAL_EVF_PARALLEL_RGB565_BW16 = 0x40,                    /**< Parallel, RGB 565, Bus Width: 16, 1 Clk/Pixel */
    DLPC347X_DUAL_EVF_PARALLEL_RGB666_BW18 = 0x41,                    /**< Parallel, RGB 666, Bus Width: 18, 1 Clk/Pixel */
    DLPC347X_DUAL_EVF_PARALLEL_RGB888_BW8 = 0x42,                     /**< Parallel, RGB 888, Bus Width: 8, 3 Clk/Pixel */
    DLPC347X_DUAL_EVF_PARALLEL_RGB888_BW24 = 0x43,                    /**< Parallel, RGB 888, Bus Width: 24, 1 Clks/Pixel */
    DLPC347X_DUAL_EVF_PARALLEL_YCBCR666_BW18 = 0x50,                  /**< Parallel, YCbCr 666, Bus Width: 18, 1 Clk/Pixel */
    DLPC347X_DUAL_EVF_PARALLEL_YCBCR888_BW24 = 0x51,                  /**< Parallel, YCbCr 888, Bus Width: 24, 1 Clk/Pixel */
    DLPC347X_DUAL_EVF_PARALLEL_YCBCR422_BW8 = 0x60,                   /**< Parallel, YCbCr 4:2:2 88, Bus Width: 8, 2 Clks/Pixel */
    DLPC347X_DUAL_EVF_PARALLEL_YCBCR422_BW16 = 0x61,                  /**< Parallel, YCbCr 4:2:2 88, Bus Width: 16, 1 Clk/Pixel */
    DLPC347X_DUAL_EVF_BT656 = 0xA0,                                   /**< BT656, YCbCr 4:2:2 88, Bus Width: 8, 2 Clk/Pixel */
} DLPC347X_DUAL_ExternalVideoFormat_e;

typedef enum
{
    DLPC347X_DUAL_C_BLACK = 0x0,
    DLPC347X_DUAL_C_RED = 0x1,
    DLPC347X_DUAL_C_GREEN = 0x2,
    DLPC347X_DUAL_C_BLUE = 0x3,
    DLPC347X_DUAL_C_CYAN = 0x4,
    DLPC347X_DUAL_C_MAGENTA = 0x5,
    DLPC347X_DUAL_C_YELLOW = 0x6,
    DLPC347X_DUAL_C_WHITE = 0x7,
} DLPC347X_DUAL_Color_e;

typedef enum
{
    DLPC347X_DUAL_DLS_DLS3 = 0x3,                                     /**< 3 pixels */
    DLPC347X_DUAL_DLS_DLS7 = 0x7,                                     /**< 7 pixels */
    DLPC347X_DUAL_DLS_DLS15 = 0xF,                                    /**< 15 pixels */
    DLPC347X_DUAL_DLS_DLS31 = 0x1F,                                   /**< 31 pixels */
    DLPC347X_DUAL_DLS_DLS63 = 0x3F,                                   /**< 63 pixels */
    DLPC347X_DUAL_DLS_DLS127 = 0x7F,                                  /**< 127 pixels */
    DLPC347X_DUAL_DLS_DLS255 = 0xFF,                                  /**< 255 pixels */
} DLPC347X_DUAL_DiagonalLineSpacing_e;

typedef enum
{
    DLPC347X_DUAL_TP_SOLID_FIELD = 0x0,                               /**< Solid Field */
    DLPC347X_DUAL_TP_HORIZONTAL_RAMP = 0x1,                           /**< Horizontal Ramp */
    DLPC347X_DUAL_TP_VERTICAL_RAMP = 0x2,                             /**< Vertical Ramp */
    DLPC347X_DUAL_TP_HORIZONTAL_LINES = 0x3,                          /**< Horizontal Lines */
    DLPC347X_DUAL_TP_DIAGONAL_LINES = 0x4,                            /**< Diagonal Lines */
    DLPC347X_DUAL_TP_VERTICAL_LINES = 0x5,                            /**< Vertical Lines */
    DLPC347X_DUAL_TP_GRID = 0x6,                                      /**< Grid */
    DLPC347X_DUAL_TP_CHECKERBOARD = 0x7,                              /**< Checkerboard */
    DLPC347X_DUAL_TP_COLORBARS = 0x8,                                 /**< Colorbars */
} DLPC347X_DUAL_TestPattern_e;

typedef enum
{
    DLPC347X_DUAL_PF_RGB565 = 0x2,                                    /**< 16-bit RGB 5-6-5 */
    DLPC347X_DUAL_PF_YCBCR422 = 0x3,                                  /**< 16-bit YCbCr 4:2:2 */
} DLPC347X_DUAL_PixelFormats_e;

typedef enum
{
    DLPC347X_DUAL_CT_UNCOMPRESSED = 0x0,                              /**< Uncompressed */
    DLPC347X_DUAL_CT_RGB_RLE_COMPRESSED = 0x1,                        /**< RGB RLE Compressed */
    DLPC347X_DUAL_CT_UNUSED = 0x2,                                    /**< Unused */
    DLPC347X_DUAL_CT_YUV_RLE_COMPRESSED = 0x3,                        /**< YUV RLE Compressed */
} DLPC347X_DUAL_CompressionTypes_e;

typedef enum
{
    DLPC347X_DUAL_CO_RGB = 0x0,                                       /**< RGB */
    DLPC347X_DUAL_CO_GRB = 0x1,                                       /**< GRB */
} DLPC347X_DUAL_ColorOrders_e;

typedef enum
{
    DLPC347X_DUAL_CO_CR_FIRST = 0x0,                                  /**< Cr first */
    DLPC347X_DUAL_CO_CB_FIRST = 0x1,                                  /**< Cb first */
} DLPC347X_DUAL_ChromaOrders_e;

typedef enum
{
    DLPC347X_DUAL_BO_LITTLE_ENDIAN = 0x0,                             /**< Little endian */
    DLPC347X_DUAL_BO_BIG_ENDIAN = 0x1,                                /**< Big endian */
} DLPC347X_DUAL_ByteOrders_e;

typedef enum
{
    DLPC347X_DUAL_IR_NO_ROTATION = 0x0,                               /**< No rotation */
    DLPC347X_DUAL_IR_MINUS90_DEGREE_ROTATION = 0x1,                   /**< Minus 90 degree rotation */
} DLPC347X_DUAL_ImageRotation_e;

typedef enum
{
    DLPC347X_DUAL_IF_IMAGE_NOT_FLIPPED = 0x0,                         /**< Image not flipped */
    DLPC347X_DUAL_IF_IMAGE_FLIPPED = 0x1,                             /**< Image flipped */
} DLPC347X_DUAL_ImageFlip_e;

typedef enum
{
    DLPC347X_DUAL_DSSM_AUTO_SYNC = 0x0,                               /**< Auto-sync */
    DLPC347X_DUAL_DSSM_FORCE_LOCK_TO_INTERNAL_VSYNC = 0x1,            /**< Force lock to internal VSYNC */
} DLPC347X_DUAL_DmdSequencerSyncMode_e;

typedef enum
{
    DLPC347X_DUAL_SASS_LOCK_TO_EXTERNAL_VSYNC = 0x0,                  /**< Lock to external VSYNC */
    DLPC347X_DUAL_SASS_LOCK_TO_INTERNAL_VSYNC = 0x1,                  /**< Lock to internal VSYNC */
} DLPC347X_DUAL_SystemAutoSyncSetting_e;

typedef enum
{
    DLPC347X_DUAL_BE_DISABLE = 0x0,                                   /**< Disable */
    DLPC347X_DUAL_BE_ENABLE = 0x1,                                    /**< Enable */
} DLPC347X_DUAL_BorderEnable_e;

typedef enum
{
    DLPC347X_DUAL_LCM_MANUAL = 0x0,                                   /**< Manual RGB LED currents (disables CAIC algorithm) */
    DLPC347X_DUAL_LCM_AUTOMATIC = 0x1,                                /**< CAIC (automatic) RGB LED power (enables CAIC algorithm) */
} DLPC347X_DUAL_LedControlMethod_e;

typedef enum
{
    DLPC347X_DUAL_LC_DISABLED = 0x0,                                  /**< Disabled */
    DLPC347X_DUAL_LC_MANUAL = 0x1,                                    /**< Enabled: Manual strength control (no light sensor) */
    DLPC347X_DUAL_LC_AUTOMATIC = 0x2,                                 /**< Enabled: Automatic strength control (uses light sensor - not supported) */
} DLPC347X_DUAL_LabbControl_e;

typedef enum
{
    DLPC347X_DUAL_CWC_DISABLED = 0x0,                                 /**< White point correction disabled */
    DLPC347X_DUAL_CWC_ENABLED = 0x1,                                  /**< White point correction enabled */
} DLPC347X_DUAL_CaicWpcControl_e;

typedef enum
{
    DLPC347X_DUAL_CGDS_P1024 = 0x0,                                   /**< 100% = 1024 pixels */
    DLPC347X_DUAL_CGDS_P512 = 0x1,                                    /**< 100% = 512 pixels */
} DLPC347X_DUAL_CaicGainDisplayScale_e;

typedef enum
{
    DLPC347X_DUAL_BCS_COMMAND = 0x0,                                  /**< Defined by the border color command */
    DLPC347X_DUAL_BCS_FLASH = 0x1,                                    /**< Flash defined 24-bit color */
} DLPC347X_DUAL_BorderColorSource_e;

typedef enum
{
    DLPC347X_DUAL_SI_NOT_COMPLETE = 0x0,                              /**< Not Complete */
    DLPC347X_DUAL_SI_COMPLETE = 0x1,                                  /**< Complete */
} DLPC347X_DUAL_SystemInit_e;

typedef enum
{
    DLPC347X_DUAL_E_NO_ERROR = 0x0,                                   /**< No Error */
    DLPC347X_DUAL_E_ERROR = 0x1,                                      /**< Error */
} DLPC347X_DUAL_Error_e;

typedef enum
{
    DLPC347X_DUAL_SE_NO_ERROR = 0x0,                                  /**< No Error */
    DLPC347X_DUAL_SE_ILLUMINATION_TIME_NOT_SUPPORTED = 0x1,           /**< Illumination Time Not Supported */
    DLPC347X_DUAL_SE_PRE_ILLUMINATION_TIME_NOT_SUPPORTED = 0x2,       /**< Pre-Illumination Time Not Supported */
    DLPC347X_DUAL_SE_POST_ILLUMINATION_TIME_NOT_SUPPORTED = 0x3,      /**< Post-Illumination Time Not Supported */
    DLPC347X_DUAL_SE_TRIGGER_OUT1_DELAY_NOT_SUPPORTED = 0x4,          /**< TriggerOut1 Delay Not Supported */
    DLPC347X_DUAL_SE_TRIGGER_OUT2_DELAY_NOT_SUPPORTED = 0x5,          /**< TriggerOut2 Delay Not Supported */
    DLPC347X_DUAL_SE_MAX_PATTERN_ORDER_TABLE_ENTRIES_EXCEEDED = 0x6,  /**< Max Pattern Order Table Entries Exceeded */
} DLPC347X_DUAL_SensingError_e;

typedef enum
{
    DLPC347X_DUAL_FE_COMPLETE = 0x0,                                  /**< Complete */
    DLPC347X_DUAL_FE_NOT_COMPLETE = 0x1,                              /**< Not Complete */
} DLPC347X_DUAL_FlashErase_e;

typedef enum
{
    DLPC347X_DUAL_A_BOOT_APP = 0x0,                                   /**< Boot App */
    DLPC347X_DUAL_A_MAIN_APP = 0x1,                                   /**< Main App */
} DLPC347X_DUAL_Application_e;

typedef enum
{
    DLPC347X_DUAL_LS_LED_OFF = 0x0,                                   /**< LED Off */
    DLPC347X_DUAL_LS_LED_ON = 0x1,                                    /**< LED On */
} DLPC347X_DUAL_LedState_e;

typedef enum
{
    DLPC347X_DUAL_PS_SUPPLY_VOLTAGE_NORMAL = 0x0,                     /**< Supply Voltage Normal */
    DLPC347X_DUAL_PS_SUPPLY_VOLTAGE_LOW = 0x1,                        /**< Supply Voltage Low */
} DLPC347X_DUAL_PowerSupply_e;

typedef enum
{
    DLPC347X_DUAL_CC_SINGLE = 0x0,                                    /**< Single */
    DLPC347X_DUAL_CC_DUAL = 0x1,                                      /**< Dual */
} DLPC347X_DUAL_ControllerConfiguration_e;

typedef enum
{
    DLPC347X_DUAL_MOSO_MASTER = 0x0,                                  /**< Master */
    DLPC347X_DUAL_MOSO_SLAVE = 0x1,                                   /**< Slave */
} DLPC347X_DUAL_MasterOrSlaveOperation_e;

typedef enum
{
    DLPC347X_DUAL_WT_NO_TIMEOUT = 0x0,                                /**< No Timeout */
    DLPC347X_DUAL_WT_TIMEOUT = 0x1,                                   /**< Timeout */
} DLPC347X_DUAL_WatchdogTimeout_e;

typedef enum
{
    DLPC347X_DUAL_MLO_DMD_INTERFACE_LOCK = 0x0,                       /**< DMD Interface Lock */
    DLPC347X_DUAL_MLO_DMD_INTERFACE_UNLOCK = 0x1,                     /**< DMD Interface Unlock */
    DLPC347X_DUAL_MLO_DMD_INTERFACE_UNLOCK_DELAY100_MS_DMD_INTERFACE_LOCK = 0x2, /**< DMD Interface Unlock, delay 100ms, DMD Interface Lock */
} DLPC347X_DUAL_MirrorLockOptions_e;

typedef enum
{
    DLPC347X_DUAL_DDS_DMD_DEVICE_ID = 0x0,
    DLPC347X_DUAL_DDS_DMD_FUSE_GROUP0 = 0x1,
    DLPC347X_DUAL_DDS_DMD_FUSE_GROUP1 = 0x2,
    DLPC347X_DUAL_DDS_DMD_FUSE_GROUP2 = 0x3,
    DLPC347X_DUAL_DDS_DMD_FUSE_GROUP3 = 0x4,
} DLPC347X_DUAL_DmdDataSelection_e;

typedef enum
{
    DLPC347X_DUAL_TT_TRIGGER1 = 0x0,                                  /**< Trigger Out 1 */
    DLPC347X_DUAL_TT_TRIGGER2 = 0x1,                                  /**< Trigger Out 2 */
} DLPC347X_DUAL_TriggerType_e;

typedef enum
{
    DLPC347X_DUAL_TE_DISABLE = 0x0,                                   /**< Disable */
    DLPC347X_DUAL_TE_ENABLE = 0x1,                                    /**< Enable */
} DLPC347X_DUAL_TriggerEnable_e;

typedef enum
{
    DLPC347X_DUAL_TI_NOT_INVERTED = 0x0,                              /**< Not Inverted */
    DLPC347X_DUAL_TI_INVERTED = 0x1,                                  /**< Inverted */
} DLPC347X_DUAL_TriggerInversion_e;

typedef enum
{
    DLPC347X_DUAL_TP_ACTIVE_LOW = 0x0,                                /**< Active Low */
    DLPC347X_DUAL_TP_ACTIVE_HI = 0x1,                                 /**< Active Hi */
} DLPC347X_DUAL_TriggerPolarity_e;

typedef enum
{
    DLPC347X_DUAL_ST_ONE_BIT_MONO = 0x0,                              /**< 1 Bit Mono */
    DLPC347X_DUAL_ST_ONE_BIT_RGB = 0x1,                               /**< 1 Bit RGB */
    DLPC347X_DUAL_ST_EIGHT_BIT_MONO = 0x2,                            /**< 8 Bit Mono */
    DLPC347X_DUAL_ST_EIGHT_BIT_RGB = 0x3,                             /**< 8 Bit RGB */
} DLPC347X_DUAL_SequenceType_e;

typedef enum
{
    DLPC347X_DUAL_STR_ONE_BIT_MONO_EXT = 0x0,                         /**< 1 Bit Mono External */
    DLPC347X_DUAL_STR_ONE_BIT_RGB_EXT = 0x1,                          /**< 1 Bit RGB External */
    DLPC347X_DUAL_STR_EIGHT_BIT_MONO_EXT = 0x2,                       /**< 8 Bit Mono External */
    DLPC347X_DUAL_STR_EIGHT_BIT_RGB_EXT = 0x3,                        /**< 8 Bit RGB External */
    DLPC347X_DUAL_STR_ONE_BIT_MONO_INT = 0x4,                         /**< 1 Bit Mono Internal */
    DLPC347X_DUAL_STR_ONE_BIT_RGB_INT = 0x5,                          /**< 1 Bit RGB Internal */
    DLPC347X_DUAL_STR_EIGHT_BIT_MONO_INT = 0x6,                       /**< 8 Bit Mono Internal */
    DLPC347X_DUAL_STR_EIGHT_BIT_RGB_INT = 0x7,                        /**< 8 Bit RGB Internal */
    DLPC347X_DUAL_STR_ONE_BIT_MONO_SPLASH = 0x8,                      /**< 1 Bit Mono Splash */
    DLPC347X_DUAL_STR_ONE_BIT_RGB_SPLASH = 0x9,                       /**< 1 Bit RGB Splash */
    DLPC347X_DUAL_STR_EIGHT_BIT_MONO_SPLASH = 0xA,                    /**< 8 Bit Mono Splash */
    DLPC347X_DUAL_STR_EIGHT_BIT_RGB_SPLASH = 0xB,                     /**< 8 Bit RGB Splash */
    DLPC347X_DUAL_STR_SEQ5050 = 0xC,                                  /**< Seq 50 50 */
} DLPC347X_DUAL_SequenceTypeRead_e;

typedef enum
{
    DLPC347X_DUAL_IS_NOT_ANY = 0x0,                                   /**< NotAny */
    DLPC347X_DUAL_IS_RED_ONLY = 0x1,                                  /**< Red Only */
    DLPC347X_DUAL_IS_RGB = 0x7,                                       /**< RGB */
} DLPC347X_DUAL_IllumSelect_e;

typedef enum
{
    DLPC347X_DUAL_IE_DISABLE = 0x0,                                   /**< Disable */
    DLPC347X_DUAL_IE_ENABLE = 0x1,                                    /**< Enable */
} DLPC347X_DUAL_IlluminatorEnable_e;

typedef enum
{
    DLPC347X_DUAL_PC_START = 0x0,                                     /**< Start */
    DLPC347X_DUAL_PC_STOP = 0x1,                                      /**< Stop */
    DLPC347X_DUAL_PC_PAUSE = 0x2,                                     /**< Pause */
    DLPC347X_DUAL_PC_STEP = 0x3,                                      /**< Step */
    DLPC347X_DUAL_PC_RESUME = 0x4,                                    /**< Resume */
    DLPC347X_DUAL_PC_RESET = 0x5,                                     /**< Reset */
} DLPC347X_DUAL_PatternControl_e;

typedef enum
{
    DLPC347X_DUAL_WC_CONTINUE = 0x0,                                  /**< Continue */
    DLPC347X_DUAL_WC_START = 0x1,                                     /**< Start */
    DLPC347X_DUAL_WC_RELOAD_FROM_FLASH = 0x2,                         /**< Reload from Flash */
} DLPC347X_DUAL_WriteControl_e;

typedef enum
{
    DLPC347X_DUAL_PRS_NOT_READY = 0x0,                                /**< Not Ready */
    DLPC347X_DUAL_PRS_READY = 0x1,                                    /**< Ready */
} DLPC347X_DUAL_PatternReadyStatus_e;

typedef enum
{
    DLPC347X_DUAL_PD_HORIZONTAL_PATTERN = 0x0,                        /**< Horizontal Pattern */
    DLPC347X_DUAL_PD_VERTICAL_PATTERN = 0x1,                          /**< Vertical Pattern */
} DLPC347X_DUAL_PatternDirection_e;

typedef enum
{
    DLPC347X_DUAL_PM_EXTERNAL = 0x0,                                  /**< External */
    DLPC347X_DUAL_PM_INTERNAL = 0x1,                                  /**< Internal */
    DLPC347X_DUAL_PM_SPACE_CODED = 0x2,                               /**< Space Coded */
} DLPC347X_DUAL_PatternMode_e;

typedef enum
{
    DLPC347X_DUAL_ETS_NO = 0x0,                                       /**< No */
    DLPC347X_DUAL_ETS_YES = 0x1,                                      /**< Yes */
} DLPC347X_DUAL_ExposureTimeSupported_e;

typedef enum
{
    DLPC347X_DUAL_ZDTS_NO = 0x0,                                      /**< No */
    DLPC347X_DUAL_ZDTS_YES = 0x1,                                     /**< Yes */
} DLPC347X_DUAL_ZeroDarkTimeSupported_e;

typedef enum
{
    DLPC347X_DUAL_FDTS_ENTIRE_FLASH = 0x0,                            /**< Entire Flash */
    DLPC347X_DUAL_FDTS_ENTIRE_FLASH_NO_OEM = 0x2,                     /**< Entire Flash except OEM Calibration and Scratchpad Data */
    DLPC347X_DUAL_FDTS_MAIN_APP = 0x10,                               /**< Main Software Application */
    DLPC347X_DUAL_FDTS_TI_APP = 0x20,                                 /**< TI Application Data (AOM) */
    DLPC347X_DUAL_FDTS_BATCH_FILES = 0x30,                            /**< OEM Batchfiles */
    DLPC347X_DUAL_FDTS_LOOKS = 0x40,                                  /**< Looks Data */
    DLPC347X_DUAL_FDTS_SEQUENCES = 0x50,                              /**< Sequence Data */
    DLPC347X_DUAL_FDTS_CMT = 0x60,                                    /**< Degamma/CMT Data */
    DLPC347X_DUAL_FDTS_CCA = 0x70,                                    /**< CCA Data */
    DLPC347X_DUAL_FDTS_GLUTS = 0x80,                                  /**< General LUT Data */
    DLPC347X_DUAL_FDTS_SPLASH = 0x90,                                 /**< OEM Splash Screens */
    DLPC347X_DUAL_FDTS_OEM_CAL = 0xA0,                                /**< OEM Calibration Data */
    DLPC347X_DUAL_FDTS_OEM_SCRATCHPAD_FULL0 = 0xB0,                   /**< Entire OEM Scratchpad Data Set 0 */
    DLPC347X_DUAL_FDTS_OEM_SCRATCHPAD_PARTIAL0 = 0xB1,                /**< Partial OEM Scrathpad Data Set 0 */
    DLPC347X_DUAL_FDTS_OEM_SCRATCHPAD_FULL1 = 0xB2,                   /**< Entire OEM Scratchpad Data Set 1 */
    DLPC347X_DUAL_FDTS_OEM_SCRATCHPAD_PARTIAL1 = 0xB3,                /**< Partial OEM Scrathpad Data Set 1 */
    DLPC347X_DUAL_FDTS_OEM_SCRATCHPAD_FULL2 = 0xB4,                   /**< Entire OEM Scratchpad Data Set 2 */
    DLPC347X_DUAL_FDTS_OEM_SCRATCHPAD_PARTIAL2 = 0xB5,                /**< Partial OEM Scrathpad Data Set 2 */
    DLPC347X_DUAL_FDTS_OEM_SCRATCHPAD_FULL3 = 0xB5,                   /**< Entire OEM Scratchpad Data Set 3 */
    DLPC347X_DUAL_FDTS_OEM_SCRATCHPAD_PARTIAL3 = 0xB7,                /**< Partial OEM Scrathpad Data Set 3 */
    DLPC347X_DUAL_FDTS_ENTIRE_SENS_PATTERN_DATA = 0xD0,               /**< Entire Sensing Pattern Data */
    DLPC347X_DUAL_FDTS_ENTIRE_SENS_SEQ_DATA = 0xE0,                   /**< Entire Sensing Sequence Data */
} DLPC347X_DUAL_FlashDataTypeSelect_e;

typedef enum
{
    DLPC347X_DUAL_TDM_TWO_D_OPERATION = 0x0,                          /**< 2-D Operation */
    DLPC347X_DUAL_TDM_THREE_D_OPERATION = 0x1,                        /**< 3-D Operation */
} DLPC347X_DUAL_ThreeDModes_e;

typedef enum
{
    DLPC347X_DUAL_TDRS_INTERNAL_REFERENCE = 0x0,                      /**< Internal Reference */
    DLPC347X_DUAL_TDRS_EXTERNAL_REFERENCE_VIA3_DR_PIN = 0x1,          /**< External Reference via 3DR pin */
} DLPC347X_DUAL_ThreeDReferenceSource_e;

typedef enum
{
    DLPC347X_DUAL_TDD_LEFT_DOMINANT_LEFT_EYE_FIRST = 0x0,             /**< Left Dominant (left eye first) */
    DLPC347X_DUAL_TDD_RIGHT_DOMINANT_RIGHT_EYE_FIRST = 0x1,           /**< Right Dominant (right eye first) */
} DLPC347X_DUAL_ThreeDDominance_e;

typedef enum
{
    DLPC347X_DUAL_TDRP_CORRECT_NO_INVERSION_REQUIRED = 0x0,           /**< Correct (no inversion required) */
    DLPC347X_DUAL_TDRP_INCORRECT_INVERSION_REQUIRED = 0x1,            /**< Incorrect (inversion required) */
} DLPC347X_DUAL_ThreeDReferencePolarity_e;

typedef enum
{
    DLPC347X_DUAL_ATS_MASTER_AND_SLAVE = 0x0,                         /**< Master and Slave */
    DLPC347X_DUAL_ATS_MASTER_ONLY = 0x1,                              /**< Master Only */
    DLPC347X_DUAL_ATS_SLAVE_ONLY = 0x2,                               /**< Slave Only */
} DLPC347X_DUAL_AsicTypeSelect_e;

typedef enum
{
    DLPC347X_DUAL_RW_WRITE = 0x0,                                     /**< Write */
    DLPC347X_DUAL_RW_READ = 0x1,                                      /**< Read */
} DLPC347X_DUAL_ReadWrite_e;

typedef struct
{
    uint16_t WidthInPixels;
    uint16_t HeightInPixels;
    uint32_t SizeInBytes;
    DLPC347X_DUAL_PixelFormats_e PixelFormat;
    DLPC347X_DUAL_CompressionTypes_e CompressionType;
    DLPC347X_DUAL_ColorOrders_e ColorOrder;
    DLPC347X_DUAL_ChromaOrders_e ChromaOrder;
    DLPC347X_DUAL_ByteOrders_e ByteOrder;
} DLPC347X_DUAL_SplashScreenHeader_s;

typedef struct
{
    DLPC347X_DUAL_BorderEnable_e Border;
    DLPC347X_DUAL_Color_e BackgroundColor;
    DLPC347X_DUAL_Color_e ForegroundColor;
    uint8_t ForegroundLineWidth;
    uint8_t BackgroundLineWidth;
} DLPC347X_DUAL_HorizontalLines_s;

typedef struct
{
    DLPC347X_DUAL_BorderEnable_e Border;
    DLPC347X_DUAL_Color_e BackgroundColor;
    DLPC347X_DUAL_Color_e ForegroundColor;
    DLPC347X_DUAL_DiagonalLineSpacing_e HorizontalSpacing;
    DLPC347X_DUAL_DiagonalLineSpacing_e VerticalSpacing;
} DLPC347X_DUAL_DiagonalLines_s;

typedef struct
{
    DLPC347X_DUAL_BorderEnable_e Border;
    DLPC347X_DUAL_Color_e BackgroundColor;
    DLPC347X_DUAL_Color_e ForegroundColor;
    uint8_t ForegroundLineWidth;
    uint8_t BackgroundLineWidth;
} DLPC347X_DUAL_VerticalLines_s;

typedef struct
{
    DLPC347X_DUAL_BorderEnable_e Border;
    DLPC347X_DUAL_Color_e BackgroundColor;
    DLPC347X_DUAL_Color_e ForegroundColor;
    uint8_t HorizontalForegroundLineWidth;
    uint8_t HorizontalBackgroundLineWidth;
    uint8_t VerticalForegroundLineWidth;
    uint8_t VerticalBackgroundLineWidth;
} DLPC347X_DUAL_GridLines_s;

typedef struct
{
    DLPC347X_DUAL_BorderEnable_e Border;
    DLPC347X_DUAL_Color_e BackgroundColor;
    DLPC347X_DUAL_Color_e ForegroundColor;
    uint16_t HorizontalCheckerCount;
    uint16_t VerticalCheckerCount;
} DLPC347X_DUAL_Checkerboard_s;

typedef struct
{
    DLPC347X_DUAL_TestPattern_e PatternSelect;
    DLPC347X_DUAL_BorderEnable_e Border;
    DLPC347X_DUAL_Color_e BackgroundColor;
    DLPC347X_DUAL_Color_e ForegroundColor;
    uint8_t StartValue;
    uint8_t EndValue;
    uint8_t ForegroundLineWidth;
    uint8_t BackgroundLineWidth;
    uint8_t HorizontalSpacing;
    uint8_t VerticalSpacing;
    uint8_t HorizontalForegroundLineWidth;
    uint8_t HorizontalBackgroundLineWidth;
    uint16_t HorizontalCheckerCount;
    uint8_t VerticalForegroundLineWidth;
    uint8_t VerticalBackgroundLineWidth;
    uint16_t VerticalCheckerCount;
} DLPC347X_DUAL_TestPatternSelect_s;

typedef struct
{
    double LookRedDutyCycle;
    double LookGreenDutyCycle;
    double LookBlueDutyCycle;
    double LookMaxFrameTime;
    double LookMinFrameTime;
    uint8_t LookMaxSequenceVectors;
    double SeqRedDutyCycle;
    double SeqGreenDutyCycle;
    double SeqBlueDutyCycle;
    double SeqMaxFrameTime;
    double SeqMinFrameTime;
    uint8_t SeqMaxSequenceVectors;
} DLPC347X_DUAL_SequenceHeaderAttributes_s;

typedef struct
{
    DLPC347X_DUAL_SystemInit_e SystemInitialized;
    DLPC347X_DUAL_Error_e CommunicationError;
    DLPC347X_DUAL_Error_e SystemError;
    DLPC347X_DUAL_FlashErase_e FlashEraseComplete;
    DLPC347X_DUAL_Error_e FlashError;
    DLPC347X_DUAL_Error_e SensingSequenceError;
    DLPC347X_DUAL_Application_e Application;
} DLPC347X_DUAL_ShortStatus_s;

typedef struct
{
    DLPC347X_DUAL_Error_e DmdDeviceError;
    DLPC347X_DUAL_Error_e DmdInterfaceError;
    DLPC347X_DUAL_Error_e DmdTrainingError;
    DLPC347X_DUAL_LedState_e RedLedEnableState;
    DLPC347X_DUAL_LedState_e GreenLedEnableState;
    DLPC347X_DUAL_LedState_e BlueLedEnableState;
    DLPC347X_DUAL_Error_e RedLedError;
    DLPC347X_DUAL_Error_e GreenLedError;
    DLPC347X_DUAL_Error_e BlueLedError;
    DLPC347X_DUAL_Error_e SequenceAbortError;
    DLPC347X_DUAL_Error_e SequenceError;
    DLPC347X_DUAL_PowerSupply_e DcPowerSupply;
    DLPC347X_DUAL_SensingError_e SensingError;
    DLPC347X_DUAL_ControllerConfiguration_e ControllerConfiguration;
    DLPC347X_DUAL_MasterOrSlaveOperation_e MasterOrSlaveOperation;
    DLPC347X_DUAL_Error_e ProductConfigurationError;
    DLPC347X_DUAL_WatchdogTimeout_e WatchdogTimerTimeout;
} DLPC347X_DUAL_SystemStatus_s;

typedef struct
{
    DLPC347X_DUAL_Error_e InvalidCommandError;
    DLPC347X_DUAL_Error_e InvalidCommandParameterValue;
    DLPC347X_DUAL_Error_e CommandProcessingError;
    DLPC347X_DUAL_Error_e FlashBatchFileError;
    DLPC347X_DUAL_Error_e ReadCommandError;
    DLPC347X_DUAL_Error_e InvalidNumberOfCommandParameters;
    DLPC347X_DUAL_Error_e BusTimeoutByDisplayError;
    uint8_t AbortedOpCode;
} DLPC347X_DUAL_CommunicationStatus_s;

typedef struct
{
    DLPC347X_DUAL_ExposureTimeSupported_e ExposureTimeSupported;
    DLPC347X_DUAL_ZeroDarkTimeSupported_e ZeroDarkTimeSupported;
    uint32_t MinimumExposureTime;
    uint32_t PreExposureDarkTime;
    uint32_t PostExposureDarkTime;
} DLPC347X_DUAL_ValidateExposureTime_s;

typedef struct
{
    uint8_t SelectedSequenceIndex;
    DLPC347X_DUAL_SequenceTypeRead_e SequenceTypeRead;
    DLPC347X_DUAL_IllumSelect_e IllumSelect;
    uint8_t NumVectors;
    uint8_t NumPatternsPerVector;
    uint8_t BitDepth;
    uint8_t PatLutIndex;
    uint32_t PreIlluminationDarkTime;
    uint32_t IlluminationTime;
    uint32_t PostIlluminationDarkTime;
    uint32_t SeqOffset;
    uint32_t SeqSize;
} DLPC347X_DUAL_SequenceInfo_s;

typedef struct
{
    DLPC347X_DUAL_SequenceType_e SequenceType;
    uint8_t NumberOfPatterns;
    DLPC347X_DUAL_IlluminatorEnable_e RedIlluminator;
    DLPC347X_DUAL_IlluminatorEnable_e GreenIlluminator;
    DLPC347X_DUAL_IlluminatorEnable_e BlueIlluminator;
    uint32_t IlluminationTime;
    uint32_t PreIlluminationDarkTime;
    uint32_t PostIlluminationDarkTime;
} DLPC347X_DUAL_PatternConfiguration_s;

typedef struct
{
    uint8_t PatSetIndex;
    uint8_t NumberOfPatternsToDisplay;
    DLPC347X_DUAL_IlluminatorEnable_e RedIlluminator;
    DLPC347X_DUAL_IlluminatorEnable_e GreenIlluminator;
    DLPC347X_DUAL_IlluminatorEnable_e BlueIlluminator;
    uint32_t PatternInvertLsword;
    uint32_t PatternInvertMsword;
    uint32_t IlluminationTime;
    uint32_t PreIlluminationDarkTime;
    uint32_t PostIlluminationDarkTime;
} DLPC347X_DUAL_PatternOrderTableEntry_s;

typedef struct
{
    DLPC347X_DUAL_PatternReadyStatus_e PatternReadyStatus;
    uint8_t NumPatOrderTableEntries;
    uint8_t CurrentPatOrderEntryIndex;
    uint8_t CurrentPatSetIndex;
    uint8_t NumPatInCurrentPatSet;
    uint8_t NumPatDisplayedFromPatSet;
    uint8_t NextPatSetIndex;
} DLPC347X_DUAL_InternalPatternStatus_s;


/**
 * Selects the image operating mode for the projection module.
 *
 * \param[in]  OperatingMode  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteOperatingModeSelect(DLPC347X_DUAL_OperatingMode_e OperatingMode);

/**
 * Reads the state of the image operating mode for the projection module.
 *
 * \param[out]  OperatingMode  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadOperatingModeSelect(DLPC347X_DUAL_OperatingMode_e *OperatingMode);

/**
 * Selects the index of a splash screen that is to be displayed. See also Write Splash Screen Execute.
 *
 * \param[in]  SplashScreenIndex  The splash screen index, zero-based (the first splash screen stored in flash is index 0).
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteSplashScreenSelect(uint8_t SplashScreenIndex);

/**
 * Returns the index of a splash screen that is to be displayed (or is being displayed).
 *
 * \param[out]  SplashScreenIndex  The splash screen index, zero-based (the first splash screen stored in flash is index 0).
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadSplashScreenSelect(uint8_t *SplashScreenIndex);

/**
 * Retrieves the select splash screen from flash for display on the projection module. See also Write Splash Screen Select.
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteSplashScreenExecute();

/**
 * Read Splash screen header
 *
 * \param[in]  SplashScreenIndex  The splash screen index, zero-based (the first splash screen stored in flash is index 0).
 * \param[out]  SplashScreenHeader  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadSplashScreenHeader(uint8_t SplashScreenIndex, DLPC347X_DUAL_SplashScreenHeader_s *SplashScreenHeader);

/**
 * Specifies the active external video port and the source data type for the projection module.
 *
 * \param[in]  VideoFormat  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteExternalVideoSourceFormatSelect(DLPC347X_DUAL_ExternalVideoFormat_e VideoFormat);

/**
 * Reads the state of the active external video port and the source data type for the projection module.
 *
 * \param[out]  VideoFormat  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadExternalVideoSourceFormatSelect(DLPC347X_DUAL_ExternalVideoFormat_e *VideoFormat);

/**
 * Specifies the characteristics of the selected YCbCr source and the type of chroma processing that will be used for the YCbCr source in the projection module.
 *
 * \param[in]  ChromaInterpolationMethod  
 * \param[in]  ChromaChannelSwap  
 * \param[in]  CscCoefficientSet  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteVideoChromaProcessingSelect(DLPC347X_DUAL_ChromaInterpolationMethod_e ChromaInterpolationMethod, DLPC347X_DUAL_ChromaChannelSwap_e ChromaChannelSwap, uint8_t CscCoefficientSet);

/**
 * Reads the specified characteristics for the selected YCrCb source and the chroma processing used.
 *
 * \param[out]  ChromaInterpolationMethod  
 * \param[out]  ChromaChannelSwap  
 * \param[out]  CscCoefficientSet  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadVideoChromaProcessingSelect(DLPC347X_DUAL_ChromaInterpolationMethod_e *ChromaInterpolationMethod, DLPC347X_DUAL_ChromaChannelSwap_e *ChromaChannelSwap, uint8_t *CscCoefficientSet);

/**
 * Specifies the 3D frame dominance and reference polarity.
 *
 * \param[in]  ThreeDFrameDominance  
 * \param[in]  ThreeDReferencePolarity  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_Write3DControl(DLPC347X_DUAL_ThreeDDominance_e ThreeDFrameDominance, DLPC347X_DUAL_ThreeDReferencePolarity_e ThreeDReferencePolarity);

/**
 * Reads the 3D frame dominance and reference polarity.
 *
 * \param[out]  ThreeDMode  
 * \param[out]  ThreeDFrameDominance  
 * \param[out]  ThreeDReferencePolarity  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_Read3DControl(DLPC347X_DUAL_ThreeDModes_e *ThreeDMode, DLPC347X_DUAL_ThreeDDominance_e *ThreeDFrameDominance, DLPC347X_DUAL_ThreeDReferencePolarity_e *ThreeDReferencePolarity);

/**
 * Specifies the active data size of the external input image to the projection module.
 *
 * \param[in]  PixelsPerLine  LSB first.
 * \param[in]  LinesPerFrame  LSB first.
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteInputImageSize(uint16_t PixelsPerLine, uint16_t LinesPerFrame);

/**
 * Reads the specified data size of the external input image to the projection module.
 *
 * \param[out]  PixelsPerLine  LSB first.
 * \param[out]  LinesPerFrame  LSB first.
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadInputImageSize(uint16_t *PixelsPerLine, uint16_t *LinesPerFrame);

/**
 * Specifies the size of the active image to be displayed on the projection module.
 *
 * \param[in]  PixelsPerLine  LSB first.
 * \param[in]  LinesPerFrame  LSB first.
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteDisplaySize(uint16_t PixelsPerLine, uint16_t LinesPerFrame);

/**
 * Reads the state of the display size settings for the projection module.
 *
 * \param[out]  PixelsPerLine  LSB first.
 * \param[out]  LinesPerFrame  LSB first.
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadDisplaySize(uint16_t *PixelsPerLine, uint16_t *LinesPerFrame);

/**
 * Specifies the image orientation of the displayed image for the projection module.
 *
 * \param[in]  LongAxisImageFlip  
 * \param[in]  ShortAxisImageFlip  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteDisplayImageOrientation(DLPC347X_DUAL_ImageFlip_e LongAxisImageFlip, DLPC347X_DUAL_ImageFlip_e ShortAxisImageFlip);

/**
 * Reads the state of the displayed image orientation function for the projection module.
 *
 * \param[out]  LongAxisImageFlip  
 * \param[out]  ShortAxisImageFlip  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadDisplayImageOrientation(DLPC347X_DUAL_ImageFlip_e *LongAxisImageFlip, DLPC347X_DUAL_ImageFlip_e *ShortAxisImageFlip);

/**
 * Controls the display image curtain for the projection module. An image curtain fills the entire display with the selected color regardless of selected operating mode (except for Internal Pattern Streaming).
 *
 * \param[in]  Enable  
 * \param[in]  Color  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteDisplayImageCurtain(DLPC347X_DUAL_ImageCurtainEnable_e Enable, DLPC347X_DUAL_Color_e Color);

/**
 * Reads the state of the image curtain control function for the projection module.
 *
 * \param[out]  Enable  
 * \param[out]  Color  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadDisplayImageCurtain(DLPC347X_DUAL_ImageCurtainEnable_e *Enable, DLPC347X_DUAL_Color_e *Color);

/**
 * Enables or disables the image freeze function for the projection module. If enabled, this preserves the current image data.
 *
 * \param[in]  Enable  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteImageFreeze(bool Enable);

/**
 * Reads the state of the image freeze function for the projection module.
 *
 * \param[out]  Enable  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadImageFreeze(bool *Enable);

/**
 * Specifies the on screen border color for the projection module. Whenever the display image size is smaller than the display active area, this color fills the unused area.
 *
 * \param[in]  DisplayBorderColor  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteBorderColor(DLPC347X_DUAL_Color_e DisplayBorderColor);

/**
 * Reads the state of the on screen border color for the projection module.
 *
 * \param[out]  DisplayBorderColor  
 * \param[out]  PillarBoxBorderColorSource  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadBorderColor(DLPC347X_DUAL_Color_e *DisplayBorderColor, DLPC347X_DUAL_BorderColorSource_e *PillarBoxBorderColorSource);

/**
 * Writes a solid field pattern as internal test pattern for display.
 *
 * \param[in]  Border  Enable 1 pixel wide white border around the test pattern. 0: Disable, 1: Enable
 * \param[in]  ForegroundColor  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteSolidField(DLPC347X_DUAL_BorderEnable_e Border, DLPC347X_DUAL_Color_e ForegroundColor);

/**
 * Writes a horizontal ramp pattern as internal test pattern for display.
 *
 * \param[in]  Border  Enable 1 pixel wide white border around the test pattern. 0: Disable, 1: Enable
 * \param[in]  ForegroundColor  
 * \param[in]  StartValue  
 * \param[in]  EndValue  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteHorizontalRamp(DLPC347X_DUAL_BorderEnable_e Border, DLPC347X_DUAL_Color_e ForegroundColor, uint8_t StartValue, uint8_t EndValue);

/**
 * Writes a vertical ramp pattern as internal test pattern for display.
 *
 * \param[in]  Border  Enable 1 pixel wide white border around the test pattern. 0: Disable, 1: Enable
 * \param[in]  ForegroundColor  
 * \param[in]  StartValue  
 * \param[in]  EndValue  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteVerticalRamp(DLPC347X_DUAL_BorderEnable_e Border, DLPC347X_DUAL_Color_e ForegroundColor, uint8_t StartValue, uint8_t EndValue);

/**
 * Writes a horizontal lines pattern as internal test pattern for display.
 *
 * \param[in]  HorizontalLines  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteHorizontalLines(DLPC347X_DUAL_HorizontalLines_s *HorizontalLines);

/**
 * Writes a diagonal lines pattern as internal test pattern for display.
 *
 * \param[in]  DiagonalLines  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteDiagonalLines(DLPC347X_DUAL_DiagonalLines_s *DiagonalLines);

/**
 * Writes a vertical lines pattern as internal test pattern for display.
 *
 * \param[in]  VerticalLines  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteVerticalLines(DLPC347X_DUAL_VerticalLines_s *VerticalLines);

/**
 * Writes a grid lines pattern as internal test pattern for display.
 *
 * \param[in]  GridLines  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteGridLines(DLPC347X_DUAL_GridLines_s *GridLines);

/**
 * Writes a checkerboard pattern as internal test pattern for display. 0: Disable, 1: Enable
 *
 * \param[in]  Checkerboard  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteCheckerboard(DLPC347X_DUAL_Checkerboard_s *Checkerboard);

/**
 * Writes a colorbars pattern as internal test pattern for display.
 *
 * \param[in]  Border  Enable 1 pixel wide white border around the test pattern. 0: Disable, 1: Enable
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteColorbars(DLPC347X_DUAL_BorderEnable_e Border);

/**
 * Reads back the host-specified parameters for an internal test pattern.
 *
 * \param[out]  TestPatternSelect  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadTestPatternSelect(DLPC347X_DUAL_TestPatternSelect_s *TestPatternSelect);

/**
 * Executes a batch file stored in the flash image.
 *
 * \param[in]  BatchFileNumber  Zero-based index.
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteExecuteFlashBatchFile(uint8_t BatchFileNumber);

/**
 * Delays for the specified number of microseconds. Only valid within a batch file.
 *
 * \param[in]  DelayInMicroseconds  LSB first.
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteBatchFileDelay(uint16_t DelayInMicroseconds);

/**
 * Specifies the method for controlling the LED outputs for the projection module.
 *
 * \param[in]  LedControlMethod  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteLedOutputControlMethod(DLPC347X_DUAL_LedControlMethod_e LedControlMethod);

/**
 * Reads the state of the LED output control method for the projection module.
 *
 * \param[out]  LedControlMethod  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadLedOutputControlMethod(DLPC347X_DUAL_LedControlMethod_e *LedControlMethod);

/**
 * Enables the LEDs for the projection module.
 *
 * \param[in]  RedLedEnable  0: Disable, 1: Enable
 * \param[in]  GreenLedEnable  0: Disable, 1: Enable
 * \param[in]  BlueLedEnable  0: Disable, 1: Enable
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteRgbLedEnable(bool RedLedEnable, bool GreenLedEnable, bool BlueLedEnable);

/**
 * Reads the state of the LED enables for the projection module.
 *
 * \param[out]  RedLedEnable  0: Disable, 1: Enable
 * \param[out]  GreenLedEnable  0: Disable, 1: Enable
 * \param[out]  BlueLedEnable  0: Disable, 1: Enable
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadRgbLedEnable(bool *RedLedEnable, bool *GreenLedEnable, bool *BlueLedEnable);

/**
 * Sets the IDAC register value of the PMIC for the red, green, and blue LEDs. This value directly controls the LED current.
 *
 * \param[in]  RedLedCurrent  LSB first.
 * \param[in]  GreenLedCurrent  LSB first.
 * \param[in]  BlueLedCurrent  LSB first.
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteRgbLedCurrent(uint16_t RedLedCurrent, uint16_t GreenLedCurrent, uint16_t BlueLedCurrent);

/**
 * Reads the state of the current for the red, green, and blue LEDs. This value directly controls the LED current.
 *
 * \param[out]  RedLedCurrent  LSB first.
 * \param[out]  GreenLedCurrent  LSB first.
 * \param[out]  BlueLedCurrent  LSB first.
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadRgbLedCurrent(uint16_t *RedLedCurrent, uint16_t *GreenLedCurrent, uint16_t *BlueLedCurrent);

/**
 * Reads the specified maximum LED power (Watts) allowed for the projection module.
 *
 * \param[out]  MaxLedPower  LSB first.
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadCaicLedMaxAvailablePower(double *MaxLedPower);

/**
 * Specifies the maximum LED current allowed for each LED in the projection module.
 *
 * \param[in]  MaxRedLedCurrent  LSB first.
 * \param[in]  MaxGreenLedCurrent  LSB first.
 * \param[in]  MaxBlueLedCurrent  LSB first.
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteRgbLedMaxCurrent(uint16_t MaxRedLedCurrent, uint16_t MaxGreenLedCurrent, uint16_t MaxBlueLedCurrent);

/**
 * Reads the specified maximum LED current allowed for each LED in the projection module.
 *
 * \param[out]  MaxRedLedCurrent  LSB first.
 * \param[out]  MaxGreenLedCurrent  LSB first.
 * \param[out]  MaxBlueLedCurrent  LSB first.
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadRgbLedMaxCurrent(uint16_t *MaxRedLedCurrent, uint16_t *MaxGreenLedCurrent, uint16_t *MaxBlueLedCurrent);

/**
 * Reads the state of the current for the red, green, and blue LEDs of the projection module.
 *
 * \param[out]  RedLedCurrent  LSB first.
 * \param[out]  GreenLedCurrent  LSB first.
 * \param[out]  BlueLedCurrent  LSB first.
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadCaicRgbLedCurrent(uint16_t *RedLedCurrent, uint16_t *GreenLedCurrent, uint16_t *BlueLedCurrent);

/**
 * Specifies the Look for the image on the projection module. A Look typically specifies a target white point.
 *
 * \param[in]  LookNumber  Zero-based index.
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteLookSelect(uint8_t LookNumber);

/**
 * Reads the state of the Look select command for the projection module.
 *
 * \param[out]  LookNumber  Zero-based index.
 * \param[out]  SequenceIndex  Zero-based index.
 * \param[out]  SequenceFrameTime  LSB first.
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadLookSelect(uint8_t *LookNumber, uint8_t *SequenceIndex, double *SequenceFrameTime);

/**
 * Reads Look and Sequence header information for the active Look and Sequence of the projection module.
 *
 * \param[out]  SequenceHeaderAttributes  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadSequenceHeaderAttributes(DLPC347X_DUAL_SequenceHeaderAttributes_s *SequenceHeaderAttributes);

/**
 * Controls the local area brightness boost image processing functionality for the projection module.
 *
 * \param[in]  LabbControl  
 * \param[in]  SharpnessStrength  
 * \param[in]  LabbStrengthSetting  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteLocalAreaBrightnessBoostControl(DLPC347X_DUAL_LabbControl_e LabbControl, uint8_t SharpnessStrength, uint8_t LabbStrengthSetting);

/**
 * Reads the state of the local area brightness boost image processing functionality for the projection module.
 *
 * \param[out]  LabbControl  
 * \param[out]  SharpnessStrength  
 * \param[out]  LabbStrengthSetting  
 * \param[out]  LabbGainValue  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadLocalAreaBrightnessBoostControl(DLPC347X_DUAL_LabbControl_e *LabbControl, uint8_t *SharpnessStrength, uint8_t *LabbStrengthSetting, uint8_t *LabbGainValue);

/**
 * Controls the CAIC functionality for the projection module.
 *
 * \param[in]  CaicGainDisplayScale  
 * \param[in]  CaicGainDisplayEnable  
 * \param[in]  CaicMaxLumensGain  
 * \param[in]  CaicClippingThreshold  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteCaicImageProcessingControl(DLPC347X_DUAL_CaicGainDisplayScale_e CaicGainDisplayScale, bool CaicGainDisplayEnable, double CaicMaxLumensGain, double CaicClippingThreshold);

/**
 * Reads the state of the CAIC functionality within the projection module.
 *
 * \param[out]  CaicGainDisplayScale  
 * \param[out]  CaicGainDisplayEnable  
 * \param[out]  CaicMaxLumensGain  
 * \param[out]  CaicClippingThreshold  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadCaicImageProcessingControl(DLPC347X_DUAL_CaicGainDisplayScale_e *CaicGainDisplayScale, bool *CaicGainDisplayEnable, double *CaicMaxLumensGain, double *CaicClippingThreshold);

/**
 * Controls the Color Coordinate Adjustment (CCA) image processing functionality for the projection module.
 *
 * \param[in]  CcaEnable  Must be enabled during normal operation. 0: Disable, 1: Enable
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteColorCoordinateAdjustmentControl(bool CcaEnable);

/**
 * Reads the state of the Color Coordinate Adjustment (CCA) image processing within the projection module.
 *
 * \param[out]  CcaEnable  0: Disable, 1: Enable
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadColorCoordinateAdjustmentControl(bool *CcaEnable);

/**
 * Provides a brief system status for the projection module.
 *
 * \param[out]  ShortStatus  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadShortStatus(DLPC347X_DUAL_ShortStatus_s *ShortStatus);

/**
 * Reads system status information for the projection module.
 *
 * \param[out]  SystemStatus  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadSystemStatus(DLPC347X_DUAL_SystemStatus_s *SystemStatus);

/**
 * Reads I2C communication status information for the projection module.
 *
 * \param[out]  CommunicationStatus  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadCommunicationStatus(DLPC347X_DUAL_CommunicationStatus_s *CommunicationStatus);

/**
 * Reads the Arm software version (main application) information for the projection module. This application is part of the firmware image.
 *
 * \param[out]  PatchVersion  
 * \param[out]  MinorVersion  
 * \param[out]  MajorVersion  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadSystemSoftwareVersion(uint16_t *PatchVersion, uint8_t *MinorVersion, uint8_t *MajorVersion);

/**
 * Reads the controller device ID for the projection module.
 *
 * \param[out]  DeviceId  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadControllerDeviceId(DLPC347X_DUAL_ControllerDeviceId_e *DeviceId);

/**
 * Reads the DMD device ID or DMD fuse data for the projection module.
 *
 * \param[in]  DmdDataSelection  
 * \param[out]  DeviceId  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadDmdDeviceId(DLPC347X_DUAL_DmdDataSelection_e DmdDataSelection, uint32_t *DeviceId);

/**
 * Reads the controller firmware version for the projection module.
 *
 * \param[out]  PatchVersion  
 * \param[out]  MinorVersion  
 * \param[out]  MajorVersion  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadFirmwareBuildVersion(uint16_t *PatchVersion, uint8_t *MinorVersion, uint8_t *MajorVersion);

/**
 * Reads the System Temperature.
 *
 * \param[out]  Temperature  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadSystemTemperature(double *Temperature);

/**
 * Verifies that a pending flash update (write) is appropriate for the specified block of the projection module flash. Must have called Write Flash Data Type Select prior.
 *
 * \param[in]  FlashUpdatePackageSize  LSB first.
 * \param[out]  PackageSizeStatus  
 * \param[out]  PacakgeConfigurationCollapsed  
 * \param[out]  PacakgeConfigurationIdentifier  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadFlashUpdatePrecheck(uint32_t FlashUpdatePackageSize, DLPC347X_DUAL_Error_e *PackageSizeStatus, DLPC347X_DUAL_Error_e *PacakgeConfigurationCollapsed, DLPC347X_DUAL_Error_e *PacakgeConfigurationIdentifier);

/**
 * Selects the data block that will be written/read from the flash.
 *
 * \param[in]  FlashSelect  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteFlashDataTypeSelect(DLPC347X_DUAL_FlashDataTypeSelect_e FlashSelect);

/**
 * Specifies the length in bytes of data that will be written/read from the flash.
 *
 * \param[in]  FlashDataLength  LSB first.
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteFlashDataLength(uint16_t FlashDataLength);

/**
 * Erases the selected flash data.
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteFlashErase();

/**
 * Writes data to the flash.
 *
 * \param[in]  DataLength  Byte Length for Data
 * \param[in]  Data  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteFlashStart(uint16_t DataLength, uint8_t* Data);

/**
 * Reads data from the flash.
 *
 * \param[in]  Length  
 * \param[out]  Data  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadFlashStart(uint16_t Length, uint8_t *Data);

/**
 * Writes data to the flash.
 *
 * \param[in]  DataLength  Byte Length for Data
 * \param[in]  Data  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteFlashContinue(uint16_t DataLength, uint8_t* Data);

/**
 * Reads data from the flash.
 *
 * \param[in]  Length  
 * \param[out]  Data  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadFlashContinue(uint16_t Length, uint8_t *Data);

/**
 * Reads the Light Control Sequence Binary version.
 *
 * \param[out]  PatchVersion  
 * \param[out]  MinorVersion  
 * \param[out]  MajorVersion  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadSequenceBinaryVersion(uint8_t *PatchVersion, uint8_t *MinorVersion, uint8_t *MajorVersion);

/**
 * Specifies the control for the Internal Pattern.
 *
 * \param[in]  PatternControl  
 * \param[in]  RepeatCount  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteInternalPatternControl(DLPC347X_DUAL_PatternControl_e PatternControl, uint8_t RepeatCount);

/**
 * Checks the sequence database for support of the exposure time.
 *
 * \param[in]  PatternMode  Pattern Mode
 * \param[in]  BitDepth  Bit Depth
 * \param[in]  ExposureTime  Exposure Time
 * \param[out]  ValidateExposureTime  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadValidateExposureTime(DLPC347X_DUAL_PatternMode_e PatternMode, DLPC347X_DUAL_SequenceType_e BitDepth, uint32_t ExposureTime, DLPC347X_DUAL_ValidateExposureTime_s *ValidateExposureTime);

/**
 * Specifies the configuration for Trigger In.
 *
 * \param[in]  TriggerEnable  
 * \param[in]  TriggerPolarity  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteTriggerInConfiguration(DLPC347X_DUAL_TriggerEnable_e TriggerEnable, DLPC347X_DUAL_TriggerPolarity_e TriggerPolarity);

/**
 * Reads the configuration for Trigger In.
 *
 * \param[out]  TriggerEnable  
 * \param[out]  TriggerPolarity  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadTriggerInConfiguration(DLPC347X_DUAL_TriggerEnable_e *TriggerEnable, DLPC347X_DUAL_TriggerPolarity_e *TriggerPolarity);

/**
 * Specifies the configuration for Trigger Out1 or Trigger Out2.
 *
 * \param[in]  TriggerType  
 * \param[in]  TriggerEnable  
 * \param[in]  TriggerInversion  
 * \param[in]  Delay  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WriteTriggerOutConfiguration(DLPC347X_DUAL_TriggerType_e TriggerType, DLPC347X_DUAL_TriggerEnable_e TriggerEnable, DLPC347X_DUAL_TriggerInversion_e TriggerInversion, int32_t Delay);

/**
 * Reads the configuration for Trigger Out1 or Trigger Out2.
 *
 * \param[in]  Trigger  Trigger Type
 * \param[out]  TriggerEnable  
 * \param[out]  TriggerInversion  
 * \param[out]  Delay  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadTriggerOutConfiguration(DLPC347X_DUAL_TriggerType_e Trigger, DLPC347X_DUAL_TriggerEnable_e *TriggerEnable, DLPC347X_DUAL_TriggerInversion_e *TriggerInversion, int32_t *Delay);

/**
 * Specifies the configuration for the Pattern Ready output signal.
 *
 * \param[in]  TriggerEnable  
 * \param[in]  TriggerPolarity  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WritePatternReadyConfiguration(DLPC347X_DUAL_TriggerEnable_e TriggerEnable, DLPC347X_DUAL_TriggerPolarity_e TriggerPolarity);

/**
 * Reads the configuration for Pattern Ready output signal.
 *
 * \param[out]  TriggerEnable  
 * \param[out]  TriggerPolarity  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadPatternReadyConfiguration(DLPC347X_DUAL_TriggerEnable_e *TriggerEnable, DLPC347X_DUAL_TriggerPolarity_e *TriggerPolarity);

/**
 * Specifies the configuration for external and internal patterns.
 *
 * \param[in]  PatternConfiguration  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WritePatternConfiguration(DLPC347X_DUAL_PatternConfiguration_s *PatternConfiguration);

/**
 * Reads the configuration for external and internal patterns.
 *
 * \param[out]  PatternConfiguration  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadPatternConfiguration(DLPC347X_DUAL_PatternConfiguration_s *PatternConfiguration);

/**
 * Specifies the configuration for the Pattern Order Table.
 *
 * \param[in]  WriteControl  
 * \param[in]  PatternOrderTableEntry  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_WritePatternOrderTableEntry(DLPC347X_DUAL_WriteControl_e WriteControl, DLPC347X_DUAL_PatternOrderTableEntry_s *PatternOrderTableEntry);

/**
 * Reads the configuration for the Pattern Order Table.
 *
 * \param[in]  PatternOrderTableEntryIndex  pattern_order_table_entry_index
 * \param[out]  PatternOrderTableEntry  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadPatternOrderTableEntry(uint8_t PatternOrderTableEntryIndex, DLPC347X_DUAL_PatternOrderTableEntry_s *PatternOrderTableEntry);

/**
 * Reads the Status of the Internal Pattern Streaming.
 *
 * \param[out]  InternalPatternStatus  
 *
 * \return 0 if successful, error code otherwise
 */
uint32_t DLPC347X_DUAL_ReadInternalPatternStatus(DLPC347X_DUAL_InternalPatternStatus_s *InternalPatternStatus);

#ifdef __cplusplus    /* matches __cplusplus construct above */
}
#endif
#endif /* DLPC347X_DUAL_H */
