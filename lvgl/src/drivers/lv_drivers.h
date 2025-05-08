/**
 * @file lv_drivers.h
 *
 */

#ifndef LV_DRIVERS_H
#define LV_DRIVERS_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/





#include "display/drm/lv_linux_drm.h"
#include "display/fb/lv_linux_fbdev.h"

#include "display/tft_espi/lv_tft_espi.h"

#include "display/lcd/lv_lcd_generic_mipi.h"
#include "display/ili9341/lv_ili9341.h"
#include "display/st7735/lv_st7735.h"
#include "display/st7789/lv_st7789.h"
#include "display/st7796/lv_st7796.h"

#include "display/renesas_glcdc/lv_renesas_glcdc.h"
#include "display/st_ltdc/lv_st_ltdc.h"
#include "display/ft81x/lv_ft81x.h"







/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_DRIVERS_H*/
