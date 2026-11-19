/**
 * @file lv_extra.h
 *
 */

#ifndef LV_EXTRA_H
#define LV_EXTRA_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

#include "../../../lvgl/src/extra/layouts/lv_layouts.h"
#include "../../../lvgl/src/extra/libs/lv_libs.h"
#include "../../../lvgl/src/extra/others/lv_others.h"
#include "../../../lvgl/src/extra/themes/lv_themes.h"
#include "../../../lvgl/src/extra/widgets/lv_widgets.h"

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/

/**
 * Initialize the extra components
 */
void lv_extra_init(void);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /*extern "C"*/
#endif

#endif /*LV_EXTRA_H*/
