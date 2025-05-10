
#include "sdram.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_ll_fmc.h"
#include "lvgl.h"
#include "main.h"
#include "lv_draw_buf.h"

extern SDRAM_HandleTypeDef hsdram1;
static FMC_SDRAM_CommandTypeDef Command;








void BSP_SDRAM_Initialization_sequence(SDRAM_HandleTypeDef *hsdram, uint32_t RefreshCount)
{
  __IO uint32_t tmpmrd = 0;

  /* Step 1: Configure a clock configuration enable command */
  Command.CommandMode            = FMC_SDRAM_CMD_CLK_ENABLE;
  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
  Command.AutoRefreshNumber      = 1;
  Command.ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

  /* Step 2: Insert 100 us minimum delay */
  /* Inserted delay is equal to 1 ms due to systick time base unit (ms) */
  HAL_Delay(1);

  /* Step 3: Configure a PALL (precharge all) command */
  Command.CommandMode            = FMC_SDRAM_CMD_PALL;
  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
  Command.AutoRefreshNumber      = 1;
  Command.ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

  /* Step 4: Configure an Auto Refresh command */
  Command.CommandMode            = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
  Command.AutoRefreshNumber      = 8;
  Command.ModeRegisterDefinition = 0;

  /* Send the command */
  HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

  /* Step 5: Program the external memory mode register */
  tmpmrd = (uint32_t)SDRAM_MODEREG_BURST_LENGTH_1          |\
                     SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL   |\
                     SDRAM_MODEREG_CAS_LATENCY_2           |\
                     SDRAM_MODEREG_OPERATING_MODE_STANDARD |\
                     SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;

  Command.CommandMode            = FMC_SDRAM_CMD_LOAD_MODE;
  Command.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
  Command.AutoRefreshNumber      = 1;
  Command.ModeRegisterDefinition = tmpmrd;

  /* Send the command */
  HAL_SDRAM_SendCommand(&hsdram1, &Command, SDRAM_TIMEOUT);

  /* Step 6: Set the refresh rate counter */
  /* Set the device refresh rate */
  HAL_SDRAM_ProgramRefreshRate(&hsdram1, RefreshCount);
}

/* Framebuffer placé en SDRAM */
__attribute__((section(".sdram"))) static uint16_t my_fb[480 * 100];  // framebuffer en SDRAM
static lv_draw_buf_t draw_buf;



/* Fonction de flush : ici on ne copie rien car LTDC lit déjà directement dans la SDRAM */
void my_flush_cb(lv_display_t * disp, const lv_area_t * area, uint8_t * px_map)
{
    lv_display_flush_ready(disp);
}


/* Fonction d'initialisation de l'afficheur avec LVGL */
void display_init(void)
{
    lv_display_t * disp = lv_display_create(480, 100);

    // Initialisation du buffer de dessin (mode direct, un seul buffer)
    lv_draw_buf_init(&draw_buf,
                     480,                     // largeur
                     272,                     // hauteur
                     LV_COLOR_FORMAT_RGB565,  // format couleur
                     480 * 2,                 // stride = largeur * 2 (car 2 octets par pixel)
                     my_fb,                   // pointeur vers les pixels
                     sizeof(my_fb));          // taille totale du buffer

    // Association du buffer au display
    lv_display_set_draw_buffers(disp, &draw_buf, NULL);

    // Définir un flush_cb (même s'il ne fait rien, obligatoire)
    lv_display_set_flush_cb(disp, my_flush_cb);
}
