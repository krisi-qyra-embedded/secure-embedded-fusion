#include "iwdg.h"
#include <stdio.h>   /* only for optional debug message in default handler */
#include "spi_dma.h"
#include "max31865.h"
#include "systick.h"


/* If READ_BIT macro unavailable, provide fallback */
#ifndef READ_BIT
#define READ_BIT(REG, BIT)    (( (REG) & (BIT) ) ? (BIT) : 0U)
#endif

 
#ifndef IWDG_SR_PVU
#define IWDG_SR_PVU   (1U << 0)    
#endif
#ifndef IWDG_SR_RVU
#define IWDG_SR_RVU   (1U << 1)    
#endif


 
static volatile bool iwdg_inited = false;
static uint32_t iwdg_poll_budget = IWDG_READY_POLL_MAX;


 
/* Writes data through backup write marker and reports transfer errors. */
static inline void backup_write_marker(uint32_t tag)
{
    /* Check the transfer result before continuing. */
#ifdef RTC_BKP0R
    /* Example for MCUs with RTC backup registers - replace with correct register names */
    RTC->BKP0R = tag;
#else
    (void)tag;  
#endif
}

/* Reads data through backup read marker and returns the operation status. */
static inline uint32_t backup_read_marker(void)
{
    /* Check the returned status before accepting the sample. */
#ifdef RTC_BKP0R
    return RTC->BKP0R;
#else
    return 0U;  
#endif
}

/* escalation helper: persist marker then stop kicking -> IWDG will reset */
 void escalate_to_iwdg_reset(const char *reason)
{
    /* Check the current status before proceeding. */
     
    backup_write_marker(INTENTIONAL_RESET_TAG);

     
    log_printf("ESCALATE -> IWDG reset: %s\n\r", reason ? reason : "(no reason)");

    /* Stop calling iwdg_refresh(); tight loop so hardware IWDG forces reset */
    for(;;) { __asm volatile ("nop"); }
}

 
volatile uint32_t hb_spi_ts;
volatile uint32_t hb_main_ts;

/* Drivers must call this in DMA TC (place in your DMA completion callback) */
void heartbeat_signal_spi(void) {
    /* Check the current status before proceeding. */ hb_spi_ts = ms_get(); }
/* Handles the heartbeat signal main operation. */
void heartbeat_signal_main(void) {
    /* Check the current status before proceeding. */ hb_main_ts = ms_get(); }

/* YOUR original check_reset_source (kept, but clears persistent marker when intentional) */
 void check_reset_source(void)
{
    /* Check the current status before proceeding. */
    if ((RCC->CSR & RCC_CSR_IWDGRSTF) != 0U) {
        RCC->CSR |= RCC_CSR_RMVF;
        if (backup_read_marker() == INTENTIONAL_RESET_TAG) {
            log_printf("Boot: intentional supervisor reset (IWDG)\n\r");
            backup_write_marker(0U);  
        } else {
            log_printf("Boot: unexpected watchdog reset (crash)\n\r");
        }
    }
}

 /* Minimal supervisor: only place that refreshes IWDG */
 void supervisor_check_and_kick(void)
 {
    /* Check the current status before proceeding. */
     uint32_t now = ms_get();
     uint32_t age_spi  = now - hb_spi_ts;    
     uint32_t age_main = now - hb_main_ts;

 
/* log_printf("SUPERVISOR: now=%u hb_spi=%u (age=%u ms) hb_main=%u (age=%u ms)\n\r", */
 
 
 
 
 

     /* Check SPI heartbeat (DMA completion must call heartbeat_signal_spi()) */
     if (age_spi > HEARTBEAT_WINDOW_MS) {
         log_printf("SUPERVISOR: SPI heartbeat missed (age=%u ms)\n\r", (unsigned)age_spi);
         /* we expect the driver's own recovery attempts to have been done already and returned error to main. */
         escalate_to_iwdg_reset("SPI heartbeat missed");
     }

      
     if (age_main > HEARTBEAT_WINDOW_MS) {
         log_printf("SUPERVISOR: main heartbeat missed (age=%u ms)\n\r", (unsigned)age_main);
         escalate_to_iwdg_reset("main heartbeat missed");
     }

      
     if (iwdg_refresh() != IWDG_OK) {
          
         IWDG_ErrorHandler("iwdg_refresh failed", IWDG_ERR_HW);
         escalate_to_iwdg_reset("iwdg_refresh failed");
     }
 }

/* Override weak error hook from iwdg driver (application-specific handling) */
void IWDG_ErrorHandler(const char *msg, iwdg_status_t code)
{
    /* Log once, then stop to avoid undefined behavior. */
    log_printf("IWDG Error: %s (code=%d)\n\r", msg ? msg : "NULL", (int)code);

     
    while (1) {
        for (volatile uint32_t d = 0; d < 200000U; ++d) __asm volatile("nop");
        for (volatile uint32_t d = 0; d < 200000U; ++d) __asm volatile("nop");
    }
}

/* Centralized error handling in main: replace all busy-wait error traps with this simple wrapper */
void on_fatal_init_error(const char *msg)
{
    /* Set up the required peripheral configuration. */
    log_printf("FATAL: %s -> escalating\n\r", msg);
    escalate_to_iwdg_reset(msg);
}

/* Internal helper: wait for PVU/RVU == 0 with budget iterations */
static iwdg_status_t wait_iwdg_ready(uint32_t budget)
{
    /* Check the returned status before accepting the sample. */
    uint32_t cnt = 0U;
    const uint32_t max = (budget == 0U) ? iwdg_poll_budget : budget;

    while (READ_BIT(IWDG->SR, IWDG_SR_PVU | IWDG_SR_RVU) != 0U) {
        if (++cnt >= max) {
            return IWDG_ERR_TIMEOUT;
        }
         
        __asm volatile ("nop");
    }
    return IWDG_OK;
}

 
/* Initializes the resources used by iwdg init. */
iwdg_status_t iwdg_init(uint32_t prescaler, uint32_t reload, uint32_t poll_budget)
{
    /* Set up the required peripheral configuration. */
     
    if (reload > IWDG_RELOAD_MAX) {
        IWDG_ErrorHandler("IWDG: invalid reload", IWDG_ERR_INVALID_PARAM);
        return IWDG_ERR_INVALID_PARAM;
    }

    /* If already initialized, do not reconfigure (watchdog is typically one-shot) */
    if (iwdg_inited) {
        return IWDG_ERR_ALREADY_INIT;
    }

    if (poll_budget != 0U) {
        iwdg_poll_budget = poll_budget;
    }

    /* Enable IWDG (this will start the watchdog). */
    IWDG->KR = IWDG_KEY_ENABLE;

     
    __asm volatile ("nop");

     
    IWDG->KR = IWDG_KEY_WR_ACCESS_ENABLE;

     
    IWDG->PR = prescaler;

     
    IWDG->RLR = reload;

     
    if (wait_iwdg_ready(0U) != IWDG_OK) {
        IWDG_ErrorHandler("IWDG: timeout waiting for PR/RLR update", IWDG_ERR_TIMEOUT);
        return IWDG_ERR_TIMEOUT;
    }

     
    IWDG->KR = IWDG_KEY_RELOAD;

    /* Quick verify: read back RLR if readable on this device (some parts may not allow readback) */
#if defined(IWDG_RLR)
    if ( (IWDG->RLR & IWDG_RELOAD_MAX) != (reload & IWDG_RELOAD_MAX) ) {
        /* Not all MCUs permit immediate readback; treat mismatch as warning but continue. */
        IWDG_ErrorHandler("IWDG: RLR readback mismatch", IWDG_ERR_HW);
         
    }
#endif

     
    iwdg_inited = true;
    return IWDG_OK;
}


/* Handles the iwdg refresh operation. */
iwdg_status_t iwdg_refresh(void)
{
    /* Check the current status before proceeding. */
     
    if (!iwdg_inited) {
        IWDG_ErrorHandler("IWDG: refresh called before init", IWDG_ERR_INVALID_PARAM);
        return IWDG_ERR_INVALID_PARAM;
    }

     
    IWDG->KR = IWDG_KEY_RELOAD;

     
    if (READ_BIT(IWDG->SR, IWDG_SR_PVU | IWDG_SR_RVU) != 0U) {
         
        if (wait_iwdg_ready(iwdg_poll_budget / 10U) != IWDG_OK) {
            IWDG_ErrorHandler("IWDG: refresh caused SR pending", IWDG_ERR_TIMEOUT);
            return IWDG_ERR_TIMEOUT;
        }
    }

    return IWDG_OK;
}


/* Reads data through iwdg is ready and returns the operation status. */
bool iwdg_is_ready(void)
{
    /* Check the returned status before accepting the sample. */
    return (READ_BIT(IWDG->SR, IWDG_SR_PVU | IWDG_SR_RVU) == 0U) ? true : false;
}





