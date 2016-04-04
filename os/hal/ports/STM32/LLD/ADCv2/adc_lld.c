/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    STM32/LLD/ADCv2/adc_lld.c
 * @brief   STM32 ADC subsystem low level driver source.
 *
 * @addtogroup ADC
 * @{
 */

#include "hal.h"

#if HAL_USE_ADC || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

#define ADC1_DMA_CHANNEL                                                    \
  STM32_DMA_GETCHANNEL(STM32_ADC_ADC1_DMA_STREAM, STM32_ADC1_DMA_CHN)

#define ADC2_DMA_CHANNEL                                                    \
  STM32_DMA_GETCHANNEL(STM32_ADC_ADC2_DMA_STREAM, STM32_ADC2_DMA_CHN)

#define ADC3_DMA_CHANNEL                                                    \
  STM32_DMA_GETCHANNEL(STM32_ADC_ADC3_DMA_STREAM, STM32_ADC3_DMA_CHN)

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief ADC1 driver identifier.*/
#if STM32_ADC_USE_ADC1 || defined(__DOXYGEN__)
ADCDriver ADCD1;
#endif

/** @brief ADC2 driver identifier.*/
#if STM32_ADC_USE_ADC2 || defined(__DOXYGEN__)
ADCDriver ADCD2;
#endif

/** @brief ADC3 driver identifier.*/
#if STM32_ADC_USE_ADC3 || defined(__DOXYGEN__)
ADCDriver ADCD3;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   ADC DMA ISR service routine.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 * @param[in] flags     pre-shifted content of the ISR register
 */
static void adc_lld_serve_rx_interrupt(ADCDriver *adcp, uint32_t flags) {

  /* DMA errors handling.*/
  if ((flags & (STM32_DMA_ISR_TEIF | STM32_DMA_ISR_DMEIF)) != 0) {
    /* DMA, this could help only if the DMA tries to access an unmapped
       address space or violates alignment rules.*/
    _adc_isr_error_code(adcp, ADC_ERR_DMAFAILURE);
  }
  else {
    /* It is possible that the conversion group has already be reset by the
       ADC error handler, in this case this interrupt is spurious.*/
    if (adcp->grpp != NULL) {

      if ((flags & STM32_DMA_ISR_TCIF) != 0) {
        /* Transfer complete processing.*/
        _adc_isr_full_code(adcp);
      }
      else if ((flags & STM32_DMA_ISR_HTIF) != 0) {
        /* Half transfer processing.*/
        _adc_isr_half_code(adcp);
      }
    }
  }
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if STM32_ADC_USE_ADC1 || STM32_ADC_USE_ADC2 || STM32_ADC_USE_ADC3 ||       \
    defined(__DOXYGEN__)
/**
 * @brief   ADC interrupt handler.
 *
 * @isr
 */
 //TODO: in dual adc mode, instead of SR registers, CSR is used
 //See if interrupt on one of the ADCs for example ADC1 is OK
 //TODO:maybe we need to define some registesr on both ADC1 and ADC2 in dual mode
 //For Example CONT bit.
 //See dual interleaved mode 13.9.3 in DM00031020.pdf
OSAL_IRQ_HANDLER(STM32_ADC_HANDLER) {
  uint32_t sr;

  OSAL_IRQ_PROLOGUE();
#if STM32_ADC_DUAL_MODE
  sr = ADC1->SR;
  sr |= ADC2->SR;
  ADC1->SR = 0;
  ADC2->SR = 0;
  /* Note, an overflow may occur after the conversion ended before the driver
     is able to stop the ADC, this is why the DMA channel is checked too.*/
  if ((sr & ADC_SR_OVR) && (dmaStreamGetTransactionSize(ADCD1.dmastp) > 0)) {
    /* ADC overflow condition, this could happen only if the DMA is unable
       to read data fast enough.*/
    if (ADCD1.grpp != NULL)
      _adc_isr_error_code(&ADCD1, ADC_ERR_OVERFLOW);
  }
  /* TODO: Add here analog watchdog handling.*/
#else /* !STM32_ADC_DUAL_MODE */
#if STM32_ADC_USE_ADC1
  sr = ADC1->SR;
  ADC1->SR = 0;
  /* Note, an overflow may occur after the conversion ended before the driver
     is able to stop the ADC, this is why the DMA channel is checked too.*/
  if ((sr & ADC_SR_OVR) && (dmaStreamGetTransactionSize(ADCD1.dmastp) > 0)) {
    /* ADC overflow condition, this could happen only if the DMA is unable
       to read data fast enough.*/
    if (ADCD1.grpp != NULL)
      _adc_isr_error_code(&ADCD1, ADC_ERR_OVERFLOW);
  }
  /* TODO: Add here analog watchdog handling.*/
#endif /* STM32_ADC_USE_ADC1 */

#if STM32_ADC_USE_ADC2
  sr = ADC2->SR;
  ADC2->SR = 0;
  /* Note, an overflow may occur after the conversion ended before the driver
     is able to stop the ADC, this is why the DMA channel is checked too.*/
  if ((sr & ADC_SR_OVR) && (dmaStreamGetTransactionSize(ADCD2.dmastp) > 0)) {
    /* ADC overflow condition, this could happen only if the DMA is unable
       to read data fast enough.*/
    if (ADCD2.grpp != NULL)
      _adc_isr_error_code(&ADCD2, ADC_ERR_OVERFLOW);
  }
  /* TODO: Add here analog watchdog handling.*/
#endif /* STM32_ADC_USE_ADC2 */

#if STM32_ADC_USE_ADC3
  sr = ADC3->SR;
  ADC3->SR = 0;
  /* Note, an overflow may occur after the conversion ended before the driver
     is able to stop the ADC, this is why the DMA channel is checked too.*/
  if ((sr & ADC_SR_OVR) && (dmaStreamGetTransactionSize(ADCD3.dmastp) > 0)) {
    /* ADC overflow condition, this could happen only if the DMA is unable
       to read data fast enough.*/
    if (ADCD3.grpp != NULL)
      _adc_isr_error_code(&ADCD3, ADC_ERR_OVERFLOW);
  }
  /* TODO: Add here analog watchdog handling.*/
#endif /* STM32_ADC_USE_ADC3 */
#endif /* !STM32_ADC_DUAL_MODE */
  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level ADC driver initialization.
 *
 * @notapi
 */
 //TODO: check DMA registers bits
void adc_lld_init(void) {

#if STM32_ADC_USE_ADC1
  /* Driver initialization.*/
  adcObjectInit(&ADCD1);
  ADCD1.adcm = ADC1;
  ADCD1.adcc = ADC;
#if STM32_ADC_DUAL_MODE
  ADCD1.adcs = ADC2;
#endif
  ADCD1.dmastp  = STM32_DMA_STREAM(STM32_ADC_ADC1_DMA_STREAM);
  ADCD1.dmamode = STM32_DMA_CR_CHSEL(ADC1_DMA_CHANNEL) |
                  STM32_DMA_CR_PL(STM32_ADC_ADC1_DMA_PRIORITY) |
                  STM32_DMA_CR_DIR_P2M |
                  STM32_DMA_CR_MSIZE_HWORD | STM32_DMA_CR_PSIZE_HWORD |
                  STM32_DMA_CR_MINC        | STM32_DMA_CR_TCIE        |
                  STM32_DMA_CR_DMEIE       | STM32_DMA_CR_TEIE;
#endif

#if STM32_ADC_USE_ADC2
  /* Driver initialization.*/
  adcObjectInit(&ADCD2);
  ADCD2.adcm = ADC2;
  ADCD2.adcc = ADC;
  ADCD2.dmastp  = STM32_DMA_STREAM(STM32_ADC_ADC2_DMA_STREAM);
  ADCD2.dmamode = STM32_DMA_CR_CHSEL(ADC2_DMA_CHANNEL) |
                  STM32_DMA_CR_PL(STM32_ADC_ADC2_DMA_PRIORITY) |
                  STM32_DMA_CR_DIR_P2M |
                  STM32_DMA_CR_MSIZE_HWORD | STM32_DMA_CR_PSIZE_HWORD |
                  STM32_DMA_CR_MINC        | STM32_DMA_CR_TCIE        |
                  STM32_DMA_CR_DMEIE       | STM32_DMA_CR_TEIE;
#endif

#if STM32_ADC_USE_ADC3
  /* Driver initialization.*/
  adcObjectInit(&ADCD3);
  ADCD3.adcm = ADC3;
  ADCD3.adcc = ADC;
  ADCD3.dmastp  = STM32_DMA_STREAM(STM32_ADC_ADC3_DMA_STREAM);
  ADCD3.dmamode = STM32_DMA_CR_CHSEL(ADC3_DMA_CHANNEL) |
                  STM32_DMA_CR_PL(STM32_ADC_ADC3_DMA_PRIORITY) |
                  STM32_DMA_CR_DIR_P2M |
                  STM32_DMA_CR_MSIZE_HWORD | STM32_DMA_CR_PSIZE_HWORD |
                  STM32_DMA_CR_MINC        | STM32_DMA_CR_TCIE        |
                  STM32_DMA_CR_DMEIE       | STM32_DMA_CR_TEIE;
#endif

  /* The shared vector is initialized on driver initialization and never
     disabled because sharing.*/
  nvicEnableVector(STM32_ADC_NUMBER, STM32_ADC_IRQ_PRIORITY);
}

/**
 * @brief   Configures and activates the ADC peripheral.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_start(ADCDriver *adcp) {

  /* If in stopped state then enables the ADC and DMA clocks.*/
  if (adcp->state == ADC_STOP) {
#if STM32_ADC_USE_ADC1
    if (&ADCD1 == adcp) {
      bool b;
      b = dmaStreamAllocate(adcp->dmastp,
                            STM32_ADC_ADC1_DMA_IRQ_PRIORITY,
                            (stm32_dmaisr_t)adc_lld_serve_rx_interrupt,
                            (void *)adcp);
      osalDbgAssert(!b, "stream already allocated");
      // dmaStreamSetPeripheral(adcp->dmastp, &ADC1->DR);
      rccEnableADC1(FALSE);
    }
#endif /* STM32_ADC_USE_ADC1 */

#if STM32_ADC_USE_ADC2
    if (&ADCD2 == adcp) {
      bool b;
      b = dmaStreamAllocate(adcp->dmastp,
                            STM32_ADC_ADC2_DMA_IRQ_PRIORITY,
                            (stm32_dmaisr_t)adc_lld_serve_rx_interrupt,
                            (void *)adcp);
      osalDbgAssert(!b, "stream already allocated");
      // dmaStreamSetPeripheral(adcp->dmastp, &ADC2->DR);
      rccEnableADC2(FALSE);
    }
#endif /* STM32_ADC_USE_ADC2 */

#if STM32_ADC_USE_ADC3
    if (&ADCD3 == adcp) {
      bool b;
      b = dmaStreamAllocate(adcp->dmastp,
                            STM32_ADC_ADC3_DMA_IRQ_PRIORITY,
                            (stm32_dmaisr_t)adc_lld_serve_rx_interrupt,
                            (void *)adcp);
      osalDbgAssert(!b, "stream already allocated");
      // dmaStreamSetPeripheral(adcp->dmastp, &ADC3->DR);
      rccEnableADC3(FALSE);
    }
#endif /* STM32_ADC_USE_ADC3 */

  /* Setting DMA peripheral-side pointer.*/
#if STM32_ADC_DUAL_MODE
  rccEnableADC2(FALSE);
  // dmaStreamSetPeripheral(adcp->dmastp, &adcp->adcc->CDR);
  dmaStreamSetPeripheral(adcp->dmastp, &adcp->adcc->CDR);
#else
  dmaStreamSetPeripheral(adcp->dmastp, &adcp->adcm->DR);
#endif

    /* This is a common register but apparently it requires that at least one
       of the ADCs is clocked in order to allow writing, see bug 3575297.*/
#if STM32_ADC_DUAL_MODE
    ADC->CCR = (ADC->CCR & (ADC_CCR_TSVREFE | ADC_CCR_VBATE)) |
               (STM32_ADC_ADCPRE << 16) |
                ADC_CCR_MULTI_0 | ADC_CCR_MULTI_1 | ADC_CCR_MULTI_2 |
                ADC_CCR_DMA_1 | ADC_CCR_DDS;
#else
    ADC->CCR = (ADC->CCR & (ADC_CCR_TSVREFE | ADC_CCR_VBATE)) |
               (STM32_ADC_ADCPRE << 16);
#endif
    /* ADC initial setup, starting the analog part here in order to reduce
       the latency when starting a conversion.*/

#if STM32_ADC_DUAL_MODE
    adcp->adcm->CR1 = 0;
    adcp->adcm->CR2 = 0;

    adcp->adcs->CR1 = 0;
    adcp->adcs->CR2 = 0;

    adcp->adcm->CR2 = ADC_CR2_ADON;
    adcp->adcs->CR2 = ADC_CR2_ADON;
#else
    adcp->adcm->CR1 = 0;
    adcp->adcm->CR2 = 0;
    adcp->adcm->CR2 = ADC_CR2_ADON;
#endif
  }
}

/**
 * @brief   Deactivates the ADC peripheral.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_stop(ADCDriver *adcp) {

  /* If in ready state then disables the ADC clock.*/
  if (adcp->state == ADC_READY) {
    dmaStreamRelease(adcp->dmastp);
    adcp->adcm->CR1 = 0;
    adcp->adcm->CR2 = 0;

#if STM32_ADC_DUAL_MODE
    adcp->adcm->CR1 = 0;
    adcp->adcm->CR2 = 0;
#endif

#if STM32_ADC_USE_ADC1
    if (&ADCD1 == adcp)
      rccDisableADC1(FALSE);
#if STM32_ADC_DUAL_MODE
    if(&ADCD1 == adcp)
      rccDisableADC2(FALSE);
#endif /*!STM32_ADC_DUAL_MODE*/
#endif /*!STM32_ADC_USE_ADC1*/

#if STM32_ADC_USE_ADC2
    if (&ADCD2 == adcp)
      rccDisableADC2(FALSE);
#endif

#if STM32_ADC_USE_ADC3
    if (&ADCD3 == adcp)
      rccDisableADC3(FALSE);
#endif


  }
}

/**
 * @brief   Starts an ADC conversion.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
 //TODO: Edit this function for DUAL_MODE
void adc_lld_start_conversion(ADCDriver *adcp) {
  uint32_t mode;
  uint32_t cr2;
  const ADCConversionGroup *grpp = adcp->grpp;

  osalDbgAssert(!STM32_ADC_DUAL_MODE || ((grpp->num_channels & 1) == 0),
                "odd number of channels in dual mode");
  /* DMA setup.*/
  mode = adcp->dmamode;
  if (grpp->circular) {
    mode |= STM32_DMA_CR_CIRC;
    if (adcp->depth > 1) {
      /* If circular buffer depth > 1, then the half transfer interrupt
         is enabled in order to allow streaming processing.*/
      mode |= STM32_DMA_CR_HTIE;
    }
  }
  dmaStreamSetMemory0(adcp->dmastp, adcp->samples);
#if STM32_ADC_DUAL_MODE
  dmaStreamSetTransactionSize(adcp->dmastp, (uint32_t)grpp->num_channels *
                                            (uint32_t)adcp->depth);
#else
  dmaStreamSetTransactionSize(adcp->dmastp, (uint32_t)grpp->num_channels *
                                            (uint32_t)adcp->depth);
#endif
  dmaStreamSetMode(adcp->dmastp, mode);
  dmaStreamEnable(adcp->dmastp);

  /* ADC setup.*/
#if STM32_ADC_DUAL_MODE
  adcp->adcm->SR    = 0;
  adcp->adcm->SMPR1 = grpp->smpr1;
  adcp->adcm->SMPR2 = grpp->smpr2;
  adcp->adcm->SQR1  = grpp->sqr1;
  adcp->adcm->SQR2  = grpp->sqr2;
  adcp->adcm->SQR3  = grpp->sqr3;
  adcp->adcs->SR    = 0;
  adcp->adcs->SMPR1 = grpp->smpr1;
  adcp->adcs->SMPR2 = grpp->smpr2;
  adcp->adcs->SQR1  = grpp->sqr1;
  adcp->adcs->SQR2  = grpp->sqr2;
  adcp->adcs->SQR3  = grpp->sqr3;

  /* ADC configuration and start.*/
  adcp->adcm->CR1   = grpp->cr1 | ADC_CR1_OVRIE | ADC_CR1_SCAN;
  adcp->adcs->CR1   = grpp->cr1 | ADC_CR1_OVRIE | ADC_CR1_SCAN;

  /* Enforcing the mandatory bits in CR2.*/
  cr2 = grpp->cr2 | ADC_CR2_DMA | ADC_CR2_DDS | ADC_CR2_ADON;

  /* The start method is different dependign if HW or SW triggered, the
     start is performed using the method specified in the CR2 configuration.*/
  if ((cr2 & ADC_CR2_SWSTART) != 0) {
    /* Initializing CR2 while keeping ADC_CR2_SWSTART at zero.*/
    adcp->adcm->CR2 = (cr2 | ADC_CR2_CONT) & ~ADC_CR2_SWSTART;
    adcp->adcs->CR2 = (cr2 | ADC_CR2_CONT) & ~ADC_CR2_SWSTART;

    /* Finally enabling ADC_CR2_SWSTART.*/
    adcp->adcm->CR2 = (cr2 | ADC_CR2_CONT);
    adcp->adcs->CR2 = (cr2 | ADC_CR2_CONT);
  }
  else{
    adcp->adcm->CR2 = cr2;
    adcp->adcs->CR2 = cr2;
  }
#else /* !STM32_ADC_DUAL_MODE */
  adcp->adcm->SR    = 0;
  adcp->adcm->SMPR1 = grpp->smpr1;
  adcp->adcm->SMPR2 = grpp->smpr2;
  adcp->adcm->SQR1  = grpp->sqr1;
  adcp->adcm->SQR2  = grpp->sqr2;
  adcp->adcm->SQR3  = grpp->sqr3;

  /* ADC configuration and start.*/
  adcp->adcm->CR1   = grpp->cr1 | ADC_CR1_OVRIE | ADC_CR1_SCAN;

  /* Enforcing the mandatory bits in CR2.*/
  cr2 = grpp->cr2 | ADC_CR2_DMA | ADC_CR2_DDS | ADC_CR2_ADON;

  /* The start method is different dependign if HW or SW triggered, the
     start is performed using the method specified in the CR2 configuration.*/
  if ((cr2 & ADC_CR2_SWSTART) != 0) {
    /* Initializing CR2 while keeping ADC_CR2_SWSTART at zero.*/
    adcp->adcm->CR2 = (cr2 | ADC_CR2_CONT) & ~ADC_CR2_SWSTART;

    /* Finally enabling ADC_CR2_SWSTART.*/
    adcp->adcm->CR2 = (cr2 | ADC_CR2_CONT);
  }
  else
    adcp->adcm->CR2 = cr2;
#endif
}

/**
 * @brief   Stops an ongoing conversion.
 *
 * @param[in] adcp      pointer to the @p ADCDriver object
 *
 * @notapi
 */
void adc_lld_stop_conversion(ADCDriver *adcp) {

  dmaStreamDisable(adcp->dmastp);
  adcp->adcm->CR1 = 0;
  adcp->adcm->CR2 = 0;
  adcp->adcm->CR2 = ADC_CR2_ADON;
#if STM32_ADC_DUAL_MODE
  adcp->adcs->CR1 = 0;
  adcp->adcs->CR2 = 0;
  adcp->adcs->CR2 = ADC_CR2_ADON;
#endif

}

/**
 * @brief   Enables the TSVREFE bit.
 * @details The TSVREFE bit is required in order to sample the internal
 *          temperature sensor and internal reference voltage.
 * @note    This is an STM32-only functionality.
 */
void adcSTM32EnableTSVREFE(void) {

  ADC->CCR |= ADC_CCR_TSVREFE;
}

/**
 * @brief   Disables the TSVREFE bit.
 * @details The TSVREFE bit is required in order to sample the internal
 *          temperature sensor and internal reference voltage.
 * @note    This is an STM32-only functionality.
 */
void adcSTM32DisableTSVREFE(void) {

  ADC->CCR &= ~ADC_CCR_TSVREFE;
}

/**
 * @brief   Enables the VBATE bit.
 * @details The VBATE bit is required in order to sample the VBAT channel.
 * @note    This is an STM32-only functionality.
 * @note    This function is meant to be called after @p adcStart().
 */
void adcSTM32EnableVBATE(void) {

  ADC->CCR |= ADC_CCR_VBATE;
}

/**
 * @brief   Disables the VBATE bit.
 * @details The VBATE bit is required in order to sample the VBAT channel.
 * @note    This is an STM32-only functionality.
 * @note    This function is meant to be called after @p adcStart().
 */
void adcSTM32DisableVBATE(void) {

  ADC->CCR &= ~ADC_CCR_VBATE;
}

#endif /* HAL_USE_ADC */

/** @} */
