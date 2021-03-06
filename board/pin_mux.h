/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_

/***********************************************************************************************************************
 * Definitions
 **********************************************************************************************************************/

/*! @brief Direction type  */
typedef enum _pin_mux_direction
{
    kPIN_MUX_DirectionInput = 0U,        /* Input direction */
    kPIN_MUX_DirectionOutput = 1U,       /* Output direction */
    kPIN_MUX_DirectionInputOrOutput = 2U /* Input or output direction */
} pin_mux_direction_t;

/*!
 * @addtogroup pin_mux
 * @{
 */

/***********************************************************************************************************************
 * API
 **********************************************************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

/*!
 * @brief
 * Selects function mode (on-chip pull-up/pull-down resistor control).
 * : Pull-down.
 * Pull-down resistor enabled.
 */
#define PIO0_21_MODE_PULL_DOWN 0x01u
/*!
 * @brief Select peripheral clock divider for input filter sampling clock. Value 0x7 is reserved.: IOCONCLKDIV2 */
#define PIO0_25_CLK_DIV_0b010 0x02u

/*! @name PIO0_20 (number 29), P3[20]/J5[6]/P0_20-ADC6
  @{ */
#define BOARD_INITPINS_RX_PERIPHERAL USART0               /*!<@brief Device name: USART0 */
#define BOARD_INITPINS_RX_SIGNAL RXD                      /*!<@brief USART0 signal: RXD */
#define BOARD_INITPINS_RX_PORT 0U                         /*!<@brief PORT device name: 0U */
#define BOARD_INITPINS_RX_PIN 20U                         /*!<@brief 0U pin index: 20 */
#define BOARD_INITPINS_RX_PIN_NAME PIO0_20                /*!<@brief Pin name */
#define BOARD_INITPINS_RX_LABEL "P3[20]/J5[6]/P0_20-ADC6" /*!<@brief Label */
#define BOARD_INITPINS_RX_NAME "RX"                       /*!<@brief Identifier name */
                                                          /* @} */

/*! @name PIO0_22 (number 27), P3[18]/J5[4]/P0_22-ADC4
  @{ */
#define BOARD_INITPINS_TX_PERIPHERAL USART0               /*!<@brief Device name: USART0 */
#define BOARD_INITPINS_TX_SIGNAL TXD                      /*!<@brief USART0 signal: TXD */
#define BOARD_INITPINS_TX_PORT 0U                         /*!<@brief PORT device name: 0U */
#define BOARD_INITPINS_TX_PIN 22U                         /*!<@brief 0U pin index: 22 */
#define BOARD_INITPINS_TX_PIN_NAME PIO0_22                /*!<@brief Pin name */
#define BOARD_INITPINS_TX_LABEL "P3[18]/J5[4]/P0_22-ADC4" /*!<@brief Label */
#define BOARD_INITPINS_TX_NAME "TX"                       /*!<@brief Identifier name */
                                                          /* @} */

/*! @name PIO0_21 (number 28), P3[19]/J5[5]/P0_21-ADC5
  @{ */
#define BOARD_INITPINS_DIR_PERIPHERAL GPIO                    /*!<@brief Device name: GPIO */
#define BOARD_INITPINS_DIR_SIGNAL PIO0                        /*!<@brief GPIO signal: PIO0 */
#define BOARD_INITPINS_DIR_GPIO GPIO                          /*!<@brief GPIO device name: GPIO */
#define BOARD_INITPINS_DIR_GPIO_PIN 21U                       /*!<@brief PIO0 pin index: 21 */
#define BOARD_INITPINS_DIR_PORT 0U                            /*!<@brief PORT device name: 0U */
#define BOARD_INITPINS_DIR_PIN 21U                            /*!<@brief 0U pin index: 21 */
#define BOARD_INITPINS_DIR_CHANNEL 21                         /*!<@brief GPIO PIO0 channel: 21 */
#define BOARD_INITPINS_DIR_PIN_NAME PIO0_21                   /*!<@brief Pin name */
#define BOARD_INITPINS_DIR_LABEL "P3[19]/J5[5]/P0_21-ADC5"    /*!<@brief Label */
#define BOARD_INITPINS_DIR_NAME "DIR"                         /*!<@brief Identifier name */
#define BOARD_INITPINS_DIR_DIRECTION kPIN_MUX_DirectionOutput /*!<@brief Direction */
                                                              /* @} */

/*! @name PIO0_25 (number 13), P3[6]/J1[6]/P4[5]/P0_25-MISO
  @{ */
#define BOARD_INITPINS_SCK_PERIPHERAL SPI0                      /*!<@brief Device name: SPI0 */
#define BOARD_INITPINS_SCK_SIGNAL SCK                           /*!<@brief SPI0 signal: SCK */
#define BOARD_INITPINS_SCK_PORT 0U                              /*!<@brief PORT device name: 0U */
#define BOARD_INITPINS_SCK_PIN 25U                              /*!<@brief 0U pin index: 25 */
#define BOARD_INITPINS_SCK_PIN_NAME PIO0_25                     /*!<@brief Pin name */
#define BOARD_INITPINS_SCK_LABEL "P3[6]/J1[6]/P4[5]/P0_25-MISO" /*!<@brief Label */
#define BOARD_INITPINS_SCK_NAME "SCK"                           /*!<@brief Identifier name */
#define BOARD_INITPINS_SCK_DIRECTION kPIN_MUX_DirectionOutput   /*!<@brief Direction */
                                                                /* @} */

/*! @name PIO0_26 (number 12), P3[5]/J1[7]/P4[3]/P0_26-MOSI
  @{ */
#define BOARD_INITPINS_MOSI_PERIPHERAL SPI0                      /*!<@brief Device name: SPI0 */
#define BOARD_INITPINS_MOSI_SIGNAL MOSI                          /*!<@brief SPI0 signal: MOSI */
#define BOARD_INITPINS_MOSI_PORT 0U                              /*!<@brief PORT device name: 0U */
#define BOARD_INITPINS_MOSI_PIN 26U                              /*!<@brief 0U pin index: 26 */
#define BOARD_INITPINS_MOSI_PIN_NAME PIO0_26                     /*!<@brief Pin name */
#define BOARD_INITPINS_MOSI_LABEL "P3[5]/J1[7]/P4[3]/P0_26-MOSI" /*!<@brief Label */
#define BOARD_INITPINS_MOSI_NAME "MOSI"                          /*!<@brief Identifier name */
#define BOARD_INITPINS_MOSI_DIRECTION kPIN_MUX_DirectionOutput   /*!<@brief Direction */
                                                                 /* @} */

/*! @name PIO0_27 (number 11), D1[3]/P3[42]/J1[9]/P0_27-BLUE
  @{ */
#define BOARD_INITPINS_MISO_PERIPHERAL SPI0                       /*!<@brief Device name: SPI0 */
#define BOARD_INITPINS_MISO_SIGNAL MISO                           /*!<@brief SPI0 signal: MISO */
#define BOARD_INITPINS_MISO_PORT 0U                               /*!<@brief PORT device name: 0U */
#define BOARD_INITPINS_MISO_PIN 27U                               /*!<@brief 0U pin index: 27 */
#define BOARD_INITPINS_MISO_PIN_NAME PIO0_27                      /*!<@brief Pin name */
#define BOARD_INITPINS_MISO_LABEL "D1[3]/P3[42]/J1[9]/P0_27-BLUE" /*!<@brief Label */
#define BOARD_INITPINS_MISO_NAME "MISO"                           /*!<@brief Identifier name */
#define BOARD_INITPINS_MISO_DIRECTION kPIN_MUX_DirectionInput     /*!<@brief Direction */
                                                                  /* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitPins(void); /* Function assigned for the Cortex-M0P */

#define IOCON_PIO_CLKDIV0 0x00u      /*!<@brief IOCONCLKDIV0 */
#define IOCON_PIO_HYS_DI 0x00u       /*!<@brief Disable hysteresis */
#define IOCON_PIO_INV_DI 0x00u       /*!<@brief Input not invert */
#define IOCON_PIO_MODE_PULLUP 0x10u  /*!<@brief Selects pull-up function */
#define IOCON_PIO_OD_DI 0x00u        /*!<@brief Disables Open-drain function */
#define IOCON_PIO_SMODE_BYPASS 0x00u /*!<@brief Bypass input filter */

/*! @name PIO0_27 (number 11), D1[3]/P3[42]/J1[9]/P0_27-BLUE
  @{ */
#define BOARD_INITLEDSPINS_MISO_PERIPHERAL GPIO                       /*!<@brief Device name: GPIO */
#define BOARD_INITLEDSPINS_MISO_SIGNAL PIO0                           /*!<@brief GPIO signal: PIO0 */
#define BOARD_INITLEDSPINS_MISO_GPIO GPIO                             /*!<@brief GPIO device name: GPIO */
#define BOARD_INITLEDSPINS_MISO_GPIO_PIN 27U                          /*!<@brief PIO0 pin index: 27 */
#define BOARD_INITLEDSPINS_MISO_PORT 0U                               /*!<@brief PORT device name: 0U */
#define BOARD_INITLEDSPINS_MISO_PIN 27U                               /*!<@brief 0U pin index: 27 */
#define BOARD_INITLEDSPINS_MISO_CHANNEL 27                            /*!<@brief GPIO PIO0 channel: 27 */
#define BOARD_INITLEDSPINS_MISO_PIN_NAME PIO0_27                      /*!<@brief Pin name */
#define BOARD_INITLEDSPINS_MISO_LABEL "D1[3]/P3[42]/J1[9]/P0_27-BLUE" /*!<@brief Label */
#define BOARD_INITLEDSPINS_MISO_NAME "MISO"                           /*!<@brief Identifier name */
#define BOARD_INITLEDSPINS_MISO_DIRECTION kPIN_MUX_DirectionOutput    /*!<@brief Direction */
                                                                      /* @} */

/*! @name PIO0_12 (number 2), SW2/D1[1]/P3[45]/J2[5]/P0_12-RED-ISP
  @{ */
/*!
 * @brief Device name: GPIO */
#define BOARD_INITLEDSPINS_LED_RED_PERIPHERAL GPIO
/*!
 * @brief GPIO signal: PIO0 */
#define BOARD_INITLEDSPINS_LED_RED_SIGNAL PIO0
/*!
 * @brief GPIO device name: GPIO */
#define BOARD_INITLEDSPINS_LED_RED_GPIO GPIO
/*!
 * @brief PIO0 pin index: 12 */
#define BOARD_INITLEDSPINS_LED_RED_GPIO_PIN 12U
/*!
 * @brief PORT device name: 0U */
#define BOARD_INITLEDSPINS_LED_RED_PORT 0U
/*!
 * @brief 0U pin index: 12 */
#define BOARD_INITLEDSPINS_LED_RED_PIN 12U
/*!
 * @brief GPIO PIO0 channel: 12 */
#define BOARD_INITLEDSPINS_LED_RED_CHANNEL 12
/*!
 * @brief Pin name */
#define BOARD_INITLEDSPINS_LED_RED_PIN_NAME PIO0_12
/*!
 * @brief Label */
#define BOARD_INITLEDSPINS_LED_RED_LABEL "SW2/D1[1]/P3[45]/J2[5]/P0_12-RED-ISP"
/*!
 * @brief Identifier name */
#define BOARD_INITLEDSPINS_LED_RED_NAME "LED_RED"
/*!
 * @brief Direction */
#define BOARD_INITLEDSPINS_LED_RED_DIRECTION kPIN_MUX_DirectionOutput
/* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitLEDsPins(void); /* Function assigned for the Cortex-M0P */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitDEBUG_UARTPins(void); /* Function assigned for the Cortex-M0P */

#define IOCON_PIO_CLKDIV0 0x00u      /*!<@brief IOCONCLKDIV0 */
#define IOCON_PIO_HYS_DI 0x00u       /*!<@brief Disable hysteresis */
#define IOCON_PIO_INV_DI 0x00u       /*!<@brief Input not invert */
#define IOCON_PIO_MODE_PULLUP 0x10u  /*!<@brief Selects pull-up function */
#define IOCON_PIO_OD_DI 0x00u        /*!<@brief Disables Open-drain function */
#define IOCON_PIO_SMODE_BYPASS 0x00u /*!<@brief Bypass input filter */

/*! @name SWCLK (number 6), P5[4]/U2[16]/TARGET_SWCLK
  @{ */
/*!
 * @brief Device name: SWD */
#define BOARD_INITSWD_DEBUGPINS_DEBUG_SWD_SWDCLK_PERIPHERAL SWD
/*!
 * @brief SWD signal: SWCLK */
#define BOARD_INITSWD_DEBUGPINS_DEBUG_SWD_SWDCLK_SIGNAL SWCLK
/*!
 * @brief Pin name */
#define BOARD_INITSWD_DEBUGPINS_DEBUG_SWD_SWDCLK_PIN_NAME SWCLK
/*!
 * @brief Label */
#define BOARD_INITSWD_DEBUGPINS_DEBUG_SWD_SWDCLK_LABEL "P5[4]/U2[16]/TARGET_SWCLK"
/*!
 * @brief Identifier name */
#define BOARD_INITSWD_DEBUGPINS_DEBUG_SWD_SWDCLK_NAME "DEBUG_SWD_SWDCLK"
/* @} */

/*! @name SWDIO (number 7), P5[2]/U2[17]/TARGET_SWDIO
  @{ */
/*!
 * @brief Device name: SWD */
#define BOARD_INITSWD_DEBUGPINS_DEBUG_SWD_SWDIO_PERIPHERAL SWD
/*!
 * @brief SWD signal: SWDIO */
#define BOARD_INITSWD_DEBUGPINS_DEBUG_SWD_SWDIO_SIGNAL SWDIO
/*!
 * @brief Pin name */
#define BOARD_INITSWD_DEBUGPINS_DEBUG_SWD_SWDIO_PIN_NAME SWDIO
/*!
 * @brief Label */
#define BOARD_INITSWD_DEBUGPINS_DEBUG_SWD_SWDIO_LABEL "P5[2]/U2[17]/TARGET_SWDIO"
/*!
 * @brief Identifier name */
#define BOARD_INITSWD_DEBUGPINS_DEBUG_SWD_SWDIO_NAME "DEBUG_SWD_SWDIO"
/* @} */

/*! @name RESETN (number 3), J4[3]/P3[4]/U2[3]/P5[10]/SW3[1]/TARGET_nRESET-P0_5
  @{ */
/*!
 * @brief Device name: SYSCON */
#define BOARD_INITSWD_DEBUGPINS_DEBUG_SWD_RESETN_PERIPHERAL SYSCON
/*!
 * @brief SYSCON signal: RESETN */
#define BOARD_INITSWD_DEBUGPINS_DEBUG_SWD_RESETN_SIGNAL RESETN
/*!
 * @brief Pin name */
#define BOARD_INITSWD_DEBUGPINS_DEBUG_SWD_RESETN_PIN_NAME RESETN
/*!
 * @brief Label */
#define BOARD_INITSWD_DEBUGPINS_DEBUG_SWD_RESETN_LABEL "J4[3]/P3[4]/U2[3]/P5[10]/SW3[1]/TARGET_nRESET-P0_5"
/*!
 * @brief Identifier name */
#define BOARD_INITSWD_DEBUGPINS_DEBUG_SWD_RESETN_NAME "DEBUG_SWD_RESETN"
/* @} */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitSWD_DEBUGPins(void); /* Function assigned for the Cortex-M0P */

/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitBUTTONsPins(void); /* Function assigned for the Cortex-M0P */

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
