/**
 * EUSART Generated Driver API Header File
 * 
 * @file eusart.c
 * 
 * @ingroup eusart
 * 
 * @brief This is the generated driver implementation file for the EUSART driver using CCL
 *
 * @version EUSART Driver Version 3.0.0
*/

/*
? [2023] Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip 
    software and any derivatives exclusively with Microchip products. 
    You are responsible for complying with 3rd party license terms  
    applicable to your use of 3rd party software (including open source  
    software) that may accompany Microchip software. SOFTWARE IS ?AS IS.? 
    NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS 
    SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,  
    MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT 
    WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
    KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF 
    MICROCHIP HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE 
    FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP?S 
    TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL NOT 
    EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR 
    THIS SOFTWARE.
*/

/**
  Section: Included Files
*/
#include "../eusart.h"

/**
  Section: Macro Declarations
*/

#define EUSART_TX_BUFFER_SIZE (32) //buffer size should be 2^n
#define EUSART_TX_BUFFER_MASK (EUSART_TX_BUFFER_SIZE - 1) 

#define EUSART_RX_BUFFER_SIZE (32) //buffer size should be 2^n
#define EUSART_RX_BUFFER_MASK (EUSART_RX_BUFFER_SIZE - 1)

/**
  Section: Driver Interface
 */

const uart_drv_interface_t UART1 = {
    .Initialize = &EUSART_Initialize,
    .Deinitialize = &EUSART_Deinitialize,
    .Read = &EUSART_Read,
    .Write = &EUSART_Write,
    .IsRxReady = &EUSART_IsRxReady,
    .IsTxReady = &EUSART_IsTxReady,
    .IsTxDone = &EUSART_IsTxDone,
    .TransmitEnable = &EUSART_TransmitEnable,
    .TransmitDisable = &EUSART_TransmitDisable,
    .AutoBaudSet = &EUSART_AutoBaudSet,
    .AutoBaudQuery = &EUSART_AutoBaudQuery,
    .BRGCountSet = NULL,
    .BRGCountGet = NULL,
    .BaudRateSet = NULL,
    .BaudRateGet = NULL,
    .AutoBaudEventEnableGet = NULL,
    .ErrorGet = &EUSART_ErrorGet,
    .TxCompleteCallbackRegister = &EUSART_TxCompleteCallbackRegister,
    .RxCompleteCallbackRegister = &EUSART_RxCompleteCallbackRegister,
    .TxCollisionCallbackRegister = NULL,
    .FramingErrorCallbackRegister = &EUSART_FramingErrorCallbackRegister,
    .OverrunErrorCallbackRegister = &EUSART_OverrunErrorCallbackRegister,
    .ParityErrorCallbackRegister = NULL,
    .EventCallbackRegister = NULL,
};

/**
  Section: EUSART variables
*/
static volatile uint8_t eusartTxHead = 0;
static volatile uint8_t eusartTxTail = 0;
static volatile uint8_t eusartTxBuffer[EUSART_TX_BUFFER_SIZE];
volatile uint8_t eusartTxBufferRemaining;

static volatile uint8_t eusartRxHead = 0;
static volatile uint8_t eusartRxTail = 0;
static volatile uint8_t eusartRxBuffer[EUSART_RX_BUFFER_SIZE];
static volatile eusart_status_t eusartRxStatusBuffer[EUSART_RX_BUFFER_SIZE];
volatile uint8_t eusartRxCount;

volatile eusart_status_t eusartRxLastError;

/**
  Section: EUSART APIs
*/

void (*EUSART_TxInterruptHandler)(void);
void (*EUSART_RxInterruptHandler)(void);
void (*EUSART_FramingErrorHandler)(void);
void (*EUSART_OverrunErrorHandler)(void);

static void EUSART_DefaultFramingErrorCallback(void);
static void EUSART_DefaultOverrunErrorCallback(void);

void EUSART_TransmitISR (void);
void EUSART_ReceiveISR(void);


/**
  Section: EUSART  APIs
*/

void EUSART_Initialize(void)
{
    PIE1bits.RCIE = 0;   
    EUSART_RxCompleteCallbackRegister(EUSART_ReceiveISR);  

    PIE1bits.TXIE = 0; 
    EUSART_TxCompleteCallbackRegister(EUSART_TransmitISR);

    // Set the EUSART module to the options selected in the user interface.

    //ABDEN disabled; WUE disabled; BRG16 16bit_generator; SCKP Non-Inverted; 
    BAUD1CON = 0x48; 
    //ADDEN disabled; CREN enabled; SREN disabled; RX9 8-bit; SPEN enabled; 
    RC1STA = 0x90; 
    //TX9D 0x0; BRGH hi_speed; SENDB sync_break_complete; SYNC asynchronous; TXEN enabled; TX9 8-bit; CSRC client; 
    TX1STA = 0x26; 
    //SPBRGL 25; 
    SP1BRGL = 0x19; 
    //SPBRGH 0; 
    SP1BRGH = 0x0; 

    EUSART_FramingErrorCallbackRegister(EUSART_DefaultFramingErrorCallback);
    EUSART_OverrunErrorCallbackRegister(EUSART_DefaultOverrunErrorCallback);
    eusartRxLastError.status = 0;  

    eusartTxHead = 0;
    eusartTxTail = 0;
    eusartTxBufferRemaining = sizeof(eusartTxBuffer);

    eusartRxHead = 0;
    eusartRxTail = 0;
    eusartRxCount = 0;

    PIE1bits.RCIE = 1; 
}

void EUSART_Deinitialize(void)
{
    PIE1bits.RCIE = 0;    
    PIE1bits.TXIE = 0; 
    BAUD1CON = 0x00;
    RC1STA = 0x00;
    TX1STA = 0x00;
    SP1BRGL = 0x00;
    SP1BRGH = 0x00;
}

inline void EUSART_Enable(void)
{
    RC1STAbits.SPEN = 1;

}

inline void EUSART_Disable(void)
{
    RC1STAbits.SPEN = 0;
}


inline void EUSART_TransmitEnable(void)
{
    TX1STAbits.TXEN = 1;
}

inline void EUSART_TransmitDisable(void)
{
    TX1STAbits.TXEN = 0;
}

inline void EUSART_ReceiveEnable(void)
{
    RC1STAbits.CREN = 1;
}

inline void EUSART_ReceiveDisable(void)
{
    RC1STAbits.CREN = 0;
}

inline void EUSART_SendBreakControlEnable(void)
{
    TX1STAbits.SENDB = 1;
}

inline void EUSART_SendBreakControlDisable(void)
{
    TX1STAbits.SENDB = 0;
}

inline void EUSART_AutoBaudSet(bool enable)
{
    if(enable)
    {
        BAUD1CONbits.ABDEN = 1;
    }
    else
    {
       BAUD1CONbits.ABDEN = 0; 
    }
}

inline bool EUSART_AutoBaudQuery(void)
{
return (bool)(!BAUD1CONbits.ABDEN);
}

inline bool EUSART_IsAutoBaudDetectOverflow(void)
{
    return (bool)BAUD1CONbits.ABDOVF; 
}

inline void EUSART_AutoBaudDetectOverflowReset(void)
{
    BAUD1CONbits.ABDOVF = 0; 
}

inline void EUSART_TransmitInterruptEnable(void)
{
    PIE1bits.TXIE = 1;

}

inline void EUSART_TransmitInterruptDisable(void)
{ 
    PIE1bits.TXIE = 0; 
}

inline void EUSART_ReceiveInterruptEnable(void)
{
    PIE1bits.RCIE = 1;
}
inline void EUSART_ReceiveInterruptDisable(void)
{
    PIE1bits.RCIE = 0; 
}

bool EUSART_IsRxReady(void)
{
    return (eusartRxCount ? true : false);
}

bool EUSART_IsTxReady(void)
{
    return (eusartTxBufferRemaining ? true : false);
}

bool EUSART_IsTxDone(void)
{
    return TX1STAbits.TRMT;
}

size_t EUSART_ErrorGet(void)
{
    eusartRxLastError.status = eusartRxStatusBuffer[(eusartRxTail + 1) & EUSART_RX_BUFFER_MASK].status;
    return eusartRxLastError.status;
}

uint8_t EUSART_Read(void)
{
    uint8_t readValue  = 0;
    uint8_t tempRxTail;
    
    readValue = eusartRxBuffer[eusartRxTail];

    tempRxTail = (eusartRxTail + 1) & EUSART_RX_BUFFER_MASK; // Buffer size of RX should be in the 2^n
    
    eusartRxTail = tempRxTail;

    eusartRxLastError = eusartRxStatusBuffer[eusartRxTail];
    

    PIE1bits.RCIE = 0; 
    if(eusartRxCount != 0)
    {
        eusartRxCount--;
    }
    PIE1bits.RCIE = 1;
    return readValue;
}

void EUSART_ReceiveISR(void)
{
    uint8_t regValue;
	uint8_t tempRxHead;

    // use this default receive interrupt handler code
    eusartRxStatusBuffer[eusartRxHead].status = 0;

    if(RC1STAbits.OERR)
    {
        eusartRxStatusBuffer[eusartRxHead].oerr = 1;
        if(NULL != EUSART_OverrunErrorHandler)
        {
            EUSART_OverrunErrorHandler();
        }   
    }   
    if(RC1STAbits.FERR)
    {
        eusartRxStatusBuffer[eusartRxHead].ferr = 1;
        if(NULL != EUSART_FramingErrorHandler)
        {
            EUSART_FramingErrorHandler();
        }   
    } 
    
    regValue = RC1REG;
    
    tempRxHead = (eusartRxHead + 1) & EUSART_RX_BUFFER_MASK;// Buffer size of RX should be in the 2^n
    if (tempRxHead == eusartRxTail) 
    {
		// ERROR! Receive buffer overflow 
	} 
    else
    {
        eusartRxBuffer[eusartRxHead] = regValue;
		eusartRxHead = tempRxHead;
		eusartRxCount++;
	}   
}

void EUSART_Write(uint8_t txData)
{
    uint8_t tempTxHead;
    
    if(0 == PIE1bits.TXIE)
    {
        TX1REG = txData;
    }
    else if(eusartTxBufferRemaining) // check if at least one byte place is available in TX buffer
    {
       eusartTxBuffer[eusartTxHead] = txData;
       tempTxHead = (eusartTxHead + 1) & EUSART_TX_BUFFER_MASK;
       
       eusartTxHead = tempTxHead;
       PIE1bits.TXIE = 0; //Critical value decrement
       eusartTxBufferRemaining--; // one less byte remaining in TX buffer
    }
    else
    {
        //overflow condition; eusartTxBufferRemaining is 0 means TX buffer is full
    }
    PIE1bits.TXIE = 1;
}

void EUSART_TransmitISR(void)
{
    uint8_t tempTxTail;
    // use this default transmit interrupt handler code
    if(sizeof(eusartTxBuffer) > eusartTxBufferRemaining) // check if all data is transmitted
    {
       TX1REG = eusartTxBuffer[eusartTxTail];
       tempTxTail = (eusartTxTail + 1) & EUSART_TX_BUFFER_MASK;
       
       eusartTxTail = tempTxTail;
       eusartTxBufferRemaining++; // one byte sent, so 1 more byte place is available in TX buffer
    }
    else
    {
        PIE1bits.TXIE = 0;
    }
}

int getch(void)
{
    while(!(EUSART_IsRxReady()));
    return EUSART_Read();
}

void putch(char txData)
{
    while(!(EUSART_IsTxReady()));
    return EUSART_Write(txData);   
}

static void EUSART_DefaultFramingErrorCallback(void)
{
    
}

static void EUSART_DefaultOverrunErrorCallback(void)
{
    //Continuous Receive must be cleared to clear Overrun Error else Rx will not receive upcoming bytes
    RC1STAbits.CREN = 0;
    RC1STAbits.CREN = 1;
}

void EUSART_FramingErrorCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
        EUSART_FramingErrorHandler = callbackHandler;
    }
}

void EUSART_OverrunErrorCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
        EUSART_OverrunErrorHandler = callbackHandler;
    }    
}

void EUSART_RxCompleteCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
       EUSART_RxInterruptHandler = callbackHandler; 
    }   
}

void EUSART_TxCompleteCallbackRegister(void (* callbackHandler)(void))
{
    if(NULL != callbackHandler)
    {
       EUSART_TxInterruptHandler = callbackHandler;
    }   
}

