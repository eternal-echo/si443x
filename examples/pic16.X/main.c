 /*
 * MAIN Generated Driver File
 * 
 * @file main.c
 * 
 * @defgroup main MAIN
 * 
 * @brief This is the generated driver implementation file for the MAIN driver.
 *
 * @version MAIN Driver Version 1.0.0
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
#include "mcc_generated_files/system/system.h"
#include "../../inc/si443x.h"
/*
    Main application
*/
#define RX_BUFFER_SIZE 64
uint8_t rx_buffer[RX_BUFFER_SIZE] = {0};
int rx_count = 0;

int main(void)
{
    uint8_t i = 0;
    SYSTEM_Initialize();

    // If using interrupts in PIC18 High/Low Priority Mode you need to enable the Global High and Low Interrupts 
    // If using interrupts in PIC Mid-Range Compatibility Mode you need to enable the Global and Peripheral Interrupts 
    // Use the following macros to: 

    // Enable the Global Interrupts 
    INTERRUPT_GlobalInterruptEnable(); 

    // Disable the Global Interrupts 
    //INTERRUPT_GlobalInterruptDisable(); 

    // Enable the Peripheral Interrupts 
    INTERRUPT_PeripheralInterruptEnable(); 

    // Disable the Peripheral Interrupts 
    //INTERRUPT_PeripheralInterruptDisable(); 
    printf("Si443x driver example\r\n");
    si443x_t si443x;
    si443x_init(&si443x);
    si443x_start_listening(&si443x);
    si443x_read_all(&si443x);

    while(1)
    {
        i++;
        if(si443x_is_packetReceived(&si443x) == SI443X_ERR_OK)
        {
            uint8_t readData[64];
            uint8_t length;
            printf("[radio][rx] Packet received\r\n");
            length = si443x_get_packet_received(&si443x, readData);
            printf("[radio][rx] payload: %.*s\r\n", length, readData);
            printf("[radio][rx] length: %d\r\n", length);
            printf("\r\n");

            si443x_start_listening(&si443x);
        }
        if(BUTTON_GetValue() == 0)
        {
            while(BUTTON_GetValue() == 0);
            printf("Button pressed\r\n");
            const uint8_t data[] = "Hello";
            si443x_send_packet(&si443x, data, sizeof(data));
            printf("Sent: %s\r\n", data);
            si443x_start_listening(&si443x);
        }
        // if (UART1__IsRxReady())
        // {
        //     rx_count = 0;
        //     while (UART1__IsRxReady() && rx_count < RX_BUFFER_SIZE)
        //     {
        //         uint8_t data = UART1_Read();
        //         rx_buffer[rx_count] = data;
        //         rx_count++;
        //         __delay_ms(1);
        //     }
        //     rx_buffer[rx_count] = '\0';
        //     printf("[UART][rx] payload: %s\r\n", rx_buffer);
        //     printf("[UART][rx] length: %d\r\n", rx_count);
        //     si443x_send_packet(&si443x, rx_buffer, rx_count);
        // }
    }    
}