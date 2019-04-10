#include "mbed.h"
 
InterruptIn button(USER_BUTTON);
Serial pc(USBTX, USBRX); // tx, rx
 
volatile int8_t Button_State;
 
void Button_Interrupt() 
{
    wait_ms(50);    //Delay for switch debounce
    pc.printf("I am in Interrupt!\n");
    Button_State = 1;
}
 
int main() 
{
    //Attach the address of the Button Interrupt function to the falling edge
    button.fall(&Button_Interrupt); 
    button.enable_irq();
    
    //Initialize button state
    Button_State = 0;
    
    //Loop Forever
    while(1)
    {
        if(Button_State == 1)
        {
            pc.printf("Hmmmmm...\n");
            break;
        }
    }
}
