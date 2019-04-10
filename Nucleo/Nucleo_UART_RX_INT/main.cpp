#include "mbed.h"

#define NODE_1_ID   0x01
#define EGLOVE_ID   0x02
#define START_ID    0x03

Serial pc(USBTX, USBRX);
Serial uart(PA_9, PA_10);

volatile char nRF_Nodes[5];

volatile char Nex_Buffer[10];
volatile int Nex_RX_Count;
volatile int Node_Count;

volatile bool Button_1_State;
volatile bool Button_2_State;
 
void callback() 
{
    Nex_Buffer[Nex_RX_Count] = uart.getc();

    Nex_RX_Count++;

    if(Nex_RX_Count == 7)
    {
        Nex_RX_Count = 0;
        if(Nex_Buffer[2] == NODE_1_ID)
        {
            //Toggle Button 1 State
            Button_1_State ^= true;
            if(Button_1_State == 0)
            {
                pc.printf("Node 1 OFF\n");
            }
            
            else
            {
                pc.printf("Node 1 ON\n");
            }
        }
        
        else if(Nex_Buffer[2] == EGLOVE_ID)
        {
            //Toggle Button 2 State
            Button_2_State ^= true;
            if(Button_2_State == 0)
            {
                pc.printf("eGlove OFF\n");
            }
            
            else
            {
                pc.printf("eGlove ON\n");
            }
        }
        
        else if(Nex_Buffer[2] == START_ID)
        {
            if(Button_1_State != 0)
            {
                nRF_Nodes[Node_Count] = 0x01;
                Node_Count++;
            }
            
            if(Button_2_State != 0)
            {
                nRF_Nodes[Node_Count] = 0x02;
                Node_Count++;
            }
            
            pc.printf("Total Nodes:%d\n", Node_Count);
            pc.printf("Active Nodes:");
            for(int j = 0; j < Node_Count; j++)
            {
                pc.printf("%d ", nRF_Nodes[j]);
            }
            pc.printf("\n");
            Button_1_State = false;
            Button_2_State = false;
            Node_Count = 0;
        }
    }
}
 
int main() {
    uart.attach(&callback);
    
    while (1)
    {
        if(Nex_Buffer[2] == START_ID)
        {
            pc.printf("Main routine\n");
            wait(0.5);
        }
        
        wait(0.5);
    }
}
/* 
Serial pc(USBTX, USBRX);
Serial uart(PA_9, PA_10);

volatile char Nex_Buffer[50];

volatile int i;
 
void callback() {
    


    Nex_Buffer[i] = uart.getc();

    pc.printf("%c", Nex_Buffer[i]);

    i++;

    if(i == 7)
    {
        i = 0;
        pc.printf("%c", Nex_Buffer[2]);
    }

//    pc.printf("%c", uart.getc());
}
 
int main() {
    uart.attach(&callback);
    
    while (1){
        wait(0.5);
    }
}
*/