#include "Nrf24l01p.h"

#define PAYLOAD_MAX_LEN		32
#define PAYLOAD_LEN 		10
#define PAYLOAD_QUAT_LEN 	10

#define NRF_TOTAL_NODES 	6
#define NRF_ADDR_LEN 		5

/* Change it new Nodes or Sub-nodes are made. */
#define TOTAL_NODES_AND_SUBNODES 2

uint8_t BS_payload_TX[PAYLOAD_LEN] = {0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint8_t RX_buffer_1[PAYLOAD_MAX_LEN];
uint8_t RX_buffer_2[PAYLOAD_MAX_LEN];

uint8_t Serial_RX_buffer[10] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
volatile uint8_t RX_done = 0;

/* add more address for nRF_Nodes and NRF_address  if more than 6 device available.
 * make sure change value for TOTAL_NODES_AND_SUBNODES, to connect to the max number of devices.
*/

uint8_t nRF_Nodes[NRF_TOTAL_NODES] = {0x01,0x02,0x03,0x04,0x05,0x06};   // Address of the Main Nodes connected to the Base station
uint8_t NRF_address[NRF_TOTAL_NODES][NRF_ADDR_LEN] = {   				// Address of Sub Nodes connected to the Main Nodes
    												 {0x11, 0x12, 0x13, 0x14, 0x15},
    												 {0x21, 0x22, 0x23, 0x24, 0x25},
    												 {0x31, 0x32, 0x33, 0x34, 0x35},
    												 {0x41, 0x42, 0x43, 0x44, 0x45},
    												 {0x51, 0x52, 0x53, 0x54, 0x55},
    												 {0x61, 0x62, 0x63, 0x64, 0x65}
    												 };


Nrf24l01p nrf(D11, D12, D13, D4, D5); 	// Create an object for the NRF24L01Plus class
Serial pc(SERIAL_TX, SERIAL_RX);		// Serial connection

// Ticker flipper; 						// Ticker for mode transmission to other node if no packet received
InterruptIn nrf_irq(D6);				// Pin in Nrf24l01p for Interrupt
volatile uint8_t mode = 0; 					// 0 - Quaternion; 1 - Quaternion + linear acceleration
volatile uint8_t nRF_Node; 				// Node number
volatile uint8_t serialrun = 1;
volatile uint8_t serial_rx_Payload_cnt = 0;

//User button with interrupt
InterruptIn button(USER_BUTTON);

Timer t;



/************************************************************************************
** Nucleo_Init function:
** - Start-up delay
** - Initializes the I/O peripherals
*************************************************************************************/
void Nucleo_Init(void)
{
    wait(0.75);         //750ms mandatory delay for BNO055 POR
}


/*
void onSerialInput(){
	char buffer[20];
		if(pc.readable()){
			pc.gets(buffer,19); // @suppress("Method cannot be resolved")
		}

	if (serialrun == 1){
		for(uint8_t i=0; i<PAYLOAD_LEN; i++)
		{
			if (buffer[i] == 0xAA)
			{
				serial_rx_Payload_cnt++;
			}
		}
		changereturndatatype(serial_rx_Payload_cnt);
		serialrun = 0;
		serial_rx_Payload_cnt = 0;
	}else{
		// Interrupt is interrupted two times so just taking the first interrup as valid interrupt
		// and again the third as the valid interrupt
		serialrun = 1;
	}
	// waiting seems to be necessary
	// if no waiting is done then no data transfer occurs so USB unplug and plug is required of the base station
	
	wait(1);
}
*/

volatile uint8_t serialCommand = 0;

void onSerialRXDone()
{
	while (pc.readable())
	{
		serialCommand = pc.getc();
		//Serial_RX_buffer[0] = pc.getc();
	}
	RX_done = 1;
}


void onSerialTXDone(int event)
{
}

void Button_Interrupt() 
{
    wait_ms(50);    //Delay for switch debounce
    //button.disable_irq();
    if (mode == 0)
    {
    	mode = 1;
    }
    else
    {
    	mode = 0;
    }
}

/**************************************************************
**                       MAIN
***************************************************************/

int main()
{
	Nucleo_Init();
	
	pc.baud(500000);					// Set Serial baud-rate
	//pc.format(8, SerialBase::Even, 1);
	//pc.attach(&onSerialRXDone, Serial::RxIrq);
	//pc.enable_input();
	
	mode = 0;							// default: Quaternion only mode
    nRF_Node = 0; 						// Initialize the node number
    
	nrf.init(0x69, DR_1M, NRF_ADDR_LEN, 1);
	nrf.setRetries(500, 1);
	
	//nrf.openTXPipe(NRF_address[nRF_Node], PAYLOAD_QUAT_LEN, true, false);
	nrf.openDynamicTXPipe(NRF_address[nRF_Node], true, false);
	//nrf.openRXPipe(0, NRF_address[nRF_Node], PAYLOAD_QUAT_LEN, true, false);
	
	// do not use interrupts, use polling instead
    nrf.maskIRQ(true, true, true);
    
    // Disable nRF IRQ (consider removing IRQ pin from code...)
    nrf_irq.disable_irq();
	
	nrf.setModeTX();
    nrf.flushTX();
    nrf.resetIRQFlags();
	
    //nrf_irq.fall(&onDataReceived);    	// Attach the address of the nRF IRQ Interrupt function to the falling edge
    
    
    //Attach the address of the Button Interrupt function to the falling edge
    button.fall(&Button_Interrupt); 
    button.enable_irq();
    
    
    bool rx, txDone, maxTry;
    uint8_t numRetries = 0;
    
    uint8_t rxLen = 0;
    
    uint8_t* currentBuffer = RX_buffer_1;
    
    event_callback_t serialCallbackTX = &onSerialTXDone;
    //event_callback_t serialCallbackRX = &onSerialRXDone;
    
    // prevent mbed from going into deep sleep
    DeepSleepLock lock;
    // t.start();
    
	uint8_t buffer[10];
	uint8_t serialRXLen = 0;
	
	//t.start();
	
    while (1)
    {
        // TODO: try to read serial port before doing something else
		/*
        serialRXLen = 0;
		t.reset();
		
		if (pc.readable())
		{
			buffer[serialRXLen++] = pc.getc();
			while (t.read_us() < 1000)
			{
				while (pc.readable() && serialRXLen < 10)
				{
					buffer[serialRXLen++] = pc.getc();
				}
				if (serialRXLen >= 10)
				{
					break;
				}
			}
			//t.stop();
			if (serialRXLen > 0)
			{
				//wait_ms(1);
				while (pc.write(buffer, serialRXLen, serialCallbackTX))
				{
				}
				BS_payload_TX[0] = buffer[0];
				
				wait(2);
			}
		}
        continue;
        */
        
        // TODO: set request data, replace BS_payload_TX (only 1 byte should be sufficient)
        
        nrf.writeTXData(BS_payload_TX + mode, 1);
        
        // get status and reset interrupt flags
        nrf.getIRQStatus(rx, txDone, maxTry);
    	nrf.resetIRQFlags();
        
        rxLen = 0;
        
        if (rx)
        {
        	uint8_t pipe;
			
			while (nrf.dataAvailable())
			{
				nrf.readRXData(currentBuffer, rxLen, pipe);
				
	        	//pc.printf("recv, rt: %i, len: %i, t_d: %i, t: %i, id: %i\n", numRetries, rxLen, currentBuffer[0], time, currentBuffer[1]);
	        	
	        	if (rxLen > 0)
	        	{
		        	while (pc.write(currentBuffer, rxLen, serialCallbackTX))
		        	{
		        		wait_us(1);
		        	}
		        	
		        	// change buffer to not accidentially ovwerwrite data while sending via serial port in parallel
		        	if (currentBuffer == RX_buffer_1)
		        	{
		        		currentBuffer = RX_buffer_2;
		        	}
		        	else
		        	{
		        		currentBuffer = RX_buffer_1;
		        	}
				}
			}
		}
		
		if ((txDone && rxLen == 0) || maxTry)
		{
			/*
			if (maxTry)
			{
        		numRetries = nrf.getRetryCount();
        		pc.printf("mrt, rt: %i\n", numRetries);
			}
			*/
			// debug
        	// uncomment for debugging
        	/*
        	int time = t.read_ms();
        	t.stop();
        	t.reset();
            t.start();
        	
			if (maxTry)
			{
        		numRetries = nrf.getRetryCount();
        		pc.printf("mrt, rt: %i, t: %i\n", numRetries, time);
			}
			else
			{
				pc.printf("done in %i\n", time);
			}
			*/
			
			// select next node
        	nRF_Node++;
	    	if (nRF_Node >= TOTAL_NODES_AND_SUBNODES)
	    	{
	    		nRF_Node = 0;
	        }
	    	// select the node to talk
	    	nrf.setTXAddress(NRF_address[nRF_Node], NRF_ADDR_LEN);
	    	nrf.setRXAddress(0, NRF_address[nRF_Node], NRF_ADDR_LEN);
	    	continue;
		}
    }
}

