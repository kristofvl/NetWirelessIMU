#include "./Nrf24l019/Nrf24l01p.h"
//#include "TARGET_NUCLEO_F411RE/TOOLCHAIN_ARM_STD/PinNames.h"
//#include "platform/mbed_retarget.h"
//#include "platform/mbed_wait_api.h"
#include <cstdint>

#include "mbed.h"


#define PAYLOAD_MAX_LEN		32
#define PAYLOAD_LEN 		10
#define PAYLOAD_QUAT_LEN 	10

#define NRF_TOTAL_NODES 	6
#define NRF_ADDR_LEN 		5

/* Change if new Nodes or Sub-nodes are made. */
#define TOTAL_NODES_AND_SUBNODES 8

#define MAX_NUM_NODES 16


uint8_t BS_payload_TX[PAYLOAD_LEN] = {0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

uint8_t RX_buffer_1[PAYLOAD_MAX_LEN];
uint8_t RX_buffer_2[PAYLOAD_MAX_LEN];
uint8_t RX_buffer_broadcast[PAYLOAD_MAX_LEN];

uint8_t Serial_RX_buffer[10] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
volatile uint8_t RX_done = 0;

/* add more address for nRF_Nodes and NRF_address  if more than 6 device available.
 * make sure change value for TOTAL_NODES_AND_SUBNODES, to connect to the max number of devices.
*/

uint8_t nrf_data_address[5] = {0xBA,0x5E,0xDA,0x7A,0xFF}; // change 0xFF to actual node
uint8_t nrf_broadcast_address[5] = {0xBA,0x5E,0xCA,0x57,0x3d};


uint32_t nrf_address_table = 0;


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
volatile uint8_t mode = 0; 				// 0 - Quaternion; 1 - Quaternion + linear acceleration
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

//event_callback_t serialCallbackRX = &onSerialRXDone;


volatile bool serial_is_sending = false;

void onSerialTXDone(int event)
{
    serial_is_sending = false;
}

event_callback_t serialCallbackTX = &onSerialTXDone;

void serial_write(uint8_t* buffer, uint8_t rxLen)
{
    while (serial_is_sending)
    {
        wait_us(1);
    }
    // wait_us(1);
    serial_is_sending = true;
    // just to be sure: try to send data until pc.write allows it (in case serial_is_sending is not sufficient)
    while (pc.write(buffer, rxLen, serialCallbackTX))
    {
        wait_us(1);
    }
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





uint8_t do_broadcast(uint8_t mode, uint8_t numNodes, uint8_t sessionID)
{
    nrf.setTXAddress(nrf_broadcast_address, NRF_ADDR_LEN);
    nrf.setRXAddress(0, nrf_broadcast_address, NRF_ADDR_LEN);

    uint8_t BS_broadcast_payload[3] = { mode, numNodes, sessionID };

    nrf.writeTXData(BS_broadcast_payload, 3);


    bool rx, txDone, maxTry;
    uint8_t rxLen = 0;
    uint8_t pipe;

    // get status and reset interrupt flags
    nrf.getIRQStatus(rx, txDone, maxTry);
    nrf.resetIRQFlags();

    if (maxTry)
    {
        return 0;
    }
    else if (rx)
    {
        nrf.readRXData(RX_buffer_broadcast, rxLen, pipe);

        if (rxLen > 0)
        {
            // TODO: work with RX data
        }
        return 1;
    }
    else
    {
        // we sent data successfully, but did not receive data.
        // There is a node waiting, but it either already sent its data or it has some kind of error
        return 0;
    }

    return 0;
}



/**************************************************************
**                       MAIN
***************************************************************/

int main()
{
	Nucleo_Init();
	
	pc.baud(500000);					// Set Serial baud-rate
    pc.SerialBase::enable_input(true);
    pc.SerialBase::enable_output(true);
	pc.set_blocking(false);

	//pc.attach(&onSerialRXDone, Serial::RxIrq);
	
	mode = 0;							// default: Quaternion only mode
    nRF_Node = 0; 						// Initialize the node number
    uint8_t numNodes = 0;               // No nodes available at beginning
	
    nrf.init(0x69, DR_1M, NRF_ADDR_LEN, 1);
	

	nrf.openDynamicTXPipe(nrf_broadcast_address, true, false);
	
	// do not use interrupts, use polling instead
    nrf.maskIRQ(true, true, true);
    
    // Disable nRF IRQ (consider removing IRQ pin from code...)
    nrf_irq.disable_irq();
	
	nrf.setModeTX();
    nrf.flushTX();
    nrf.flushRX();
    nrf.resetIRQFlags();
	
    //nrf_irq.fall(&onDataReceived);    	// Attach the address of the nRF IRQ Interrupt function to the falling edge
    
    
    //Attach the address of the Button Interrupt function to the falling edge
    button.fall(&Button_Interrupt); 
    button.enable_irq();
    
    bool rx, txDone, maxTry;
    uint8_t numRetries = 0;
    
    uint8_t rxLen = 0;
    uint8_t pipe = 0;
    

	uint8_t serial_RX_buffer[10];    

    uint8_t* currentBuffer = RX_buffer_1;
    
    
    // prevent mbed from going into deep sleep
    DeepSleepLock lock;
    // t.start();
    
	
    // NRF_NodeTable nrf_node;
	//t.start();




    // set num retries higher and set it back to 1 after init phase
	nrf.setRetries(500, 5);

    // check if nodes already are online (e.g. when base station was reset during a session)
    // if so, recover session
    for (uint8_t i = 0; i < MAX_NUM_NODES; ++i)
    {
        nrf_data_address[4] = i;
        
        nrf.setTXAddress(nrf_data_address, NRF_ADDR_LEN);
        nrf.setRXAddress(0, nrf_data_address, NRF_ADDR_LEN);
        
        // TODO: write command to flush node
        nrf.writeTXData(BS_payload_TX + 2, 1);

        // get status and reset interrupt flags
        nrf.getIRQStatus(rx, txDone, maxTry);
        nrf.resetIRQFlags();
        
        if (txDone)
        {
            // sensor i responded -> at least i + 1 sensors have been present before
            numNodes = i + 1;

            // TODO: recover session id or upload new session id
        }
        if (rx)
        {
			while (nrf.dataAvailable())
			{
				nrf.readRXData(currentBuffer, rxLen, pipe);
				
	        	if (rxLen > 0)
	        	{
                    // TODO: extract mode, set address as available
				}
			}
		}
        else if (maxTry)
        {
            // TODO: no sensor available at this address
        }
    }

    // only 1 retry after 500us to ensure high performance and short reaction times
	nrf.setRetries(500, 1);




    // ######################################################################################################
    // Main loop starts here:
    // Loop through all sensors, receive and forward data, and regularly check if new sensors available
    // ######################################################################################################

    while (1)
    {
        // TODO: try to read serial port before doing something else
        if (pc.readable())
        {
            ssize_t serial_RX_len = pc.read(serial_RX_buffer, sizeof(serial_RX_buffer), NULL);
            if (serial_RX_len > 0)
            {
                // TODO: serial data available
            }
        }
        

        // TODO: set request data, replace BS_payload_TX (only 1 byte should be sufficient)
        nrf.writeTXData(BS_payload_TX + mode, 1);
        

        // get status and reset interrupt flags
        nrf.getIRQStatus(rx, txDone, maxTry);
    	nrf.resetIRQFlags();
        
        rxLen = 0;
        
        if (rx)
        {
			while (nrf.dataAvailable())
			{
				nrf.readRXData(currentBuffer, rxLen, pipe);
				
	        	//pc.printf("recv, rt: %i, len: %i, t_d: %i, t: %i, id: %i\n", numRetries, rxLen, currentBuffer[0], time, currentBuffer[1]);
	        	
	        	if (rxLen > 0)
	        	{
                    serial_write(currentBuffer, rxLen);
		        	
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
            // TODO: find nxt node ID (sensor might go offline, new ones come online etc, simply node++ does not work)
			// select next node
        	nRF_Node++;
            // ++nrf_node;

	    	if (nRF_Node >= numNodes)
	    	{
	    		nRF_Node = 0;

                // TODO: set session ID to some random number
                uint8_t sessionID = 1;
                // broadcast and check if new sensors are available
                uint8_t retVal = do_broadcast(mode, numNodes, sessionID);
                if (retVal > 0)
                {
                    // uint8_t retVal = do_broadcast(mode, numNodes, sessionID);
                    // nrf.resetIRQFlags();
                    // success, found a new sensor
                    ++numNodes;
                }

	        }
	    	// select the node to talk
            nrf_data_address[4] = nRF_Node;
            // nrf_data_address[4] = nrf_node.getCurrent();
            
            nrf.setTXAddress(nrf_data_address, NRF_ADDR_LEN);
            nrf.setRXAddress(0, nrf_data_address, NRF_ADDR_LEN);

	    	//nrf.setTXAddress(NRF_address[nRF_Node], NRF_ADDR_LEN);
	    	//nrf.setRXAddress(0, NRF_address[nRF_Node], NRF_ADDR_LEN);
	    	continue;
		}
    }
}


