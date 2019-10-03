
//////////////////////////////////////////////////////////////////////////////////////////////
//// Communication protocol start
//// ----the only thing you need to do is modifying the numbers of variables according to your demand
//
//****define what to send to cell phone*******************
#define NUM_TX_BOOL   0
#define NUM_TX_BYTE   0
#define NUM_TX_SHORT  0
#define NUM_TX_INT    0
#define NUM_TX_FLOAT  0
//*******************************************************

//****define what we want to get from cell phone********
#define NUM_RX_BOOL   0
#define NUM_RX_BYTE   3
#define NUM_RX_SHORT  0
#define NUM_RX_INT    0
#define NUM_RX_FLOAT  0
//*******************************************************

// size of Loop Buffer is editable depending on the data packet transmission rate and memory space
#define LOOP_BUFFER_SIZE 512  
#define SIZE_RECV_PERTIME 64  // fixed

/////////////////////////////////////////////////////
//Do not change the code below+++++++++++++++++++++++
/////////////////////////////////////////////////////
// data structure to send to phone
struct {
  bool bools        [NUM_TX_BOOL]; 
  int8_t bytes      [NUM_TX_BYTE];
  int16_t shorts    [NUM_TX_SHORT];
  int32_t  integers [NUM_TX_INT];
  float   floats    [NUM_TX_FLOAT];
  }txPack;


// data structure to recv from phone
struct {
  bool bools        [NUM_RX_BOOL]; 
  int8_t bytes      [NUM_RX_BYTE];
  int16_t shorts    [NUM_RX_SHORT];
  int32_t  integers [NUM_RX_INT];
  float   floats    [NUM_RX_FLOAT];
  } rxPack;


byte loopBuffer[LOOP_BUFFER_SIZE];
byte rxPacketBuffer[((NUM_RX_BOOL+7)>>3)+NUM_RX_BYTE+(NUM_RX_SHORT<<1)+(NUM_RX_INT<<2)+(NUM_RX_FLOAT<<2)+3];
byte txPacketBuffer[((NUM_TX_BOOL+7)>>3)+NUM_TX_BYTE+(NUM_TX_SHORT<<1)+(NUM_TX_INT<<2)+(NUM_TX_FLOAT<<2)+3];

int16_t RXPACK_SIZE = sizeof(rxPacketBuffer);
int16_t TXPACK_SIZE = sizeof(txPacketBuffer);
int32_t INDEX_RANGE = LOOP_BUFFER_SIZE*2;

HardwareSerial *pSerial;

void initPack(HardwareSerial &serial, int32_t baudRate)
{
  serial.begin(baudRate);
  serial.setTimeout(0);
  pSerial = &serial;
}
void getVariables(void)
{
  unsigned char byte_pos=0,bit_pos=0;
  #if (NUM_RX_BOOL>0)
  for(int i=0;i<NUM_RX_BOOL;i++)
  {
    rxPack.bools[i] = (0x01<<bit_pos)&rxPacketBuffer[byte_pos]; 
    bit_pos++;
    if(bit_pos>=8)
    {
      byte_pos++;
      bit_pos=0;
      }
   }
   if(bit_pos!=0)
     byte_pos++;
  #endif
  
  #if (NUM_RX_BYTE>0)
    for(int i=0;i<NUM_RX_BYTE;i++)
    {
      rxPack.bytes[i] = rxPacketBuffer[byte_pos];
      byte_pos++;
    }
  #endif

  #if (NUM_RX_SHORT>0)
    for(int i=0;i<NUM_RX_SHORT;i++)
    {
      rxPack.shorts[i] = (rxPacketBuffer[byte_pos+1]<<8)|rxPacketBuffer[byte_pos];
      byte_pos+=2;
    }
  #endif

  #if (NUM_RX_INT>0)
    for(int i=0;i<NUM_RX_INT;i++)
    {
      rxPack.integers[i] = (rxPacketBuffer[byte_pos+3]<<24)| (rxPacketBuffer[byte_pos+2]<<16)| (rxPacketBuffer[byte_pos+1]<<8)|rxPacketBuffer[byte_pos];
      byte_pos+=4;
    }
  #endif

   #if (NUM_RX_FLOAT>0)
   float f;
   byte *p = (byte *) &f;
   int pp=0;
    for(int i=0;i<NUM_RX_FLOAT;i++)
    {
      p[0+pp] =  rxPacketBuffer[byte_pos];
      p[1+pp] =  rxPacketBuffer[byte_pos+1];
      p[2+pp] =  rxPacketBuffer[byte_pos+2];
      p[3+pp] =  rxPacketBuffer[byte_pos+3];
      pp+=4;
      byte_pos+=4;
      rxPack.floats[i] = f;
    }
  #endif
  
  }
int32_t  rxIndex=0;
int32_t rdIndex=0;
int32_t err=0;
int32_t sum;
bool recvPack(void)
{
  
  int start_index;
  int tail_index; 
  int cut_size;
  int rx_length;
  bool isOK = 0;
  int rx_pack_index;
  
  // read bytes to loop buffer
  start_index = rxIndex%LOOP_BUFFER_SIZE;
  if(start_index+SIZE_RECV_PERTIME<=LOOP_BUFFER_SIZE)
  {
     rx_length+=pSerial->readBytes(loopBuffer+start_index,SIZE_RECV_PERTIME);
     rxIndex+=rx_length;
  }else
  {
    cut_size = LOOP_BUFFER_SIZE-start_index;
    rx_length=pSerial->readBytes(loopBuffer+start_index,cut_size);
    rxIndex+=rx_length;
    if(rx_length==cut_size)
    {
    cut_size = SIZE_RECV_PERTIME-cut_size;
    rx_length=pSerial->readBytes(loopBuffer,cut_size);
    rxIndex+=rx_length;
    }
   }

   // extract a complete packet
   while(rdIndex<(rxIndex-2*RXPACK_SIZE))
     rdIndex+=RXPACK_SIZE;

   while(rdIndex<=rxIndex-RXPACK_SIZE)
   {
    start_index = rdIndex%LOOP_BUFFER_SIZE;
    isOK = 0;
    if(loopBuffer[start_index]==0xA5)
    {
      tail_index = (start_index+RXPACK_SIZE-1)%LOOP_BUFFER_SIZE;
      if(loopBuffer[tail_index]==0x5A)  // Head and Tail match 
      {
        rx_pack_index = 0;
        // Check Summing 
        sum = 0;
        // if data packet is divided into two segments
        if(tail_index<start_index)
        {
          for(int i = start_index+1;i<LOOP_BUFFER_SIZE;i++)
          {
            rxPacketBuffer[rx_pack_index] = loopBuffer[i];
            rx_pack_index++;
            sum+=loopBuffer[i];
          }
          for(int i = 0;i<tail_index-1;i++)
          {
            rxPacketBuffer[rx_pack_index] = loopBuffer[i];
            rx_pack_index++;
            sum+=loopBuffer[i];
          }
          tail_index--;
          if(tail_index<0)
            tail_index+=LOOP_BUFFER_SIZE;
          
          if(loopBuffer[tail_index]==(sum&0xff))
            isOK = 1;
          }else // data packet is contiguous
        {
          for(int i = start_index+1;i<tail_index-1;i++)
          {
            rxPacketBuffer[rx_pack_index] = loopBuffer[i];
            rx_pack_index++;
            sum+=loopBuffer[i];
          }
          if(loopBuffer[tail_index-1]==(sum&0xff))
            isOK = 1;
          }
        if(isOK) // parse the data to rxPack
          {
            getVariables();
            rdIndex+=RXPACK_SIZE;
          }
      }
      }
      if(!isOK)
      {
        rdIndex++;
        err++;
      }
   }
  // limit the range of read index and recv index
  if(rxIndex>INDEX_RANGE&&rdIndex>INDEX_RANGE)
  {
    rxIndex-=INDEX_RANGE;
    rdIndex-=INDEX_RANGE;
    }
   return isOK; 
  }


void sendPack(void)
{
  short byte_pos=0,bit_pos=0;
  int32_t sum=0;
  txPacketBuffer[byte_pos++] = 0xA5;

 
  #if (NUM_TX_BOOL>0)
  for(int i=0;i<NUM_TX_BOOL;i++)
  {
    if(txPack.bools[i])
      txPacketBuffer[byte_pos] |= 0x01<<bit_pos;
    else
      txPacketBuffer[byte_pos] &= ~(0x01<<bit_pos);
    bit_pos++;
    if(bit_pos>=8)
    {
      byte_pos++;
      bit_pos=0;
      } 
   }
   if(bit_pos!=0)
     byte_pos++;
  #endif

  #if (NUM_TX_BYTE>0)
  
  for(int i=0;i<NUM_TX_BYTE;i++)
    txPacketBuffer[byte_pos++] = txPack.bytes[i];
  
  #endif
  
  #if (NUM_TX_SHORT>0)
  for(int i=0;i<NUM_TX_SHORT;i++)
  {
    txPacketBuffer[byte_pos++] = txPack.shorts[i]&0xff;
    txPacketBuffer[byte_pos++] = (txPack.shorts[i]>>8)&0xff;
  }
  #endif 
  
  #if (NUM_TX_INT>0)
  for(int i=0;i<NUM_TX_INT;i++)
  {
    txPacketBuffer[byte_pos++] = txPack.integers[i]&0xff;
    txPacketBuffer[byte_pos++] = (txPack.integers[i]>>8)&0xff;
    txPacketBuffer[byte_pos++] = (txPack.integers[i]>>16)&0xff;
    txPacketBuffer[byte_pos++] = (txPack.integers[i]>>24)&0xff;
  }
  #endif 

  #if (NUM_TX_FLOAT>0)
  float f;
  byte *add;
  for(int i=0;i<NUM_TX_FLOAT;i++)
  {
    f = txPack.floats[i];
    add = (byte *)&f;
    
    txPacketBuffer[byte_pos++] = add[0];
    txPacketBuffer[byte_pos++] = add[1];
    txPacketBuffer[byte_pos++] = add[2];
    txPacketBuffer[byte_pos++] = add[3];
  }
  #endif   

  for(int i=1;i<TXPACK_SIZE-2;i++)
    sum+=txPacketBuffer[i];
  txPacketBuffer[byte_pos++] = sum&0xff;
  txPacketBuffer[byte_pos] = 0x5a; 
  
  pSerial->write(txPacketBuffer,TXPACK_SIZE);
  }

  
/////////////////////////////////////////////////////
//Do not change the code above-----------------------
/////////////////////////////////////////////////////

//// Communication protocol end
//////////////////////////////////////////////////////////////////////////////////////////////

#define LED_RED   2
#define LED_GREEN 3
#define LED_BLUE  4

void setup() {
initPack(Serial1,9600);
}
void loop() {
   if(recvPack()) // if recved
   {
     analogWrite(LED_RED,rxPack.bytes[0]*2);
     analogWrite(LED_GREEN,rxPack.bytes[1]*2);
     analogWrite(LED_BLUE,rxPack.bytes[2]*2);
   } 
   delay(10);
}
