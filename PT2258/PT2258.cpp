/* Arduino PT2258 Library
 *
 * This file is part of the Arduino PT2258 Library
 *
 */ 

#include <arduino.h>
#include <Wire.h>
#include <PT2258.h>

#define  ADDR_00                  0x40 
#define  ADDR_11                  0x46

unsigned char channell_address01[8] = 
  {CHANNEL1_VOLUME_STEP_01, CHANNEL2_VOLUME_STEP_01, 
   CHANNEL3_VOLUME_STEP_01, CHANNEL4_VOLUME_STEP_01,
   CHANNEL5_VOLUME_STEP_01, CHANNEL6_VOLUME_STEP_01, 
   CHANNEL7_VOLUME_STEP_01, CHANNEL8_VOLUME_STEP_01};

unsigned char channell_address10[8] = 
  {CHANNEL1_VOLUME_STEP_10, CHANNEL2_VOLUME_STEP_10,
   CHANNEL3_VOLUME_STEP_10, CHANNEL4_VOLUME_STEP_10,
   CHANNEL5_VOLUME_STEP_10, CHANNEL6_VOLUME_STEP_10,
   CHANNEL7_VOLUME_STEP_10, CHANNEL8_VOLUME_STEP_10};

unsigned char addr[2] = {ADDR_00, ADDR_11};

// helper method
unsigned char PT2258::HEX2BCD (unsigned char x)
{
    unsigned char y;
    y = (x / 10) << 4;
    y = y | (x % 10);
    return (y);
}
   
// helper method
int PT2258::writeI2CChar(unsigned char c)   
{   for(int a=0; a<2; a++)
   {
//  shift address to right - Wire library always uses 7 bit addressing
    Wire.beginTransmission(addr[a]); // transmit to device 0x88, PT2258
    Wire.write(c);   
    int rtnCode = Wire.endTransmission(); // stop transmitting
    return rtnCode;
   }

}

 

// initialize PT2258
int PT2258::init(void)   
{   
    delay(300); // in case this is first time - I2C bus not ready for this long on power on with 10uF cref

    unsigned char masterVolumeValue = 0x00;    //master volume 0dB
    unsigned char channelVolume = 0x00;   // channel volume 0dB
        
    // initialize device
    writeI2CChar(SYSTEM_RESET);

    // set channell volumes to zero
    for(int chno=0; chno<8; chno++)
  {
   
   if (chno <=5 )
    {
     Wire.beginTransmission(ADDR_00); // transmit to device 0x88, PT2258
     Wire.write(channell_address01[chno] | (HEX2BCD(channelVolume)   &  0x0f));   
     Wire.write(channell_address10[chno] | ((HEX2BCD(channelVolume)  &  0xf0)>>4));   
     Wire.endTransmission();       // stop transmitting
    }

    if (chno >=6 )
    {
     Wire.beginTransmission(ADDR_11); // transmit to device 0x88, PT2258
     Wire.write(channell_address01[chno] | (HEX2BCD(channelVolume)   &  0x0f));   
     Wire.write(channell_address10[chno] | ((HEX2BCD(channelVolume)  &  0xf0)>>4));   
     Wire.endTransmission();       // stop transmitting
    }  
  }
    
    // set the master voume
    Wire.beginTransmission(ADDR_00); // transmit to device 0x88, PT2258
    Wire.write(MASTER_VOLUME_1STEP | (HEX2BCD(masterVolumeValue)  &  0x0f));   
    Wire.write(MASTER_VOLUME_10STEP | ((HEX2BCD(masterVolumeValue)  &  0xf0)>>4));   
    Wire.endTransmission();       // stop transmitting

    Wire.beginTransmission(ADDR_11); // transmit to device 0x88, PT2258
    Wire.write(MASTER_VOLUME_1STEP | (HEX2BCD(masterVolumeValue)  &  0x0f));   
    Wire.write(MASTER_VOLUME_10STEP | ((HEX2BCD(masterVolumeValue)  &  0xf0)>>4));   
    Wire.endTransmission();       // stop transmitting

    // set mute off
    return writeI2CChar(MUTE | 0x00);
}   

// Set mute: 1 -> mute on, 0 -> mute off
void PT2258::setMute(char in_mute)   
{   
  writeI2CChar(MUTE | in_mute);
}   

// Set channel volume, attenuation range : 0 to 79dB
void PT2258::setChannelVolume(unsigned char chvol, char chno)
{   

 if (chno <=5 )
  {
  Wire.beginTransmission(ADDR_00); // transmit to device 0x88, PT2258
  Wire.write(channell_address01[chno] | (HEX2BCD(chvol)   &  0x0f));   
  Wire.write(channell_address10[chno] | ((HEX2BCD(chvol)  &  0xf0)>>4));   
  Wire.endTransmission();       // stop transmitting
  }

  if (chno >=6 )
  {  
  Wire.beginTransmission(ADDR_11); // transmit to device 0x88, PT2258
  Wire.write(channell_address01[chno] | (HEX2BCD(chvol)   &  0x0f));   
  Wire.write(channell_address10[chno] | ((HEX2BCD(chvol)  &  0xf0)>>4));   
  Wire.endTransmission();       // stop transmitting
  }
}

// Set master volume, attenuation range : 0 to 79dB
void PT2258::setMasterVolume(unsigned char mvol)   
{   
  Wire.beginTransmission(ADDR_00); // transmit to device 0x88, PT2258
  Wire.write(MASTER_VOLUME_1STEP  | (HEX2BCD(mvol)   &  0x0f));   
  Wire.write(MASTER_VOLUME_10STEP | ((HEX2BCD(mvol)  &  0xf0)>>4));   
  Wire.endTransmission();       // stop transmitting

  Wire.beginTransmission(ADDR_11); // transmit to device 0x88, PT2258
  Wire.write(MASTER_VOLUME_1STEP  | (HEX2BCD(mvol)   &  0x0f));   
  Wire.write(MASTER_VOLUME_10STEP | ((HEX2BCD(mvol)  &  0xf0)>>4));   
  Wire.endTransmission();       // stop transmitting
}
