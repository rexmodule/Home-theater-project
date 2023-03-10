/* Arduino PT2258 Library
 *
 * This file is part of the Arduino PT2258 Library
 *
 */ 

/* channel adresses */
#define  CHANNEL1_VOLUME_STEP_01  0x90
#define  CHANNEL1_VOLUME_STEP_10  0x80 
#define  CHANNEL2_VOLUME_STEP_01  0x50
#define  CHANNEL2_VOLUME_STEP_10  0x40
#define  CHANNEL3_VOLUME_STEP_01  0x10
#define  CHANNEL3_VOLUME_STEP_10  0x00
#define  CHANNEL4_VOLUME_STEP_01  0x30
#define  CHANNEL4_VOLUME_STEP_10  0x20
#define  CHANNEL5_VOLUME_STEP_01  0x70
#define  CHANNEL5_VOLUME_STEP_10  0x60
#define  CHANNEL6_VOLUME_STEP_01  0xb0
#define  CHANNEL6_VOLUME_STEP_10  0xa0
#define  CHANNEL7_VOLUME_STEP_01  0x90
#define  CHANNEL7_VOLUME_STEP_10  0x80
#define  CHANNEL8_VOLUME_STEP_01  0x50
#define  CHANNEL8_VOLUME_STEP_10  0x40
#define  CHANNEL9_VOLUME_STEP_01  0x10
#define  CHANNEL9_VOLUME_STEP_10  0x00
#define  CHANNEL10_VOLUME_STEP_01  0x30
#define  CHANNEL10_VOLUME_STEP_10  0x20
#define  CHANNEL11_VOLUME_STEP_01  0x70
#define  CHANNEL11_VOLUME_STEP_10  0x60
#define  CHANNEL12_VOLUME_STEP_01  0xb0
#define  CHANNEL12_VOLUME_STEP_10  0xa0
#define  CHANNEL13_VOLUME_STEP_01  0x90
#define  CHANNEL13_VOLUME_STEP_10  0x80
#define  CHANNEL14_VOLUME_STEP_01  0x50
#define  CHANNEL14_VOLUME_STEP_10  0x40
#define  CHANNEL15_VOLUME_STEP_01  0x10
#define  CHANNEL15_VOLUME_STEP_10  0x00
#define  CHANNEL16_VOLUME_STEP_01  0x30
#define  CHANNEL16_VOLUME_STEP_10  0x20
#define  CHANNEL17_VOLUME_STEP_01  0x70
#define  CHANNEL17_VOLUME_STEP_10  0x60
#define  CHANNEL18_VOLUME_STEP_01  0xb0
#define  CHANNEL18_VOLUME_STEP_10  0xa0
#define  CHANNEL19_VOLUME_STEP_01  0x90
#define  CHANNEL19_VOLUME_STEP_10  0x80
#define  CHANNEL20_VOLUME_STEP_01  0x50
#define  CHANNEL20_VOLUME_STEP_10  0x40
#define  CHANNEL21_VOLUME_STEP_01  0x10
#define  CHANNEL21_VOLUME_STEP_10  0x00
#define  CHANNEL22_VOLUME_STEP_01  0x30
#define  CHANNEL22_VOLUME_STEP_10  0x20
#define  CHANNEL23_VOLUME_STEP_01  0x70
#define  CHANNEL23_VOLUME_STEP_10  0x60
#define  CHANNEL24_VOLUME_STEP_01  0xb0
#define  CHANNEL24_VOLUME_STEP_10  0xa0
#define  MASTER_VOLUME_1STEP     0xe0
#define  MASTER_VOLUME_10STEP    0xd0
#define  MUTE                    0xf8
#define  UNMUTE                  0xf9
#define  SYSTEM_RESET            0xc0 


#ifndef PT2258_h
#define PT2258_h

class PT2258 {
    
private:
    unsigned char HEX2BCD (unsigned char x);
    int writeI2CChar(unsigned char c);

public:
    int init(void); 
    void setMute(char);
    void setChannelVolume(unsigned char chvol, char chno);
    void setMasterVolume(unsigned char mvol);
};

#endif
