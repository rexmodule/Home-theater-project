// 7.1 Surround System by Rhythm Dey

#include <Arduino.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <IRremote.hpp>
#include <PT2258.h>
#include <RotaryEncoder.h>

PT2258 pt2258;
RotaryEncoder *encoder = nullptr;

#define temp_lim 60
#define temp_cooldown 48
#define fan_min 40
#define fan_max 100
#define lcd_addr 0x27
#define btn_delay 150
#define ir_delay 1
#define vol_delay 100
#define lcd_timeout 30000
      
#define THERMISTORNOMINAL 10000                               // resistance at 25 degrees C     
#define TEMPERATURENOMINAL 25                                 // temp. for nominal resistance (almost always 25 C)
#define NUMSAMPLES 5                                          //how many samples to take and average, more takes longer but is more 'smooth'
#define BCOEFFICIENT 3950                                     // The beta coefficient of the thermistor (usually 3000-4000)
#define SERIESRESISTOR 10000                                  // the value of the 'other' resistor

int samples1[NUMSAMPLES];
int samples2[NUMSAMPLES];

// Analog pins
#define sw_pwr A0    // Power
#define sw_mute A1   // Mute
#define stb_led A2  // stand by LED
#define t_sens1 A6   // NTC 1
#define t_sens2 A7   // NTC 2 

// Digital Pins
#define enc_dt 2     // DT
#define enc_clk 3    // CLK
#define enc_sw 4     // SW
#define t_mute 5         //npn 5v
#define t_aux 6          //npn 5v
#define t_bt 7           //npn 5v
#define t_surr 8         //npn 5v
#define t_subp A3         //npn 12v
#define t_sub_ph 10      //npn 12v
#define ir_sens 11       // IR
#define t_bt_power 12   //pnp 5v
#define t_amp_power 13  //npn 24v


// IR HEX code
#define ir_power 0xBF40FF00        // IR power ON/OFF
#define ir_mute 0xFF00FF00         // IR mute
#define ir_in_1 0xA25DFF00         // IR input BT
#define ir_in_2 0xE817FF00         // IR input AUX
#define ir_in_3 0xEB14FF00         // IR input PC
#define ir_vol_i 0xE21DFF00        // IR vol++
#define ir_vol_d 0xBD42FF00        // IR vol--
#define ir_fl_i 0xE619FF00         // IR fl++
#define ir_fl_d 0xEA15FF00         // IR fl--
#define ir_fr_i 0xF807FF00         // IR fr++
#define ir_fr_d 0xF906FF00         // IR fr--
#define ir_sl_i 0xB24DFF00         // IR sl++
#define ir_sl_d 0xBE41FF00         // IR sl--
#define ir_sr_i 0xAE51FF00         // IR sr++
#define ir_sr_d 0xA758FF00         // IR sr--
#define ir_rl_i 0xFA05FF00         // IR rl++
#define ir_rl_d 0xFB04FF00         // IR rl--
#define ir_rr_i 0xE51AFF00         // IR rr++
#define ir_rr_d 0xE916FF00         // IR rr--
#define ir_cn_i 0xFD02FF00         // IR cn++
#define ir_cn_d 0xFC03FF00         // IR cn--
#define ir_sub_i 0xB54AFF00        // IR sub++
#define ir_sub_d 0xE11EFF00        // IR sub--
#define ir_sp_mode 0xB748FF00      // IR speaker mode change
#define ir_sub_ph_mode 0xE01FFF00  // IR subwoofer phase
#define ir_reset 0xB946FF00        // IR reset

const byte OC1A_PIN = 9; // fan

const word PWM_FREQ_HZ =  25000; //Adjust this value to adjust the frequency (Frequency in HZ!) (Set currently to 25kHZ)
const word TCNT1_TOP = 16000000/(2*PWM_FREQ_HZ);


byte custom_num[8][8] = {
  { B00111, B01111, B11111, B11111, B11111, B11111, B11111, B11111 },
  { B11111, B11111, B11111, B00000, B00000, B00000, B00000, B00000 },
  { B11100, B11110, B11111, B11111, B11111, B11111, B11111, B11111 },
  { B11111, B11111, B11111, B11111, B11111, B11111, B01111, B00111 },
  { B00000, B00000, B00000, B00000, B00000, B11111, B11111, B11111 },
  { B11111, B11111, B11111, B11111, B11111, B11111, B11110, B11100 },
  { B11111, B11111, B11111, B00000, B00000, B00000, B11111, B11111 },
  { B11111, B11111, B11111, B11111, B11111, B11111, B11111, B11111 }
};

const int digit_width = 3;
const char custom_num_top[10][digit_width] = { 0, 1, 2, 1, 2, 32, 6, 6, 2, 6, 6, 2, 3, 4, 7, 7, 6, 6, 0, 6, 6, 1, 1, 2, 0, 6, 2, 0, 6, 2 };
const char custom_num_bot[10][digit_width] = { 3, 4, 5, 4, 7, 4, 7, 4, 4, 4, 4, 5, 32, 32, 7, 4, 4, 5, 3, 4, 5, 32, 32, 7, 3, 4, 5, 4, 4, 5 };


byte arrow_right[8] = { B00000, B10000, B11000, B11100, B11110, B11100, B11000, B10000 };

LiquidCrystal_I2C lcd(lcd_addr, 20, 4);  // Set the LCD address
bool backlight = 0;

unsigned long time;
unsigned long sleepTimer;
unsigned long dim;
unsigned long currentTime;
unsigned long currentSleepTime;
unsigned long previousTime = 0;

int in, mute, return_d, sub_ph, a, b, pos, newPos, power, menu, menu_active, speaker_mode, btn_press, long_press, vol_menu, vol_menu_jup, reset, fan_pwm;
int fl, fr, sl, sr, rr, rl, cn, sub, ir_menu, ir_on, vol_on, mas_vol, fl_vol, fr_vol, sl_vol, sr_vol, rl_vol, rr_vol, cn_vol, sub_vol, pt2258_mute, sleepTime;

unsigned long btn_timer = 0;
unsigned long enc_long_press_time = 500;
float TX;
bool ir_Receiver;

//Rotary Encoder ----------------------------------------------------------------------------//
void checkPosition() {
  encoder->tick();  // just call tick() to check the state.
}

//backlight timout-------------------------------------------------------//
void lcdBacklight() {
  if(backlight == 1){
    lcd.backlight();
    currentTime = millis();
  } 
}

//Button debounce--------------------------------------------------------//
void btn_cl() {
  delay(btn_delay);
  time = millis();
  return_d = 1;
  backlight = 1;
  lcdBacklight();
}
void vol_cl() {
  delay(vol_delay);
  time = millis();
  return_d = 1;
  backlight = 1;
  lcdBacklight();
}
void ir_cl() {
  time = millis();
  return_d = 1;
  backlight = 1;
  lcdBacklight();
}
void return_delay() {
  if (millis() - time > 50000 && return_d == 1 && mute == 0 && menu_active != 0) {
    menu_active = 0;
    vol_menu = 0;
    reset = 0;
    return_d = 0;
    lcd.clear();
    backlight = 1;
    lcdBacklight();   
  } else if (millis() - time > 5000 && return_d == 1 && mute == 0 && menu_active == 0) {
    vol_menu = 0;
    return_d = 0;
    backlight = 1;
    lcdBacklight();   
  }
}

//Sleep timer------------------------------------------------------------//
void set_sleep() {
  if (sleepTime > 4) {
    sleepTime = -1;      
  }      
  if (sleepTime != -1) {
    currentSleepTime = millis();
  } 
}

//Thermal----------------------------------------------------------------//
void thermal() {
 uint8_t i;
  float average1, average2, t1, t2 ;

  // take N samples in a row, with a slight delay
  for (i=0; i< NUMSAMPLES; i++) {
   samples1[i] = analogRead(t_sens1);
   samples2[i] = analogRead(t_sens2);
   delay(1);
  }
  
  // average all the samples out
  average1 = average2 = 0;

  for (i=0; i< NUMSAMPLES; i++) {
     average1 += samples1[i];
     average2 += samples2[i];
  }
  average1 /= NUMSAMPLES;
  average2 /= NUMSAMPLES; 
 
   average1 = analogRead(t_sens1);
   average2 = analogRead(t_sens2); 
  
  // convert the value to resistance
  average1 = 1023 / average1 - 1;
  average1 = SERIESRESISTOR / average1;

  average2 = 1023 / average2 - 1;
  average2 = SERIESRESISTOR / average2;

  
  t1 = average1 / THERMISTORNOMINAL;     // (R/Ro)
  t1 = log(t1);                  // ln(R/Ro)
  t1 /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  t1 += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  t1 = 1.0 / t1;                 // Invert
  t1 -= 273.15;                         // convert absolute temp to C

  t2 = average2 / THERMISTORNOMINAL;     // (R/Ro)
  t2 = log(t2);                  // ln(R/Ro)
  t2 /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  t2 += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  t2 = 1.0 / t2;                 // Invert
  t2 -= 273.15;                         // convert absolute temp to C

  if(t1 > t2){
    TX = t1;
  }else{
    TX = t2;
  }
}

//Fan PWM----------------------------------------------------------------//
void setPwmDuty (byte duty) {
  OCR1A = (word) (duty*TCNT1_TOP)/100;
}

//SUB Phase--------------------------------------------------------------//
void set_sub_ph() {
  if (sub_ph > 1) {
    sub_ph = 0;
  }

  switch (sub_ph) {
    case 0: digitalWrite(t_sub_ph, LOW); break;   // Sub Woofer Phase 0 degree
    case 1: digitalWrite(t_sub_ph, HIGH); break;  // Sub Woofer Phase 180 degree
  }
}

//Mutting----------------------------------------------------------------//
void set_mute() {
  if (mute > 1) {
    mute = 0;
  }
  if (mute == 1) {
    vol_on = 1;
  } else {
    vol_on = 0;
  }
  switch (mute) {
    case 0: if(speaker_mode == 0) {                                // All CH UN_MUTE
               digitalWrite(t_mute, HIGH);
               digitalWrite(t_surr, HIGH);
            }
            if(speaker_mode == 1){
              digitalWrite(t_surr, LOW);
              digitalWrite(t_mute, HIGH);
            }                        
    break;  
    case 1: digitalWrite(t_mute, LOW);                             // All CH MUTE
            digitalWrite(t_surr, LOW);
    break;   
  }
}

//speaker mode ----------------------------------------------------------//
void set_speaker_mode() {
  if (speaker_mode > 1) {
    speaker_mode = 0;
  }
  switch (speaker_mode) {
    case 0:  // 7.1 mode
      if(in == 2) {
      digitalWrite(t_surr, HIGH);
      digitalWrite(t_subp, LOW);
      }else {speaker_mode = 1;}
      break;
    case 1:  // 2.1 mode
      digitalWrite(t_surr, LOW);
      digitalWrite(t_subp, HIGH);
      break;
  }
  //set_sl();
  //set_sr();
  //set_rl();
  //set_rr();
  //set_cn();
}

//input settings --------------------------------------------------------//
void set_in() {
  if (in > 2) {
    in = 0;
  }
  switch (in) {
    case 0:
      //speaker_mode = 1;
      //set_speaker_mode();
      digitalWrite(t_bt, HIGH);  // BLUETOOTH
      digitalWrite(t_aux, LOW);
      speaker_mode = 1;

      break;

    case 1:
      //speaker_mode = 1;
      //set_speaker_mode();
      digitalWrite(t_bt, LOW);  // AUX
      digitalWrite(t_aux, HIGH);
      speaker_mode = 1;
      break;

    case 2:
      digitalWrite(t_bt, LOW);  // PC
      digitalWrite(t_aux, LOW);
      break;
  }
  set_speaker_mode();
}

//pt2258 settings -------------------------------------------------------//
void set_mas_vol() {
  if (mas_vol > 69) {
    mas_vol = 69;
  }
  if (mas_vol < 10) {
    mas_vol = 10;
  }
  if (mas_vol == 69) {
    pt2258_mute = 1;
    mute = 1;
    //set_mute();
  } else {
    pt2258_mute = 0;
    mute = 0;
    //set_mute();
  }
  //pt2258.setMasterVolume(mas_vol);
  //set_pt2258_mute();
  set_mute();
}
void set_fl() {
  if (fl_vol > 10) {
    fl_vol = 10;
  }
  if (fl_vol < -10) {
    fl_vol = -10;
  }
  fl = mas_vol + fl_vol;
  pt2258.setChannelVolume(fl, 0);  //CH1
}
void set_fr() {
  if (fr_vol > 10) {
    fr_vol = 10;
  }
  if (fr_vol < -10) {
    fr_vol = -10 ;
  }
  fr = mas_vol + fr_vol;
  pt2258.setChannelVolume(fr, 1);  // CH2
}
void set_sl() {
  if (sl_vol > 10) {
    sl_vol = 10;
  }
  if (sl_vol < -10) {
    sl_vol = -10;
  }
  sl = mas_vol + sl_vol;
  pt2258.setChannelVolume(sl, 2);  // CH3
}
void set_sr() {
  if (sr_vol > 10) {
    sr_vol = 10;
  }
  if (sr_vol < -10) {
    sr_vol = -10;
  }
  sr = mas_vol + sr_vol;
  pt2258.setChannelVolume(sr, 3);  // CH4
}
void set_rl() {
  if (rl_vol > 10) {
    rl_vol = 10;
  }
  if (rl_vol < -10) {
    rl_vol = -10;
  }
  rl = mas_vol + rl_vol;
  pt2258.setChannelVolume(rl, 4);  // CH5
}
void set_rr() {
  if (rr_vol > 10) {
    rr_vol = 10;
  }
  if (rr_vol < -10) {
    rr_vol = -10;
  }
  rr = mas_vol + rr_vol;
  pt2258.setChannelVolume(rr, 5);  // CH6
}
void set_cn() {
  if (cn_vol > 10) {
    cn_vol = 10;
  }
  if (cn_vol < -10) {
    cn_vol = -10;
  }
  cn = mas_vol + cn_vol;
  pt2258.setChannelVolume(cn, 6);  // CH7
}
void set_sub() {
  if (sub_vol > 10) {
    sub_vol = 10;
  }
  if (sub_vol < -10) {
    sub_vol = -10;
  }
  sub = mas_vol + sub_vol;
  pt2258.setChannelVolume(sub, 7);  // CH8
}
void set_pt2258_mute() {
  switch (pt2258_mute) {
    case 0:
      pt2258.setMute(0);
      break;  // mute disabled all ch
    case 1:
      pt2258.setMute(1);
      break;  // mute all ch
  }
}

//bluetooth power-------------------------------------------------------//
void bt_power_sw() {
  if (power == 1) {
    if (in == 0) {
      digitalWrite(t_bt_power, LOW);
    } else {
      digitalWrite(t_bt_power, HIGH);
    }
  } else {
    digitalWrite(t_bt_power, HIGH);
  }
}

//all reset ------------------------------------------------------------//
void set_reset() {
  if (reset == 1) {
    lcd.setCursor(0, 0);
    lcd.print(F("                    "));
    lcd.setCursor(0, 1);
    lcd.print(F("                    "));
    lcd.setCursor(0, 2);
    lcd.print(F("    AMP RESTORED    "));
    lcd.setCursor(0, 3);
    lcd.print(F("                    "));
    delay(2000);
    in = 0;
    sleepTime = -1;
    mas_vol = 40;
    fl_vol = 0;
    fr_vol = 0;
    sl_vol = 0;
    sr_vol = 0;
    rl_vol = 0;
    rr_vol = 0;
    cn_vol = 0;
    sub_vol = 0;
    speaker_mode = 1;
    sub_ph = 0;
    vol_menu = 0;
    menu_active = 0;
    reset = 0;
    lcd.clear();
  }
  set_in();
  set_fl();
  set_fr();
  set_sl();
  set_sr();
  set_rl();
  set_rr();
  set_cn();
  set_sub();
  set_speaker_mode();
  set_sub_ph();
  bt_power_sw();
  
}

//power up -------------------------------------------------------------//
void power_up() {
  if (power == 1) {
    digitalWrite(stb_led, LOW);
    digitalWrite(t_amp_power, HIGH);
    setPwmDuty(fan_max);
    lcd.clear();
    delay(500);
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print(F("    SPEAKER MODE:   "));
    lcd.setCursor(0, 2);
    if (speaker_mode == 1) {
      lcd.print(F("       Stereo       "));
    }
    if (speaker_mode == 0) {
      lcd.print(F("      Surround      "));
    }
    delay(1200);    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("                    "));
    lcd.setCursor(0, 2);
    if (in == 0) {
      lcd.print(F("     Bluetooth     "));
    }
    if (in == 1) {
      lcd.print(F("        AUX        "));
    }
    if (in == 2) {
      lcd.print(F("         PC        "));
    }
    delay(1200);
    setPwmDuty(fan_pwm);
    lcd.clear();
    mute = 0;
    set_mute();
    vol_menu = 0;
    menu_active = 0;
    delay(500);
    bt_power_sw();
    ir_on = 1;
    vol_on = 0;
    sleepTimer = -1;
    currentSleepTime = millis();
    currentTime = millis();
    vol_menu_jup = 0;

  }
  else if (TX > temp_lim) {
    digitalWrite(stb_led, HIGH);
    digitalWrite(t_amp_power, LOW);
    mute = 1;
    set_mute();
    delay(100);
    bt_power_sw();
    menu_active = 110;
    ir_on = 0;
    sleepTime = -1; 
  }  
  else {
    digitalWrite(stb_led, HIGH);
    digitalWrite(t_amp_power, LOW);
    mute = 1;
    set_mute();
    delay(100);
    bt_power_sw();
    backlight = 1;
    menu_active = 100;
    lcdBacklight();
    ir_on = 0;
    sleepTime = -1;
  }
}

//AMP Guard--------------------------------------------------------------//
void amp_guard() {
  if ( (power == 1 || power == 0 ) && TX > temp_lim) {
    power = 0;
    power_up();
    menu_active = 110;
  }
  else if ( power == 0 && TX < temp_cooldown && menu_active == 110) { 
    backlight = 1;
    menu_active = 100;
    lcdBacklight();
     
  }

  int temp = TX  ;
  if ( temp < 25) {
    temp = 25;
  }
  
  if ( temp > 50) {
    temp = 50;
  }
  fan_pwm = map(temp, 25, 50, fan_min, fan_max);

  if (power == 1){
    setPwmDuty(fan_pwm);
  }
  if (power == 0 && TX < temp_cooldown)  {
    setPwmDuty(0);
  }
  else if (power == 0 && TX > temp_cooldown){
    setPwmDuty(fan_pwm);
  }
}

void start_up() {
  mute = 1;
  digitalWrite(stb_led, HIGH);
  set_mute();
  set_in();
  set_sub_ph();  
  setPwmDuty(fan_max);
  delay(100);
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(F("     HT701 rev2    "));
  delay(300);
  lcd.setCursor(0, 2);
  lcd.print(F("   HiFi Amplifier  "));
  delay(1300);
  lcd.clear();
//  set_in();
  delay(300);
  setPwmDuty(0);
//  set_sub_ph();  
  pt2258.init();
  set_fl();
  set_fr();
  set_sl();
  set_sr();
  set_rl();
  set_rr();
  set_cn();
  set_sub();
   
}

// IR control ----------------------------------------------------------//
void ir_control() {
  if (IrReceiver.decode()) {

    switch (IrReceiver.decodedIRData.decodedRawData) {
      //power -------------------------------------------------//
      case ir_power:
      if (menu_active != 110){
        power++;
        if (power > 1) {
          power = 0;
        }
        power_up();
      }
        break;
    }
    if (ir_on == 1) {
      switch (IrReceiver.decodedIRData.decodedRawData) {
        //mute -------------------------------------------------//
        case ir_mute:
          mute++;
          if (mute == 1) {
            menu_active = 99;
          } else {
            menu_active = 0;
          }
          set_mute();
          lcd.clear();
          break;

        //select input -------------------------------------------------//
        case ir_in_1:
          in = 0;
          set_in();
          bt_power_sw();
          ir_cl();
          break;

        case ir_in_2:
          in = 1;
          set_in();
          bt_power_sw();
          ir_cl();
          break;

        case ir_in_3:
          in = 2;
          set_in();
          bt_power_sw();
          ir_cl();
          break;
      }
    }

    if (ir_on == 1 && menu_active == 0) {
      switch (IrReceiver.decodedIRData.decodedRawData) {
        //VOL -------------------------------------------------//
        case ir_vol_i:
            if (vol_menu == 0) {
             mas_vol--;
            }
            if (vol_menu == 1) {
              fl_vol--;
            }
            if (vol_menu == 2) {
              fr_vol--;
            }
            if (vol_menu == 3) {
              sl_vol--;
            }
            if (vol_menu == 4) {
              sr_vol--;
            }
            if (vol_menu == 5) {
              rl_vol--;
            }
            if (vol_menu == 6) {
              rr_vol--;
            }
            if (vol_menu == 7) {
              cn_vol--;
            }
            if (vol_menu == 8) {
              sub_vol--;
            }
          break;

        case ir_vol_d:
            if (vol_menu == 0) {
             mas_vol++;
            }
            if (vol_menu == 1) {
              fl_vol++;
            }
            if (vol_menu == 2) {
              fr_vol++;
            }
            if (vol_menu == 3) {
              sl_vol++;
            }
            if (vol_menu == 4) {
              sr_vol++;
            }
            if (vol_menu == 5) {
              rl_vol++;
            }
            if (vol_menu == 6) {
              rr_vol++;
            }
            if (vol_menu == 7) {
              cn_vol++;
            }
            if (vol_menu == 8) {
              sub_vol++;
            }
          break;

        // UP/DOWN -------------------------------------------------//
        case ir_sub_i:
         if (speaker_mode == 0) {
          vol_menu++;
          if (vol_menu > 8) {
            vol_menu = 0;
          }
         } else if (speaker_mode == 1) {
          vol_menu++;
          if (vol_menu_jup == 0) {
            if (vol_menu > 2) {
              vol_menu = 8;
              vol_menu_jup = 1;
            }
          }
         }
        if (vol_menu_jup == 1) {
            if (vol_menu > 8) {
              vol_menu = 0;
              vol_menu_jup = 0;
            }
        }
          break;

        case ir_sub_d:
         if (speaker_mode == 0) {
          vol_menu--;
          if (vol_menu < 0) {
            vol_menu = 8;
          }
         } else if (speaker_mode == 1) {
          vol_menu--;
           if (vol_menu_jup == 0) {
            if (vol_menu < 0) {
              vol_menu = 8;
              vol_menu_jup = 1;
            }
          }
         }
          if (vol_menu_jup == 1) {
            if (vol_menu < 8) {
              vol_menu = 2;
              vol_menu_jup = 0;
            }
          }
          break;

        //speaker mode -------------------------------------------------//
        case ir_sp_mode:
          speaker_mode++;
          vol_menu = 0;
          break;

        //surround -------------------------------------------------//
        case ir_sub_ph_mode:
          sub_ph++;
          vol_menu = 0;
          break;

        // -------------------------------------------------//
        case ir_reset:
          //reset++;
          sleepTime++;
          vol_menu = 0;
          if(sleepTime > 4){
            sleepTime = -1;
          }
          break;
      }
      set_mas_vol();
      set_fl();
      set_fr();
      set_sl();
      set_sr();
      set_rl();
      set_rr();
      set_cn();
      set_sub();
      set_speaker_mode();
      set_sub_ph();
      //set_reset();
      ir_cl();
    }
    IrReceiver.resume();
    ir_Receiver = true;
  }
}

//custom shape ---------------------------------------------------------//
void custom_num_shape() {
  for (int i = 0; i < 8; i++)
    lcd.createChar(i, custom_num[i]);
}
void custom_shape() {
  lcd.createChar(1, arrow_right);
}

//lcd ------------------------------------------------------------------//
void lcd_update() {
  int c;
  switch (menu_active) {
    case 0:
      //input -------------------------------------------------//
      lcd.setCursor(0, 0);
      if (in == 0) {
        lcd.print(F("Bluetooth"));
      }
      if (in == 1) {
        lcd.print(F("AUX      "));
      }
      if (in == 2) {
        lcd.print(F("PC       "));
      }

      //speaker mode ------------------------------------------//
     
      lcd.setCursor(12, 0);
      if (speaker_mode == 0) {
        lcd.print(F("Surround"));
      }
      if (speaker_mode == 1) {
        lcd.print(F("  Stereo"));
      }

      //thermal ------------------------------------------//
      lcd.setCursor(16, 1);
      lcd.print((int)TX);
      lcd.setCursor(18, 1);
      lcd.print((char)223);
      lcd.setCursor(19, 1);
      lcd.print(F("C"));

      //subwoofer phase ------------------------------------------//
      lcd.setCursor(0, 1);
      lcd.print(F("SWp"));
      lcd.setCursor(3, 1);
      if (sub_ph == 0) {
        lcd.print(F("+"));
      }
      if (sub_ph == 1) {
        lcd.print(F("-"));
      }
      
      //auto sleep ---------------------------------------------------//    
      if(sleepTime == 0) {
        lcd.setCursor(0, 2); 
        lcd.print(F("zZ"));
        lcd.setCursor(17, 2);
        lcd.print(F("30m"));       
      }
      else if(sleepTime == 1) {
        lcd.setCursor(0, 2); 
        lcd.print(F("zZ"));
        lcd.setCursor(17, 2);
        lcd.print(F("45m"));       
      }
      else if(sleepTime == 2) {
        lcd.setCursor(0, 2); 
        lcd.print(F("zZ"));
        lcd.setCursor(17, 2);
        lcd.print(F("1hr"));       
      }
      else if(sleepTime == 3) {
        lcd.setCursor(0, 2); 
        lcd.print(F("zZ"));
        lcd.setCursor(17, 2);
        lcd.print(F("2hr"));       
      }
      else if(sleepTime == 4) {
        lcd.setCursor(0, 2); 
        lcd.print(F("zZ"));
        lcd.setCursor(17, 2);
        lcd.print(F("3hr"));       
      }else if(sleepTime == -1){
        lcd.setCursor(0, 2);
        lcd.print(F("  "));
        lcd.setCursor(17, 2);
        lcd.print(F("   "));  
      }     

      //vol ----------------------------------------------//
      switch (vol_menu) {
        case 0:
          lcd.setCursor(3, 3);
          //       ("   ");
          lcd.print(F("MAS "));
          c = mas_vol - 69 ;
          break;

        case 1:
          lcd.setCursor(3, 3);
          //       ("   ");
          lcd.print(F("FL "));
          c = fl_vol;
          break;

        case 2:
          lcd.setCursor(3, 3);
          //       ("   ");
          lcd.print(F("FR "));
          c = fr_vol;
          break;

        case 3:
          lcd.setCursor(3, 3);
          //       ("   ");
          lcd.print(F("SL "));
          c = sl_vol;
          break;

        case 4:
          lcd.setCursor(3, 3);
          //       ("   ");
          lcd.print(F("SR "));
          c = sr_vol;
          break;

        case 5:
          lcd.setCursor(3, 3);
          //       ("   ");
          lcd.print(F("RL "));
          c = rl_vol;
          break;

        case 6:
          lcd.setCursor(3, 3);
          //       ("   ");
          lcd.print(F("RR "));
          c = rr_vol;
          break;

        case 7:
          lcd.setCursor(3, 3);
          //       ("   ");
          lcd.print(F("CEN "));
          c = cn_vol;
          break;

        case 8:
          lcd.setCursor(3, 3);
          //       ("   ");
          lcd.print(F("SUB "));
          c = sub_vol;
          break;
      }
      break;

    case 1:
      switch (menu) {
        case 3:
          lcd.setCursor(0, 0);
          lcd.print(F("Sub Woofer Phase >"));
          lcd.setCursor(1, 2);
          lcd.print(F("Normal "));
          lcd.setCursor(1, 3);
          lcd.print(F("Reverse"));
          if (sub_ph == 0) {
            lcd.setCursor(0, 2);
            lcd.write(1);
            lcd.setCursor(0, 3);
            lcd.print(F(" "));
          }
          if (sub_ph == 1) {
            lcd.setCursor(0, 2);
            lcd.print(F(" "));
            lcd.setCursor(0, 3);
            lcd.write(1);
          }
          break;

        case 2:
          lcd.setCursor(0, 0);
          lcd.print(F("Speaker Mode >"));
          lcd.setCursor(1, 2);
          lcd.print(F("Surround"));
          lcd.setCursor(1, 3);
          lcd.print(F("Stereo  "));
          if (speaker_mode == 0) {
            lcd.setCursor(0, 2);
            lcd.write(1);
            lcd.setCursor(0, 3);
            lcd.print(F(" "));
          }
          if (speaker_mode == 1) {
            lcd.setCursor(0, 2);
            lcd.print(F(" "));
            lcd.setCursor(0, 3);
            lcd.write(1);
          }
          break;

        case 1:
          lcd.setCursor(0, 0);
          lcd.print(F("Input Source >"));
          lcd.setCursor(1, 1);
          lcd.print(F("BLUETOOTH"));
          lcd.setCursor(1, 2);
          lcd.print(F("AUX"));
          lcd.setCursor(1, 3);
          lcd.print(F("PC"));
          if (in == 0) {
            lcd.setCursor(0, 1);
            lcd.write(1);
            lcd.setCursor(0, 2);
            lcd.print(F(" "));
            lcd.setCursor(0, 3);
            lcd.print(F(" "));
          }
          if (in == 1) {
            lcd.setCursor(0, 1);
            lcd.print(F(" "));
            lcd.setCursor(0, 2);
            lcd.write(1);
            lcd.setCursor(0, 3);
            lcd.print(F(" "));
          }
          if (in == 2) {
            lcd.setCursor(0, 1);
            lcd.print(F(" "));
            lcd.setCursor(0, 2);
            lcd.print(F(" "));
            lcd.setCursor(0, 3);
            lcd.write(1);
          }
          break;

        case 4:
          lcd.setCursor(0, 0);
          lcd.print(F("Sleep Timer >"));
          if (sleepTime == -1){
          lcd.setCursor(0, 1);
          lcd.write(1);
          lcd.setCursor(1, 1);
          lcd.print(F("Off       "));
          lcd.setCursor(0, 2);
          lcd.print(F(" "));
          lcd.setCursor(1, 2);
          lcd.print(F("30 Minutes"));
          lcd.setCursor(0, 3);
          lcd.print(F(" "));
          lcd.setCursor(1, 3);
          lcd.print(F("45 Minutes"));
          }
          if (sleepTime == 0){
          lcd.setCursor(0, 1);
          lcd.print(F(" "));
          lcd.setCursor(1, 1);
          lcd.print(F("Off       "));
          lcd.setCursor(0, 2);
          lcd.write(1);
          lcd.setCursor(1, 2);
          lcd.print(F("30 Minutes"));
          lcd.setCursor(0, 3);
          lcd.print(F(" "));
          lcd.setCursor(1, 3);
          lcd.print(F("45 Minutes"));
          }
          if (sleepTime == 1){
          lcd.setCursor(0, 1);
          lcd.print(F(" "));
          lcd.setCursor(1, 1);
          lcd.print(F("Off       "));
          lcd.setCursor(0, 2);
          lcd.print(F(" "));
          lcd.setCursor(1, 2);
          lcd.print(F("30 Minutes"));
          lcd.setCursor(0, 3);
          lcd.write(1);
          lcd.setCursor(1, 3);
          lcd.print(F("45 Minutes"));
          }
          if (sleepTime == 2){     // Page 2 
          lcd.setCursor(0, 1);
          lcd.write(1);
          lcd.setCursor(1, 1);
          lcd.print(F("1 Hour    "));
          lcd.setCursor(0, 2);
          lcd.print(F(" "));
          lcd.setCursor(1, 2);
          lcd.print(F("2 Hours   "));
          lcd.setCursor(0, 3);
          lcd.print(F(" "));
          lcd.setCursor(1, 3);
          lcd.print(F("3 Hours   "));
          }
          if (sleepTime == 3){     
          lcd.setCursor(0, 1);
          lcd.print(F(" "));
          lcd.setCursor(1, 1);
          lcd.print(F("1 Hours   "));
          lcd.setCursor(0, 2);
          lcd.write(1);
          lcd.setCursor(1, 2);
          lcd.print(F("2 Hours   "));
          lcd.setCursor(0, 3);
          lcd.print(F(" "));
          lcd.setCursor(1, 3);
          lcd.print(F("3 Hours   "));
          }
          if (sleepTime == 4){     
          lcd.setCursor(0, 1);
          lcd.print(F(" "));
          lcd.setCursor(1, 1);
          lcd.print(F("1 Hours   "));
          lcd.setCursor(0, 2);
          lcd.print(F(" "));
          lcd.setCursor(1, 2);
          lcd.print(F("2 Hours   "));
          lcd.setCursor(0, 3);
          lcd.write(1);
          lcd.setCursor(1, 3);
          lcd.print(F("3 Hours   "));
          }
          break;

        case 5:
          lcd.setCursor(0, 0);
          lcd.print(F("                    "));
          lcd.setCursor(0, 2);
          lcd.print(F("  < Restore AMP >   "));
          lcd.setCursor(0, 3);
          lcd.print(F(" Exit          YES  "));

          if (reset == 1) {
            lcd.setCursor(0, 3);
            lcd.write(1);
          }

          break;
      }
      break;

    case 99:
      lcd.setCursor(0, 0);
      lcd.print(F("                    "));
      lcd.setCursor(0, 2);
      lcd.print(F("        MUTE        "));
      break;

    case 100:
      lcd.setCursor(0, 0);
      lcd.print(F("                    "));
      lcd.setCursor(0, 1);
      lcd.print(F("      STANDBY       "));
      lcd.setCursor(0, 2);
      lcd.print(F("                    "));
      lcd.setCursor(0, 3);
      lcd.print(F("        "));
      lcd.setCursor(8, 3);
      lcd.print((int)TX);
      lcd.setCursor(10, 3);
      lcd.print((char)223);
      lcd.setCursor(11, 3);
      lcd.print(F("C"));
      lcd.setCursor(12, 3);
      lcd.print(F("        "));
      break;

     case 110:
      lcd.setCursor(0, 0);
      lcd.print(F("                    "));
      lcd.setCursor(0, 1);
      lcd.print(F("  ! AMP OVERHEAT !  "));
      lcd.setCursor(0, 2);
      lcd.print(F("                    "));
      lcd.setCursor(0, 3);
      lcd.print(F("        "));      
      lcd.setCursor(8, 3);
      lcd.print((int)TX);
      lcd.setCursor(10, 3);
      lcd.print((char)223);
      lcd.setCursor(11, 3);
      lcd.print(F("C"));
      lcd.setCursor(12, 3);
      lcd.print(F("        "));     
      break; 
  }

  if (menu_active == 0) {
    int y;
    if (c < 0) {
      lcd.setCursor(14, 3);
      lcd.print(F("+"));
      lcd.setCursor(15, 3);
      lcd.print(F("dB"));
      y = 10 - (c + 10);
    } else if (c == -10) {
      lcd.setCursor(14, 3);
      lcd.print(F("+"));
      lcd.setCursor(15, 3);
      lcd.print(F("dB"));
      y = 10;
    } else {
      lcd.setCursor(14, 3);
      lcd.print(F("-"));
      lcd.setCursor(15, 3);
      lcd.print(F("dB"));
      y = c;
    }
    a = y / 10;
    b = y - a * 10;

    lcd.setCursor(7, 2);
    for (int i = 0; i < digit_width; i++)
      lcd.print(custom_num_top[a][i]);

    lcd.setCursor(7, 3);
    for (int i = 0; i < digit_width; i++)
      lcd.print(custom_num_bot[a][i]);

    lcd.setCursor(11, 2);
    for (int i = 0; i < digit_width; i++)
      lcd.print(custom_num_top[b][i]);

    lcd.setCursor(11, 3);
    for (int i = 0; i < digit_width; i++)
      lcd.print(custom_num_bot[b][i]);
  }
}

//EEPROM---------------------------------------------------------------//
void eeprom_update() {
  EEPROM.update(0, in);
  EEPROM.update(1, mas_vol);
  EEPROM.update(2, fl_vol + 10);
  EEPROM.update(3, fr_vol + 10);
  EEPROM.update(4, sl_vol + 10);
  EEPROM.update(5, sr_vol + 10);
  EEPROM.update(6, rl_vol + 10);
  EEPROM.update(7, rr_vol + 10);
  EEPROM.update(8, cn_vol + 10);
  EEPROM.update(9, sub_vol + 10);
  EEPROM.update(10, sub_ph);
  EEPROM.update(11, speaker_mode);
}
void eeprom_read() { 
  in = EEPROM.read(0);
  mas_vol = EEPROM.read(1);
  fl_vol = EEPROM.read(2) - 10;
  fr_vol = EEPROM.read(3) - 10;
  sl_vol = EEPROM.read(4) - 10;
  sr_vol = EEPROM.read(5) - 10;
  rl_vol = EEPROM.read(6) - 10;
  rr_vol = EEPROM.read(7) - 10;
  cn_vol = EEPROM.read(8) - 10;
  sub_vol = EEPROM.read(9) - 10;
  sub_ph = EEPROM.read(10);
  speaker_mode = EEPROM.read(11);
}

//Setup----------------------------------------------------------------//
void setup() {
   
  Wire.setClock(100000);
  wdt_enable(WDTO_8S);
  IrReceiver.begin(ir_sens, DISABLE_LED_FEEDBACK);
  lcd.init(); 
  lcd.clear();
  
  pinMode(sw_mute, INPUT_PULLUP);   // Mute
  pinMode(sw_pwr, INPUT_PULLUP);    // Power
  pinMode(enc_sw, INPUT_PULLUP);
  pinMode(enc_dt, INPUT);  
  pinMode(enc_clk, INPUT); 
  pinMode(t_amp_power, OUTPUT);
  pinMode(t_bt_power, OUTPUT);
  pinMode(t_mute, OUTPUT);
  pinMode(t_aux, OUTPUT);
  pinMode(stb_led, OUTPUT);
  pinMode(t_bt, OUTPUT);
  pinMode(t_surr, OUTPUT);
  pinMode(t_subp, OUTPUT);
  pinMode(t_sub_ph, OUTPUT);
  pinMode(OC1A_PIN, OUTPUT);

  analogReference(EXTERNAL);

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  TCCR1A |= (1 << COM1A1) | (1 << WGM11);
  TCCR1B |= (1 << WGM13) | (1 << CS10);
  ICR1 = TCNT1_TOP;

  digitalWrite(t_amp_power, LOW);
  digitalWrite(t_bt_power, HIGH);

  encoder = new RotaryEncoder(enc_clk, enc_dt, RotaryEncoder::LatchMode::TWO03);

  // register interrupt routine
  attachInterrupt(digitalPinToInterrupt(enc_clk), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_dt), checkPosition, CHANGE);


  power = 0;
  sleepTime = -1;
  backlight = 1;
  eeprom_read();
  thermal();
  amp_guard();
  start_up();
  power_up();

} //setup end

//Loop-----------------------------------------------------------------//
void loop() {
  thermal();
  amp_guard();
  lcd_update();
  eeprom_update();
  ir_control();
  return_delay();
  static int pos = 0;
  encoder->tick();
  newPos = encoder->getPosition();

  if (menu_active == 0) {
    custom_num_shape();
  } else {
    custom_shape();
  }

  //power -----------------------------------------------------//
  if (power == 0) { 
    if ((digitalRead(sw_pwr) == LOW || digitalRead(enc_sw) == LOW) && (TX <= temp_cooldown) && (menu_active != 110)) {
      power = 1;
      power_up();
      lcd.clear();
      delay(btn_delay);
    }   
  }
  if (power == 1) {
    if ((digitalRead(sw_pwr) == LOW)) {
      power = 0;
      power_up();
      lcd.clear();
      delay(btn_delay);
    }
  }

  //select menu -----------------------------------------------//
  if (digitalRead(enc_sw) == LOW) {
    if (btn_press == 0) {
      btn_press = 1;
      btn_timer = millis();
    }
    if ((millis() - btn_timer > enc_long_press_time) && (long_press == 0) && (menu_active == 0)) {
      long_press = 1;
      menu_active = 1;
      menu = 1;
      btn_cl();
      lcd.clear();
    } else if ((millis() - btn_timer > enc_long_press_time) && (long_press == 0) && (menu_active == 1)) {
      long_press = 1;
      menu_active = 0;
      vol_menu = 0;
      reset = 0;
      btn_cl();
      lcd.clear();
    }
  } else {
    if (btn_press == 1) {
      if (long_press == 1) {
        long_press = 0;
      } else {
        if (menu_active == 1) {
          menu++;
          if (menu > 5) {
            menu = 1;
          }
          btn_cl();
          lcd.clear();
        } else if (menu_active == 0 && speaker_mode == 0) {
          vol_menu++;
          if (vol_menu > 8) {
            vol_menu = 0;
          }
          btn_cl();
        } else if (menu_active == 0 && speaker_mode == 1) {
          vol_menu++;
          if (vol_menu_jup == 0) {
            if (vol_menu > 2) {
              vol_menu = 8;
              vol_menu_jup = 1;
            }
          }
          if (vol_menu_jup == 1) {
            if (vol_menu > 8) {
              vol_menu = 0;
              vol_menu_jup = 0;
            }
          }
          btn_cl();
        }
      }
      btn_press = 0;
    }
  }
    
  if (sleepTime != -1) {
  const uint32_t sleepTimes[] = {30, 45, 60, 120, 180}; // in minutes
  uint32_t sleepTimer = (millis() - currentSleepTime) / 60000; // convert to minutes
  if (sleepTimer >= sleepTimes[sleepTime]) {
    power = 0;
    power_up();
  }
}

    if (menu_active == 0 && backlight == 1 ){
      dim = millis() - currentTime;
      if(dim >= lcd_timeout){
      lcd.noBacklight();
      backlight= 0;
      }
    }
    if (menu_active == 100 && backlight == 1){
      dim = millis() - currentTime;
      if(dim >= 10000){
      lcd.noBacklight();
      backlight= 0;
      }
    }
    if (menu_active == 99) {
      currentTime = millis();
      if(currentTime - previousTime >= 750){
        previousTime = currentTime;
        if(backlight == 1){
        lcd.backlight();
        backlight = 0;
        }else{
        lcd.noBacklight();
        backlight = 1;
        }
      }
    }
      
  
    if (menu_active == 110) {
      currentTime = millis();
      if(currentTime - previousTime >= 500){
        previousTime = currentTime;
        if(backlight == 1){
        lcd.backlight();
        backlight = 0;
        }else{
        lcd.noBacklight();
        backlight = 1;
        }
      }
    }

  //mute ------------------------------------------------------//
  if (digitalRead(sw_mute) == LOW) {
    mute++;
    if (mute == 1) {
      menu_active = 99;
    } else {
      menu_active = 0;
    }
    set_mute();
    delay(btn_delay);
    lcd.clear();
  }


  //menu active 0 ---------------------------------------------//  volume up  //
  if (menu_active == 0) {
    if (pos < newPos) {
      if (vol_menu == 0) {
        mas_vol--;
      }
      if (vol_menu == 1) {
        fl_vol--;
      }
      if (vol_menu == 2) {
        fr_vol--;
      }
      if (vol_menu == 3) {
        sl_vol--;
      }
      if (vol_menu == 4) {
        sr_vol--;
      }
      if (vol_menu == 5) {
        rl_vol--;
      }
      if (vol_menu == 6) {
        rr_vol--;
      }
      if (vol_menu == 7) {
        cn_vol--;
      }
      if (vol_menu == 8) {
        sub_vol--;
      }
      set_mas_vol();
      set_fl();
      set_fr();
      set_sl();
      set_sr();
      set_rl();
      set_rr();
      set_cn();
      set_sub();
      vol_cl();
      pos = newPos;
    }

    //menu active 0----------------------------------------------// volume down //
    if (pos > newPos) {
      if (vol_menu == 0) {
        mas_vol++;
      }
      if (vol_menu == 1) {
        fl_vol++;
      }
      if (vol_menu == 2) {
        fr_vol++;
      }
      if (vol_menu == 3) {
        sl_vol++;
      }
      if (vol_menu == 4) {
        sr_vol++;
      }
      if (vol_menu == 5) {
        rl_vol++;
      }
      if (vol_menu == 6) {
        rr_vol++;
      }
      if (vol_menu == 7) {
        cn_vol++;
      }
      if (vol_menu == 8) {
        sub_vol++;
      }
      set_mas_vol();
      set_fl();
      set_fr();
      set_sl();
      set_sr();
      set_rl();
      set_rr();
      set_cn();
      set_sub();
      vol_cl();
      pos = newPos;
    }
  }

  //menu active 1 ---------------------------------------------// Menu //
  if (menu_active == 1) {

    //subwoofer phase--------------------------------------------//
    if (menu == 3) {

      if (pos < newPos) {
        sub_ph++;
        if (sub_ph > 1) {
          sub_ph = 1;
        }
        set_sub_ph();
        btn_cl();
        pos = newPos;
      }
      if (pos > newPos) {
        sub_ph--;
        if (sub_ph < 0) {
          sub_ph = 0;
        }
        set_sub_ph();
        btn_cl();
        pos = newPos;
      }
    }
    //channel mode-----------------------------------------------//
    if (menu == 2) {

      if (pos < newPos) {
        speaker_mode++;
        if (speaker_mode > 1) {
          speaker_mode = 1;
        }
        if (speaker_mode == 1) {
          vol_menu_jup = 0;
        }
        set_speaker_mode();
        btn_cl();
        pos = newPos;
      }
      if ((pos > newPos) && (in == 2)) {
        speaker_mode--;
        if (speaker_mode < 0) {
          speaker_mode = 0;
        }
        if (speaker_mode == 1) {
          vol_menu_jup = 0;
        }
        set_speaker_mode();
        btn_cl();
        pos = newPos;
      }
    }
    //input source----------------------------------------------//
    if (menu == 1) {

      if (pos < newPos) {
        in++;
        if (in > 2) {
          in = 2;
        }
        set_in();
        bt_power_sw();
        btn_cl();
        pos = newPos;
      }
      if (pos > newPos) {
        in--;
        if (in < 0) {
          in = 0;
        }
        set_in();
        bt_power_sw();
        btn_cl();
        pos = newPos;
      }
    }
    if (menu == 4) {
      if (pos < newPos) {
        sleepTime++;
        if (sleepTime > 4) {
          sleepTime = 4;
        }
        set_sleep();
        btn_cl();
        pos = newPos;
      }
      if (pos > newPos) {
        sleepTime--;
        if (sleepTime < -1) {
          sleepTime = -1;
        }
        set_sleep();
        btn_cl();
        pos = newPos;
      }
    }
    if (menu == 5) {
      if (pos > newPos) {
        lcd.clear();
        menu_active = 0;
        pos = newPos;
      } 

      if (pos < newPos) {
        reset++;
        set_reset();
        btn_cl();
        pos = newPos;
      }
    }
  }
  wdt_reset();
}
//end code
