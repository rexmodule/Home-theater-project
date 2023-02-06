// 7.1 Surround System
// by Rhythm Dey

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
#define fan_min 45

#define lcd_addr 0x27
#define btn_delay 300
#define ir_delay 1
#define vol_delay 10
#define pwr_delay 2000
#define lcd_timeout 60000


       
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 10000 

int samples1[NUMSAMPLES];
int samples2[NUMSAMPLES];

// Analog pins
#define sw_pwr A0    // Power
#define sw_mute A1   // Mute
#define stb_led A2  // stand by LED
#define SDA A4       // I2C SDA
#define SCL A5       // I2C SCL
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
#define sw_bt_power 12   //pnp 5v
#define sw_amp_power 13  //npn 24v


// IR HEX code
#define ir_power 0xBF40FF00        // IR power ON/OFF
#define ir_mute 0xFF00FF00         // IR mute
#define ir_in_1 0xA25DFF00         // IR input MP3
#define ir_in_2 0xE817FF00         // IR input AUX
#define ir_in_3 0xEB14FF00         // IR input DVD
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

byte minus[8] = { 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b11111 };

const int digit_width = 3;
const char custom_num_top[10][digit_width] = { 0, 1, 2, 1, 2, 32, 6, 6, 2, 6, 6, 2, 3, 4, 7, 7, 6, 6, 0, 6, 6, 1, 1, 2, 0, 6, 2, 0, 6, 2 };
const char custom_num_bot[10][digit_width] = { 3, 4, 5, 4, 7, 4, 7, 4, 4, 4, 4, 5, 32, 32, 7, 4, 4, 5, 3, 4, 5, 32, 32, 7, 3, 4, 5, 4, 4, 5 };


byte arrow_right[8] = { B00000, B10000, B11000, B11100, B11110, B11100, B11000, B10000 };

LiquidCrystal_I2C lcd(lcd_addr, 20, 4);  // Set the LCD address
int backlight = 0;

unsigned long time;
unsigned long currentTime;
unsigned long time_now = 0;
unsigned long previousTime = 0;
int in, mute, return_d, sub_ph, mix, a, b, x, pos, newPos, power, menu, menu_active, ch_mute, speaker_mode, btn_press, long_press, vol_menu, vol_menu_jup, reset, fan_pwm;
int fl, fr, sl, sr, rr, rl, cn, sub, ir_menu, ir_on, vol_on, mas_vol, fl_vol, fr_vol, sl_vol, sr_vol, rl_vol, rr_vol, cn_vol, sub_vol, pt2258_mute;

int period = 1000;
long btn_timer = 0;
long enc_long_press_time = 600;
float TX;
bool ir_Receiver;

void setup() {

  //Wire.setClock(100000);
  //Serial.begin(9600);
  pt2258.init();
  IrReceiver.begin(11, DISABLE_LED_FEEDBACK);
  lcd.init();
  lcd.backlight();
  lcd.clear();

  pinMode(sw_mute, INPUT_PULLUP);   // Mute
  pinMode(sw_pwr, INPUT_PULLUP);    // Power
  pinMode(enc_sw, INPUT_PULLUP);      
  pinMode(sw_amp_power, OUTPUT);
  pinMode(sw_bt_power, OUTPUT);
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

  digitalWrite(sw_amp_power, LOW);
  digitalWrite(sw_bt_power, HIGH);

  encoder = new RotaryEncoder(enc_clk, enc_dt, RotaryEncoder::LatchMode::TWO03);

  // register interrupt routine
  attachInterrupt(digitalPinToInterrupt(enc_clk), checkPosition, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_dt), checkPosition, CHANGE);


  power = 0;
  //backlight = 1;
  eeprom_read();
  thermal();
  amp_guard();
  start_up();
  power_up();

} //setup end

void loop() {
  thermal();
  amp_guard();
  lcdBacklight();
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
    if (digitalRead(sw_pwr) == LOW || digitalRead(enc_sw) == LOW && TX <= temp_cooldown) {
      power = 1;
      power_up();
      lcd.clear();
      delay(btn_delay);
    }else if(digitalRead(sw_pwr) == LOW || digitalRead(enc_sw) == LOW && TX > temp_lim ) {
      menu_active = 110;
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
/*  //select input ----------------------------------------------//
  if (digitalRead(sw_input) == LOW) {
    in++;
    set_in();
    bt_power_sw();
    delay(btn_delay);
  }
*/
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
          if (menu > 4) {
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
        vol_cl();
        pos = newPos;
      }
      if (pos > newPos) {
        sub_ph--;
        if (sub_ph < 0) {
          sub_ph = 0;
        }
        set_sub_ph();
        vol_cl();
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
        vol_cl();
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
        vol_cl();
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
        vol_cl();
        pos = newPos;
      }
      if (pos > newPos) {
        in--;
        if (in < 0) {
          in = 0;
        }
        set_in();
        bt_power_sw();
        vol_cl();
        pos = newPos;
      }
    }
    if (menu == 4) {
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
}

//eeprom--------------------------------------------//

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

void btn_cl() {
  delay(btn_delay);
  time = millis();
  return_d = 1;
}

void vol_cl() {
  delay(vol_delay);
  time = millis();
  return_d = 1;
}
void ir_cl() {
  time = millis();
  return_d = 1;
}
void return_delay() {
  if (millis() - time > 50000 && return_d == 1 && mute == 0 && menu_active != 0) {
    menu_active = 0;
    vol_menu = 0;
    reset = 0;
    return_d = 0;
    lcd.clear();
  } else if (millis() - time > 5000 && return_d == 1 && mute == 0 && menu_active == 0) {
    vol_menu = 0;
    return_d = 0;
  }
}

//Thermal---------------------------------------------------------------------/
void thermal() {
 uint8_t i;
  float average1, average2, t1, t2 ;

  // take N samples in a row, with a slight delay
  for (i=0; i< NUMSAMPLES; i++) {
   samples1[i] = analogRead(t_sens1);
   samples2[i] = analogRead(t_sens2);
   delay(10);
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

//AMP Guard------------------------------------------------------//
void amp_guard() {
  //delay(100);
  if ( (power == 1 || power == 0 ) && TX > temp_lim) {
    power = 0;
    menu_active = 110;
  }
  else if ( power == 0 && TX < temp_cooldown) { 
    if (menu_active == 110) {
     menu_active = 100;
    } 
  }

  int temp = TX  ;
  if ( temp < 25) {
    temp = 25;
  }
  
  if ( temp > 50) {
    temp = 50;
  }
  fan_pwm = map(temp, 25, 50, fan_min, 100);

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

void setPwmDuty (byte duty) {
  OCR1A = (word) (duty*TCNT1_TOP)/100;
}


//power up -----------------------------------------------------//

void power_up() {
  if (power == 1) {
    setPwmDuty(100);
    lcd.clear();
    delay(500);
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("    SPEAKER MODE:   ");
    lcd.setCursor(0, 2);
    if (speaker_mode == 1) {
      lcd.print("       Stereo       ");
    }
    if (speaker_mode == 0) {
      lcd.print("      Surround      ");
    }
    delay(1200);    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("                    ");
    //lcd.print("       INPUT:       ");
    lcd.setCursor(0, 2);
    if (in == 0) {
      lcd.print("     Bluetooth     ");
    }
    if (in == 1) {
      lcd.print("        AUX        ");
    }
    if (in == 2) {
      lcd.print("         PC        ");
    }
    delay(1200);
    setPwmDuty(fan_pwm);
    lcd.clear();
    mute = 0;
    set_mute();
    vol_menu = 0;
    menu_active = 0;
    delay(300);
    bt_power_sw();
    ir_on = 1;
    vol_on = 0;
    vol_menu_jup = 0;
    digitalWrite(stb_led, LOW);
    digitalWrite(sw_amp_power, HIGH);

  }  
  else {
    digitalWrite(stb_led, HIGH);
    digitalWrite(sw_amp_power, LOW);
    mute = 1;
    set_mute();
    delay(100);
    bt_power_sw();
    menu_active = 110;
    ir_on = 0;
  }
}

void bt_power_sw() {
  if (power == 1) {
    if (in == 0) {
      digitalWrite(sw_bt_power, LOW);
    } else {
      digitalWrite(sw_bt_power, HIGH);
    }
  } else {
    digitalWrite(sw_bt_power, HIGH);
  }
}

void start_up() {
  mute = 1;
  set_mute();
  setPwmDuty(100);
  delay(500);
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("     HT701 rev1    ");
  delay(500);
  lcd.setCursor(0, 2);
  lcd.print("   HiFi Amplifier  ");
  delay(1500);
  lcd.clear();
  delay(300);
  setPwmDuty(0);
  set_in();
  //pt2258.init();
  set_fl();
  set_fr();
  set_sl();
  set_sr();
  set_rl();
  set_rr();
  set_cn();
  set_sub();
  set_sub_ph();
   
}

//Rotary Encoder ----------------------------------------------------------------------------//
void checkPosition() {
  encoder->tick();  // just call tick() to check the state.
}

// IR control --------------------------------------------------------------------------------//

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
          if (speaker_mode == 1) {
            vol_menu_jup = 0;
          }
          break;

        //surround -------------------------------------------------//
        case ir_sub_ph_mode:
          sub_ph++;
          vol_menu = 0;
          break;

        // -------------------------------------------------//
        case ir_reset:
          reset++;
          vol_menu = 0;
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
      set_reset();
      ir_cl();
    }
    IrReceiver.resume();
    ir_Receiver = true;
  }
}



//custom shape --------------------------------------------------------------------------------//

void custom_num_shape() {
  for (int i = 0; i < 8; i++)
    lcd.createChar(i, custom_num[i]);
}

void custom_shape() {
  lcd.createChar(1, arrow_right);
}

//lcd ---------------------------------------------------------//

void lcd_update() {
  int c;
  switch (menu_active) {
    case 0:
      //input -------------------------------------------------//
      lcd.setCursor(0, 0);
      if (in == 0) {
        lcd.print("Bluetooth");
      }
      if (in == 1) {
        lcd.print("AUX      ");
      }
      if (in == 2) {
        lcd.print("PC       ");
      }

      //speaker mode ------------------------------------------//
     
      lcd.setCursor(12, 0);
      if (speaker_mode == 0) {
        lcd.print("Surround");
      }
      if (speaker_mode == 1) {
        lcd.print("  Stereo");
      }

      //thermal ------------------------------------------//
      lcd.setCursor(16, 1);
      lcd.print((int)TX);
      lcd.setCursor(18, 1);
      lcd.print((char)223);
      lcd.setCursor(19, 1);
      lcd.print("C");

      //subwoofer phase ------------------------------------------//
      lcd.setCursor(0, 1);
      lcd.print("SWp");
      lcd.setCursor(3, 1);
      if (sub_ph == 0) {
        lcd.print("+");
      }
      if (sub_ph == 1) {
        lcd.print("-");
      }

      //vol ----------------------------------------------//
      switch (vol_menu) {
        case 0:
          lcd.setCursor(3, 3);
          //       ("   ");
          lcd.print("MAS ");
          c = mas_vol - 69 ;
          break;

        case 1:
          lcd.setCursor(3, 3);
          //       ("   ");
          lcd.print("FL ");
          c = fl_vol;
          break;

        case 2:
          lcd.setCursor(3, 3);
          //       ("   ");
          lcd.print("FR ");
          c = fr_vol;
          break;

        case 3:
          lcd.setCursor(3, 3);
          //       ("   ");
          lcd.print("SL ");
          c = sl_vol;
          break;

        case 4:
          lcd.setCursor(3, 3);
          //       ("   ");
          lcd.print("SR ");
          c = sr_vol;
          break;

        case 5:
          lcd.setCursor(3, 3);
          //       ("   ");
          lcd.print("RL ");
          c = rl_vol;
          break;

        case 6:
          lcd.setCursor(3, 3);
          //       ("   ");
          lcd.print("RR ");
          c = rr_vol;
          break;

        case 7:
          lcd.setCursor(3, 3);
          //       ("   ");
          lcd.print("CEN ");
          c = cn_vol;
          break;

        case 8:
          lcd.setCursor(3, 3);
          //       ("   ");
          lcd.print("SUB ");
          c = sub_vol;
          break;
      }
      break;

    case 1:
      switch (menu) {
        case 3:
          lcd.setCursor(0, 0);
          lcd.print("Sub Woofer Phase >");
          lcd.setCursor(1, 2);
          lcd.print("Normal ");
          lcd.setCursor(1, 3);
          lcd.print("Reverse");
          if (sub_ph == 0) {
            lcd.setCursor(0, 2);
            lcd.write(1);
            lcd.setCursor(0, 3);
            lcd.print(" ");
          }
          if (sub_ph == 1) {
            lcd.setCursor(0, 2);
            lcd.print(" ");
            lcd.setCursor(0, 3);
            lcd.write(1);
          }
          break;

        case 2:
          lcd.setCursor(0, 0);
          lcd.print("Speaker Mode >");
          lcd.setCursor(1, 2);
          lcd.print("Surround");
          lcd.setCursor(1, 3);
          lcd.print("Stereo  ");
          if (speaker_mode == 0) {
            lcd.setCursor(0, 2);
            lcd.write(1);
            lcd.setCursor(0, 3);
            lcd.print(" ");
          }
          if (speaker_mode == 1) {
            lcd.setCursor(0, 2);
            lcd.print(" ");
            lcd.setCursor(0, 3);
            lcd.write(1);
          }
          break;

        case 1:
          lcd.setCursor(0, 0);
          lcd.print("Input Source >");
          lcd.setCursor(1, 1);
          lcd.print("BLUETOOTH");
          lcd.setCursor(1, 2);
          lcd.print("AUX");
          lcd.setCursor(1, 3);
          lcd.print("PC");
          if (in == 0) {
            lcd.setCursor(0, 1);
            lcd.write(1);
            lcd.setCursor(0, 2);
            lcd.print(" ");
            lcd.setCursor(0, 3);
            lcd.print(" ");
          }
          if (in == 1) {
            lcd.setCursor(0, 1);
            lcd.print(" ");
            lcd.setCursor(0, 2);
            lcd.write(1);
            lcd.setCursor(0, 3);
            lcd.print(" ");
          }
          if (in == 2) {
            lcd.setCursor(0, 1);
            lcd.print(" ");
            lcd.setCursor(0, 2);
            lcd.print(" ");
            lcd.setCursor(0, 3);
            lcd.write(1);
          }
          break;

        case 4:
          lcd.setCursor(0, 0);
          lcd.print("                    ");
          lcd.setCursor(0, 2);
          lcd.print("  < Restore AMP >   ");
          lcd.setCursor(0, 3);
          lcd.print(" Exit          YES  ");

          if (reset == 1) {
            lcd.setCursor(0, 3);
            lcd.write(1);
          }

          break;
      }
      break;

    case 99:
      lcd.setCursor(0, 0);
      lcd.print("                    ");
      lcd.setCursor(0, 2);
      lcd.print("        MUTE        ");
      break;

    case 100:
      lcd.setCursor(0, 0);
      lcd.print("                    ");
      lcd.setCursor(0, 1);
      lcd.print("      STANDBY       ");
      lcd.setCursor(0, 2);
      lcd.print("                    ");
      lcd.setCursor(0, 3);
      lcd.print("        ");
      lcd.setCursor(8, 3);
      lcd.print((int)TX);
      lcd.setCursor(10, 3);
      lcd.print((char)223);
      lcd.setCursor(11, 3);
      lcd.print("C");
      lcd.setCursor(12, 3);
      lcd.print("        ");
      break;

     case 110:
      lcd.setCursor(0, 0);
      lcd.print("                    ");
      lcd.setCursor(0, 1);
      lcd.print("  ! AMP OVERHEAT !  ");
      lcd.setCursor(0, 2);
      lcd.print("                    ");
      lcd.setCursor(0, 3);
      lcd.print("        ");      
      lcd.setCursor(8, 3);
      lcd.print((int)TX);
      lcd.setCursor(10, 3);
      lcd.print((char)223);
      lcd.setCursor(11, 3);
      lcd.print("C");
      lcd.setCursor(12, 3);
      lcd.print("        ");
      lcd.backlight();
      
      break; 
  }

  if (menu_active == 0) {
    int y;
    if (c < 0) {
      lcd.setCursor(14, 3);
      lcd.print("+");
      lcd.setCursor(15, 3);
      lcd.print("dB");
      y = 10 - (c + 10);
    } else if (c == -10) {
      lcd.setCursor(14, 3);
      lcd.print("+");
      lcd.setCursor(15, 3);
      lcd.print("dB");
      y = 10;
    } else {
      lcd.setCursor(14, 3);
      lcd.print("-");
      lcd.setCursor(15, 3);
      lcd.print("dB");
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

void lcdBacklight() {

  unsigned long currentTime = millis();

  if ((menu_active == 0 ) || (menu_active == 100 )){
    if(currentTime - previousTime >= lcd_timeout){
     lcd.noBacklight();
     previousTime = currentTime;
    }
  } 
    
  if (((menu_active == 0) || (menu_active == 100 )) &&  (pos != newPos || digitalRead(enc_sw) == LOW || digitalRead(sw_mute) == LOW || ir_Receiver == true)) {
      lcd.backlight();
      previousTime = currentTime;
      pos = newPos;
      ir_Receiver = false;
  } 
    if (menu_active == 99) {
      if (currentTime - previousTime >= 1000){
       previousTime = currentTime;
      if (backlight == 0) {
        lcd.backlight();
        backlight = 1;
      } else {
        lcd.noBacklight();
        backlight = 0 ;
      }
    }
  } 
     if (menu_active == 110) {
     if (currentTime - previousTime >= 500) {
      previousTime = currentTime;
      if (backlight == 0) {
        lcd.backlight();
        backlight = 1;
      } else {
        lcd.noBacklight();
        backlight = 0 ;
      }
    }
  }
}
//all reset --------------------------------------------------------------------------------//

void set_reset() {
  if (reset == 1) {
    lcd.setCursor(0, 0);
    lcd.print("                    ");
    lcd.setCursor(0, 1);
    lcd.print("                    ");
    lcd.setCursor(0, 2);
    lcd.print("    AMP RESTORED    ");
    lcd.setCursor(0, 3);
    lcd.print("                    ");
    delay(2000);
    in = 0;
    mas_vol = 20;
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

//speaker mode --------------------------------------------------------------------------------//

void set_speaker_mode() {
  if (speaker_mode > 1) {
    speaker_mode = 0;
  }
  switch (speaker_mode) {
    case 0:  // 7.1 mode
      digitalWrite(t_bt, LOW);
      digitalWrite(t_aux, LOW);
      digitalWrite(t_surr, HIGH);
      digitalWrite(t_subp, LOW);
      break;
    case 1:  // 2.1 mode
      digitalWrite(t_surr, LOW);
      digitalWrite(t_subp, HIGH);
      break;
  }
  set_sl();
  set_sr();
  set_rl();
  set_rr();
  set_cn();
}

//input settings -----------------------------------------------------//

void set_in() {
  if (in > 2) {
    in = 0;
  }
  switch (in) {
    case 0:
      speaker_mode = 1;
      digitalWrite(t_bt, HIGH);  // BLUETOOTH
      digitalWrite(t_aux, LOW);

      break;

    case 1:
      speaker_mode = 1;
      digitalWrite(t_bt, LOW);  // AUX
      digitalWrite(t_aux, HIGH);
      break;

    case 2:
      digitalWrite(t_bt, LOW);  // PC
      digitalWrite(t_aux, LOW);
      break;
  }
  set_speaker_mode();
}
void set_sub_ph() {
  if (sub_ph > 1) {
    sub_ph = 0;
  }

  switch (sub_ph) {
    case 0: digitalWrite(t_sub_ph, LOW); break;   // Sub Woofer Phase 0 degree
    case 1: digitalWrite(t_sub_ph, HIGH); break;  // Sub Woofer Phase 180 degree
  }
}

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
    case 0: digitalWrite(t_mute, HIGH);
           /* if (speaker_mode == 1) {
             digitalWrite(t_surr, HIGH);
            } */
     break;  // All CH mute disabled
    case 1: digitalWrite(t_mute, LOW);
            digitalWrite(t_surr, LOW);
     break;   // All CH mute
  }
}


//pt2258 settings -----------------------------------------------------//

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
  } else {
    pt2258_mute = 0;
    mute = 0;
  }
  //pt2258.setMasterVolume(mas_vol);
  set_pt2258_mute();
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
      ;
      break;  // mute disabled all ch
    case 1:
      pt2258.setMute(1);
      ;
      break;  // mute all ch
  }
}
//end code
