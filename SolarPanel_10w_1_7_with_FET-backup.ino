/*--------------------------------------------------------------
  Program:      Portable Solar controller

  Description:  Monitors voltage from Panel and battery and determines
                which devices to allow to function.
  
  Hardware:     Arduino Uno with voltage divider on A2.
                Resisotrs R1 (ie: 216k) and R2 (ie: 33.1K)
                        |--------------probe
                        Z   R1
                        Z               probe
                        |                |
                A2--------------        ____ +
                        |                -- -   battery
                        Z   R2            |
                        Z                 |
                        |                 |
                GND-------------------------
  
  Date:         2017-04-29
 
  Author:       Sherman Stebbins
  Modifications:2017-05-02 
                Added second input and added two outputs for controling Solar power devices.

  Updated: 2017-05-14
                Changed chip Hz to 1 for a 6ma savings in power.
                also adjusted flashing time to save power.
  Updated: 2017-05-30
                Made major change to voltage divider and 
                calculation code. Added readVcc(), for actual referrence voltage.
                Also added code for denominator.  
  Updated 2017-06-12
                More major changes, put voltage indicator in millis delay so it would
                only run a little bit. As well as having issues with usb cutting out and might be
                coding issue. if not, then will change resistor, it might be not getting high 
                from pin, might even have to add transistor.  
  Updated 2017-07-26
                More major changes, took out buck but did leave in FET after changing to P-channel 9540, but 
                to use this I had to add transister and use USB pin for control of fet to shut off panel 
                depening on condition (ie: over charge, and to read accurate volatages from battery and no load panel)    
  Updated 2017-7-30
                Using FET to shut off as PWM is working great, Voltage count is adjusted for bvoltage accuracy.
                It seems to still have high voltage during direct sunlight charge    
--------------------------------------------------------------*/
/*                         Attiny85 
 *                           ___
 *                  Reset 1-|   |-8 5v
 *     input Panel   a3-3 2-|   |-7 2 a1 voltage indicator led
 *     input Battery a2-4 3-|   |-6 1 (pwm)Night light
 *                    Gnd 4-|   |-5 0 (pwm)used for turning off solar
 *                           ---
 */
// number of analog samples to take per reading
#define NUM_SAMPLES 10
//#define DEBUG
unsigned char sample_count = 0; // current sample number



int sversion = 7; //current firmware version for startup
#ifdef DEBUG
int outport_led = 1;  //change port for led 1 for tiny
int outport_fet = 0; //change port for usb 0 for tiny
#else
int outport_led = 1;  //change port for led 1 for tiny
int outport_fet = 0; //change port for usb 0 for tiny
#endif


int inport_battery = A2; //voltage read for battery A2 for tiny
int inport_panel = A3;  //voltage read for panel A3 for tiny
int outport_led_voltage_indicator = 2; //voltage read from battery to meter  2 = A1 for tiny
int bcount = 1; //count voltage flashes if >11v for battery volage indicator
int led_brightness = 60; //How bright for night light pwm
int led_bright_reduce = 30; //lower voltage, subract this from pwm led
int pwm_rate = 127; //half on at first
float bvoltage_correction = 1.7; //Alter to adjust for inaccuracy of resistor, 
                            //after mesuring voltage from battery add or subtract for more accurate count

float pvoltage = 0.0;   // calculated panel voltage
float pwmvoltage = 0.0; // calculate and adjust panel load voltage to battery
float bvoltage = 0.0;   // calculated panel voltage
float minvoltage_usb = 9.8; //cutoff for usb
float minvoltage_led = 9.0; //cutoff for night light
float minvoltage_panel = 2.5; // detect dark adjust for darkness
float voltage_low = 12.4; //low voltage indicator 
float voltage_critical =11.8 ; //crital voltage indicator,   +.3 volts due to it reads .3 volts low.

unsigned long previousMillis = 0;
unsigned long prevMillis = 0; //Previous millis for voltage indicator
unsigned long hold_state_voltage_check = 10000; //hold for sec * 1000 seconds
unsigned long hold_state_voltage_indicator = 50000; //hold for voltage indicator
unsigned long panel_prevmill = 0;


//for accurate voltage:
float Vcc = 0.0;
float denominator = 0.0;
float resistor1 = 220000; //Large Resistor change to match divider circuit
float resistor2 = 33000; //Small Resistor change to match divider circuit

bool usb_state = LOW;  //usb on or off

//critical low
bool critical_low = LOW; //if HIGH it will shut all down till voltage recovers
unsigned long pMillis = 0; //for low and critical low reaction
unsigned long hold_state_critical_low_reaction = 60000;//react to critical and low voltage
int critical_low_count = 0; //count how many times before retesting and allowing critical_low state to change.
int critcal_low_max_loops = 3; //how many times it goes past hold_state_critical_low_reaction max for critical_low_count


void setup()
{
    #ifdef DEBUG
    Serial.begin(9600);
    #endif
    pinMode(outport_led,OUTPUT);
    pinMode(outport_fet,OUTPUT);
    pinMode(outport_led_voltage_indicator,OUTPUT);
    analogWrite(outport_fet,0);
    delay(1000);
    for(int i=1;i<=sversion;i++){
      //analogWrite(outport_fet,255);  //for attiny85 programming shield led
      digitalWrite(outport_led_voltage_indicator,HIGH);
      //digitalWrite(outport_led,HIGH);
      delay(300);
      //analogWrite(outport_fet,LOW);
      digitalWrite(outport_led_voltage_indicator,LOW);
      digitalWrite(outport_led,LOW);
      delay(600);
    }   
    delay(150);
    denominator = resistor2 / (resistor1 + resistor2);
}

void loop()
{
  //new code added using internal ref  
    Vcc = readVcc()/1000.0;
    #ifdef DEBUG
    Serial.print("Ref Vcc: ");
    Serial.println(Vcc);
    #endif
    
  //Panel shutdown for reading voltages:
    analogWrite(outport_fet,0);
    delay(20);
    
  //Battery Section:    
    bvoltage = analogRead(inport_battery);
    bvoltage = (bvoltage / 1024.0) * Vcc; //Vcc
    bvoltage = bvoltage / denominator;
    bvoltage = bvoltage - bvoltage_correction; //Change + or - depenting correction of voltage from meter
    
    #ifdef DEBUG
    Serial.print ("Battery: ");
    //Serial.println (analogRead(inport_battery));    
    Serial.println (bvoltage);
    #endif
    delay(10);
  //turn panel back on:
    analogWrite(outport_fet,pwm_rate);
    
  //Panel Section:
    pvoltage = analogRead(inport_panel);
    pvoltage = (pvoltage / 1024.0) * Vcc; //Vcc
    pvoltage = pvoltage / denominator;

  //batter PWM LOAD Section:  (Possibly later )
    pwmvoltage = analogRead(inport_battery);
    pwmvoltage = (pwmvoltage / 1024.0) * Vcc; //Vcc
    pwmvoltage = pwmvoltage / denominator;  
    
    #ifdef DEBUG
    Serial.print ("Panel: ");
    Serial.println(pvoltage);
    //Serial.println (analogRead(inport_panel));
    #endif
    
    
 //Check for condition on both panel and battery:   
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= hold_state_voltage_check) {
      // Save next check state time:
      previousMillis = currentMillis;
      #ifdef DEBUG
      Serial.print("\n current: ");
      Serial.print(currentMillis);
      Serial.print("\n");
      #endif
      //check if battery good, if not power down devices:
      if (bvoltage >= minvoltage_usb){
        //analogWrite(outport_fet,255); 
        usb_state = HIGH;
        #ifdef DEBUG
        Serial.print(" battery >= minvoltage\n");
        #endif       
      //Power to low shut down usb:
      }else{
        #ifdef DEBUG
        Serial.print(" battery <= minvoltage\n");
        #endif
        //analogWrite(outport_fet,LOW);
        usb_state = LOW;
      } 
      //added to debug port, remove later
      //usb_state = HIGH;
      //Check to see if it is night, if so, turn on night lighting
      if (bvoltage >= minvoltage_led && pvoltage < minvoltage_panel){
          //digitalWrite(outport_led,HIGH);
          analogWrite(outport_led, led_brightness); 
      }else if(bvoltage >= minvoltage_led - .5 && bvoltage < minvoltage_led && pvoltage < minvoltage_panel)
      {
        analogWrite(outport_led, led_brightness - led_bright_reduce); //power going lower dim
      }else{
        
          digitalWrite(outport_led,LOW);
          #ifdef DEBUG
        Serial.print(" battery <= minvoltage_led or pvoltage < minvoltage_panel \n");
        #endif
      } 
      if (pwmvoltage>16.2){
        pwm_rate--;
        if (pwm_rate<5)pwm_rate=5;
      }else{
        pwm_rate++;
        if (pwm_rate>255)pwm_rate=255;
      }
      //pwm_rate=50;
    }

    //Poor mans Voltage indicator   added so it would only show voltage flash after designated time (saving power)
    if (currentMillis - prevMillis >= hold_state_voltage_indicator) {
      prevMillis = currentMillis;
      //Indicator light to show voltage and decimal if above low voltage:
      if(bvoltage >= voltage_low){
        //analogWrite(outport_fet,LOW);
        
        //BATTERY
        int num = (int)bvoltage;
        int dnum = num -10;
        for(int i = 0;i<dnum;i++){
          digitalWrite(outport_led_voltage_indicator,HIGH);
          delay(220); //flash to count voltage.
          digitalWrite(outport_led_voltage_indicator,LOW);
          delay(760); 
        }        
            
        //decimal readout:
        delay(2000);
        float dec = bvoltage - num;
        dec = dec * 10;
        num = (int)dec;
        for(int i = 0;i<num;i++){
            digitalWrite(outport_led_voltage_indicator,HIGH);
            delay(220); //flash to count voltage.
            digitalWrite(outport_led_voltage_indicator,LOW);
            delay(880); 
        }

        //PWM
        //display pwm voltage:
        delay(4000);
        num = (int)pwmvoltage;
        dnum = num -10;
        for(int i = 0;i<dnum;i++){
          digitalWrite(outport_led_voltage_indicator,HIGH);
          delay(220); //flash to count voltage.
          digitalWrite(outport_led_voltage_indicator,LOW);
          delay(760); 
        }        
            
        //PWM decimal readout:
        delay(2000);
        dec = pwmvoltage - num;
        dec = dec * 10;
        num = (int)dec;
        for(int i = 0;i<num;i++){
            digitalWrite(outport_led_voltage_indicator,HIGH);
            delay(220); //flash to count voltage.
            digitalWrite(outport_led_voltage_indicator,LOW);
            delay(880); 
        }
        //analogWrite(outport_fet,255);
        
      }
    }

    //Check for low and Critical low 
    if (currentMillis - pMillis >= hold_state_critical_low_reaction) {
      pMillis = currentMillis;
      if (critical_low and critical_low_count>=critcal_low_max_loops){
        critical_low = LOW;
        critical_low_count = 0;
      }
      critical_low_count++;
      if (bvoltage >= voltage_critical && bvoltage <=voltage_low){
        digitalWrite(outport_led_voltage_indicator,HIGH);
        delay(50); // flash for .85 second low voltage
        digitalWrite(outport_led_voltage_indicator,LOW);
       delay(700);          
      }
      
      if (bvoltage < voltage_critical){
        digitalWrite(outport_led_voltage_indicator,HIGH);
        delay(120); //flash quickly for critical low voltage.
        digitalWrite(outport_led_voltage_indicator,LOW);
        delay(120); //flash quickly for critical low voltage.
        digitalWrite(outport_led_voltage_indicator,HIGH);
        delay(820); //flash quickly for critical low voltage.
        digitalWrite(outport_led_voltage_indicator,LOW);
        delay(520); //flash quickly for critical low voltage.
        digitalWrite(outport_led_voltage_indicator,HIGH);
        delay(120); //flash quickly for critical low voltage.
        digitalWrite(outport_led_voltage_indicator,LOW);
        delay(120); //flash quickly for critical low voltage.  
        critical_low = HIGH;  //put critical low state on
        critical_low_count = 0;        
        #ifdef DEBUG
          Serial.print(" bvoltage < voltage_critical \n");
        #endif
      }
      digitalWrite(outport_led_voltage_indicator,LOW);
      //analogWrite(outport_fet,usb_state);
    }
}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}
