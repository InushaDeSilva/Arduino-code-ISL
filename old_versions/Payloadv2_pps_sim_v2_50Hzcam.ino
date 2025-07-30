#include <LibPrintf.h>

//------- PIN definitions--PIN means the pin of the port------------------------------
#define POWER_LED A1    //POWER LED
#define PPS_LED A2      //PPS LED
#define RTK_LED A3      //RTK LED
#define POWER_LED_PIN 1    //PORTF
#define PPS_LED_PIN 2      //PORTF
#define RTK_LED_PIN 3      //PORTF
#define SIM_PPS_PIN 5   //PORTJ PJ5
#define FLIR_SYNC_PIN 4 //PORTJ PJ4
#define CAM_SYNC_PIN 3  //PORTJ PJ3
#define JETSON_REC 27   //Jetson Force Recovery //SYNCN
#define JETSON_RST 26   //Jetson Reset //SYNCP
#define JETSON_PWR 25   //Jetson PWR   
#define JETSON_REC_PIN 5   //Jetson Force Recovery //SYNCN PORTA
#define JETSON_RST_PIN 4   //Jetson Reset //SYNCP PORTA
#define JETSON_PWR_PIN 3   //Jetson PWR PORTA  
#define LEVEL_CNV_ENABLE 53 //Level converter enable 
#define GPS_PPS_PIN 0 //PORT K ADC8 PC INT16   (USE PINK to work)
#define RTC_PPS_PIN 1 //PORT K ADC9 PC INT17   (USE PINK to work)

//Efficient port set/clear functions ----Does not work for analog pins-----------------
#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))
#define TOGGLE(x,y) (x=((~x)&(1<<y))|((x)&(~(1<<y))))
#define READ(x,y,z) (z=(x>>y)&1)  // read to z
#define READ2(x,y) ((x>>y)&1)     // read out

// variable for state machines---------------------------------------------------------
int cam_pulse_count = 0;   // stores the pulse count of camera control timer (2x) 
int cam_pulse_count_for_GPRMC = 0;   // stores the pulse count of camera control timer -for GPRMC write (2x) 
int gps_pulse_count = 0 ;
int gps_pulse_count_last = 0 ;
int pps_width_count = 0;  // used to control the pulse width of pps_sim
bool flag_pps_sim_high = false; // stores the current pps sim signal value
bool flag_cam_sim_high = false; // stores the current cam signal value
bool flag_write_serial = false; // used to control serial write to Jetson
unsigned int preload = 25536;
unsigned int max_val = 65535;
int cam_pps_preload =0;
int cam_pps_correction =0;


//variables for pps syncing
unsigned int gps_pps_period = 0 ;
long gps_tmr_val_pre = 0;
long gps_tmr_val_crr = 0;
long phase_diff = 0;
bool flag_flock = false; 
bool flag_plock_ready = false;
int flock_count = 0;
long phase_diff_corr = 0;
unsigned int preload_flock = 0;
bool flag_write_serial2 = false; 

//Vars LVI_handhold MCU time to LiDAR via gprmc ---
char gprmcSTR[8]="$GPRMC,";
int chckNum=0;
char chckNumChar[2];
char value_1[100]="";
char value_2[100]="";
int ss=0;
int mm=0;
int hh=0;


 // variables to hold the parsed NMEA data
bool flag_send_next_nema = false;
const int numChars = 256; // increase as needed
char receivedChars[numChars];
char tempChars[numChars]; 
char messageFromPC[numChars] = {0};
char messageToLidar[numChars] = {0};
int integerFromPC = 0;
float floatFromPC = 0.0;

long onTime = 0;
int lastReading = LOW;
int bounceTime = 1000;

boolean newData = false;

const byte buff_len = 90;
char CRCbuffer[buff_len];

// setup
void setup() {

  // set input output
  pinMode(LEVEL_CNV_ENABLE, OUTPUT);  //Level converter (TXS0108E) output enable (Low = High Impedence)
  pinMode(PPS_LED, OUTPUT);  // Green LED
  pinMode(RTK_LED, OUTPUT);  // Red LED
  pinMode(POWER_LED, OUTPUT);  // Yellow LED
  pinMode(JETSON_REC, OUTPUT);   //Jetson Force Recovery
  pinMode(JETSON_RST, OUTPUT);   //Jetson RST
  pinMode(JETSON_PWR, OUTPUT);   //Jetson PWR

  // Set using DDR regiter for non mapped pins
  SET(DDRJ,SIM_PPS_PIN);   //PORTJ PJ5
  SET(DDRJ,FLIR_SYNC_PIN); //PORTJ PJ4
  SET(DDRJ,CAM_SYNC_PIN); //PORTJ PJ3
  CLR(DDRK,GPS_PPS_PIN); //PORTJ PK0
  CLR(DDRK,RTC_PPS_PIN); //PORTJ PK1
  //DDRJ |= B00101000;  // Set using DDR regiter for non mapped pins

  // Startup devices
  digitalWrite(POWER_LED, HIGH);  // Turns on level converter
  digitalWrite(LEVEL_CNV_ENABLE, HIGH);  // Turns on level converter

  //Serial ports
  Serial.begin(115200); // Jetson Communication
  Serial1.begin(38400); // GPS Reciever
  Serial3.begin(38400); // GPS Reciever

  // Configure Timer 4 for 100Hz -  toggle achieves the cam trigger of 50 Hz
  noInterrupts();           // disable all interrupts
  TCCR4A = 0;               // disable compare capture A
  TCCR4B = 0;               // disable compare capture B
  TCNT4 = 25536;            // preload timer 65536-16MHz/8/100Hz  - 50Hz overflow
  TCCR4B |= (1 << CS41);    // PS 8 prescaler :Timer resolution 1/16e6*PS
  TIMSK4 |= (1 << TOIE4);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts


  //Masked External interrupt (GPS)  PCINT 16
  PCICR |= B00000100; // Pin change interrupt 1 enabled (PCINT15:8)
  PCMSK2 |= B00000001; // Specify which pin is enabled (PCINT16 and 17)

  //Enable eny external inturupts
  //EICRA |= B00001100;
  //EIMSK |= B00000010;
  

  printf("test \n");
}

// Loop functions
void loop() {
  //digitalWrite(JETSON_PWR,HIGH);
  //digitalWrite(JETSON_REC,HIGH);
  //digitalWrite(JETSON_RST,HIGH);
  if (flag_write_serial == true){
      sprintf(value_2, "%s%02d%02d%02d%s", gprmcSTR, hh, mm, ss, ".00,A,2237.496474,N,11356.089515,E,0.0,225.5,230520,2.3,S,A*");
      strcpy(value_1,value_2);
      chckNum =checkNum(value_1);
      sprintf(chckNumChar, "%02X", chckNum);
      printf("%s", value_2);
      printf("%s\n", chckNumChar);
      //Serial.write("%s",value_2);
      //Serial.write("s%",chckNumChar);
      flag_write_serial = false;
    }

    if(flag_write_serial2){
      // this for debugging
      /*
      Serial.println(cam_pps_correction);
      Serial.println(gps_pulse_count_last);
      Serial.println(gps_tmr_val_crr);
      Serial.println(gps_tmr_val_pre);
      Serial.println(phase_diff);
      Serial.println(preload_flock);
      //Serial.println(preload);
      flag_write_serial2 = false;*/
    }
 
 //Process GPS
  recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        parseData();
        showParsedData();
        newData = false;
    }

 /*while (Serial3.available()){
    printf("test \n");
    Serial.write(Serial3.read());
 }*/

}

// ISR -  PC inturupts - Masked Block 2
// This is only invoked when GPS PPS is there
ISR(PCINT2_vect) {
  //digitalWrite(RTK_LED,READ2(PORTK,GPS_PPS_PIN));
  //digitalWrite(RTK_LED,!digitalRead(RTK_LED));
  
  //digitalWrite(RTK_LED,digitalRead(A8));
  //digitalWrite(RTK_LED,READ2(PINK,GPS_PPS_PIN)); //read pin 0
  if(READ2(PINK,GPS_PPS_PIN)){
    SET(PORTF,RTK_LED_PIN);
    // Coarse correction
    // check PPSsim count 
    gps_pulse_count_last = gps_pulse_count;
    gps_pulse_count = 0;
    cam_pps_correction = cam_pulse_count-50;    
    
    if(READ2(PORTJ,CAM_SYNC_PIN)){
      //if not in sync tolerance(10 Hz i,e. 100 counts)   
      gps_tmr_val_pre = gps_tmr_val_crr;   
      gps_tmr_val_crr = long(TCNT4);
      
    }
    else{
      gps_tmr_val_pre = gps_tmr_val_crr;    
      gps_tmr_val_crr = -long(TCNT4);
      
      //Serial.println(TCNT4);
      //Serial.println(-65536+TCNT4);
    }
    // check if in range
      if(gps_tmr_val_crr<long(preload) && gps_tmr_val_crr>long(-preload) ){
        Serial.println("corrected...");
        Serial.println(gps_tmr_val_crr);
        Serial.println("corrected...");
        gps_tmr_val_crr= gps_tmr_val_pre;   
      }
    // calculate coarse diff value - to do fix remaining quadrants
    //and pulse count handling
    if (gps_pulse_count_last==100){ // only fine tuning
    if(gps_tmr_val_crr>=0 && gps_tmr_val_pre>=0){
      phase_diff = gps_tmr_val_crr - gps_tmr_val_pre; }
    if(gps_tmr_val_crr>=0 && gps_tmr_val_pre<0){
      //phase_diff = -preload + gps_tmr_val_crr + gps_tmr_val_pre + 65535; }
      phase_diff = phase_diff;} // skip overflow points
    if(gps_tmr_val_crr<0 && gps_tmr_val_pre>=0){
      //phase_diff = 65535 - gps_tmr_val_crr - gps_tmr_val_pre - preload; }
      phase_diff = phase_diff;} // skip overflow points
    if(gps_tmr_val_crr<0 && gps_tmr_val_pre<0){
      phase_diff = -gps_tmr_val_crr + gps_tmr_val_pre; }  
    }
    else{
         // go to nominal value  ?    
    }

    //FLOCK - check the phase diff for 5 cycles and then apply correction to preload
    if(!flag_flock){
      flock_count++;
      phase_diff_corr += phase_diff;
      if(flock_count>5){
        phase_diff_corr = phase_diff_corr/500;
        preload_flock = preload - phase_diff_corr;
        preload= preload_flock;
        flock_count =0;
        flag_plock_ready = true;
        //plock_amount = gps_tmr_val_crr;
        //flag_flock= true;
      }  
    }
  

    // handle - phase diff lock
    if(flag_plock_ready && gps_pulse_count_last ==100){
      if(gps_tmr_val_crr < -max_val+2000){
          preload= preload_flock +5;
      }
      else if(gps_tmr_val_crr < -max_val+1000){
          preload= preload_flock +4;
      }
      else if(gps_tmr_val_crr < -max_val+30){
          preload= preload_flock +1;
      }
      else{
          preload= preload_flock;
      }

      if(gps_tmr_val_crr > preload+2000){
          preload= preload_flock -5;      
      }
      else if(gps_tmr_val_crr > preload+1000){
          preload= preload_flock -4;      
      }
      else if(gps_tmr_val_crr > preload+30){
          preload= preload_flock -1;      
      }
      else{
          preload= preload_flock;
      }
    }

    // phase lock of simulated PPS
    if(flag_plock_ready){
      
    }
    
    flag_write_serial2 =true;
    // pseudo code for pps fine syncing
    // 1. check the correct preload amount to use based on latest pps cycle.
    // 2. do sanity check to make sure its not an outlier and report. 
    // 3. check if the clock is ahead or behind
    // 4. if its ahead above thresh slow down
    // 5. if its lagging above thresh speed up
    // 6. if its within tolerance set to latest pps period


    // pseudo code for pps coarse syncing
    // 1. Wait for  fine syncing
    // 2. check the correct period amount (should be 100)
    // 3. if its ahead above thresh slow down
    // 5. if its lagging above thresh speed up
    // 6. if its within tolerance set to latest pps period
  }
  else{
    CLR(PORTF,RTK_LED_PIN);
  }
}

// Inturupt service routine - timer overflow inturupt 100Hz
ISR(TIMER4_OVF_vect)        // interrupt service routine that wraps a user defined function supplied by attachInterrupt
{
  cam_pulse_count++;
  cam_pulse_count_for_GPRMC++;
  pps_width_count++;
  gps_pulse_count++;
  flag_pps_sim_high = READ2(PORTJ,CAM_SYNC_PIN);
  TOGGLE(PORTJ,CAM_SYNC_PIN); // 25Hz pulse width 50%
  flag_pps_sim_high = !flag_pps_sim_high;
  TCNT4 = preload;

 /* if(cam_pps_correction>40){
      cam_pps_preload = -20;
  }
  else if(cam_pps_correction>20){
      cam_pps_preload = -10;
  }
  else if(cam_pps_correction>5){
      cam_pps_preload = -3;
  }
  else if(cam_pps_correction<-40){
      cam_pps_preload = 20;
  }
  else if(cam_pps_correction<-20){
      cam_pps_preload = 10;
  }
  else if(cam_pps_correction<-5){
      cam_pps_preload = 3;
  }
  else {
       cam_pps_preload = 0;
  }*/

  if(cam_pulse_count>=cam_pps_preload+50 && flag_pps_sim_high){ //50 => 1s
    SET(PORTJ,SIM_PPS_PIN);
    SET(PORTA,JETSON_RST_PIN);
    CLR(PORTA,JETSON_REC_PIN);
    //digitalWrite(JETSON_RST,HIGH);
    //digitalWrite(JETSON_REC,LOW);
    digitalWrite(PPS_LED,HIGH); //IMU Sync LED on
    cam_pulse_count = 0;
    pps_width_count = 0;
    flag_pps_sim_high = true;
  }

  if(flag_pps_sim_high && pps_width_count>=cam_pps_preload+19){  // 19 =>380ms
    CLR(PORTJ,SIM_PPS_PIN);
    CLR(PORTA,JETSON_RST_PIN);
    SET(PORTA,JETSON_REC_PIN);
    //digitalWrite(JETSON_RST,LOW);
    //digitalWrite(JETSON_REC,HIGH);
    digitalWrite(PPS_LED,LOW); //IMU Sync LED on
    pps_width_count = 0;
    flag_pps_sim_high = false;
  }

  if(cam_pulse_count_for_GPRMC>=50 && flag_pps_sim_high){ //writes serial every second
    if(ss<59){
      ss++;
		}else{
      ss=0;
      if(mm<59){
				 mm++;
			 }else{
         mm=0;
         if(hh<23){
           hh++;
           }else{
             hh=0;
             }
			 }
		}
    cam_pulse_count_for_GPRMC=0; 
    flag_write_serial = true;


    //PPS LOCK handling
    /*if(plock_ready){
        

    }*/
  }
}

unsigned char result;
int i;
int checkNum(const char *gprmcContext)
{
    if (gprmcContext == NULL) 
    {
        // printf("Input is NULL.\n");
        return -1;
		}

    result = gprmcContext[1];

    for (i = 2; gprmcContext[i] != '*' && gprmcContext[i] != '\0'; i++)
    {
        // printf("Processing character: %c (ASCII: %d)\n", gprmcContext[i], gprmcContext[i]);
        result ^= gprmcContext[i];
    }

    if (gprmcContext[i] != '*') 
    {
        // printf("No '*' found in the string.\n");
        return -1;
    }

    //printf("Final result before returning: %02X\n", result);
    return result;
}


void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '$';
    char endMarker = '\n';
    char rc;

    while (Serial1.available() > 0 && newData == false) {
        rc = Serial1.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
            recvInProgress = true;
            receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
        }
    }
}

//============

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index
    byte crc_calc = 0;
    byte crc_recv = 0;
    char crc_str[] = "4C";
    
    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    if (strcmp(strtokIndx,"$GNRMC")==0){
      strcpy(messageFromPC, strtokIndx);
      strcat(messageFromPC, ",");
      strtokIndx = strtok(NULL, "\n");
      strcat(messageFromPC, strtokIndx); // copy it to messageFromPC
      
      //CRC calculation
      if(messageFromPC[strlen(messageFromPC)-4]=='*'){
        crc_calc= convertToCRC(messageFromPC);
        crc_str[0] = messageFromPC[strlen(messageFromPC)-3];
        crc_str[1] = messageFromPC[strlen(messageFromPC)-2]; 
        //Serial.println(crc_calc);
        //crc_str = strcat(messageFromPC[strlen(messageFromPC)-2],messageFromPC[strlen(messageFromPC)-1]);
        crc_recv = strtol(crc_str, NULL, 16);
        if(crc_recv!=crc_calc){
          memset(messageFromPC, 0,strlen(messageFromPC));
          Serial.println("Checksum error");
          } 
        }
        else{
          memset(messageFromPC, 0,strlen(messageFromPC));   
          Serial.println("No * in NMEA");
        }
    }
    else{
      if (strcmp(strtokIndx,"$GNGGA")==0){
        strcpy(messageFromPC, strtokIndx);
        strcat(messageFromPC, ",");
        strtokIndx = strtok(NULL, "\n");
        strcat(messageFromPC, strtokIndx); 
        Serial.println(messageFromPC);
        memset(messageFromPC, 0,strlen(messageFromPC));
      }
      else {
          memset(messageFromPC, 0,strlen(messageFromPC));
      }   
    }
    /*strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    integerFromPC = atoi(strtokIndx);     // convert this part to an integer

    strtokIndx = strtok(NULL, ",");
    floatFromPC = atof(strtokIndx);     // convert this part to a float
*/
}

//============

void showParsedData() {
  if (strlen(messageFromPC)>0 && flag_send_next_nema){
    //Serial.print("Message ");
    //Serial2.println(messageFromPC);
    Serial.println(messageFromPC);
    //flag_send_next_nema = false;
    //flag_send_next_nema_delayed = true;
    //modifyNEMAData();
    //strcpy(messageToLidar,messageFromPC);   
  }
}

byte convertToCRC(char *buff) {
  // NMEA CRC: XOR each byte with previous for all chars between '$' and '*'
  char c;
  byte i;
  byte start_with = 0;
  byte end_with = 0;
  byte crc = 0;

  for (i = 0; i < buff_len; i++) {
    c = buff[i];
    if(c == '$'){
      start_with = i;
    }
    if(c == '*'){
      end_with = i;
    }      
  }
  if (end_with > start_with){
    for (i = start_with+1; i < end_with; i++){ // XOR every character between '$' and '*'
      crc = crc ^ buff[i] ;  // compute CRC
    }
  }
  else { // else if error, print a msg (to both ports)
    Serial.println("CRC ERROR");
    //SERIALN.println("CRC ERROR");
  }
  return crc;
  //based on code by Elimeléc López - July-19th-2013
}

