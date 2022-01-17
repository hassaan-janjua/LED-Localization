#include <Rotap.h>
#include <avr/wdt.h>
/* 16/01/2020 16:00:00 */
/********************************************************************/
/* Serial communication protocol:                                   */
/********************************************************************/
/* -----------------------------                                    */
/* VersaSense -> SleepyPi                                           */
/* -----------------------------                                    */
/*                                                                  */
/* ---------------------                                            */
/* | Code    | Data    |                                            */
/* ---------------------                                            */
/* | 2 Byte  | 30 Byte |                                            */
/* ---------------------                                            */
/*                                                                  */
/* Length: 32 Bytes                                                 */
/* Code  : ASCII                                                    */
/*         00  -> RPi Shutdown                                      */
/*         01  -> RPi Power up                                      */
/*         02  -> RPi WiFi Off                                      */
/*         03  -> RPi WiFi On                                       */
/*         04  -> Sleepy Pi Reboot                                  */
/*         05  -> Sleepy Pi Heartbeat                               */
/*                                                                  */
/********************************************************************/
/* ----------------------                                           */
/* SleepyPi -> RPi                                                  */
/* ----------------------                                           */
/*                                                                  */
/* --------------------                                             */
/* | Code   | Data    |                                             */
/* --------------------                                             */
/* | 2 Byte | 30 Byte |                                             */
/* --------------------                                             */
/*                                                                  */
/* Length: Variable                                                 */
/* Code:   ASCII                                                    */
/*         00[Windows size left or -1] -> ACK                       */
/*         01[Arbitrary data]          -> Data                      */
/*         0102                        -> WiFi Off                  */
/*         0103                        -> WiFi On                   */
/*         0104                        -> Sleepy Pi Reboot          */
/*         0105                        -> Sleepy Pi Heartbeat       */
/*                                                                  */
/********************************************************************/
/* ----------------------                                           */
/* RPi -> SleepyPi                                                  */
/* ----------------------                                           */
/*                                                                  */
/* -----------------------------------------------------            */
/* Low Battery Indicator |   ID    |    X    |    Y    |            */
/* -----------------------------------------------------            */
/* |       1 bit         | 15 bits | 2 Bytes | 2 Bytes |            */
/* -----------------------------------------------------            */
/*                                                                  */
/* ID Range: 1 - 32766                                              */
/* X/Y Unit: Deci-Meter with respect to real-world origin.          */
/*                                                                  */
/********************************************************************/
/*                                                                  */
/* SleepyPi -> VersaSense                                           */
/* ------------------------------------------------------------     */
/* | Current | Power On | PiStatus | SleepyPi Ticks | Data    |     */
/* ------------------------------------------------------------     */
/* | 1 Byte  | 1 bit    | 1 bit    | 6 bits         | 30 Byte |     */
/* ------------------------------------------------------------     */
/*                                                                  */
/* Data is same as sent by RPi                                      */
/*                                                                  */
/********************************************************************/ 

#define ENABLE_PI_PWR_PIN       16        // PC2 - O/P take high to enable the RaspPi - Active High
#define ENABLE_EXT_PWR_PIN      4         // PD4 - O/P take high to enable the External Supplies
#define CMD_PI_TO_SHDWN_PIN     17        // PC3 - 0/P Handshake to request the Pi to shutdown - Active high
#define PI_IS_RUNNING           7         // PD7 - I/P Handshake to show that the Pi is running - Active High
#define V_SUPPLY_PIN            A6        // I/P - A/I Supply monitoring pin
#define I_MONITOR_PIN           A7        // I/P - A/I Current monitoring pin
#define POWER_BUTTON_PIN        3         // PD3 - I/P User Power-on Button (INT1) - Active Low
#define ALARM_PIN               2         // PD2 - I/P Pin that pulses when the alarm has expired (INT0) - Active Low
#define kFAILSAFETIME_MS        1000      // Failsafe shutdown time in milliseconds
#define kONBUTTONTIME_MS        3000      //
#define kFORCEOFFBUTTONTIME_MS  1000
#define RPI_REBOOT_TIMEOUT      900000UL  // 15 Minutes - Time since last message from RPi
#define SLEEPYPI_UPTIME_COUNTER 30000UL   // 30 Seconds - Incremenent uptime counter every 30 seconds
#define SLEEPYPI_REBOOT_TIMEOUT 3600000UL // 60Minutes - Time since last message from VSMote

#define DEBUG_MESSAGES          1

// Buffers 
uint8_t to_mote[32];
uint8_t from_rpi[32];
uint8_t to_rpi[64];


uint8_t uptime;
uint8_t packet_position;
uint8_t window_size;

bool          sampled                         = false;

unsigned long uptime_track_time               = 0;
unsigned long pi_reboot_heartbeat_time        = 0;
unsigned long sleepypi_reboot_heartbeat_time  = 0;

unsigned long shutdown_cmd_time               = 0;
bool          shutdown_cmd                    = false;

// Reset flags
bool simulationMode       = false;
bool simPiOn              = false;
bool pi_running           = false;
bool power_on             = false;
bool ext_power_on         = false;
bool force_sleepypi_reset = false;
bool force_rpi_reset      = false;
bool is_wdt_enabled       = false;

// Init ROTAP with; URI, get() and put() handlers
void get(uint8_t *size, uint8_t *buf);
void put(uint8_t size, uint8_t *buf);
void send_message_to_pi(int type, char *msg, int msg_len);

Rotap rotap("9999/2001", get, put);

// Setup serial port, ROTAP, Software I2C.
void setup(void)
{
  // Setup serial port
  Serial.begin(9600);
  Serial.println("02 ");
  Serial.println("02 Serial Initialised");

  enablePiPower(true);

  // Setup ROTAP
  rotap.start();
  Serial.println("02 ROTAP Initialised");
  Serial.println("02 VersaSense Pi Connector");
  Serial.println("02 Version 1.0");
  pinMode(9, OUTPUT);
  pinMode(10, INPUT);


  force_rpi_reset = false;
  force_sleepypi_reset = false;
  
  //Delay is required for ROTAP start-up
  delay(3000);
  
  //wdt_enable(WDTO_8S);
  is_wdt_enabled = false;
  pi_reboot_heartbeat_time = millis();
  sleepypi_reboot_heartbeat_time = pi_reboot_heartbeat_time;
  uptime_track_time = pi_reboot_heartbeat_time;
}

void loop()
{
  uint8_t       current;
  unsigned long current_time;
  
  delay(50);

  // Check if data was requested
  if (digitalRead(10) == 1)
  {
    sampled = true;
    sleepypi_reboot_heartbeat_time = millis();
    rotap.populateData(32, (uint8_t *)to_mote);
    
    // Set pin 9 to high to signal that the data has been sent
    digitalWrite(9, HIGH);
  }
  else
  {
    digitalWrite(9, LOW);
    if (sampled)
    {
      // Blank to_mote and reset packet_position
      memset(to_mote, 0, 32);
      current = rpiCurrent()/4;
      to_mote[0] = current;
      to_mote[1] = (power_on << 7) | (checkPiStatus(false) << 6) | uptime & 0x3F;
      packet_position = 2;
      sampled = false;
#if DEBUG_MESSAGES
      send_message_to_pi(5, " Data Sent to mote", strlen(" Data Sent to mote"));
#endif    
    }
  }

  current_time = millis();
  
  if (power_on == false) 
  {
#if DEBUG_MESSAGES
    send_message_to_pi(4, " force rpi turn on", strlen(" force rpi turn on"));
#endif
    pi_reboot_heartbeat_time = current_time;
    power_on = true;
    enablePiPower(true);
  }
  
  if (shutdown_cmd) 
  {
    finish_shutdown();
  }
  
  receiveData();
  
  current_time = millis();

  /* Integer overflow */
  if (current_time < uptime_track_time)
  {
    uptime_track_time = current_time;
  }
  
  if ((current_time - uptime_track_time) > SLEEPYPI_UPTIME_COUNTER) /* update every 30seconds. */
  {
    uptime++;
    uptime_track_time = current_time;
  }  

  /* Integer overflow */
  if (current_time < pi_reboot_heartbeat_time) 
  {
    pi_reboot_heartbeat_time = current_time;    
#if DEBUG_MESSAGES
    send_message_to_pi(4, " Integer overflow pi_reboot_heartbeat_time", strlen(" Integer overflow pi_reboot_heartbeat_time"));
#endif    
  }
  
  /* If more than timeout time has passed without receiving any to_mote */
  if (force_rpi_reset || ((current_time - pi_reboot_heartbeat_time)  > RPI_REBOOT_TIMEOUT)) /* Thresh is 15 minutes. */
  {
#if DEBUG_MESSAGES
    send_message_to_pi(4, " init_pi_shutdown", strlen(" init_pi_shutdown"));
#endif    
    /* Reset the pi reboot time. */
    force_rpi_reset = false;
    pi_reboot_heartbeat_time = current_time;
    init_pi_shutdown();
  }
  
  /* Integer overflow */
  if (current_time < sleepypi_reboot_heartbeat_time) 
  {
    sleepypi_reboot_heartbeat_time = current_time;    
#if DEBUG_MESSAGES
    send_message_to_pi(4, " Integer overflow sleepypi_reboot_heartbeat_time", strlen(" Integer overflow sleepypi_reboot_heartbeat_time"));
#endif    
  }
  
  /* If more than timeout time has passed without receiving any packet */
  if (force_sleepypi_reset || ((current_time - sleepypi_reboot_heartbeat_time)  > SLEEPYPI_REBOOT_TIMEOUT)) /* Thresh is 60 minutes. */
  {
#if DEBUG_MESSAGES
    send_message_to_pi(4, " Preparing to restart sleepy pi", strlen(" Preparing to restart sleepy pi"));
#endif    
    // Do not reset the watchdog
    // If watchdog is not reseted in 8 seconds, it will reboot the sleepypi
  }
  else 
  {
    // Everything is good, reset the watchdog
    if (is_wdt_enabled)
    {
      wdt_reset();
    }
    else 
    {
#if DEBUG_MESSAGES
      send_message_to_pi(4, " wdt not enabled", strlen(" wdt not enabled"));
#endif    

    }
  }
  
}

// Reads a block of data from the serial port
void receiveData()
{
  bool          written           = false;
  bool          overrun           = false;
  bool          complete_packet   = true;
  bool          can_wait_more     = false;
  uint8_t       from_rpi_index = 0;
  
  uint8_t       ack_buffer[6];
  uint8_t       incomingByte;
  unsigned long start_time;
  unsigned long current_time;
  
  start_time = millis();
  
  while (Serial.available() > 0 || !complete_packet || can_wait_more)
  {
    if (Serial.available() > 0) 
    {
      incomingByte = (uint8_t)Serial.read();
      if (from_rpi_index < 32)
      {
        from_rpi[from_rpi_index] = incomingByte;
        from_rpi_index++;
      }
      else
      {
        overrun = true;
      }
      if (from_rpi_index == 6) 
      {
        // Ignore heartbeat message
        if ((*((uint16_t*)from_rpi) == ((uint16_t)0xFFFF))) 
        {
          // Signal from RPi to reboot the sleepypi
          if ((*((uint16_t*)(from_rpi+2)) == ((uint16_t)0x0001))) 
          {
            force_sleepypi_reset = true;
            
            /* Init proper RPi Shutdown */
            force_rpi_reset = true;
#if DEBUG_MESSAGES
            send_message_to_pi(3, " received force_sleepypi_reset", strlen(" received force_rpi_reset"));
#endif   
          }
          // Signal from RPi to reboot the RPi
          else if ((*((uint16_t*)(from_rpi+2)) == ((uint16_t)0x0002))) 
          {
            force_rpi_reset = true;
#if DEBUG_MESSAGES
            send_message_to_pi(3, " received force_rpi_reset", strlen(" received force_rpi_reset"));
#endif   
          }
          // Signal from RPi to disable wdt
          else if ((*((uint16_t*)(from_rpi+2)) == ((uint16_t)0x0003))) 
          {
            wdt_disable();
            is_wdt_enabled = false;
#if DEBUG_MESSAGES
            send_message_to_pi(3, " received wdt disable", strlen(" received wdt disable"));
#endif   
          }
          // Signal from RPi to enable wdt
          else if ((*((uint16_t*)(from_rpi+2)) == ((uint16_t)0x0004))) 
          {
            wdt_enable(WDTO_8S);
            is_wdt_enabled = true;
#if DEBUG_MESSAGES
            send_message_to_pi(3, " received wdt enable", strlen(" received wdt enable"));
#endif   
          }
          from_rpi_index = 0;
        }
      }

      written = (from_rpi_index > 0);
      pi_reboot_heartbeat_time = millis();
    }
    else if (complete_packet) 
    {
      break;
    }
    
    complete_packet = ((from_rpi_index % 6) == 0);
    current_time = millis();
    
    if ((current_time - start_time) > 50) 
    {
      can_wait_more = false;
    }
    delay(1);
  }
  

  // Only send forward if received complete 6 bytes packets
  if (written && ((from_rpi_index % 6) == 0))
  {
    if (overrun || from_rpi_index > (32 - packet_position))
    {
      ack_buffer[0] = '-';
      ack_buffer[1] = '1';
    }
    else
    {
      // Append data to sensor-buffer
      memcpy(to_mote+packet_position, from_rpi, from_rpi_index);
      
      // calculate ack
      packet_position += from_rpi_index;
      window_size = 32 - packet_position;
      
      ack_buffer[0] = (window_size/10) + '0';
      ack_buffer[1] = (window_size%10) + '0';
    }
    send_message_to_pi(0, (char*)ack_buffer, 2);
  }
  
}

// GET function: no longer used
void get(uint8_t *size, uint8_t *buf)
{
}

// PUT function: from_mote contains payload received
void put(uint8_t size, uint8_t *from_mote)
{
  uint8_t sz = (size < 32)?size:32;

  if (sz >= 2 && from_mote[0] == '0')
  {
    /* 00: RPi Shutdown */
    if (from_mote[1] == '0')
    {
      init_pi_shutdown();
    }
    /* 01: RPi Power up */
    else if (from_mote[1] == '1')
    {
      enablePiPower(true);
    }
    /* 04: SleepyPiRestart */
    else if (from_mote[1] == '4') 
    {
      init_pi_shutdown();
      force_sleepypi_reset = true;
    }
  }
  
  sleepypi_reboot_heartbeat_time = millis();

  /* Forward the message to RPi */
  send_message_to_pi(1, (char*)from_mote, sz);
}

void init_pi_shutdown() 
{
  digitalWrite(CMD_PI_TO_SHDWN_PIN,HIGH);
  shutdown_cmd_time = millis();
  shutdown_cmd = true;
}

void finish_shutdown() 
{
  unsigned long current_time;
  int           handShake;
  
  handShake = digitalRead(PI_IS_RUNNING);
  current_time = millis();
  if ((current_time - shutdown_cmd_time) > kFAILSAFETIME_MS || handShake == 0 ) 
  {
    enablePiPower(false);
    digitalWrite(CMD_PI_TO_SHDWN_PIN,LOW);
    shutdown_cmd = false;
  }
}

void enablePiPower(bool enable)
{
  if(simulationMode == true)
  {
    if(enable == true)
    {
      // Turn on the Pi
      // digitalWrite(ENABLE_PI_PWR_PIN,HIGH);
      power_on = true;
    }
    else
    {
      // Turn off the Pi
      // digitalWrite(ENABLE_PI_PWR_PIN,LOW);
      power_on = false;
    }
  }
  else {
    if(enable == true)
    {
      // Turn on the Pi
      digitalWrite(ENABLE_PI_PWR_PIN,HIGH);
      power_on = true;
    }
    else
    {
      // Turn off the Pi
      digitalWrite(ENABLE_PI_PWR_PIN,LOW);
      power_on = false;
    }
  }
  return;
}

float rpiCurrent(void)
{
  int     reading;
  float   current;

  // Read
  reading = analogRead(I_MONITOR_PIN);
  // remove lower bit noise
  if(reading <= 3)
  {
    reading = 0;
  }
  // Convert
  current =  3.22 * (float)reading;     // Raw current reading
  // 10-bit ADC resolution = 3.3 / 1024 = 3.22mV

  return  current;             // in mA

}

bool checkPiStatus(bool forceShutdownIfNotRunning)
{
  int  handShake;

  if(simulationMode == true)
  {
    handShake = power_on;
    pi_running = true;
  }
  else
  {
    handShake = digitalRead(PI_IS_RUNNING);
  }

  if(handShake > 0)
  {
    // RasPi is still running
    pi_running = true;
    return true;
  }
  else
  {
    // Pi not handshaking - either booting or manually shutdown
    if(forceShutdownIfNotRunning == true)
    {
      // Pi not running - has it been on?
      if(pi_running == true)
      {
        // Pi has been running and now isn't
        // so cut the power
        enablePiPower(false);
        pi_running = false;
      }
    }
    return false;
  }

}

void send_message_to_pi(int type, char *msg, int msg_len)
{
  to_rpi[0] = '0' + type/10;
  to_rpi[1] = '0' + type%10;
  memcpy(to_rpi+2, msg, msg_len);  
  to_rpi[msg_len + 2] = ' ';
  to_rpi[msg_len + 3] = 'V';
  to_rpi[msg_len + 4] = '7';
  to_rpi[msg_len + 5] = '\r';
  to_rpi[msg_len + 6] = '\n';
  
  Serial.write((uint8_t*)to_rpi, msg_len+7);
  
}
