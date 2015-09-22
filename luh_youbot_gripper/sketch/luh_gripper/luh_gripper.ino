#include <RunningMedian.h>

//#define READ_POSITION 0
//#define READ_EFFORT 1
//#define SET_POSITION 2
//#define SET_EFFORT 3

#define READ_GRIPPER_STATE 0
#define SET_POSITION 2
#define SET_EFFORT 4
#define SET_RELATIVE_POSITION 8

#define LEFT_SERVO 3
#define RIGHT_SERVO 5

#define MIN_ANGLE -950.0
#define MAX_ANGLE 950.0

#define MIN_PULSE 600.0
#define MAX_PULSE 2300.0

#define MIN_POS_FEEDBACK 188.0
#define MAX_POS_FEEDBACK 1916.0

#define POSITION_FEEDBACK 31
#define FORCE_FEEDBACK 90
#define ZERO_EFFORT 1100

#define TIMEOUT 500

#define EFFORT_BUFFER_SIZE 11
#define POSITION_BUFFER_SIZE 21

#define CYCLE_TIME 10000 // micro seconds

#define POSITION_STEP 8

#define LED_PIN 13

short task;
short response[8];
short request[2];
short max_effort[2];
short position_command[2];
short positions[2];
short efforts[2];
short velocities[2];
short current_position[2];
//short effort_buffer_left[EFFORT_BUFFER_SIZE];
//short effort_buffer_right[EFFORT_BUFFER_SIZE];
//short buffer_index = 0;
//float gauss_kernel[11] = {0.1995, 0.1760, 0.1210, 0.0648, 0.0270, 0.0088, 0.088, 0.027, 0.0648, 0.1210, 0.1760};
short usb_voltage;

unsigned long led_time = 0;
byte led_state = HIGH;
boolean position_reached = false;

RunningMedian effort_buffer_left = RunningMedian(EFFORT_BUFFER_SIZE);
RunningMedian effort_buffer_right = RunningMedian(EFFORT_BUFFER_SIZE);
RunningMedian position_buffer_left = RunningMedian(POSITION_BUFFER_SIZE);
RunningMedian position_buffer_right = RunningMedian(POSITION_BUFFER_SIZE);

//########## INITIALISE ############################################
void initialise()
{  
  // init everything with 0
  request[0] = 0;
  request[1] = 0; 
  max_effort[0] = 10000;
  max_effort[1] = 10000;
  position_command[0] = 0;
  position_command[1] = 0; 
  positions[0] = 0;
  positions[1] = 0;
  efforts[0] = 0;
  efforts[1] = 0;
  velocities[0] = 0;
  velocities[1] = 0;
  current_position[0] = 0;
  current_position[1] = 0;
  
  for(int i=0; i<6; i++)
    response[i] = 0;

  setPosition();
  delay(10);
}

//########## SETUP ############################################
void setup()  
{  
      // Open serial communications and wait for port to open:
  Serial.begin(57600);
  while (!Serial) 
  {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, led_state);
  led_time = micros();  

  initialise();

}

//########## FEEDBACK TO ANGLE ###############################
short feedbackToAngle(short feedback)
{
    return (feedback - MIN_POS_FEEDBACK) * (MAX_ANGLE - MIN_ANGLE) / (MAX_POS_FEEDBACK - MIN_POS_FEEDBACK) + MIN_ANGLE;
    //return feedback;
}

//########## ANGLE TO PULSE ###############################
short angleToPulse(short angle)
{ 
    return (short)(((float)angle - MIN_ANGLE) * (MAX_PULSE - MIN_PULSE) / (MAX_ANGLE - MIN_ANGLE) + MIN_PULSE);
    //return angle;
}

//########## SET POSITION ####################################
void setPosition()
{
   short pulse_left = angleToPulse(current_position[0]);
   short pulse_right = angleToPulse(current_position[1]);  
  
   if(pulse_left >= MIN_PULSE && pulse_right >= MIN_PULSE && pulse_left <= MAX_PULSE && pulse_right <= MAX_PULSE)
   {
     
   pinMode(LEFT_SERVO, OUTPUT);
   pinMode(RIGHT_SERVO, OUTPUT);
   digitalWrite(LEFT_SERVO, LOW);
   digitalWrite(RIGHT_SERVO, LOW);   
   delay(1);
   
   if(pulse_left == pulse_right)
   {
       digitalWrite(LEFT_SERVO, HIGH);
       digitalWrite(RIGHT_SERVO, HIGH);
              
       delayMicroseconds(pulse_right);
       digitalWrite(RIGHT_SERVO, LOW);
       digitalWrite(LEFT_SERVO, LOW);
   }
   else if(pulse_left < pulse_right)
   {
       digitalWrite(LEFT_SERVO, HIGH);
       digitalWrite(RIGHT_SERVO, HIGH);
       
       delayMicroseconds(pulse_left);
       digitalWrite(LEFT_SERVO, LOW);
              
       delayMicroseconds(pulse_right - pulse_left);
       digitalWrite(RIGHT_SERVO, LOW);
   }
   else
   {
       digitalWrite(LEFT_SERVO, HIGH);
       digitalWrite(RIGHT_SERVO, HIGH);
       
       delayMicroseconds(pulse_right);
       digitalWrite(RIGHT_SERVO, LOW);
              
       delayMicroseconds(pulse_left - pulse_right);
       digitalWrite(LEFT_SERVO, LOW);
   }  
   }
}

//########## GET FEEDBACK ####################################
short getFeedback(short pin, short pulse_width)
{
   // send pulse for feedback
   pinMode(pin, OUTPUT);
   digitalWrite(pin, LOW);
   delay(1);
   digitalWrite(pin, HIGH);   
   delayMicroseconds(pulse_width);
   digitalWrite(pin, LOW);  
   
   // read feedback signal
   pinMode(pin, INPUT);
   unsigned long start_time = micros();
   unsigned long low_time;
   
   while(digitalRead(pin) == LOW)
   {
      low_time = micros();
      
      if(low_time - start_time > TIMEOUT)
      {
         return 0;  
      }
   }
      
   while(digitalRead(pin) == HIGH)
   {;}

   return micros() - low_time;   
}

//########## GET POSITION ####################################
void getPosition()
{
     //positions[0] = feedbackToAngle(getFeedback(LEFT_SERVO, POSITION_FEEDBACK));
     //positions[1] = feedbackToAngle(getFeedback(RIGHT_SERVO, POSITION_FEEDBACK));
 
     position_buffer_left.add(feedbackToAngle(getFeedback(LEFT_SERVO, POSITION_FEEDBACK)));
     position_buffer_right.add(feedbackToAngle(getFeedback(RIGHT_SERVO, POSITION_FEEDBACK))); 
     
     short old_left_pos = positions[0];
     short old_right_pos = positions[1];
     
     positions[0] = position_buffer_left.getMedian();
     positions[1] = position_buffer_right.getMedian();
     
     velocities[0] = 1000000.0 * float((positions[0] - old_left_pos)) / CYCLE_TIME;
     velocities[1] = 1000000.0 * float((positions[1] - old_right_pos)) / CYCLE_TIME;     
}

//########## GET EFFORT ######################################
void getEffort()
{
     effort_buffer_left.add(getFeedback(LEFT_SERVO, FORCE_FEEDBACK) - ZERO_EFFORT);
     effort_buffer_right.add(getFeedback(RIGHT_SERVO, FORCE_FEEDBACK) - ZERO_EFFORT); 
     
     efforts[0] = effort_buffer_left.getMedian();
     efforts[1] = effort_buffer_right.getMedian();
  
  // save effort to buffer
//   effort_buffer_left[buffer_index] = getFeedback(LEFT_SERVO, FORCE_FEEDBACK) - ZERO_EFFORT;
//   effort_buffer_right[buffer_index] = getFeedback(RIGHT_SERVO, FORCE_FEEDBACK) - ZERO_EFFORT; 
   
   // filter buffer
//   float left_mean = 0;
//   float right_mean = 0;
//   int j = buffer_index;
//   for(int i=0; i<EFFORT_BUFFER_SIZE; i++)
//   {
//       left_mean += effort_buffer_left[j] * gauss_kernel[i];
//       right_mean += effort_buffer_right[j] * gauss_kernel[i];
//       
//       j = (j+1) % EFFORT_BUFFER_SIZE;
//   }
//   
//  
//   
//   efforts[0] = (short)left_mean;
//   efforts[1] = (short)right_mean;
//   
//   //efforts[0] = effort_buffer_left[buffer_index];
//   
//   buffer_index = (buffer_index + 1) % EFFORT_BUFFER_SIZE; 
}

//########## READ REQUEST ####################################
void readRequest()
{
    char * b = (char *) request;
    Serial.readBytes(b, 4);
}

//########## SEND RESPONSE ###################################
void sendResponse()
{    
     response[0] = positions[0] / 10;
     response[1] = positions[1] / 10;     
     response[2] = velocities[0] / 10;
     response[3] = velocities[1] / 10;
     response[4] = efforts[0];
     response[5] = efforts[1];  
     if(position_reached)
       response[6] = 255;
     else
       response[6] = 0;

     response[7]=usb_voltage; 
     byte * b = (byte *) response;
     Serial.write(b, 16);
}

//########## Calculate USB-Voltage ###################################
short readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  // ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1125300L / result /10; // Back-calculate AVcc in mV
  short result_short=result;
  //return 10;
  return result_short;
}



//########## LOOP ############################################
void loop() // run over and over
{ 
    unsigned long start_time = micros();
    
    // blink to indicate program is running
//    if(start_time - led_time > 100000)
//    {
//       led_time = start_time;
//      
//        if(led_state == LOW)
//        {
//          led_state = HIGH;
//        }
//        else
//        {
//          led_state = LOW;
//        }
//        
//        digitalWrite(LED_PIN, led_state);
//    }
    if(position_reached)
       digitalWrite(LED_PIN, HIGH);
    else
       digitalWrite(LED_PIN, LOW);
    
      // === HANDLE REQUESTS === 
    if(Serial.available())
    {
       task = Serial.read();

       if(task == READ_GRIPPER_STATE)
       {
           sendResponse();
       }
       else if(task == SET_POSITION)
       {
          readRequest();
          position_command[0] = request[0] * 10;
          position_command[1] = request[1] * 10; 
          current_position[0] = positions[0];
          current_position[1] = positions[1];
          position_reached = false;
          //setPosition(); // TODO: use contol loop instead        
       }
       else if(task == SET_EFFORT)
       {
          readRequest();
          max_effort[0] = request[0]; 
          max_effort[1] = request[1];           
       }
       else if(task == SET_RELATIVE_POSITION)
       {
          readRequest();
          position_command[0] = current_position[0] + request[0] * 10;
          position_command[1] = current_position[1] + request[1] * 10; 

          position_reached = false;
          
       }
    }
    
    // === READ STATE ===
    getPosition();
    getEffort();    
   
    // === CONTROLLER ===  
    
    if((fabs(efforts[0]) + fabs(efforts[1])) > (max_effort[0] + max_effort[1]))
      position_reached = true;
    
    if(!position_reached)
    {
        for(int i=0; i<2; i++)
        {  
            if(current_position[i] > position_command[i])
            {
               current_position[i] = max(current_position[i] - POSITION_STEP, position_command[i]); 
            }
            else if(current_position[i] < position_command[i])
            {
               current_position[i] = min(current_position[i] + POSITION_STEP, position_command[i]);
            }
        }
        
        if(current_position[0] != position_command[0] || current_position[1] != position_command[1])
          setPosition(); 
        else
          position_reached = true;
    }
   
   // === Calculate the USB-Voltage
   
   usb_voltage=readVcc();
   
   
   
   // === SLEEP ===
   unsigned long cycle_duration = micros() - start_time;
   
   if(cycle_duration < CYCLE_TIME)
   {
      delayMicroseconds(CYCLE_TIME - cycle_duration); 
   }

}

