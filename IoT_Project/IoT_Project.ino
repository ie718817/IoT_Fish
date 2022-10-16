#include <Servo.h>
//#include "semphr.h" //Sempahore library for freertos

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

////////////// Definitions ////////////////////////////////////

#define LDR_PIN  15
#define T_Sensor_PIN 27
#define RELE_PIN 4
#define PH_SENSOR_PIN 33
#define SERVO_PIN 2

//I2C pins
#define SDA_PIN 21
#define SCL_PIN 22
#define SLAVE_ADDR 0x05

#define LDR_REF_RESISTOR 10000 //value for reference resistor in ohms

#define ADC_mV 5000
#define ADC_Res 4096
#define MAX_VALUE_PH  3475    //Max possible value obtained from an ADC read

#define  sensors_buffer_size  3 //buffer size for sensors to be stored
#define  LDR_buffer_position 0
#define  Temperature_buffer_position 1




////////////////////// Tasks /////////////////////////
void Task_LDR_Read( void *pvParameters );
void Task_Read_Temperature( void *pvParameters );
void Task_Rele( void *pvParameters );
void Task_pH( void *pvParameters );
void Task_feed_fish( void *pvParameters );


///////////////////// Global Variables //////////////////////////////////
volatile static char Global_variable_buffer[sensors_buffer_size] = {0};
SemaphoreHandle_t rele_semaphore = NULL;     // Waits for parameter to be read
SemaphoreHandle_t feed_fish = NULL;


void setup() {
  //make a semaphore for signal to activate a relé or feed the fish
  rele_semaphore = xSemaphoreCreateBinary();
  feed_fish = xSemaphoreCreateBinary();
  
  // initialize serial communication at 115200 bits per second:
  Serial.begin(9600);

  // Task to run forever
  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
              Task_LDR_Read,  // Function to be called
              "Read LDR",   // Name of task
              1024,         // Stack size (bytes in ESP32, words in FreeRTOS)
              NULL,         // Parameter to pass to function
              1,            // Task priority (0 to configMAX_PRIORITIES - 1)
              NULL,         // Task handle
              ARDUINO_RUNNING_CORE);     // Run on one core for demo purposes (ESP32 only)

  // Task to run forever
  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
              Task_Read_Temperature,  // Function to be called
              "Temperature task",   // Name of task
              2048,         // Stack size (bytes in ESP32, words in FreeRTOS)
              NULL,         // Parameter to pass to function
              1,            // Task priority (0 to configMAX_PRIORITIES - 1)
              NULL,         // Task handle
              ARDUINO_RUNNING_CORE);     // Run on one core for demo purposes (ESP32 only)
              
  // Task to run forever
  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
              Task_Rele,  // Function to be called
              "Rele task",   // Name of task
              2048,         // Stack size (bytes in ESP32, words in FreeRTOS)
              NULL,         // Parameter to pass to function
              1,            // Task priority (0 to configMAX_PRIORITIES - 1)
              NULL,         // Task handle
              ARDUINO_RUNNING_CORE);     // Run on one core for demo purposes (ESP32 only)

    // Task to run forever
  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
              Task_pH,  // Function to be called
              "Read ph",   // Name of task
              2048,         // Stack size (bytes in ESP32, words in FreeRTOS)
              NULL,         // Parameter to pass to function
              1,            // Task priority (0 to configMAX_PRIORITIES - 1)
              NULL,         // Task handle
              ARDUINO_RUNNING_CORE);     // Run on one core for demo purposes (ESP32 only)
              

  // Task to run forever
  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
              Task_feed_fish,  // Function to be called
              "Feed the fish in the tank",   // Name of task
              2048,         // Stack size (bytes in ESP32, words in FreeRTOS)
              NULL,         // Parameter to pass to function
              1,            // Task priority (0 to configMAX_PRIORITIES - 1)
              NULL,         // Task handle
              ARDUINO_RUNNING_CORE);     // Run on one core for demo purposes (ESP32 only)

  // If this was vanilla FreeRTOS, you'd want to call vTaskStartScheduler() in
  // main after setting up your tasks.
}

void loop() {
  // Do nothing
  // setup() and loop() run in their own task with priority 1 in core 1
  // on ESP32
}


/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/


void Task_LDR_Read( void *pvParameters )
{
    (void) pvParameters;
/*
  AnalogReadSerial for LDR
  Reads an analog input on pin LDR in defines and stores the value in a global arry for later be printed in a serial terminal
  This example code is in the public domain.
*/

  for (;;)
  {
    // read the input on analog pin LDR_PIN:
    int LDR_a_read_value = analogRead(LDR_PIN);
    //Process data 

    //pre proces data
    int light = map(LDR_a_read_value, 0, ADC_Res, 0, 100);
    
 
    // print the temperature in the Serial Monitor:
    Serial.print("LDR_Value: ");
    Serial.print(LDR_a_read_value );   // print the temperature in °C
    Serial.print("LDR_brightness: ");
    Serial.print(light );   // brightness
    Serial.println("end LDR");
    
    vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
  }
}

void Task_Read_Temperature( void *pvParameters )
{
      (void) pvParameters;
/*
  AnalogReadSerial for LM35 temperature sensor
  Reads an analog input on pin T_sensor in defines and stores the value in a global arry for later be printed in a serial terminal
  This example code is in the public domain.
*/
  for (;;)
  {
    // read the input on analog pin LDR_PIN:
    int Temperature_a_read_value = analogRead(T_Sensor_PIN);
    //Process data 
    float volt = Temperature_a_read_value * (ADC_mV/ADC_Res);
    float Temp = volt/10;

    //store data
    
    // print the temperature in the Serial Monitor:
    Serial.print("Temperature: ");
    Serial.print(Temp);   // print the temperature in °C
    Serial.println("°C");
    vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
  }
}

void Task_Rele( void *pvParameters )
{
    pinMode(RELE_PIN, OUTPUT);
    for (;;)
  {
      digitalWrite(RELE_PIN, LOW);
      Serial.println("Current Flowing in Rele");
      vTaskDelay(300);  // one tick delay (15ms) in between reads for stability 
      
      // Normally Open configuration, send HIGH signal stop current flow
      // (if you're usong Normally Closed configuration send LOW signal)
      digitalWrite(RELE_PIN, HIGH);
      Serial.println("Current not Flowing in rele");
      vTaskDelay(300);  // one tick delay (15ms) in between reads for stability

    
  }
}

void Task_feed_fish( void *pvParameters )
{
    //initialize pwm pin for servo
    static Servo myservo;
    static int pos = 0;    // variable to store the servo position
    myservo.attach(SERVO_PIN);
    for (;;)
  {
      for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      vTaskDelay(10);  // one tick delay (15ms) in between reads for stability
    }
    for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
      myservo.write(pos);              // tell servo to go to position in variable 'pos'
      vTaskDelay(10);  // one tick delay (15ms) in between reads for stability
    }
  }
}

void Task_pH( void *pvParameters )
{
      (void) pvParameters;
/*
  AnalogReadSerial for LM35 temperature sensor
  Reads an analog input on pin T_sensor in defines and stores the value in a global arry for later be printed in a serial terminal
  This example code is in the public domain.
*/

  for (;;)
  {
    // read the input on analog pin LDR_PIN:
    static int pH_a_read_value = analogRead(PH_SENSOR_PIN);
    //Process data 

    static int water_quality = map(pH_a_read_value, 0, MAX_VALUE_PH, 100, 0);

    //store data
     
    // print the temperature in the Serial Monitor:
       
    Serial.print("Calidad agua: ");
    Serial.println(water_quality);
    
    vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
  }
}
