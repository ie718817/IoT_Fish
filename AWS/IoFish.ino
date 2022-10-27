#include <Servo.h>
//#include "semphr.h" //Sempahore library for freertos

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

////////////// Definitions ////////////////////////////////////

#define LDR_PIN  15
#define T_Sensor_PIN 16
#define RELE_PIN 18
#define PH_SENSOR_PIN 14
#define SERVO_PIN 19

//I2C pins
#define SDA_PIN 21
#define SCL_PIN 22
#define SLAVE_ADDR 0x05

#define ADC_mV 3300
#define ADC_Res 4096

#define  sensors_buffer_size  3 //buffer size for sensors to be stored
#define  LDR_buffer_position 0
#define  Temperature_buffer_position 1




////////////////////// Tasks /////////////////////////
void Task_LDR_Read( void *pvParameters );
void Task_Serial_Print( void *pvParameters );
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
  Serial.begin(115200);

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
              Task_Serial_Print,  // Function to be called
              "Print data to serial terminal",   // Name of task
              2048,         // Stack size (bytes in ESP32, words in FreeRTOS)
              NULL,         // Parameter to pass to function
              2,            // Task priority (0 to configMAX_PRIORITIES - 1)
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

    //store data
    Global_variable_buffer[0] = LDR_a_read_value;
 
    // print the temperature in the Serial Monitor:
    Serial.print("LDR_Value: ");
    Serial.print(LDR_a_read_value);   // print the temperature in °C
    Serial.println("end LDR");
    
    vTaskDelay(10);  // one tick delay (15ms) in between reads for stability
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
    
    vTaskDelay(10);  // one tick delay (15ms) in between reads for stability
  }
}

void Task_Rele( void *pvParameters )
{
    //make statements and semaphores for this
    pinMode(RELE_PIN, OUTPUT);
    for (;;)
  {
    if (NULL != rele_semaphore)
    {
      if( pdTRUE == xSemaphoreTake( rele_semaphore, ( TickType_t ) 10 )) //check the semaphor to take it or wait n ticks to see if it becomes available
      {
          digitalWrite(RELE_PIN, HIGH);   // turn the Rele on (HIGH is the voltage level)
          vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
          digitalWrite(RELE_PIN, LOW);    // turn the Rele off by making the voltage LOW
      }
    }
    vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
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
    if (NULL != feed_fish)
    {
      if( pdTRUE == xSemaphoreTake( feed_fish, ( TickType_t ) 10 )) //check the semaphor to take it or wait n ticks to see if it becomes available
      {
        //take actions to feed fish 
        myservo.write(pos); 
      }
    }
    vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
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
    int pH_a_read_value = analogRead(PH_SENSOR_PIN);
    //Process data 
    int temperature = 0, averageVoltage = 0;
    
    float pH_voltage =  pH_a_read_value *3.3 /ADC_Res;
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
    float compensationVolatge = averageVoltage / compensationCoefficient; //temperature compensation
    float ph = (133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5; //convert voltage value to tds value

    //store data
    
    // print the temperature in the Serial Monitor:
    Serial.print("Ph: ");
    Serial.print(ph);   // print the temperature in °C
    Serial.println("end");
    
    vTaskDelay(10);  // one tick delay (15ms) in between reads for stability
  }
}

void Task_Serial_Print( void *pvParameters )
{
  /*
  serial print of data adquired in a terminal
*/
  //Serial.print("Data adquired from the LDR -> ");
  
  vTaskDelay(50);  // one tick delay (15ms) in between reads for stability
}
