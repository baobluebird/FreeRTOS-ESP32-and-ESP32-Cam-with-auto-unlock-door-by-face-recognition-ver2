
//This code is used to receive signals from the button. If the button is active, it will send a signal to Blynk so that Blynk can send a signal to the ESP32 Cam to take a photo and send it to the AI ​​server.
// If the response status is OK, send a signal to Blynk so that BLynk can send it to ESP32 to open the door (servo)
// If the response status is ERR, send a signal to Blynk so that BLynk can send it to ESP32 to warn (buzzer) and the app
#define BLYNK_TEMPLATE_ID "YOUR_TEMPLATE_ID"
#define BLYNK_TEMPLATE_NAME "YOUR_TEMPLATE_NAME"
#define BLYNK_AUTH_TOKEN "YOUR_AUTH_TOKEN"

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial

/* Comment this out to disable prints and save space */
#define BLYNK_PRINT Serial
#define BUZZER_PIN 4 // GPIO pin connected to buzzer

#define TRIG_PIN_OUTDOOR 12 // GPIO pin connected to Trig of HC-SR04 OutDoor
#define ECHO_PIN_OUTDOOR 14 // GPIO pin connected to Echo of HC-SR04 OutDoor

#define TRIG_PIN_INDOOR 5 // GPIO pin connected to Trig of HC-SR04 InDoor
#define ECHO_PIN_INDOOR 18 // GPIO pin connected to Echo of HC-SR04 InDoor

#define SERVO_PIN 13 // GPIO pin connected to servo

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <ESP32Servo.h>


// Your WiFi credentials.
// Set password to "" for open networks.
char ssid[] = "Your_NAME_WIFI";
char pass[] = "Your_PASS_WIFI";


//SERVO
Servo servo1;

bool isDoorOpen = false;

//FreeRTOS
QueueHandle_t distanceQueue;
SemaphoreHandle_t doorSemaphoreBinary;
SemaphoreHandle_t doorMutex;
TaskHandle_t distanceOutDoorTaskHandle = NULL;
TaskHandle_t distanceInDoorTaskHandle = NULL;

const char *pcTextForTask1 = "Task distanceOutDoorTask is running\r\n";
const char *pcTextForTask2 = "Task processDistanceTask is running\t\n";
const char *pcTextForTask3 = "Task distanceInDoorTask is running\t\n";
const char *pcTextForTask4 = "Task closeDoorTask is running\t\n";
const char *pcTextForTask5 = "Task blynkTask is running\t\n";
// Function to measure distance from HC-SR04
long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);
  return duration * 0.034 / 2;
}

void setup() {
  Serial.begin(9600);

  // Blynk setup
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  // GPIO setup

  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(TRIG_PIN_OUTDOOR, OUTPUT);
  pinMode(ECHO_PIN_OUTDOOR, INPUT);

  pinMode(TRIG_PIN_INDOOR, OUTPUT);
  pinMode(ECHO_PIN_INDOOR, INPUT);

  // Servo setup
  servo1.attach(SERVO_PIN);


  //FreeRTOS
  // Queue and Semaphore Setup
  distanceQueue = xQueueCreate(10, sizeof(long));
  doorSemaphoreBinary = xSemaphoreCreateBinary();
  doorMutex = xSemaphoreCreateMutex();
 // xSemaphoreGive(doorSemaphore); // Initial door state closed

  // Create tasks
  xTaskCreatePinnedToCore(distanceOutDoorTask, "Distance OutDoor Task", 3000, (void*)pcTextForTask1, 1, &distanceOutDoorTaskHandle,0);
  xTaskCreatePinnedToCore(processDistanceTask, "Process Distance Task", 3000, (void*)pcTextForTask2, 1, NULL,0);

  xTaskCreatePinnedToCore(distanceInDoorTask, "Distance InDoor Task", 3000, (void*)pcTextForTask3, 2, NULL,0);
  xTaskCreatePinnedToCore(closeDoorTask, "Close Door Task", 3000, (void*)pcTextForTask4, 2, NULL,0);

  xTaskCreatePinnedToCore(blynkTask, "Blynk Task", 3000, (void*)pcTextForTask5, 1, NULL,1);
}

// Task to check OUTDOOR sensor and send data to queue
void distanceOutDoorTask(void *pvParameters) {
  char *pcTaskName;
  pcTaskName = ( char * ) pvParameters;
  while (1) {
    // Chỉ chạy khi có thể lấy được mutex
    if (xSemaphoreTake(doorMutex, portMAX_DELAY) == pdTRUE) {

        Serial.println(pcTaskName);
        
        long distanceOutDoor = getDistance(TRIG_PIN_OUTDOOR, ECHO_PIN_OUTDOOR);
        if (distanceOutDoor <= 10 && distanceOutDoor != 0) {
          Serial.println("Object detected near OUTDOOR sensor!");
          xQueueSend(distanceQueue, &distanceOutDoor, portMAX_DELAY);
        } else if (distanceOutDoor != 0){
          Blynk.virtualWrite(V0, 0);
        }
      
      
      // Trả lại mutex để task khác có thể sử dụng tài nguyên
      xSemaphoreGive(doorMutex);
    }
    
    vTaskDelay(pdMS_TO_TICKS(1000)); // Kiểm tra mỗi giây
  }
}

// Task to process OUTDOOR sensor data and control door opening
void processDistanceTask(void *pvParameters) {
  char *pcTaskName;
  pcTaskName = ( char * ) pvParameters;
  long receivedDistance;
  while (1) {

    Serial.println(pcTaskName);

    if (xQueueReceive(distanceQueue, &receivedDistance, portMAX_DELAY)) {
      Serial.print("Processing distance: ");
      Serial.println(receivedDistance);
      if (isDoorOpen == false) {
        Blynk.virtualWrite(V0, 1); // Trigger Blynk to take a photo
      }
    }
  }
}

void closeDoorTask(void *pvParameters) {
  char *pcTaskName;
  pcTaskName = ( char * ) pvParameters;
  while (1) {
    Serial.println(pcTaskName);
    if (xSemaphoreTake(doorSemaphoreBinary, portMAX_DELAY) == pdTRUE){
          for (int pos = 90; pos >= 0; pos--) {
            servo1.write(pos);
            delay(20);
          }
          Blynk.virtualWrite(V1, 0);
          Blynk.virtualWrite(V3, 0);
          isDoorOpen = false; // Cập nhật trạng thái cửa là đóng
          vTaskResume(distanceOutDoorTaskHandle);
    }
  }
}

// Task to check INDOOR sensor and close door if someone passes
void distanceInDoorTask(void *pvParameters) {
  char *pcTaskName;
  pcTaskName = ( char * ) pvParameters;
  while (1) {
    if (isDoorOpen) {
      // Lấy mutex để chiếm quyền sử dụng tài nguyên
      if (xSemaphoreTake(doorMutex, portMAX_DELAY) == pdTRUE) {
        
        Serial.println(pcTaskName);
        
        long distanceInDoor = getDistance(TRIG_PIN_INDOOR, ECHO_PIN_INDOOR);
        if (distanceInDoor <= 5 && distanceInDoor != 0) {
          Serial.println("Closing door due to indoor object detection...");
          xSemaphoreGive(doorSemaphoreBinary);
        }
        xSemaphoreGive(doorMutex);
      }
      
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Blynk task
void blynkTask(void *pvParameters) {
  char *pcTaskName;
  pcTaskName = ( char * ) pvParameters;
  while (1) {

    //Serial.println(pcTaskName);

    Blynk.run();
    vTaskDelay(pdMS_TO_TICKS(10)); // Allow other tasks to run
  }
}

BLYNK_WRITE(V1) {
  int pinValue = param.asInt();
  Serial.print("V1 Slider value is: ");
  Serial.println(pinValue);

  if (pinValue == 1) {
    Serial.println("Door Opening..........................");
    
    for (int posDegrees = 0; posDegrees <= 90; posDegrees++) {
      servo1.write(posDegrees);
      delay(20);
    }
    isDoorOpen = true;  
    Blynk.virtualWrite(V0, 0); //stop capture image when door opened
    vTaskSuspend(distanceOutDoorTaskHandle);
  } 
}

BLYNK_WRITE(V3) {
  int pinValue = param.asInt();
  Serial.print("V3 slider for manual value is: ");
  Serial.println(pinValue);

  if (pinValue == 1) {
    Serial.println("Door Opening By App..........................");
    
    for (int posDegrees = 0; posDegrees <= 90; posDegrees++) {
      servo1.write(posDegrees);
      delay(20);
    }

    isDoorOpen = true;  
    vTaskSuspend(distanceOutDoorTaskHandle);

  } else {
    Serial.println("Door Closing By App..........................");
    
    for (int posDegrees = 90; posDegrees >= 0; posDegrees--) {
      servo1.write(posDegrees);
      delay(20);
    }
    isDoorOpen = false; // Cập nhật trạng thái cửa là đóng
    vTaskResume(distanceOutDoorTaskHandle);
  }
}

BLYNK_WRITE(V2) {
  int pinValue = param.asInt();
  if (pinValue == 1) {
    Serial.println(pinValue);
    Serial.print("Another People trying unlock your door: ");
    digitalWrite(BUZZER_PIN, 1);
  }else{
    Serial.println(pinValue);
    Serial.print("Turn Off Warning: ");
    digitalWrite(BUZZER_PIN, 0);
  }
}



void loop() {
  // Empty because FreeRTOS tasks are handling everything
}
