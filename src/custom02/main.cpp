#include <Arduino.h>
#include <ArduinoOTA.h>
#include <ESP32Servo.h>
#include <IRremote.hpp>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <base64.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_camera.h"
#include "SD_MMC.h" // SD Card ESP32

const char *networkHostname = "turret_mod";

// REPLACE WITH YOUR NETWORK CREDENTIALS
const char *ssid = "***";
const char *password = "***";

// Azure Computer Vision resource
// Free tier: "Free F0" (20 Calls per minute, 5K Calls per month)
// https://portal.azure.com/#create/Microsoft.CognitiveServicesComputerVision
// https://learn.microsoft.com/en-us/azure/ai-services/computer-vision/how-to/call-analyze-image-40?tabs=csharp&pivots=programming-language-csharp
// https://learn.microsoft.com/en-us/azure/ai-services/computer-vision/quickstarts-sdk/image-analysis-client-library-40?tabs=visual-studio%2Cwindows&pivots=programming-language-rest-api

const char *computerVisionEndpoint = "https://***.cognitiveservices.azure.com/computervision/imageanalysis:analyze?features=caption,denseCaptions,read,objects,people&model-version=latest&language=en&api-version=2024-02-01";
const char *computerVisionApiKey = "***";

// Declare a mutex handle
SemaphoreHandle_t turretMovementMutex;
SemaphoreHandle_t webServerOperationMutex;
SemaphoreHandle_t computerVisionToggleMutex;
SemaphoreHandle_t testMutex;

bool computerVisionToggle = true;

const long cameraInterval = 30000;
unsigned long previousMillis = 0;

// REPLACE WITH YOUR TIMEZONE STRING: https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv
String myTimezone = "CST6CDT,M3.2.0,M11.1.0";

// ===================
// Select camera model
// ===================
// #define CAMERA_MODEL_WROVER_KIT // Has PSRAM
// #define CAMERA_MODEL_ESP_EYE // Has PSRAM
#define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
// #define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
// #define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
// #define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
// #define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
// #define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
// #define CAMERA_MODEL_AI_THINKER // Has PSRAM
// #define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
//  ** Espressif Internal Boards **
// #define CAMERA_MODEL_ESP32_CAM_BOARD
// #define CAMERA_MODEL_ESP32S2_CAM_BOARD
// #define CAMERA_MODEL_ESP32S3_CAM_LCD

#include "camera_pins.h"

#define LED_PIN LED_BUILTIN

#define DECODE_NEC // defines the type of IR transmission to decode based on the remote. See IRremote library for examples on how to decode other types of remote

// Purple wire (IR receiver) (Pin 2 on freenove_esp32_s3_wroom is an on-board LED indicator)
#define IR_PIN 14
#define IR_FEEDBACK_PIN 2
// Blue wire (Rotation)
#define YAW_PIN 21
// Green wire (Pitch - up and down)
#define PITCH_PIN 47
// Yellow wire (Rotation of top half of turret)
#define ROLL_PIN 1

/*
** if you want to add other remotes (as long as they're on the same protocol above):
** press the desired button and look for a hex code similar to those below (ex: 0x11)
** then add a new line to #define newCmdName 0x11,
** and add a case to the switch statement like case newCmdName:
** this will let you add new functions to buttons on other remotes!
** the best remotes to try are cheap LED remotes, some TV remotes, and some garage door openers
*/

// SD Card pins
#define SD_MMC_CMD 38 // Please do not modify it.
#define SD_MMC_CLK 39 // Please do not modify it.
#define SD_MMC_D0 40  // Please do not modify it.

// defines the specific command code for each button on the remote
#define left 0x8
#define right 0x5A
#define up 0x18
#define down 0x52
#define ok 0x1C
#define cmd1 0x45
#define cmd2 0x46
#define cmd3 0x47
#define cmd4 0x44
#define cmd5 0x40
#define cmd6 0x43
#define cmd7 0x7
#define cmd8 0x15
#define cmd9 0x9
#define cmd0 0x19
#define star 0x16
#define hashtag 0xD

Servo yawServo;   // names the servo responsible for YAW rotation, 360 spin around the base
Servo pitchServo; // names the servo responsible for PITCH rotation, up and down tilt
Servo rollServo;  // names the servo responsible for ROLL rotation, spins the barrel to fire darts

int yawServoVal; // initialize variables to store the current value of each servo
int pitchServoVal = 100;
int rollServoVal;

int pitchMoveSpeed = 8; // this variable is the angle added to the pitch servo to control how quickly the PITCH servo moves - try values between 3 and 10
int yawMoveSpeed = 90;  // this variable is the speed controller for the continuous movement of the YAW servo motor. It is added or subtracted from the yawStopSpeed, so 0 would mean full speed rotation in one direction, and 180 means full rotation in the other. Try values between 10 and 90;
int yawStopSpeed = 90;  // value to stop the yaw motor - keep this at 90
int rollMoveSpeed = 90; // this variable is the speed controller for the continuous movement of the ROLL servo motor. It is added or subtracted from the rollStopSpeed, so 0 would mean full speed rotation in one direction, and 180 means full rotation in the other. Keep this at 90 for best performance / highest torque from the roll motor when firing.
int rollStopSpeed = 90; // value to stop the roll motor - keep this at 90

int yawPrecision = 150;  // this variable represents the time in milliseconds that the YAW motor will remain at it's set movement speed. Try values between 50 and 500 to start (500 milliseconds = 1/2 second)
int rollPrecision = 158; // this variable represents the time in milliseconds that the ROLL motor with remain at it's set movement speed. If this ROLL motor is spinning more or less than 1/6th of a rotation when firing a single dart (one call of the fire(); command) you can try adjusting this value down or up slightly, but it should remain around the stock value (160ish) for best results.

int pitchMax = 175; // this sets the maximum angle of the pitch servo to prevent it from crashing, it should remain below 180, and be greater than the pitchMin
int pitchMin = 10;  // this sets the minimum angle of the pitch servo to prevent it from crashing, it should remain above 0, and be less than the pitchMax

// Stores the camera configuration parameters
camera_config_t config;

WebServer webServer(80);

// Function type definition
typedef void (*FunctionPtr)();

/// @brief To move the turret/servos, call this function to prevent attempting to move the turret/servos from more than 1 task simultaneously.
/// @param func The function performing the turret/servos movements.
void safeTurretMove(FunctionPtr func)
{
  if (xSemaphoreTake(turretMovementMutex, portMAX_DELAY))
  {
    func();
    xSemaphoreGive(turretMovementMutex);
  }
}

void safeWebServerOperation(FunctionPtr func)
{
  if (xSemaphoreTake(webServerOperationMutex, portMAX_DELAY))
  {
    func();
    xSemaphoreGive(webServerOperationMutex);
  }
}

String SendHTML();

// Web server handler prototypes
void webServerHandle_OnRoot();
void webServerHandle_NotFound();
void webServerHandle_moveUp();
void webServerHandle_moveDown();
void webServerHandle_moveLeft();
void webServerHandle_moveRight();
void webServerHandle_fire();
void webServerHandle_fireAll();
void webServerHandle_shakeHeadNo();
void webServerHandle_shakeHeadYes();
void webServerHandle_yosemiteSam();
void webServerHandle_toggleComputerVision();

// Servo movement prototypes
void handleCommand_moveUp();
void handleCommand_moveDown();
void handleCommand_moveLeft();
void handleCommand_moveRight();
void handleCommand_fire();
void handleCommand_fireAll();
void handleCommand_shakeHeadNo();
void handleCommand_shakeHeadYes();
void handleCommand_yosemiteSam();
void homeServos();

void initOTA()
{
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname(networkHostname);

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
  ArduinoOTA.setTimeout(10000);

  ArduinoOTA
      .onStart([]()
               {
        Serial.println("OTA update starting.");

        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
          type = "sketch";
        else // U_SPIFFS
          type = "filesystem";

        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type); })
      .onEnd([]()
             { Serial.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total)
                  { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
      .onError([](ota_error_t error)
               {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed"); });

  ArduinoOTA.begin();
}

// Initializes the camera
void configInitCamera()
{
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  // for larger pre-allocated frame buffer.
  if (psramFound())
  {
    config.jpeg_quality = 10;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  }
  else
  {
    // Limit the frame size when PSRAM is not available
    config.frame_size = FRAMESIZE_SVGA;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  // Initialize the Camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  // s->set_vflip(s, 1); // flip it back
  s->set_brightness(s, 1); // up the brightness just a bit
  s->set_saturation(s, 0); // lower the saturation
}

// Connect to wifi
void initWiFi()
{
  WiFi.begin(ssid, password);
  Serial.println("Connecting Wifi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
}

// Initialize web server (for remote control)
void initWebServer()
{
  webServer.on("/", []()
               { safeWebServerOperation(webServerHandle_OnRoot); });
  webServer.onNotFound([]()
                       { safeWebServerOperation(webServerHandle_NotFound); });

  webServer.on("/moveUp", []()
               { safeWebServerOperation(webServerHandle_moveUp); });
  webServer.on("/moveDown", []()
               { safeWebServerOperation(webServerHandle_moveDown); });
  webServer.on("/moveLeft", []()
               { safeWebServerOperation(webServerHandle_moveLeft); });
  webServer.on("/moveRight", []()
               { safeWebServerOperation(webServerHandle_moveRight); });
  webServer.on("/fire", []()
               { safeWebServerOperation(webServerHandle_fire); });
  webServer.on("/fireAll", []()
               { safeWebServerOperation(webServerHandle_fireAll); });
  webServer.on("/shakeHeadNo", []()
               { safeWebServerOperation(webServerHandle_shakeHeadNo); });
  webServer.on("/shakeHeadYes", []()
               { safeWebServerOperation(webServerHandle_shakeHeadYes); });
  webServer.on("/yosemiteSam", []()
               { safeWebServerOperation(webServerHandle_yosemiteSam); });
  webServer.on("/toggleComputerVision", []()
               { safeWebServerOperation(webServerHandle_toggleComputerVision); });

  webServer.begin();
  Serial.println("HTTP server started.");
}

// Function to set timezone
void setTimezone(String timezone)
{
  Serial.printf("  Setting Timezone to %s\n", timezone.c_str());
  setenv("TZ", timezone.c_str(), 1); //  Now adjust the TZ.  Clock settings are adjusted to show the new local time
  tzset();
}

// Connect to NTP server and adjust timezone
void initTime(String timezone)
{
  struct tm timeinfo;
  Serial.println("Setting up time");
  configTime(0, 0, "pool.ntp.org"); // First connect to NTP server, with 0 TZ offset
  if (!getLocalTime(&timeinfo))
  {
    Serial.println(" Failed to obtain time");
    return;
  }
  Serial.println("Got the time from NTP");
  // Now we can set the real timezone
  setTimezone(timezone);
}

// Get the picture filename based on the current ime
String getPictureFilename()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
    return "";
  }
  char timeString[20];
  strftime(timeString, sizeof(timeString), "%Y-%m-%d_%H-%M-%S", &timeinfo);
  Serial.println(timeString);
  String filename = "/picture_" + String(timeString) + ".jpg";
  return filename;
}

// Initialize the micro SD card
void initMicroSDCard()
{
  // Start Micro SD card
  Serial.println("Starting SD Card");

  SD_MMC.setPins(SD_MMC_CLK, SD_MMC_CMD, SD_MMC_D0);
  if (!SD_MMC.begin("/sdcard", true, true, SDMMC_FREQ_DEFAULT, 5))
  {
    Serial.println("SD Card Mount Failed");
    return;
  }

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE)
  {
    Serial.println("No SD Card attached");
    return;
  }

  Serial.print("SD_MMC Card Type: ");
  if (cardType == CARD_MMC)
  {
    Serial.println("MMC");
  }
  else if (cardType == CARD_SD)
  {
    Serial.println("SDSC");
  }
  else if (cardType == CARD_SDHC)
  {
    Serial.println("SDHC");
  }
  else
  {
    Serial.println("UNKNOWN");
  }
}

// Take photo and save to microSD card
void takePhotoAndSave()
{
  Serial.println("\ntakePhotoAndSave(): Begin");

  // Take Picture with Camera
  camera_fb_t *fb = esp_camera_fb_get();

  // Uncomment the following lines if you're getting old pictures
  // esp_camera_fb_return(fb); // dispose the buffered image
  // fb = NULL; // reset to capture errors
  // fb = esp_camera_fb_get();

  if (!fb)
  {
    Serial.println("Camera capture failed. Restarting...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP.restart();
  }

  // Path where new picture will be saved in SD Card
  Serial.println("\ntakePhotoAndSave(): getPictureFilename");
  String path = getPictureFilename();
  Serial.printf("Picture file name: %s\n", path.c_str());

  // Save picture to microSD card
  Serial.println("\ntakePhotoAndSave(): Open file for output...");
  fs::FS &fs = SD_MMC;
  File file = fs.open(path.c_str(), FILE_WRITE);
  if (!file)
  {
    Serial.printf("Failed to open file in writing mode");
  }
  else
  {
    file.write(fb->buf, fb->len); // payload (image), payload length
    Serial.printf("Saved: %s\n", path.c_str());
  }

  Serial.println("\ntakePhotoAndSave(): Close file...");
  file.close();
  esp_camera_fb_return(fb);

  // Send to Azure Cognitive Services

  Serial.println("Opening file to send to Azure Cognitive Services...");

  // Open the image file
  File imageFile = fs.open(path.c_str());
  if (!imageFile)
  {
    Serial.println("Failed to open image file");
    return;
  }

  size_t fileSize = imageFile.size();

  Serial.printf("Sending image to Azure Cognitive Services (File size %d)...\n", fileSize);

  Serial.printf("Sending image to Azure Cognitive Services (File size %d)...\n", fileSize);

  // Send the HTTP POST request to Azure Cognitive Services
  HTTPClient http;
  http.begin(computerVisionEndpoint);
  http.addHeader("Content-Type", "application/octet-stream");
  http.addHeader("Ocp-Apim-Subscription-Key", computerVisionApiKey);

  Serial.println("Streaming file to endpoint...");

  // Stream file into http POST request
  int httpResponseCode = http.sendRequest("POST", &imageFile, fileSize);

  // Check for a successful request
  if (httpResponseCode == HTTP_CODE_OK)
  {
    String response = http.getString();
    Serial.println("Response from Azure Cognitive Services:");
    Serial.println(response);

    // Extract filename without extension
    int dotIndex = path.lastIndexOf('.');
    String fileName = path.substring(0, dotIndex);

    // Create a text file with the same name as the image file
    String textFileName = fileName + ".json";
    File textFile = fs.open(textFileName, FILE_WRITE);
    if (textFile)
    {
      textFile.println(response);
      textFile.close();
      Serial.println("Response saved to " + textFileName);
    }
    else
    {
      Serial.println("Failed to create text file");
    }
  }
  else
  {
    Serial.print("HTTP POST request failed, error: ");
    Serial.println(httpResponseCode);
    String response = http.getString();
    Serial.println("Response content:");
    Serial.println(response);
  }

  // Clean up
  imageFile.close();
  http.end();

  Serial.println("\ntakePhotoAndSave(): End");
}

/// @brief Task for checking for and handling any web server requests.
/// @param pvParameters
void webServerTask(void *pvParameters)
{
  while (true)
  {
    if (xSemaphoreTake(testMutex, portMAX_DELAY))
    {
      // Serial.println("\nWeb server task running");

      // Check for input from web site
      webServer.handleClient();

      xSemaphoreGive(testMutex);
    }

    // Delay to allow other tasks to run
    vTaskDelay(pdMS_TO_TICKS(5)); // Delay for 5ms
  }
}

/// @brief Task for checking for and handling any IR (infrared) commands.
/// @param pvParameters
void irTask(void *pvParameters)
{
  while (true)
  {
    // Serial.println("\nIR task running");

    // Check if received data is available and if yes, try to decode it.
    // Serial.println("\nChecking IR input...");
    if (IrReceiver.decode())
    {
      // Print a short summary of received data
      IrReceiver.printIRResultShort(&Serial);
      IrReceiver.printIRSendUsage(&Serial);
      if (IrReceiver.decodedIRData.protocol == UNKNOWN)
      { // command garbled or not recognized
        Serial.println(F("Received noise or an unknown (or not yet enabled) protocol - if you wish to add this command, define it at the top of the file with the hex code printed below (ex: 0x8)"));
        // We have an unknown protocol here, print more info
        IrReceiver.printIRResultRawFormatted(&Serial, true);
      }
      Serial.println();

      // !!!Important!!! Enable receiving of the next value,
      // since receiving has stopped after the end of the current received data packet.
      IrReceiver.resume(); // Enable receiving of the next value

      // Finally, check the received data and perform actions according to the received command
      // this is where the commands are handled
      switch (IrReceiver.decodedIRData.command)
      {
      case up:
        safeTurretMove(handleCommand_moveUp); // Pitch up
        break;

      case down:
        safeTurretMove(handleCommand_moveDown); // Pitch down
        break;

      case left:
        safeTurretMove(handleCommand_moveLeft);
        break;

      case right:
        safeTurretMove(handleCommand_moveRight);
        break;

      case ok:
        safeTurretMove(handleCommand_fire);
        break;

      case star:
        safeTurretMove(handleCommand_fireAll);
        break;

      case cmd0:
        safeTurretMove(handleCommand_shakeHeadNo);
        break;

      case cmd9:
        safeTurretMove(handleCommand_shakeHeadYes);
        break;

      case cmd7:
        safeTurretMove(handleCommand_yosemiteSam);
        break;
      }
    }

    // Delay to allow other tasks to run
    vTaskDelay(pdMS_TO_TICKS(5)); // Delay for 5ms
  }
}

/// @brief Task for checking for and handling any OTA (Over the Air) update requests.
/// @param pvParameters
void otaTask(void *pvParameters)
{
  while (true)
  {
    // Serial.println("\nOTA task running");

    // Check for an OTA request
    ArduinoOTA.handle();

    vTaskDelay(pdMS_TO_TICKS(5)); // Delay for 5ms
  }
}

/// @brief Task for running logic for AI services.
/// @param pvParameters
void aiTask(void *pvParameters)
{
  while (true)
  {
    if (xSemaphoreTake(testMutex, portMAX_DELAY))
    {
      // Serial.println("\nAI task running");

      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= cameraInterval)
      {
        previousMillis = currentMillis;

        bool shouldProceed = false;
        if (xSemaphoreTake(computerVisionToggleMutex, portMAX_DELAY))
        {
          shouldProceed = computerVisionToggle;
          xSemaphoreGive(computerVisionToggleMutex);
        }

        if (shouldProceed)
        {
          Serial.println("\nComputer vision is ON. Proceeding to take a photo...");

          digitalWrite(LED_PIN, HIGH);
          vTaskDelay(pdMS_TO_TICKS(100));
          digitalWrite(LED_PIN, LOW);
          vTaskDelay(pdMS_TO_TICKS(100));
          digitalWrite(LED_PIN, HIGH);
          vTaskDelay(pdMS_TO_TICKS(100));
          digitalWrite(LED_PIN, LOW);

          takePhotoAndSave();

          digitalWrite(LED_PIN, HIGH);
          vTaskDelay(pdMS_TO_TICKS(100));
          digitalWrite(LED_PIN, LOW);
        }
        else
        {
          Serial.println("\nComputer vision is OFF. Skipping photo.");
        }
      }

      xSemaphoreGive(testMutex);
    }

    // Delay to allow other tasks to run
    vTaskDelay(pdMS_TO_TICKS(5)); // Delay for 5ms
  }
}

// Setup
void setup()
{
  Serial.begin(115200); // initializes the Serial communication between the computer and the microcontroller

  Serial.println("Starting...");

  // initialize digital pin LED_PIN as an output
  pinMode(LED_PIN, OUTPUT);

  Serial.println("\nCreating mutexes...");
  turretMovementMutex = xSemaphoreCreateMutex();
  if (turretMovementMutex == NULL)
  {
    Serial.println("Failed to turret movement mutex");
    while (true)
      ; // Halt if mutex creation fails
  }

  webServerOperationMutex = xSemaphoreCreateMutex();
  if (webServerOperationMutex == NULL)
  {
    Serial.println("Failed to create web server operation mutex");
    while (true)
      ; // Halt if mutex creation fails
  }

  computerVisionToggleMutex = xSemaphoreCreateMutex();
  if (computerVisionToggleMutex == NULL)
  {
    Serial.println("Failed to create computer vision toggle mutex");
    while (true)
      ; // Halt if mutex creation fails
  }

  testMutex = xSemaphoreCreateMutex();
  if (testMutex == NULL)
  {
    Serial.println("Failed to create test mutex");
    while (true)
      ; // Halt if mutex creation fails
  }
  Serial.println("Done creating mutex.");

  // Initialize Wi-Fi
  Serial.println("\nInitializing wifi...");
  initWiFi();
  Serial.println("\nDone initializing wifi - OK!");

  // Initialize OTA (Over the Air) updates
  Serial.println("\nInitializing OTA...");
  initOTA();
  Serial.println("\nDone initializing OTA - OK!");

  // Initialize OTA (Over the Air) updates
  Serial.println("\nInitializing web server...");
  initWebServer();
  Serial.println("\nDone initializing web server - OK!");

  // Initialize time with timezone
  Serial.println("\nInitializing time...");
  initTime(myTimezone);
  Serial.println("\nDone initializing time - OK!");

  // Initialize the camera
  Serial.println("\nInitializing the camera module...");
  configInitCamera();
  Serial.println("\nDone initializing the camera module - OK!");

  // Initialize MicroSD
  Serial.println("\nInitializing the micro sd card module...");
  initMicroSDCard();
  Serial.println("\nDone initializing the micro sd card module - OK!");

  yawServo.attach(YAW_PIN);     // attach YAW servo to pin YAW_PIN
  pitchServo.attach(PITCH_PIN); // attach PITCH servo to pin 11
  rollServo.attach(ROLL_PIN);   // attach ROLL servo to pin 12

  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing IR Remote library version " VERSION_IRREMOTE));

  // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
  Serial.print("\nInitializing the IR receiver...");
  IrReceiver.begin(IR_PIN, ENABLE_LED_FEEDBACK, IR_FEEDBACK_PIN);
  Serial.println("\nDone initializing the IR receiver - OK!");

  Serial.print(F("\nReady to receive IR signals of protocols: "));
  printActiveIRProtocols(&Serial);

  Serial.println("\nHoming servos...");
  homeServos(); // set servo motors to home position
  Serial.println("Done homing servos.");

  Serial.println("\nCreating tasks...");
  xTaskCreate(webServerTask, "Web Server Task", 4048, NULL, 1, NULL);
  xTaskCreate(irTask, "IR Task", 2048, NULL, 1, NULL);
  xTaskCreate(otaTask, "OTA Task", 2048, NULL, 1, NULL);
  xTaskCreate(aiTask, "AI Task", 4048, NULL, 1, NULL);
  Serial.println("Done creating tasks.");
}

// Loop
void loop()
{
  // Tasks are handling everything.
}

void shakeHeadYes(int moves = 3)
{
  Serial.println("YES");

  int startAngle = pitchServoVal; // Current position of the pitch servo
  int lastAngle = pitchServoVal;
  int nodAngle = startAngle + 20; // Angle for nodding motion

  for (int i = 0; i < moves; i++)
  {
    // Repeat nodding motion three times

    // Nod up
    for (int angle = startAngle; angle <= nodAngle; angle++)
    {
      pitchServo.write(angle);
      vTaskDelay(pdMS_TO_TICKS(7)); // Adjust delay for smoother motion
    }

    vTaskDelay(pdMS_TO_TICKS(50)); // Pause at nodding position

    // Nod down
    for (int angle = nodAngle; angle >= startAngle; angle--)
    {
      pitchServo.write(angle);
      vTaskDelay(pdMS_TO_TICKS(7)); // Adjust delay for smoother motion
    }

    vTaskDelay(pdMS_TO_TICKS(50)); // Pause at starting position
  }
}

void shakeHeadNo(int moves = 3)
{
  Serial.println("NO");

  int startAngle = pitchServoVal; // Current position of the pitch servo
  int lastAngle = pitchServoVal;
  int nodAngle = startAngle + 60; // Angle for nodding motion

  for (int i = 0; i < moves; i++)
  {
    // Repeat nodding motion three times
    // rotate right, stop, then rotate left, stop
    yawServo.write(140);
    vTaskDelay(pdMS_TO_TICKS(190)); // Adjust delay for smoother motion
    yawServo.write(yawStopSpeed);
    vTaskDelay(pdMS_TO_TICKS(50));
    yawServo.write(40);
    vTaskDelay(pdMS_TO_TICKS(190)); // Adjust delay for smoother motion
    yawServo.write(yawStopSpeed);
    vTaskDelay(pdMS_TO_TICKS(50)); // Pause at starting position
  }
}

void moveLeft(int moves)
{
  for (int i = 0; i < moves; i++)
  {
    Serial.println("LEFT");
    yawServo.write(yawStopSpeed + yawMoveSpeed); // adding the servo speed = 180 (full counterclockwise rotation speed)
    vTaskDelay(pdMS_TO_TICKS(yawPrecision));     // stay rotating for a certain number of milliseconds
    yawServo.write(yawStopSpeed);                // stop rotating
    vTaskDelay(pdMS_TO_TICKS(5));                // delay for smoothness
  }
}

void moveRight(int moves)
{
  for (int i = 0; i < moves; i++)
  {
    Serial.println("RIGHT");
    yawServo.write(yawStopSpeed - yawMoveSpeed); // subtracting the servo speed = 0 (full clockwise rotation speed)
    vTaskDelay(pdMS_TO_TICKS(yawPrecision));
    yawServo.write(yawStopSpeed);
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void moveDown(int moves)
{
  for (int i = 0; i < moves; i++)
  {
    if (pitchServoVal > pitchMin)
    { // make sure the servo is within rotation limits (greater than 10 degrees by default)
      Serial.println("DOWN");
      pitchServoVal = pitchServoVal - pitchMoveSpeed; // decrement the current angle and update
      pitchServo.write(pitchServoVal);
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }
}

void moveUp(int moves)
{
  for (int i = 0; i < moves; i++)
  {
    if (pitchServoVal < pitchMax)
    { // make sure the servo is within rotation limits (less than 175 degrees by default)
      Serial.println("UP");
      pitchServoVal = pitchServoVal + pitchMoveSpeed; // increment the current angle and update
      pitchServo.write(pitchServoVal);
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }
}

void fire()
{ // function for firing a single dart
  Serial.println("FIRING");
  rollServo.write(rollStopSpeed + rollMoveSpeed); // start rotating the servo
  vTaskDelay(pdMS_TO_TICKS(rollPrecision));       // time for approximately 60 degrees of rotation
  rollServo.write(rollStopSpeed);                 // stop rotating the servo
  vTaskDelay(pdMS_TO_TICKS(5));                   // delay for smoothness
}

void fireAll()
{ // function to fire all 6 darts at once
  Serial.println("FIRING ALL");
  rollServo.write(rollStopSpeed + rollMoveSpeed); // start rotating the servo
  vTaskDelay(pdMS_TO_TICKS(rollPrecision * 6));   // time for 360 degrees of rotation
  rollServo.write(rollStopSpeed);                 // stop rotating the servo
  vTaskDelay(pdMS_TO_TICKS(5));                   // delay for smoothness
}

/// @brief Shoot around aimlessly.
void yosemiteSam()
{
  // Ready, fire, aim!
  Serial.print("FIRE AIMLESSLY");
  rollServo.write(rollStopSpeed + rollMoveSpeed); // start rotating the servo
  Serial.println(" AND SPIN");
  moveLeft(7);
  rollServo.write(rollStopSpeed); // stop rotating the servo
  vTaskDelay(pdMS_TO_TICKS(5));   // delay for smoothness
}

void homeServos()
{
  Serial.println("HOMING");
  yawServo.write(yawStopSpeed); // setup YAW servo to be STOPPED (90)
  vTaskDelay(pdMS_TO_TICKS(20));
  rollServo.write(rollStopSpeed); // setup ROLL servo to be STOPPED (90)
  vTaskDelay(pdMS_TO_TICKS(100));
  pitchServo.write(100); // set PITCH servo to 100 degree position
  vTaskDelay(pdMS_TO_TICKS(100));
  pitchServoVal = 100; // store the pitch servo value
}

void webServerHandle_OnRoot()
{
  Serial.println("/");
  webServer.send(200, "text/html", SendHTML());
}

void webServerHandle_NotFound()
{
  Serial.println("NotFound");
  webServer.send(404, "application/json", "{ \"status\": \"Not Found\" }");
}

void webServerHandle_moveUp()
{
  Serial.println("Handling moveUp");
  safeTurretMove(handleCommand_moveUp);

  webServer.send(200, "application/json", "{ \"status\": \"ok\" }");
}

void webServerHandle_moveDown()
{
  Serial.println("Handling moveDown");
  safeTurretMove(handleCommand_moveDown);

  webServer.send(200, "application/json", "{ \"status\": \"ok\" }");
}

void webServerHandle_moveLeft()
{
  Serial.println("Handling moveLeft");
  safeTurretMove(handleCommand_moveLeft);

  webServer.send(200, "application/json", "{ \"status\": \"ok\" }");
}

void webServerHandle_moveRight()
{
  Serial.println("Handling moveRight");
  safeTurretMove(handleCommand_moveRight);

  webServer.send(200, "application/json", "{ \"status\": \"ok\" }");
}

void webServerHandle_fire()
{
  Serial.println("Handling fire");
  safeTurretMove(handleCommand_fire);

  webServer.send(200, "application/json", "{ \"status\": \"ok\" }");
}

void webServerHandle_fireAll()
{
  Serial.println("Handling fireAll");
  safeTurretMove(handleCommand_fireAll);

  webServer.send(200, "application/json", "{ \"status\": \"ok\" }");
}

void webServerHandle_shakeHeadNo()
{
  Serial.println("Handling shakeHeadNo");
  safeTurretMove(handleCommand_shakeHeadNo);

  webServer.send(200, "application/json", "{ \"status\": \"ok\" }");
}

void webServerHandle_shakeHeadYes()
{
  Serial.println("Handling shakeHeadYes");
  safeTurretMove(handleCommand_shakeHeadYes);

  webServer.send(200, "application/json", "{ \"status\": \"ok\" }");
}

void webServerHandle_yosemiteSam()
{
  Serial.println("Handling yosemiteSam");
  safeTurretMove(handleCommand_yosemiteSam);

  webServer.send(200, "application/json", "{ \"status\": \"ok\" }");
}

void webServerHandle_toggleComputerVision()
{
  if (xSemaphoreTake(computerVisionToggleMutex, portMAX_DELAY))
  {
    computerVisionToggle = !computerVisionToggle;

    Serial.printf("Handling toggleComputerVision (Setting to %s)\n", computerVisionToggle ? "true" : "false");

    // If computer vision was just toggled on, reset timer so it is used in the very next loop.
    if (computerVisionToggle)
      previousMillis = 0;

    xSemaphoreGive(computerVisionToggleMutex);

    webServer.send(200, "application/json", "{ \"status\": \"ok\" }");
  }
}

void handleCommand_moveUp()
{
  moveUp(1);
}

void handleCommand_moveDown()
{
  moveDown(1);
}

void handleCommand_moveLeft()
{
  moveLeft(1);
}

void handleCommand_moveRight()
{
  moveRight(1);
}

void handleCommand_fire()
{
  fire();
}

void handleCommand_fireAll()
{
  fireAll();
  vTaskDelay(pdMS_TO_TICKS(50));
}

void handleCommand_shakeHeadNo()
{
  shakeHeadNo(3);
  vTaskDelay(pdMS_TO_TICKS(50));
}

void handleCommand_shakeHeadYes()
{
  shakeHeadYes(3);
  vTaskDelay(pdMS_TO_TICKS(50));
}

void handleCommand_yosemiteSam()
{
  yosemiteSam();
}

String SendHTML()
{
  std::string htmlString = R"(
  <!DOCTYPE html>
  <html>
      <head>
          <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no" />
          <title>Turret Control</title>
          <style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}
              body{margin-top: 50px;} h1 {color: #444444; margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}
              .button {
                  display: flex; /* Use flexbox */
                  justify-content: center; /* Center content horizontally */
                  align-items: center; /* Center content vertically */
                  width: 100px;
                  height: 75px; /* Allow button height to adjust based on content */
                  background-color: #3498db;
                  border: none;
                  color: white;
                  padding: 5px 10px;
                  text-decoration: none;
                  font-size: 25px;
                  margin: 0px auto 0px;
                  cursor: pointer;
                  border-radius: 4px;
                  text-align: center; /* Center-align text */
              }
              .button-move {background-color: #3498db;}
              .button-fire {background-color: #ff3333;}
              .button-fire-all {background-color: #e40000;}
              .button-yosemite-sam {background-color: #bb0000;}
              .button-neutral {background-color: #71a6c9;}
              p {font-size: 14px;color: #888;margin-bottom: 10px;}
          </style>
      </head>
      <body>
          <h1>Turret Control</h1>
          <p>
              <table>
                <tr>
                    <td>&nbsp;</td>
                    <td>
                        <a class="button button-move" href="#" onclick="makeWebRequest('/moveUp');">Move Up</a>
                    </td>
                    <td>&nbsp;</td>
                </tr>
                <tr>
                    <td>
                        <a class="button button-move" href="#" onclick="makeWebRequest('/moveLeft');">Move Left</a>
                    </td>
                    <td>
                        <a class="button button-fire" href="#" onclick="makeWebRequest('/fire');">Fire</a>
                    </td>
                    <td>
                        <a class="button button-move" href="#" onclick="makeWebRequest('/moveRight');">Move Right</a>
                    </td>
                </tr>
                <tr>
                    <td>&nbsp;</td>
                    <td>
                        <a class="button button-move" href="#" onclick="makeWebRequest('/moveDown');">Move Down</a>
                    </td>
                    <td>&nbsp;</td>
                </tr>
              </table>
          </p>
          <p>
              <table>
                  <tr>
                      <td>
                          <a class="button button-fire-all" href="#" onclick="makeWebRequest('/fireAll');">Fire All</a>
                      </td>
                      <td>&nbsp;</td>
                      <td>
                          <a class="button button-yosemite-sam" href="#" onclick="makeWebRequest('/yosemiteSam');">Yosemite Sam</a>
                      </td>
                  </tr>
                  <tr>
                      <td>&nbsp;</td>
                      <td>&nbsp;</td>
                      <td>&nbsp;</td>
                  </tr>
                  <tr>
                      <td>
                          <a class="button button-neutral" href="#" onclick="makeWebRequest('/shakeHeadNo');">Shake No</a>
                      </td>
                      <td>&nbsp;</td>
                      <td>
                          <a class="button button-neutral" href="#" onclick="makeWebRequest('/shakeHeadYes');">Nod Yes</a>
                      </td>
                  </tr>
                  <tr>
                      <td>&nbsp;</td>
                      <td>
  )";

  if (xSemaphoreTake(computerVisionToggleMutex, portMAX_DELAY))
  {
    if (computerVisionToggle)
    {
      htmlString += R"(
                          <p>Computer Vision On</p>
                          <a class="button button-neutral" href="#" onclick="makeWebRequest('/toggleComputerVision'); location.reload();">Off</a>
      )";
    }
    else
    {
      htmlString += R"(
                          <p>Comptuer Vision Off</p>
                          <a class="button button-neutral" href="#" onclick="makeWebRequest('/toggleComputerVision'); location.reload();">On</a>
      )";
    }

    xSemaphoreGive(computerVisionToggleMutex);
  }

  htmlString += R"(
                      </td>
                      <td>&nbsp;</td>
                  </tr>
              </table>
          </p>

      <script>
          function makeWebRequest(path) {
              fetch(path, {
                  method: 'GET',
                  headers: {
                      'Content-Type': 'application/json',
                  },
              })
              .then(response => {
                  // Handle the response as needed, e.g., parse JSON, update UI, etc.
              })
              .catch(error => {
                  // Handle errors, e.g., show an error message, retry logic, etc.
              });
          }
      </script>

      </body>
  </html>
  )";

  return htmlString.c_str();
}
