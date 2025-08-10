/*
  ESP32-CAM QR Code Reader + Stream Web Server
  ------------------------------------------------
  Notes:
   - Uses quirc library for QR decoding.
   - Starts an Access Point so you can connect directly to the ESP32-CAM.
   - Provides a simple webpage (index) and a MJPEG-style stream on :81/stream.
   - Creates a FreeRTOS task to continuously read QR codes when the stream isn't active.
*/

#include "esp_camera.h"            // Camera control
#include "soc/soc.h"               // Low-level SOC access (used to disable brownout)
#include "soc/rtc_cntl_reg.h"      // Brownout register
#include "quirc.h"                 // QR code detection/decoding (quirc library)
#include <WiFi.h>                  // WiFi AP
#include "esp_http_server.h"       // HTTP server library used by ESP-IDF/Arduino-ESP32
#include "esp32-hal-ledc.h"        // LEDC (PWM) functions for LED flash control
#include <HTTPClient.h>

#define RXD0 3                     // Serial RX pin to use for Serial1 (if swapping UART pins)
#define TXD0 1                     // Serial TX pin
const char* esp8266_ip = "192.168.1.2"; // (unused) IP placeholder for potential HTTP client calls


#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

/* ======================================== */

// LED pins (onboard and user LEDs on some boards)
#define LED_OnBoard 4
#define LED_Green   12
#define LED_Blue    13

// FreeRTOS task handle for QR code reader task
TaskHandle_t QRCodeReader_Task; 

/* ======================================== Variables declaration (QR structures & quirc) */
/*
  The code uses the quirc library for QR detection/decoding. We're keeping
  some convenience structures and global variables to hold the decoded result.
*/
struct QRCodeData
{
  bool valid;
  int dataType;
  uint8_t payload[1024];
  int payloadLen;
};

struct quirc *q = NULL;              // quirc instance pointer (allocated per capture)
uint8_t *image = NULL;               // pointer returned by quirc_begin() for raw image buffer
struct quirc_code code;              // quirc struct to hold a found symbol
struct quirc_data data;              // quirc decoded data
quirc_decode_error_t err;            // decode error code
struct QRCodeData qrCodeData;  
String QRCodeResult = "";            // Holds last decoded QR payload as String
String QRCodeResultSend = "";        // Value returned by /getqrcodeval endpoint
/* ======================================== */

/* ======================================== Webstream / task coordination flags */
/*
  ws_run: true when stream is active (so the QRCodeReader task should stop reading camera)
  wsLive_val / last_wsLive_val / get_wsLive_interval / get_wsLive_val:
    used for a simple heartbeat to detect if a streaming client is still connected.
*/
bool ws_run = false;
int wsLive_val = 0;
int last_wsLive_val;
byte get_wsLive_interval = 0;
bool get_wsLive_val = true;
/* ======================================== */

/* ======================================== millis() timing for heartbeat checks */
unsigned long previousMillis = 0;
const long interval = 1000; // check every 1000ms
/* ======================================== */

/* ======================================== Access Point Declaration and Configuration */
const char* ssid = "ESP32-CAM";           // Access Point SSID
const char* password = "helloesp32cam";  // AP password

// Static IP config for the AP
IPAddress local_ip(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
/* ======================================== */

/* ======================================== MJPEG stream boundary declarations */
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
/* ======================================== */

/* ======================================== HTTP server handles (two servers: index & stream) */
httpd_handle_t index_httpd = NULL;
httpd_handle_t stream_httpd = NULL;
/* ======================================== */

/* ======================================== HTML code for index / main page
   The page displays:
     - video stream (src set to :81/stream by JS)
     - slider that sends commands to control LED flash (via /action?go=S,VALUE)
     - area to show QR Code reading results (polled via /getqrcodeval)
*/
static const char PROGMEM INDEX_HTML[] = R"rawliteral(
<html>
  <head>
    <title>ESP32-CAM QR Code Reader Stream Web Server</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
      body { font-family: Arial; text-align: center; margin:0px auto; padding-top: 10px;}
      .slidecontainer { width: 100%; }
      .slider { -webkit-appearance: none; width: 50%; height: 10px; border-radius: 5px; background: #d3d3d3; outline: none; opacity: 0.7; -webkit-transition: .2s; transition: opacity .2s; }
      .slider:hover { opacity: 1; }
      .slider::-webkit-slider-thumb { -webkit-appearance: none; appearance: none; width: 20px; height: 20px; border-radius: 50%; background: #04AA6D; cursor: pointer; }
      .slider::-moz-range-thumb { width: 20px; height: 20px; border-radius: 50%; background: #04AA6D; cursor: pointer; }
      img { width: auto ; max-width: 100% ; height: auto ; }
    </style>
  </head>
  <body>
    <h3>ESP32-CAM QR Code Reader Stream Web Server</h3>
    <img src="" id="vdstream">
    <br><br>
    <div class="slidecontainer">
      <span style="font-size:15;">LED Flash &nbsp;</span>
      <input type="range" min="0" max="20" value="0" class="slider" id="mySlider">
    </div>
    <br>
    <p>QR Code Scan Result :</p>
    <div style="padding: 5px; border: 3px solid #075264; text-align: center; width: 70%; margin: auto; color:#0A758F;" id="showqrcodeval"></div>
    <br>
    <button type="button" onclick="CopyQRCodeRslt()">Copy Result</button>
    <button type="button" onclick="send_btn_cmd('clr')">Clear Result</button>
    <script>
      /* Load stream: window.location.href.slice(0, -1) removes trailing slash then adds :81/stream */
      window.onload = document.getElementById("vdstream").src = window.location.href.slice(0, -1) + ":81/stream";
      var slider = document.getElementById("mySlider");
      var myTmr;
      let qrcodeval = "...";
      start_timer();

      // When user moves slider, send new slider value to ESP32
      slider.oninput = function() {
        let slider_pwm_val = "S," + slider.value;
        send_cmd(slider_pwm_val);
      }

      // Generic GET to /action?go=...
      function send_cmd(cmds) {
        var xhr = new XMLHttpRequest();
        xhr.open("GET", "/action?go=" + cmds, true);
        xhr.send();
      }

      function start_timer() {
        myTmr = setInterval(myTimer, 500)
      }
      function stop_timer() {
        clearInterval(myTmr)
      }

      // Polling routine to get QR code value via /getqrcodeval every 500ms
      function myTimer() {
        getQRCodeVal();
        textQRCodeVal();
      }
      function textQRCodeVal() {
        document.getElementById("showqrcodeval").innerHTML = qrcodeval;
      }
      function send_btn_cmd(cmds) {
        let btn_cmd = "B," + cmds;
        send_cmd(btn_cmd);
      }

      // Copy text to clipboard (temporary textarea used)
      function CopyQRCodeRslt() {
        var el = document.createElement('textarea');
        el.value = qrcodeval;
        el.setAttribute('readonly', '');
        el.style = {position: 'absolute', left: '-9999px'};
        document.body.appendChild(el);
        el.select();
        document.execCommand('copy');
        document.body.removeChild(el);
        alert("The result of reading the QR Code has been copied.");
      }

      // Fetch the current QR code value from the device
      function getQRCodeVal() {
        var xhttp = new XMLHttpRequest();
        xhttp.onreadystatechange = function() {
          if (this.readyState == 4 && this.status == 200) {
            qrcodeval = this.responseText;
          }
        };
        xhttp.open("GET", "/getqrcodeval", true);
        xhttp.send();
      }
    </script>
  </body>
</html>
)rawliteral";
/* ======================================== */

/* ________________________________________________________________________________
   index_handler - serves the HTML index page when a GET request is received on "/"
*/
static esp_err_t index_handler(httpd_req_t *req){
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________
   stream_handler - serves MJPEG multipart stream (used by <img src="..."> on index page)
   IMPORTANT: When the streaming handler starts, it stops the QRCodeReader task
              (vTaskDelete(QRCodeReader_Task)) because the camera cannot be accessed
              simultaneously from both the streaming code and the QR reader task.
*/
static esp_err_t stream_handler(httpd_req_t *req){
  ws_run = true;                      // indicate streaming is active
  vTaskDelete(QRCodeReader_Task);     // stop the QR reader task so we can use the camera
  Serial.print("stream_handler running on core ");
  Serial.println(xPortGetCoreID());

  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  // Tell client to expect multipart/x-mixed-replace content
  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK){
    return res;
  }

  /* ----------------------------------------
     Main streaming loop:
     - Capture frame
     - Attempt QR detection/decoding for this frame (quirc)
     - Convert frame to JPEG if necessary
     - Send chunked MJPEG parts to the HTTP client
     - Keep running until a send error happens or client disconnects
  */
  while(true){
    ws_run = true;               // keep setting flag inside loop to signal stream is active
    fb = esp_camera_fb_get();    // capture frame from camera
    if (!fb) {
      Serial.println("Camera capture failed (stream_handler)");
      res = ESP_FAIL;
    } else {
      // Initialize quirc and feed the grayscale frame for decoding
      q = quirc_new();
      if (q == NULL){
        Serial.print("can't create quirc object\r\n");  
        continue;
      }
      
      // Match quirc internal buffer to frame resolution
      quirc_resize(q, fb->width, fb->height);
      image = quirc_begin(q, NULL, NULL);     // get pointer to quirc image buffer
      memcpy(image, fb->buf, fb->len);        // copy camera buffer into quirc buffer
      quirc_end(q);                           // finish feeding image

      // Count possible symbols and decode first one (if any)
      int count = quirc_count(q);
      if (count > 0) {
        quirc_extract(q, 0, &code);          // extract first code
        err = quirc_decode(&code, &data);    // decode it
    
        if (err){
          QRCodeResult = "Decoding FAILED";
          Serial.println(QRCodeResult);
        } else {
          Serial.printf("Decoding successful:\n");
          dumpData(&data);                    // stores decoded payload into QRCodeResult
        } 
        Serial.println();
      } 
      
      image = NULL;  
      quirc_destroy(q);                       // free quirc instance for this frame
      
      // If frame is not JPEG, convert it to JPEG before streaming
      if(fb->width > 200){
        if(fb->format != PIXFORMAT_JPEG){
          bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
          esp_camera_fb_return(fb);
          fb = NULL;
          if(!jpeg_converted){
            Serial.println("JPEG compression failed");
            res = ESP_FAIL;
          }
        } else {
          // already JPEG, can use the buffer directly
          _jpg_buf_len = fb->len;
          _jpg_buf = fb->buf;
        }
      }
    }

    // Send multipart chunk header
    if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    // Send JPEG data chunk
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    // Send boundary between parts
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }

    // Cleanup frame buffers
    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if(_jpg_buf){
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if(res != ESP_OK){
      break;   // error while sending -> break loop and close stream
    }
    
    // Increment live counter (used as a heartbeat to detect client activity)
    wsLive_val++;
    if (wsLive_val > 999) wsLive_val = 0;
  }
  /* ---------------------------------------- */
  return res;
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________
   cmd_handler - handles control commands from index page (/action?go=...)
   Supports:
     - Slider commands: "S,VALUE" where VALUE is 0..20 (mapped to 0..255 PWM)
     - Button commands: "B,clr" to clear the displayed QR result
*/
static esp_err_t cmd_handler(httpd_req_t *req){
  char*  buf;
  size_t buf_len;
  char variable[32] = {0,};
   
  // Get query length and allocate buffer
  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char*)malloc(buf_len);
    if(!buf){
      httpd_resp_send_500(req);
      return ESP_FAIL;
    }
    // Copy query string
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "go", variable, sizeof(variable)) == ESP_OK) {
        // 'variable' now contains the value of the 'go' query param
      } else {
        free(buf);
        httpd_resp_send_404(req);
        return ESP_FAIL;
      }
    } else {
      free(buf);
      httpd_resp_send_404(req);
      return ESP_FAIL;
    }
    free(buf);
  } else {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
 
  int res = 0;

  Serial.print("Incoming command : ");
  Serial.println(variable);
  Serial.println();
  String getData = String(variable);
  String resultData = getValue(getData, ',', 0); // first token before comma

  /* ----------------------------------------
     Controlling the LEDs on the ESP32 Cam board with PWM.
     Example: Incoming command = S,10
       - "S" indicates slider command
       - "10" is slider value (0..20 in UI)
     Map slider range 0..20 to PWM 0..255.
  */
  if (resultData == "S") {
    resultData = getValue(getData, ',', 1); // get second token (slider value)
    int pwm = map(resultData.toInt(), 0, 20, 0, 255);

    // NOTE: The original code uses ledcAttach(4, 5000, 8) then ledcWrite(2, pwm).
    // Arduino-ESP32 typical usage is ledcSetup(channel, freq, resolution) and ledcAttachPin(pin, channel)
    // But we keep original calls to avoid changing behaviour.
    ledcAttach(4, 5000, 8);  
    ledcWrite(2,pwm);
  }
  /* ---------------------------------------- */

  /* ----------------------------------------
     Clear QR Code reading result:
     Incoming Command = B,clr
     B = Button, clr = clear
  */
  if (resultData == "B") {
    resultData = getValue(getData, ',', 1);
    if (resultData == "clr") {
      QRCodeResult = "";
    }
  }
  /* ---------------------------------------- */
  
  if(res){
    return httpd_resp_send_500(req);
  }
 
  // Allow cross-origin (useful when testing from different origins)
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________
   qrcoderslt_handler - returns the current QRCodeResult to the web UI.
   The web UI polls /getqrcodeval and displays this string.
*/
static esp_err_t qrcoderslt_handler(httpd_req_t *req){
  if (QRCodeResult != "Decoding FAILED") QRCodeResultSend = QRCodeResult;
  httpd_resp_send(req, QRCodeResultSend.c_str(), HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________
   startCameraWebServer - register URI handlers & start HTTP daemon(s)
   We run two httpd instances:
     - index_httpd on port 80 (serves index and action/getqrcodeval)
     - stream_httpd on port 81 (serves MJPEG stream)
*/
void startCameraWebServer(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;  // index server

  // Set up URI handlers for index server
  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = index_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t cmd_uri = {
    .uri       = "/action",
    .method    = HTTP_GET,
    .handler   = cmd_handler,
    .user_ctx  = NULL
  };

  httpd_uri_t qrcoderslt_uri = {
    .uri       = "/getqrcodeval",
    .method    = HTTP_GET,
    .handler   = qrcoderslt_handler,
    .user_ctx  = NULL
  };

  // Stream URI will be registered on the second httpd instance
  httpd_uri_t stream_uri = {
    .uri       = "/stream",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };
    
  // Start first server (index)
  if (httpd_start(&index_httpd, &config) == ESP_OK) {
      httpd_register_uri_handler(index_httpd, &index_uri);
      httpd_register_uri_handler(index_httpd, &cmd_uri);
      httpd_register_uri_handler(index_httpd, &qrcoderslt_uri);
  }

  // Start second server (stream) on port 81 by incrementing base port and ctrl port
  config.server_port += 1;
  config.ctrl_port += 1;  
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
      httpd_register_uri_handler(stream_httpd, &stream_uri);
  }

  Serial.println();
  Serial.println("Camera Server started successfully");
  Serial.print("Camera Stream Ready! Go to: http://");
  Serial.println(local_ip);
  Serial.println();
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ SETUP */
bool startQRCodeReader = false; // Flag to be set by Arduino (unused in current sketch)

/* The setup configures:
   - brownout disable (optional)
   - serial port
   - LED pins
   - camera parameters & init
   - WiFi AP
   - HTTP servers
   - creates the QRCodeReader task
*/
void setup() {
  // Disable brownout detector (some boards may reset on undervoltage; disabling may help)
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  /* Initialize Serial console using custom pins RXD0/TXD0 */
  Serial.begin(115200,SERIAL_8N1,RXD0,TXD0);
  Serial.setDebugOutput(true);
  Serial.println();

  // Configure LED pins as outputs
  pinMode(LED_OnBoard, OUTPUT);
  pinMode(LED_Green, OUTPUT);
  pinMode(LED_Blue, OUTPUT);

  /* ---------------------------------------- Camera configuration. */
  Serial.println("Start configuring and initializing the camera...");
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0; // used by camera XCLK
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

  // XCLK freq: 10 MHz is used here (common default)
  config.xclk_freq_hz = 10000000;
  // Use grayscale to make quirc QR-decoding faster (less memory & processing)
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  // QVGA is small (320x240) which is a good tradeoff for speed
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 15; // not used for grayscale frames but kept for stream conversion
  config.fb_count = 1;      // single framebuffer (low memory usage)
  
  #if defined(CAMERA_MODEL_ESP_EYE)
    // If using ESP-EYE, these pins are used for buttons and should be pulled-up
    pinMode(13, INPUT_PULLUP);
    pinMode(14, INPUT_PULLUP);
  #endif

  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    ESP.restart(); // restart to attempt recovery
  }
  
  // Ensure frame size is set in sensor (redundant but explicit)
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
  
  Serial.println("Configure and initialize the camera successfully.");
  Serial.println();

  /* ---------------------------------------- Wi-Fi Access Point setup */
  WiFi.softAP(ssid, password); // create AP using configured SSID/password
  WiFi.softAPConfig(local_ip, gateway, subnet); // set AP static IP
  /* ---------------------------------------- */

  // Start HTTP servers and register URI handlers
  startCameraWebServer(); 
  
  // Create FreeRTOS task that continuously reads QR codes when stream isn't active
  createTaskQRCodeReader();
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________ MAIN LOOP */
/*
  The main loop performs a heartbeat check every 'interval' milliseconds to see
  whether the streaming client is still active (using wsLive_val). If the stream dies,
  it will recreate the QRCodeReader task so QR decoding continues when there's no client.
*/
void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (ws_run == true) {                  // if stream was active
      if (get_wsLive_val == true) {
        last_wsLive_val = wsLive_val;
        get_wsLive_val = false;
      }
   
      get_wsLive_interval++;
      if (get_wsLive_interval > 2) {
        get_wsLive_interval = 0;
        get_wsLive_val = true;
        // If wsLive_val didn't change since last check, assume stream ended
        if (wsLive_val == last_wsLive_val) {
          ws_run = false;                  // mark stream not running
          last_wsLive_val = 0;
          createTaskQRCodeReader();        // restart QR reader task
        }
      }
    }
  }
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________
   createTaskQRCodeReader - creates a pinned FreeRTOS task for QRCodeReader()
   - name: "QRCodeReader_Task"
   - stack: 10000 bytes (large because camera + quirc can be memory heavy)
   - priority: 1
   - pinned to core 0
*/
void createTaskQRCodeReader() {
  xTaskCreatePinnedToCore(
             QRCodeReader,          /* Task function. */
             "QRCodeReader_Task",   /* name of task. */
             10000,                 /* Stack size of task (bytes) */
             NULL,                  /* parameter of the task */
             1,                     /* priority of the task */
             &QRCodeReader_Task,    /* Task handle to keep track of created task */
             0);                    /* pin task to core 0 */
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________
   QRCodeReader - FreeRTOS task that continuously reads frames and decodes QR codes
   It runs while ws_run is false (no active stream). It:
     - captures a frame
     - passes it to quirc for detection/decoding
     - updates QRCodeResult if a code is found
*/
void QRCodeReader( void * pvParameters ){
  Serial.print("QRCodeReader running on core ");
  Serial.println(xPortGetCoreID());

  // Continue running until streaming starts (ws_run becomes true)
  while(!ws_run){
      camera_fb_t * fb = NULL;
      q = quirc_new();                 // create quirc instance
      if (q == NULL){
        Serial.print("can't create quirc object\r\n");  
        continue;
      }
      
      fb = esp_camera_fb_get();        // take frame
      if (!fb)
      {
        Serial.println("Camera capture failed (QRCodeReader())");
        continue;
      }
      
      // prepare quirc and copy the camera buffer to it
      quirc_resize(q, fb->width, fb->height);
      image = quirc_begin(q, NULL, NULL);
      memcpy(image, fb->buf, fb->len);
      quirc_end(q);
      
      int count = quirc_count(q);      // number of detected symbols
      if (count > 0) {
        // Extract and decode the first found code
        quirc_extract(q, 0, &code);
        err = quirc_decode(&code, &data);
    
        if (err){
          QRCodeResult = "Decoding FAILED";
          Serial.println(QRCodeResult);
        } else {
          Serial.printf("Decoding successful:\n");
          dumpData(&data);              // store decoded payload to QRCodeResult
        } 
        Serial.println();
      } 
      
      // Return frame buffer to driver & free quirc instance
      esp_camera_fb_return(fb);
      fb = NULL;
      image = NULL;  
      quirc_destroy(q);
  }
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________
   dumpData - helper to convert quirc_data into the QRCodeResult String
   Also prints on serial and provides simple mapping for "small/medium/large" for
   debugging outputs (S/M/L).
*/
void dumpData(const struct quirc_data *data)
{
  // data->payload is a byte array; cast to char* to build string
  QRCodeResult = (const char *)data->payload;
  Serial.println(QRCodeResult);

  // Example: if QR contains the words 'small', 'medium', 'large' print shorthand.
  // These prints are only informative and do not change QRCodeResult itself.
  if(QRCodeResult=="small"){
    Serial.println("S\n");
  }
  else if(QRCodeResult=="medium"){
    Serial.println("M\n");
  }
  if(QRCodeResult=="large"){
    Serial.println("L\n");
  }
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________
   cmd_execution - placeholder function to execute commands based on QRCodeResult.
   NOTE: This function is not called anywhere automatically in this sketch.
         You can call it after reading a QR code if you want QR-driven actions.
*/
void cmd_execution() {
  if (QRCodeResult == "LED BLUE ON") digitalWrite(LED_Blue, HIGH);
  if (QRCodeResult == "LED BLUE OFF") digitalWrite(LED_Blue, LOW);
  if (QRCodeResult == "LED GREEN ON") digitalWrite(LED_Green, HIGH);
  if (QRCodeResult == "LED GREEN OFF") digitalWrite(LED_Green, LOW);
}
/* ________________________________________________________________________________ */

/* ________________________________________________________________________________
   getValue - simple helper to split a string by separator and return token at index
   Example: getValue("S,10", ',', 1) -> "10"
   Source: electroniclinic (kept as original)
*/
String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
