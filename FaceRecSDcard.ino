const char* ssid = "A";
const char* password = "99993333Xz";
const char* mdnsName = "camera";

String LineToken = "";  

const char* apssid = "A";
const char* appassword = "99993333Xz";

boolean streamState = false;

#define ENROLL_CONFIRM_TIMES 5

String filepath[7] = {"/1.jpg", "/2.jpg", "/3.jpg", "/4.jpg", "/5.jpg", "/6.jpg", "/7.jpg"};  //1.jpg, 2.jpg, ...., 35.jpg
int image_width = 400;  
int image_height = 296;

#define ENROLL_CONFIRM_TIMES 5
#define FACE_ID_SAVE_NUMBER 7

String recognize_face_matched_name[7] = {"Name0", 
                      "Name1", 
                      "Name2", 
                      "Name3", 
                      "Name4", 
                      "Name5", 
                      "Name6"};
boolean controlState = false;  

#include <WiFi.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
HTTPClient http;
#include "soc/soc.h"             
#include "soc/rtc_cntl_reg.h"    
#include "esp_camera.h"          
#include "img_converters.h"      
#include "fb_gfx.h"              
#include "fd_forward.h"          
#include "fr_forward.h"          
#include "FS.h"                  
#include "SD_MMC.h"              
#include "esp_http_server.h"     
#include "esp_timer.h"           
#include <EEPROM.h> 
#include "index_html.h"

// define the number of bytes you want to access
#define EEPROM_SIZE 1
int pictureNumber = 0;

String Feedback="";   
String Command="",cmd="",P1="",P2="",P3="",P4="",P5="",P6="",P7="",P8="",P9="";
byte ReceiveState=0,cmdState=1,strState=1,questionstate=0,equalstate=0,semicolonstate=0;

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

#define FACE_COLOR_WHITE  0x00FFFFFF
#define FACE_COLOR_BLACK  0x00000000
#define FACE_COLOR_RED    0x000000FF
#define FACE_COLOR_GREEN  0x0000FF00
#define FACE_COLOR_BLUE   0x00FF0000
#define FACE_COLOR_YELLOW (FACE_COLOR_RED | FACE_COLOR_GREEN)
#define FACE_COLOR_CYAN   (FACE_COLOR_BLUE | FACE_COLOR_GREEN)
#define FACE_COLOR_PURPLE (FACE_COLOR_BLUE | FACE_COLOR_RED)

typedef struct {
        httpd_req_t *req;
        size_t len;
} jpg_chunking_t;


typedef struct {
  size_t size;
  size_t index;
  size_t count;
  int sum;
  int * values;
} ra_filter_t;


#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static ra_filter_t ra_filter;
httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

static mtmn_config_t mtmn_config = {0};
static int8_t detection_enabled = 1; 
static int8_t recognition_enabled = 1;
static int8_t is_enrolling = 0;
static face_id_list id_list = {0};
static int8_t flash_value = 0;
int8_t enroll_id = 0;

box_array_t *net_boxes = NULL;

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  

  Serial.begin(115200);
  Serial.setDebugOutput(true);  
  Serial.println();

  camera_config_t config;
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
  config.pixel_format = PIXFORMAT_JPEG;
  
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }
  
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    ESP.restart();
  }

  //Serial.println("Starting SD Card");
  if(!SD_MMC.begin()){
    Serial.println("SD Card Mount Failed");
    return;
  }
  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE){
    Serial.println("No SD Card attached");
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_VGA);    

  ledcAttachPin(4, 4);  
  ledcSetup(4, 5000, 8);

  encrollImageSD();  

  WiFi.mode(WIFI_AP_STA);

  for (int i=0;i<2;i++) {
    WiFi.begin(ssid, password);    
  
    delay(1000);
    Serial.print("Connecting to ");
    Serial.println(ssid);
    
    long int StartTime=millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        if ((StartTime+5000) < millis()) break;
    } 
  
    startCameraServer();
  
    if (WiFi.status() == WL_CONNECTED) {
      WiFi.softAP((WiFi.localIP().toString()+"_"+(String)apssid).c_str(), appassword); 
      Serial.println("");
      Serial.print("Camera Ready! Use: http://");
      Serial.println(WiFi.localIP());
      Serial.println("");
      delay(1000);
      MDNSConnect();
  
      for (int i=0;i<5;i++) {
        ledcWrite(4,10);
        delay(200);
        ledcWrite(4,0);
        delay(200);    
      }
    }
  } 

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.softAP((WiFi.softAPIP().toString()+"_"+(String)apssid).c_str(), appassword);         

    for (int i=0;i<2;i++) {
      ledcWrite(4,10);
      delay(1000);
      ledcWrite(4,0);
      delay(1000);    
    }
  } 
  
  Serial.println("");
  Serial.println("APIP address: ");
  Serial.println(WiFi.softAPIP());  
  Serial.println("");
  
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
}


//void takePicture(){
//  camera_fb_t * fb = NULL;
//  // initialize EEPROM with predefined size
//  EEPROM.begin(EEPROM_SIZE);
//  pictureNumber = EEPROM.read(0) + 1;
//
//  // Path where new picture will be saved in SD Card
//  String path = "/" + String(pictureNumber) +".jpg";
//
//  fs::FS &fs = SD_MMC; 
//  Serial.printf("Picture file name: %s\n", path.c_str());
//  
//  File file = fs.open(path.c_str(), FILE_WRITE);
//  if(!file){
//    Serial.println("Failed to open file in writing mode");
//  } 
//  else {
//    file.write(fb->buf, fb->len); // payload (image), payload length
//    Serial.printf("Saved file to path: %s\n", path.c_str());
//    EEPROM.write(0, pictureNumber);
//    EEPROM.commit();
//  }
//  file.close();  
//
//  Serial.println(" Take PHOTO ");
//}

void loop() {
  if (streamState == false&&recognition_enabled==1) {
    faceRecognition();
  }
  delay(100);
}

void MDNSConnect() {
  if (!MDNS.begin(mdnsName)) {
    Serial.print("mDNS responder not started");
    while (1) {
      delay(1000);
    }
  }
  mdns_hostname_set(mdnsName);
  MDNS.addService("ws", "tcp", 81);
  MDNS.addService("http", "tcp", 80);
  Serial.print("mDNS responder started: http://");
  Serial.print(mdnsName);
  Serial.println(".local");
}

void FaceMatched(int faceid) {
  Serial.println(recognize_face_matched_name[faceid]);
  if (faceid==0) {  
  } else if (faceid==1) { 
  } else if (faceid==2) { 
  } else if (faceid==3) { 
  } else if (faceid==4) { 
  } else if (faceid==5) { 
  } else if (faceid==6) {
  } else {
  }   
}

void FaceNoMatched() {
  
}

void encrollImageSD() {
  mtmn_config.type = FAST;  //FAST or NORMAL
  mtmn_config.min_face = 80;
  mtmn_config.pyramid = 0.707;
  mtmn_config.pyramid_times = 4;
  mtmn_config.p_threshold.score = 0.6;
  mtmn_config.p_threshold.nms = 0.7;
  mtmn_config.p_threshold.candidate_number = 20;
  mtmn_config.r_threshold.score = 0.7;
  mtmn_config.r_threshold.nms = 0.7;
  mtmn_config.r_threshold.candidate_number = 10;
  mtmn_config.o_threshold.score = 0.7;
  mtmn_config.o_threshold.nms = 0.7;
  mtmn_config.o_threshold.candidate_number = 1;

  if(!SD_MMC.begin()){
    Serial.println("Card Mount Failed");
    ESP.restart();
  }  
  
  fs::FS &fs = SD_MMC;
  
  face_id_init(&id_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
  dl_matrix3du_t *aligned_face = NULL;
  int8_t left_sample_face = NULL;
  dl_matrix3du_t *image_matrix = NULL;
  
  for (int j=0;j<sizeof(filepath)/sizeof(*filepath);j++) {
    File file = fs.open(filepath[j]);
    Serial.println("detect file: "+filepath[j]);
    if(!file){
      Serial.println("Failed to open file for reading");
      SD_MMC.end();    
    } else {
      Serial.println("file size: "+String(file.size())); 
      char *buf;
      buf = (char*) malloc (sizeof(char)*file.size());
      long i = 0;
      while (file.available()) {
        buf[i] = file.read(); 
        i++;  
      }
      
      image_matrix = dl_matrix3du_alloc(1, image_width, image_height, 3);
      if (!image_matrix) {
          Serial.println("dl_matrix3du_alloc failed");
      } else {          
          fmt2rgb888((uint8_t*)buf, file.size(), PIXFORMAT_JPEG, image_matrix->item);
          box_array_t *net_boxes = face_detect(image_matrix, &mtmn_config); 
          if (net_boxes){
            Serial.println("faces = " + String(net_boxes->len));  
            Serial.println();
            for (int i = 0; i < net_boxes->len; i++){ 
                Serial.println("index = " + String(i));
                int x = (int)net_boxes->box[i].box_p[0];
                Serial.println("x = " + String(x));
                int y = (int)net_boxes->box[i].box_p[1];
                Serial.println("y = " + String(y));
                int w = (int)net_boxes->box[i].box_p[2] - x + 1;
                Serial.println("width = " + String(w));
                int h = (int)net_boxes->box[i].box_p[3] - y + 1;
                Serial.println("height = " + String(h));
                Serial.println();

                if (i==0) {
                  aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
                  if (align_face(net_boxes, image_matrix, aligned_face) == ESP_OK){
                    if(!aligned_face){
                        Serial.println("Could not allocate face recognition buffer");
                    } 
                    else {
                      int8_t left_sample_face = enroll_face(&id_list, aligned_face);
          
                      if(left_sample_face == (ENROLL_CONFIRM_TIMES - 1)){
                          enroll_id = id_list.tail;
                          Serial.printf("Enrolling Face ID: %d\n", enroll_id);
                      }
                      Serial.printf("Enrolling Face ID: %d sample %d\n", enroll_id, ENROLL_CONFIRM_TIMES - left_sample_face);
                      if (left_sample_face == 0){
                          enroll_id = id_list.tail;
                          //Serial.printf("Enrolled Face ID: %d\n", enroll_id);
                      }
                      Serial.println();
                    }
                    dl_matrix3du_free(aligned_face);
                  }
                }
            } 
            dl_lib_free(net_boxes->score);
            dl_lib_free(net_boxes->box);
            dl_lib_free(net_boxes->landmark);
            dl_lib_free(net_boxes);                                
            net_boxes = NULL;
          }
          else {
            Serial.println("No Face");  
            Serial.println();
          }
          dl_matrix3du_free(image_matrix);
      }
      free(buf);
    }
    file.close();
  } 
  
  SD_MMC.end();
  Serial.println();

  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);   
}

static void rgb_print(dl_matrix3du_t *image_matrix, uint32_t color, const char * str){
    fb_data_t fb;
    fb.width = image_matrix->w;
    fb.height = image_matrix->h;
    fb.data = image_matrix->item;
    fb.bytes_per_pixel = 3;
    fb.format = FB_BGR888;
    fb_gfx_print(&fb, (fb.width - (strlen(str) * 14)) / 2, 10, color, str);
}

static int rgb_printf(dl_matrix3du_t *image_matrix, uint32_t color, const char *format, ...){
    char loc_buf[64];
    char * temp = loc_buf;
    int len;
    va_list arg;
    va_list copy;
    va_start(arg, format);
    va_copy(copy, arg);
    len = vsnprintf(loc_buf, sizeof(loc_buf), format, arg);
    va_end(copy);
    if(len >= sizeof(loc_buf)){
        temp = (char*)malloc(len+1);
        if(temp == NULL) {
            return 0;
        }
    }
    vsnprintf(temp, len+1, format, arg);
    va_end(arg);
    rgb_print(image_matrix, color, temp);
    if(len > 64){
        free(temp);
    }
    return len;
}

static void draw_face_boxes(dl_matrix3du_t *image_matrix, box_array_t *boxes, int face_id){
    int x, y, w, h, i;
    uint32_t color = FACE_COLOR_YELLOW;
    if(face_id < 0){
        color = FACE_COLOR_RED;
    } else if(face_id > 0){
        color = FACE_COLOR_GREEN;
    }
    fb_data_t fb;
    fb.width = image_matrix->w;
    fb.height = image_matrix->h;
    fb.data = image_matrix->item;
    fb.bytes_per_pixel = 3;
    fb.format = FB_BGR888;
    for (i = 0; i < boxes->len; i++){
        // rectangle box
        x = (int)boxes->box[i].box_p[0];
        y = (int)boxes->box[i].box_p[1];
        w = (int)boxes->box[i].box_p[2] - x + 1;
        h = (int)boxes->box[i].box_p[3] - y + 1;
        fb_gfx_drawFastHLine(&fb, x, y, w, color);
        fb_gfx_drawFastHLine(&fb, x, y+h-1, w, color);
        fb_gfx_drawFastVLine(&fb, x, y, h, color);
        fb_gfx_drawFastVLine(&fb, x+w-1, y, h, color);
        #if 0
          // landmark
          int x0, y0, j;
          for (j = 0; j < 10; j+=2) {
              x0 = (int)boxes->landmark[i].landmark_p[j];
              y0 = (int)boxes->landmark[i].landmark_p[j+1];
              fb_gfx_fillRect(&fb, x0, y0, 3, 3, color);
          }
        #endif
    }
}

void faceRecognition() {
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  if (!fb) {
      Serial.println("Camera capture failed");
      ESP.restart();
  }
  size_t out_len, out_width, out_height;
  uint8_t * out_buf;
  bool s;
  dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
  if (!image_matrix) {
      esp_camera_fb_return(fb);
      Serial.println("dl_matrix3du_alloc failed");
      return;
  }
  out_buf = image_matrix->item;
  out_len = fb->width * fb->height * 3;
  out_width = fb->width;
  out_height = fb->height;
  s = fmt2rgb888(fb->buf, fb->len, fb->format, out_buf);
  esp_camera_fb_return(fb);
  if(!s){
      dl_matrix3du_free(image_matrix);
      Serial.println("to rgb888 failed");
      return;
  }
  box_array_t *net_boxes = face_detect(image_matrix, &mtmn_config);  
  if (net_boxes){
      run_face_recognition(image_matrix, net_boxes);  
      dl_lib_free(net_boxes->score);
      dl_lib_free(net_boxes->box);
      dl_lib_free(net_boxes->landmark);
      dl_lib_free(net_boxes);                                
      net_boxes = NULL;
  }
  dl_matrix3du_free(image_matrix);
}

static int run_face_recognition(dl_matrix3du_t *image_matrix, box_array_t *net_boxes){
    dl_matrix3du_t *aligned_face = NULL;
    int matched_id = 0;

    aligned_face = dl_matrix3du_alloc(1, FACE_WIDTH, FACE_HEIGHT, 3);
    if(!aligned_face){
        Serial.println("Could not allocate face recognition buffer");
        return matched_id;
    }
    if (align_face(net_boxes, image_matrix, aligned_face) == ESP_OK){
        if (is_enrolling == 1){  // Enrolled face
            int8_t left_sample_face = enroll_face(&id_list, aligned_face);

            if(left_sample_face == (ENROLL_CONFIRM_TIMES - 1)){
                enroll_id = id_list.tail;
                Serial.printf("Enrolling Face ID: %d\n", enroll_id);
            }
            Serial.printf("Enrolling Face ID: %d sample %d\n", enroll_id, ENROLL_CONFIRM_TIMES - left_sample_face);
            rgb_printf(image_matrix, FACE_COLOR_CYAN, "ID[%u] Sample[%u]", id_list.tail, ENROLL_CONFIRM_TIMES - left_sample_face);
            if (left_sample_face == 0){
                is_enrolling = 0;
                enroll_id = id_list.tail;
                Serial.printf("Enrolled Face ID: %d\n", enroll_id);
            }
        } else {  //Face recognition
            matched_id = recognize_face(&id_list, aligned_face);
            if (matched_id >= 0) {  //check if it is recognized as a registered face
                Serial.printf("Match Face ID: %u\n", matched_id);
                int name_length = sizeof(recognize_face_matched_name) / sizeof(recognize_face_matched_name[0]);
                if (matched_id<name_length) {
                  Serial.printf("Match Face Name: %s\n", recognize_face_matched_name[matched_id]);
                  rgb_printf(image_matrix, FACE_COLOR_GREEN, "[%u] %s", matched_id, recognize_face_matched_name[matched_id]);
                } else {
                  Serial.printf("Match Face Name: No name");
                  rgb_printf(image_matrix, FACE_COLOR_GREEN, "[%u] No Name", matched_id);
                }
                FaceMatched(matched_id);  //Recognize the registered face to execute command control
            } else {  //If it shows an unregistered face
                Serial.println("No Match Found");
                rgb_print(image_matrix, FACE_COLOR_RED, "Intruder Alert!");
                matched_id = -1;
                FaceNoMatched();  //Recognize as a stranger face and execute command control
            }
        }
    } else {  //If a human face is detected, but cannot be recognized
        Serial.println("Face Not Aligned");
        rgb_print(image_matrix, FACE_COLOR_YELLOW, "Human Detected");
    }
    Serial.println();
    
    dl_matrix3du_free(aligned_face);
    return matched_id;
}

static size_t jpg_encode_stream(void * arg, size_t index, const void* data, size_t len){
    jpg_chunking_t *j = (jpg_chunking_t *)arg;
    if(!index){
        j->len = 0;
    }
    if(httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK){
        return 0;
    }
    j->len += len;
    return len;
}

//Image screenshot
static esp_err_t capture_handler(httpd_req_t *req){
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    int64_t fr_start = esp_timer_get_time();

    fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Camera capture failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    size_t out_len, out_width, out_height;
    uint8_t * out_buf;
    bool s;
    bool detected = false;
    int face_id = 0;
    if(!detection_enabled || fb->width > 400){
        size_t fb_len = 0;
        if(fb->format == PIXFORMAT_JPEG){
            fb_len = fb->len;
            res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
        } else {
            jpg_chunking_t jchunk = {req, 0};
            res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk)?ESP_OK:ESP_FAIL;
            httpd_resp_send_chunk(req, NULL, 0);
            fb_len = jchunk.len;
        }
        esp_camera_fb_return(fb);
        int64_t fr_end = esp_timer_get_time();
        Serial.printf("JPG: %uB %ums\n", (uint32_t)(fb_len), (uint32_t)((fr_end - fr_start)/1000));
        return res;
    }

    dl_matrix3du_t *image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);
    if (!image_matrix) {
        esp_camera_fb_return(fb);
        Serial.println("dl_matrix3du_alloc failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    out_buf = image_matrix->item;
    out_len = fb->width * fb->height * 3;
    out_width = fb->width;
    out_height = fb->height;

    s = fmt2rgb888(fb->buf, fb->len, fb->format, out_buf);
    esp_camera_fb_return(fb);
    if(!s){
        dl_matrix3du_free(image_matrix);
        Serial.println("to rgb888 failed");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    box_array_t *net_boxes = face_detect(image_matrix, &mtmn_config);  //Perform face detection to obtain face frame data

    if (net_boxes){
        //Serial.println("faces = " + String(net_boxes->len));  //Number of detected faces
        detected = true;
        if(recognition_enabled){
            face_id = run_face_recognition(image_matrix, net_boxes);  //Perform face recognition
        }
        draw_face_boxes(image_matrix, net_boxes, face_id);  //Draw a face box
        dl_lib_free(net_boxes->score);
        dl_lib_free(net_boxes->box);
        dl_lib_free(net_boxes->landmark);
        dl_lib_free(net_boxes);                                
        net_boxes = NULL;
    }

    jpg_chunking_t jchunk = {req, 0};
    s = fmt2jpg_cb(out_buf, out_len, out_width, out_height, PIXFORMAT_RGB888, 90, jpg_encode_stream, &jchunk);
    dl_matrix3du_free(image_matrix);
    if(!s){
        Serial.println("JPEG compression failed");
        return ESP_FAIL;
    }

    int64_t fr_end = esp_timer_get_time();
    Serial.printf("FACE: %uB %ums %s%d\n", (uint32_t)(jchunk.len), (uint32_t)((fr_end - fr_start)/1000), detected?"DETECTED ":"", face_id);
    return res;
}

//Video streaming
static esp_err_t stream_handler(httpd_req_t *req){
    streamState = true;
  
    camera_fb_t * fb = NULL;
    esp_err_t res = ESP_OK;
    size_t _jpg_buf_len = 0;
    uint8_t * _jpg_buf = NULL;
    char * part_buf[64];
    dl_matrix3du_t *image_matrix = NULL;
    bool detected = false;
    int face_id = 0;
    int64_t fr_start = 0;
    int64_t fr_ready = 0;
    int64_t fr_face = 0;
    int64_t fr_recognize = 0;
    int64_t fr_encode = 0;

    static int64_t last_frame = 0;
    if(!last_frame) {
        last_frame = esp_timer_get_time();
    }

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if(res != ESP_OK){
        streamState = false;
        return res;
    }
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    while(true){
        detected = false;
        face_id = 0;
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Camera capture failed");
            res = ESP_FAIL;
        } else {
            fr_start = esp_timer_get_time();
            fr_ready = fr_start;
            fr_face = fr_start;
            fr_encode = fr_start;
            fr_recognize = fr_start;
            if(!detection_enabled || fb->width > 400){
                if(fb->format != PIXFORMAT_JPEG){
                    bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
                    esp_camera_fb_return(fb);
                    fb = NULL;
                    if(!jpeg_converted){
                        Serial.println("JPEG compression failed");
                        res = ESP_FAIL;
                    }
                } else {
                    _jpg_buf_len = fb->len;
                    _jpg_buf = fb->buf;
                }
            } else {

                image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);

                if (!image_matrix) {
                    Serial.println("dl_matrix3du_alloc failed");
                    res = ESP_FAIL;
                } else {
                    if(!fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item)){
                        Serial.println("fmt2rgb888 failed");
                        res = ESP_FAIL;
                    } else {
                        fr_ready = esp_timer_get_time();
                        box_array_t *net_boxes = NULL;
                        if(detection_enabled){
                            net_boxes = face_detect(image_matrix, &mtmn_config);  //執行人臉偵測取得臉框數據
                        }
                        fr_face = esp_timer_get_time();
                        fr_recognize = fr_face;
                        if (net_boxes || fb->format != PIXFORMAT_JPEG){
                            if(net_boxes){
                                //Serial.println("faces = " + String(net_boxes->len));  //偵測到的人臉數
                                detected = true;
                                if(recognition_enabled){
                                    face_id = run_face_recognition(image_matrix, net_boxes);  //執行人臉辨識
                                }
                                fr_recognize = esp_timer_get_time();
                                draw_face_boxes(image_matrix, net_boxes, face_id);  //繪製人臉方框
                                dl_lib_free(net_boxes->score);
                                dl_lib_free(net_boxes->box);
                                dl_lib_free(net_boxes->landmark);
                                dl_lib_free(net_boxes);                                
                                net_boxes = NULL;
                            }
                            if(!fmt2jpg(image_matrix->item, fb->width*fb->height*3, fb->width, fb->height, PIXFORMAT_RGB888, 90, &_jpg_buf, &_jpg_buf_len)){
                                Serial.println("fmt2jpg failed");
                                res = ESP_FAIL;
                            }
                            esp_camera_fb_return(fb);
                            fb = NULL;
                        } else {
                            _jpg_buf = fb->buf;
                            _jpg_buf_len = fb->len;
                        }
                        fr_encode = esp_timer_get_time();
                    }
                    dl_matrix3du_free(image_matrix);
                }
            }
        }
        if(res == ESP_OK){
            size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
            res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
        }
        if(res == ESP_OK){
            res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        }
        if(fb){
            esp_camera_fb_return(fb);
            fb = NULL;
            _jpg_buf = NULL;
        } else if(_jpg_buf){
            free(_jpg_buf);
            _jpg_buf = NULL;
        }
        if(res != ESP_OK){
            streamState = false;
            break;
        }
        int64_t fr_end = esp_timer_get_time();

        int64_t ready_time = (fr_ready - fr_start)/1000;
        int64_t face_time = (fr_face - fr_ready)/1000;
        int64_t recognize_time = (fr_recognize - fr_face)/1000;
        int64_t encode_time = (fr_encode - fr_recognize)/1000;
        int64_t process_time = (fr_encode - fr_start)/1000;
        
        int64_t frame_time = fr_end - last_frame;
        last_frame = fr_end;
        frame_time /= 1000;
    }

    last_frame = 0;
    streamState = false;
    return res;
}

//Command parameter control
static esp_err_t cmd_handler(httpd_req_t *req){
    char*  buf;  //The parameter string after the access URL
    size_t buf_len;
    char variable[128] = {0,};  //Access parameter var value, can modify the array length.
    char value[128] = {0,};  //The length of the array can be modified by accessing the parameter val value.
    String myCmd = "";
    
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = (char*)malloc(buf_len);
        if(!buf){
            httpd_resp_send_500(req);
            return ESP_FAIL;
        }
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {  
          if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK &&
            httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK) {
          } 
          else {
            myCmd = String(buf);
          }
        }
        free(buf);
    } else {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    Feedback="";Command="";cmd="";P1="";P2="";P3="";P4="";P5="";P6="";P7="";P8="";P9="";
    ReceiveState=0,cmdState=1,strState=1,questionstate=0,equalstate=0,semicolonstate=0;     
    if (myCmd.length()>0) {
      myCmd = "?"+myCmd;  //The parameter string after the URL is converted into a custom parameter format
      for (int i=0;i<myCmd.length();i++) {
        getCommand(char(myCmd.charAt(i)));  //Disassemble parameter string
      }
    }

    if (cmd.length()>0) {
      Serial.println("");
      //Serial.println("Command: "+Command);
      Serial.println("cmd= "+cmd+" ,P1= "+P1+" ,P2= "+P2+" ,P3= "+P3+" ,P4= "+P4+" ,P5= "+P5+" ,P6= "+P6+" ,P7= "+P7+" ,P8= "+P8+" ,P9= "+P9);
      Serial.println(""); 
      
      if (cmd=="your cmd") {
        // You can do anything
      }
      else if (cmd=="facename") {  //Setting person name
        recognize_face_matched_name[P1.toInt()] = P2;
        
        int name_length = sizeof(recognize_face_matched_name) / sizeof(recognize_face_matched_name[0]);
        Feedback="<table style=\"color:white\"><tr><td>matched_id</td><td>name</td></tr>";
        for (int i=0;i<name_length;i++) {
          Feedback+="<tr><td>"+String(i)+"</td><td>"+recognize_face_matched_name[i]+"</td></tr>";
        }
        Feedback+="</table>";
      }  
      else if (cmd=="clearface") {  //Delete registered face
        delete_face(&id_list);
        face_id_init(&id_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
        
        int name_length = sizeof(recognize_face_matched_name) / sizeof(recognize_face_matched_name[0]);
        Feedback="<table style=\"color:white\"><tr><td>matched_id</td><td>name</td></tr>";
        for (int i=0;i<name_length;i++) {
          Feedback+="<tr><td>"+String(i)+"</td><td>"+recognize_face_matched_name[i]+"</td></tr>";
        }
        Feedback+="</table>";
        
      }      
      else {
        Feedback="Command is not defined";
      }

      if (Feedback=="") Feedback=Command;  //If the return data is not set, return the Command value
    
      const char *resp = Feedback.c_str();
      httpd_resp_set_type(req, "text/html");  //Set the format of the returned data
      httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");  //Allow cross-domain reading
      return httpd_resp_send(req, resp, strlen(resp));
    } 
    else {
      int val = atoi(value);  //Convert the parameter val value to an integer, the original char format
      sensor_t * s = esp_camera_sensor_get();
      int res = 0;
      
      if(!strcmp(variable, "framesize")) {  //Resolution
          if(s->pixformat == PIXFORMAT_JPEG) res = s->set_framesize(s, (framesize_t)val);
      }
      else if(!strcmp(variable, "quality")) res = s->set_quality(s, val);  
      else if(!strcmp(variable, "contrast")) res = s->set_contrast(s, val);  
      else if(!strcmp(variable, "brightness")) res = s->set_brightness(s, val);  
      else if(!strcmp(variable, "face_detect")) {  
          detection_enabled = val;
          if(!detection_enabled) {
              recognition_enabled = 0;
          }
      }
      else if(!strcmp(variable, "face_enroll")) is_enrolling = val;  //Face registration
      else if(!strcmp(variable, "face_recognize")) {  //Face recognition
          recognition_enabled = val;
          if(recognition_enabled){
              detection_enabled = val;
          }
      }
      else if(!strcmp(variable, "flash")) {  //flash
        ledcWrite(4,val);
        flash_value = val;
      }    
      else {
          res = -1;
      }
  
      if(res){
          return httpd_resp_send_500(req);
      }
  
      if (buf) {
        Feedback = String(buf);
        const char *resp = Feedback.c_str();
        httpd_resp_set_type(req, "text/html");
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        return httpd_resp_send(req, resp, strlen(resp));  //Return parameter string
      }
      else {
        httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
        return httpd_resp_send(req, NULL, 0);
      }
    }
}

//Display the status of video parameters (must return json format)
static esp_err_t status_handler(httpd_req_t *req){
    static char json_response[1024];

    sensor_t * s = esp_camera_sensor_get();
    char * p = json_response;
    *p++ = '{';

    p+=sprintf(p, "\"framesize\":%u,", s->status.framesize);
    p+=sprintf(p, "\"quality\":%u,", s->status.quality);
    p+=sprintf(p, "\"brightness\":%d,", s->status.brightness);
    p+=sprintf(p, "\"contrast\":%d,", s->status.contrast);
    p+=sprintf(p, "\"face_detect\":%u,", detection_enabled);
    p+=sprintf(p, "\"face_enroll\":%u,", is_enrolling);
    p+=sprintf(p, "\"face_recognize\":%u,", recognition_enabled);
    p+=sprintf(p, "\"flash\":%u", flash_value);
    *p++ = '}';
    *p++ = 0;
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, json_response, strlen(json_response));
}


static esp_err_t index_handler(httpd_req_t *req){
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");     
    return httpd_resp_send(req, (const char *)INDEX_HTML, strlen(INDEX_HTML));
}

void startCameraServer(){
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();  //Server Port can be set in HTTPD_DEFAULT_CONFIG()

    //Customizable URL path corresponding to the executed function
    httpd_uri_t index_uri = {
        .uri       = "/",             
        .method    = HTTP_GET,
        .handler   = index_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t status_uri = {
        .uri       = "/status",       //http://192.168.xxx.xxx/status
        .method    = HTTP_GET,
        .handler   = status_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t cmd_uri = {
        .uri       = "/control",      //http://192.168.xxx.xxx/control
        .method    = HTTP_GET,
        .handler   = cmd_handler,
        .user_ctx  = NULL
    };

    httpd_uri_t capture_uri = {
        .uri       = "/capture",      //http://192.168.xxx.xxx/capture
        .method    = HTTP_GET,
        .handler   = capture_handler,
        .user_ctx  = NULL
    };

   httpd_uri_t stream_uri = {
        .uri       = "/stream",       //http://192.168.xxx.xxx:81/stream
        .method    = HTTP_GET,
        .handler   = stream_handler,
        .user_ctx  = NULL
    };
    
    Serial.printf("Starting web server on port: '%d'\n", config.server_port);  //TCP Port
    if (httpd_start(&camera_httpd, &config) == ESP_OK) {
        //Register a custom URL path corresponding to the executed function
        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &cmd_uri);
        httpd_register_uri_handler(camera_httpd, &status_uri);
        httpd_register_uri_handler(camera_httpd, &capture_uri);
    }

    config.server_port += 1;  //Stream Port
    config.ctrl_port += 1;  //UDP Port
    Serial.printf("Starting stream server on port: '%d'\n", config.server_port);
    if (httpd_start(&stream_httpd, &config) == ESP_OK) {
        httpd_register_uri_handler(stream_httpd, &stream_uri);
    }
}

//Disassemble command string into variable
void getCommand(char c)
{
  if (c=='?') ReceiveState=1;
  if ((c==' ')||(c=='\r')||(c=='\n')) ReceiveState=0;
  
  if (ReceiveState==1)
  {
    Command=Command+String(c);
    
    if (c=='=') cmdState=0;
    if (c==';') strState++;
  
    if ((cmdState==1)&&((c!='?')||(questionstate==1))) cmd=cmd+String(c);
    if ((cmdState==0)&&(strState==1)&&((c!='=')||(equalstate==1))) P1=P1+String(c);
    if ((cmdState==0)&&(strState==2)&&(c!=';')) P2=P2+String(c);
    if ((cmdState==0)&&(strState==3)&&(c!=';')) P3=P3+String(c);
    if ((cmdState==0)&&(strState==4)&&(c!=';')) P4=P4+String(c);
    if ((cmdState==0)&&(strState==5)&&(c!=';')) P5=P5+String(c);
    if ((cmdState==0)&&(strState==6)&&(c!=';')) P6=P6+String(c);
    if ((cmdState==0)&&(strState==7)&&(c!=';')) P7=P7+String(c);
    if ((cmdState==0)&&(strState==8)&&(c!=';')) P8=P8+String(c);
    if ((cmdState==0)&&(strState>=9)&&((c!=';')||(semicolonstate==1))) P9=P9+String(c);
    
    if (c=='?') questionstate=1;
    if (c=='=') equalstate=1;
    if ((strState>=9)&&(c==';')) semicolonstate=1;
  }
}
