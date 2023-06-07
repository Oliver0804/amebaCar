//
const byte header[4] = { 0x53, 0x59, 0x82, 0x02 };  //數據幀頭
const byte objectDataLength = 11;                   //每個目標點數據長度

#include "WiFi.h"
#include "StreamIO.h"
#include "VideoStream.h"
#include "RTSP.h"
#include "NNObjectDetection.h"
#include "VideoStreamOverlay.h"
#include "ObjectClassList.h"

#define CHANNEL 0
#define CHANNELNN 3

// Lower resolution for NN processing
#define NNWIDTH 576
#define NNHEIGHT 320

// OSD layers
#define RECTLAYER OSDLAYER0
#define TEXTLAYER OSDLAYER1

VideoSetting config(VIDEO_FHD, 30, VIDEO_H264, 0);
VideoSetting configNN(NNWIDTH, NNHEIGHT, 10, VIDEO_RGB, 0);
NNObjectDetection ObjDet;
RTSP rtsp;
StreamIO videoStreamer(1, 1);
StreamIO videoStreamerNN(1, 1);

char ssid[] = "OpenWrt";   // your network SSID (name)
char pass[] = "08041218";  // your network password
int status = WL_IDLE_STATUS;

IPAddress ip;
int rtsp_portnum;

//mmwave
int16_t mmwave_x_pos = 0;
int16_t mmwave_y_pos = 0;
int16_t mmwave_distance = 0;

//OSD
// raw image size
#define FHD_WIDTH 1920
#define FHD_HEIGHT 1080



void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);
  // attmpt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attmpting to connect to WPA SSID: ");
    Serial.println(ssid);
    status = WiFi.begin(ssid, pass);

    // wait 2 seconds for connection:
    delay(2000);
  }
  ip = WiFi.localIP();

  // Configure camera video channels with video format information
  // Adjust the bitrate based on your WiFi network quality
  config.setBitrate(2 * 1024 * 1024);  // Recommend to use 2Mbps for RTSP streaming to prevent network congestion
  Camera.configVideoChannel(CHANNEL, config);
  Camera.configVideoChannel(CHANNELNN, configNN);
  Camera.videoInit();

  // Configure RTSP with corresponding video format information
  rtsp.configVideo(config);
  rtsp.begin();
  rtsp_portnum = rtsp.getPort();

  // Configure object detection with corresponding video format information
  // Select Neural Network(NN) task and models
  ObjDet.configVideo(configNN);
  ObjDet.setResultCallback(ODPostProcess);
  ObjDet.modelSelect(OBJECT_DETECTION, DEFAULT_YOLOV4TINY, NA_MODEL, NA_MODEL);
  ObjDet.begin();

  // Configure StreamIO object to stream data from video channel to RTSP
  videoStreamer.registerInput(Camera.getStream(CHANNEL));
  videoStreamer.registerOutput(rtsp);
  if (videoStreamer.begin() != 0) {
    Serial.println("StreamIO link start failed");
  }

  // Start data stream from video channel
  Camera.channelBegin(CHANNEL);

  // Configure StreamIO object to stream data from RGB video channel to object detection
  videoStreamerNN.registerInput(Camera.getStream(CHANNELNN));
  videoStreamerNN.setStackSize();
  videoStreamerNN.setTaskPriority();
  videoStreamerNN.registerOutput(ObjDet);
  if (videoStreamerNN.begin() != 0) {
    Serial.println("StreamIO link start failed");
  }

  // Start video channel for NN
  Camera.channelBegin(CHANNELNN);

  // Start OSD drawing on RTSP video channel
  OSD.configVideo(CHANNEL, config);
  OSD.begin();
}

void loop() {
  //Serial1.println("Helloworld");
  if (Serial1.available()) {
    // 讀取第一個字節並確認是否匹配
    if (Serial1.read() == header[0]) {
      // 如果有足夠的數據則繼續讀取
      if (Serial1.available() >= 3) {
        if (Serial1.read() == header[1] && Serial1.read() == header[2] && Serial1.read() == header[3]) {
          if (Serial1.available() >= 2) {
            byte length[2];
            length[0] = Serial1.read();
            length[1] = Serial1.read();

            int dataLength = (length[0] << 8) + length[1];

            // 確認可用數據是否足夠
            if (Serial1.available() >= dataLength) {
              if (dataLength == 0) {
                // 沒有目標物，跳過後續兩個byte
                Serial1.read();
                Serial1.read();
              } else {
                // 有目標物，進行解析
                while (dataLength >= objectDataLength) {
                  byte objectIndex = Serial1.read();
                  byte objectSize = Serial1.read();
                  byte objectFeature = Serial1.read();

                  byte xPos[2];
                  xPos[1] = Serial1.read();
                  xPos[0] = Serial1.read();
                  byte yPos[2];
                  yPos[1] = Serial1.read();
                  yPos[0] = Serial1.read();
                  byte height[2];
                  height[1] = Serial1.read();
                  height[0] = Serial1.read();
                  byte speed[2];
                  speed[1] = Serial1.read();
                  speed[0] = Serial1.read();

                  // 你可以在這裡添加對物體數據的處理
                  // 比如打印出來
                  Serial.println("===========");

                  int tmp = int((xPos[1] << 8) + xPos[0]);
                  if (tmp > 32767) {
                    mmwave_x_pos = 32767 - tmp;
                  } else {
                    mmwave_x_pos = tmp;
                  }
                  tmp = int((yPos[1] << 8) + yPos[0]);
                  if (tmp > 32767) {
                    mmwave_y_pos = 32767 - tmp;
                  } else {
                    mmwave_y_pos = tmp;
                  }
                  Serial.println("X pos: " + String(mmwave_x_pos));
                  Serial.println("Y pos: " + String(mmwave_y_pos));
                  Serial.println("height: " + String((height[1] << 8) + height[0]));
                  Serial.println("speed: " + String((speed[0] << 8) + speed[1]));

                  dataLength -= objectDataLength;
                }
              }
            }
          }
        }
      }
    }
  }
}

// User callback function for post processing of object detection results
void ODPostProcess(std::vector<ObjectDetectionResult> results) {
  uint16_t im_h = config.height();
  uint16_t im_w = config.width();
  /*
    Serial.print("Network URL for RTSP Streaming: ");
    Serial.print("rtsp://");
    Serial.print(ip);
    Serial.print(":");
    Serial.println(rtsp_portnum);
    Serial.println(" ");

    printf("Total number of objects detected = %d\r\n", ObjDet.getResultCount());
    */
  OSD.createBitmap(CHANNEL, RECTLAYER);
  OSD.createBitmap(CHANNEL, TEXTLAYER);
  if (ObjDet.getResultCount() > 0) {
    for (uint16_t i = 0; i < ObjDet.getResultCount(); i++) {
      int obj_type = results[i].type();
      if (itemList[obj_type].filter) {  // check if item should be ignored
        ObjectDetectionResult item = results[i];
        // Result coordinates are floats ranging from 0.00 to 1.00
        // Multiply with RTSP resolution to get coordinates in pixels
        int xmin = (int)(item.xMin() * im_w);
        int xmax = (int)(item.xMax() * im_w);
        int ymin = (int)(item.yMin() * im_h);
        int ymax = (int)(item.yMax() * im_h);

        // Draw boundary box
        printf("Item %d %s:\t%d %d %d %d\n\r", i, itemList[obj_type].objectName, xmin, xmax, ymin, ymax);
        OSD.drawRect(CHANNEL, xmin, ymin, xmax, ymax, 3, OSD_COLOR_WHITE, RECTLAYER);

        // Print identification text
        char text_str[20];
        snprintf(text_str, sizeof(text_str), "%s %d", itemList[obj_type].objectName, item.score());
        OSD.drawText(CHANNEL, xmin, ymin - OSD.getTextHeight(CHANNEL), text_str, OSD_COLOR_CYAN, TEXTLAYER);
      }
    }
  }
  char mmwave_text_str[20];
  snprintf(mmwave_text_str, sizeof(mmwave_text_str), "X:%d Dist:%d", mmwave_x_pos, mmwave_y_pos);
  OSD.drawText(CHANNEL, (FHD_WIDTH / 2) - 200, (FHD_HEIGHT)-OSD.getTextHeight(CHANNEL), mmwave_text_str, OSD_COLOR_CYAN, TEXTLAYER);
  //OSD.drawLine(CHANNEL,(FHD_WIDTH/2), (FHD_HEIGHT),0,0 ,3, OSD_COLOR_CYAN,TEXTLAYER);
  OSD.drawLine(CHANNEL, 10, 10, 100, 100, 3, OSD_COLOR_CYAN, TEXTLAYER);
  delay(10);
  OSD.drawLine(CHANNEL, 70, 70, 9, 9, 5, OSD_COLOR_CYAN, TEXTLAYER);

  OSD.update(CHANNEL, RECTLAYER);
  OSD.update(CHANNEL, TEXTLAYER);
}
