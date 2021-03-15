#include "esp32_digital_led_lib.h"
#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <String.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include "DHT.h"

DynamicJsonBuffer jsonBuffer;

WiFiClient client; //声明一个客户端对象，用于与服务器进行连接

bool flag;        //读取EEPROM联网，是否成功标志位

const IPAddress serverIP(124,70,130,79); //欲访问的地址
uint16_t serverPort = 9998;         //服务器端口号

#define R 25
#define G 26
#define B 27

// 0 关 1开
int state = 0;

int relay = 1;
int relay_gpio = 15;

// 1彩虹 2滚动彩虹
int type = 1;
int lednum = 32;
int old_lednum = 32;
int delays = 20;
int brightness = 64;

int reset = 0;
int reset1 = 0;

int i = 0;

unsigned long int color = 0;

uint64_t chipid;  

void http_server(void *clients);

#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

void espPinMode(int pinNum, int pinDir) {
    if (pinNum == 32 || pinNum == 33) {
        uint64_t gpioBitMask = (pinNum == 32) ? 1ULL<<GPIO_NUM_32 : 1ULL<<GPIO_NUM_33;
        gpio_mode_t gpioMode = (pinDir == OUTPUT) ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT;
        gpio_config_t io_conf;
        io_conf.intr_type = GPIO_INTR_DISABLE;
        io_conf.mode = gpioMode;
        io_conf.pin_bit_mask = gpioBitMask;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
        gpio_config(&io_conf);
    } else pinMode(pinNum, pinDir);
}

void gpioSetup(int gpioNum, int gpioMode, int gpioVal) {
  #if defined(ARDUINO) && ARDUINO >= 100
    espPinMode(gpioNum, gpioMode);
    digitalWrite (gpioNum, gpioVal);
  #elif defined(ESP_PLATFORM)
    gpio_num_t gpioNumNative = static_cast<gpio_num_t>(gpioNum);
    gpio_mode_t gpioModeNative = static_cast<gpio_mode_t>(gpioMode);
    gpio_pad_select_gpio(gpioNumNative);
    gpio_set_direction(gpioNumNative, gpioModeNative);
    gpio_set_level(gpioNumNative, gpioVal);
  #endif
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"  // It's noisy here with `-Wall`
strand_t strand = {.rmtChannel = 0, .gpioNum = 16, .ledType = LED_WS2812B_V2, .brightLimit = 64, .numPixels = 240};
strand_t * STRANDS [] = { &strand };
int STRANDCNT = COUNT_OF(STRANDS); 
#pragma GCC diagnostic pop


void save_wifi(String ssid, String password){
    EEPROM.begin(64);
    String smart_data="{\"ssid\":\""+ssid+"\",\"password\":\""+password+"\"}";
    int size_data = String(smart_data).length();
    for (int addr = 0; addr < size_data + 1; addr++){
        EEPROM.write(addr, toascii(String(smart_data).charAt(addr))); //写数据
    }
    EEPROM.commit(); //保存更改的数据
}

JsonObject& read_wifi(){
    EEPROM.begin(64);
    String line = "";
    for (int addr = 0; addr < 64; addr++){
        int data = EEPROM.read(addr);
        if (addr == 0 and data==255){
            break;
        }
        line += char(data);
        if(data==125){
            break;
        }
    }

    Serial.println(line);
    return jsonBuffer.parseObject(line);
}

bool Config_wifi(const char *wifiname, const char *psw) {
    
    // for (size_t ii = 0; ii <= 10; ii++){}

    int i = 0;
    Serial.printf("Configuring network SSID:%s Password:%s\n",wifiname,psw);
    WiFi.begin(wifiname, psw);
    while (WiFi.status() != WL_CONNECTED){
        digitalWrite (B, 1);
        delay(100);
        digitalWrite (B, 0);
        delay(400);
        Serial.print(".");
        i++;
        if (i == 20){
            Serial.println("Failed to connect");
            break;
        }
    }
    if (WiFi.isConnected()){
        Serial.println("Network connection successful");
        Serial.print("Local IP:");
        Serial.println(WiFi.localIP());
        // break;
        return true;
    }else{
        Serial.println("Connection ERROR restart");
        ESP.restart();
        // return Config_wifi(wifiname, psw);
    }

    // if (!WiFi.isConnected()){
    //     flag = true;
    // }
    
}

void smartConfig(){
    WiFi.mode(WIFI_STA);
    Serial.println("\r\nWait for Smartconfig");
    delay(2000);
    // 等待配网
    WiFi.beginSmartConfig();
    while (1){
        Serial.print(".");
        digitalWrite (G, 1);
        digitalWrite (B, 1);
        delay(100);
        digitalWrite (G, 0);
        digitalWrite (B, 0);
        delay(400);
        if (WiFi.smartConfigDone()){
            Serial.println("SmartConfig Success");
            Serial.printf("SSID:%s\r\n", WiFi.SSID().c_str());
            Serial.printf("PSW:%s\r\n", WiFi.psk().c_str());
            WiFi.setAutoConnect(true); // 设置自动连接
            save_wifi(WiFi.SSID().c_str(), WiFi.psk().c_str());
            Serial.println("End write");
            break;
        }
    }
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

void setup(){

    Serial.begin(115200);
    //初始化按钮
    pinMode(21, INPUT_PULLUP);
    pinMode(relay_gpio, OUTPUT);
    // digitalWrite(relay_gpio, HIGH);??HIGH or LOW??
    digitalWrite(relay_gpio, LOW);

    digitalLeds_initDriver();
    // Init unused outputs low to reduce noise
    gpioSetup(14, OUTPUT, LOW);
    gpioSetup(15, OUTPUT, LOW);
    gpioSetup(25, OUTPUT, LOW);//?
    gpioSetup(26, OUTPUT, LOW);
    gpioSetup(27, OUTPUT, LOW);

    gpioSetup(strand.gpioNum, OUTPUT, LOW);
    digitalLeds_addStrands(STRANDS, STRANDCNT);
    digitalLeds_initDriver();
    digitalLeds_resetPixels(STRANDS, STRANDCNT);

    chipid=ESP.getEfuseMac();
    delay(2000);
    Serial.printf("chipid is :%04X-%08X\n",(uint16_t)(chipid>>32),(uint32_t)chipid);

    flag = false;

    for (size_t ii = 0; ii <= 50; ii++){
        if(!flag and digitalRead(21) == LOW){
            flag = true;
            smartConfig();
            break;
        }
        delay(100);
    }

    JsonObject& data = read_wifi();

    String ssid = data["ssid"];
    String password = data["password"];

    Config_wifi(ssid.c_str(), password.c_str());

    // if(){
        
    // }
    // if (flag){
    //     smartConfig();
    // }

    xTaskCreatePinnedToCore(hb_server,  "hb_server",  2048, NULL, 2 ,  NULL, 1);
    xTaskCreatePinnedToCore(http_server,  "http_server",  2048, NULL, 1 ,  NULL, 1);

    digitalWrite(relay_gpio, HIGH);
    delay(500);
    digitalWrite(relay_gpio, LOW);

    digitalWrite (G, 1);
    colorWipe(pixelFromRGB(255, 0, 0),lednum);
    colorWipe(pixelFromRGB(0, 255, 0),lednum);
    colorWipe(pixelFromRGB(0, 0, 255),lednum);
    delay(2000);
    digitalLeds_resetPixels(STRANDS, STRANDCNT);

    digitalWrite (G, 0);
}


void loop(){
    if(reset==1){
        reset=0;
        digitalLeds_resetPixels(STRANDS, STRANDCNT);
    }
    rainbowCycle(lednum);

    if(WiFi.status() != WL_CONNECTED) {
        digitalWrite (R, 1);
        JsonObject& data = read_wifi();
        String ssid = data["ssid"];
        String password = data["password"];

        WiFi.begin(ssid.c_str(), password.c_str());
        Serial.println("WiFi re-connected!");
        client.stop();
    }else{
        digitalWrite (R, 0);
    }
}

void colorWipe(pixelColor_t c,int lednums) {
    for(uint16_t i=0; i<lednums; i++) {
        STRANDS[0]->pixels[i] = c;
        digitalLeds_drawPixels(STRANDS, STRANDCNT);
        delay(20);
    }
}

void rainbowCycle(int lednums) {
    uint16_t i, j;
    for(j=0; j<256*5; j++) { 
        // 5 cycles of all colors on wheel
        if(state!=0){
            if(reset==1){
                return;
            }
            reset1 = 0;
            if(type==3){
                int blue = (char)color;
                int green = (char)(color >> 8);
                int red = (char)(color >> 16);
                for(i=0; i< lednums; i++) {
                    STRANDS[0]->pixels[i] =  pixelFromRGB(red, green, blue);
                }
            }else{
                for(i=0; i< lednums; i++) {
                    if(type==1){
                        STRANDS[0]->pixels[i] = Wheel((i+j) & 255);
                    }else if(type==2){
                        STRANDS[0]->pixels[i] = Wheel(((i * 256 / lednums) + j) & 255);
                    }
                }
            }
            digitalLeds_drawPixels(STRANDS, STRANDCNT);
        }else{
            if(reset1 == 0){
                for(i=0; i< lednums; i++) {
                    STRANDS[0]->pixels[i] =  pixelFromRGBL(0, 0, 0, 0);
                }
                digitalLeds_drawPixels(STRANDS, STRANDCNT);
            }
            reset1 = 1;
        }
        delay(delays);
    }
}
 
pixelColor_t Wheel(byte WheelPos) {
    WheelPos = 255 - WheelPos;
    if(WheelPos < 85) {
        return pixelFromRGBL(255 - WheelPos * 3, 0, WheelPos * 3, brightness);
    }
    if(WheelPos < 170) {
        WheelPos -= 85;
        return pixelFromRGBL(0, WheelPos * 3, 255 - WheelPos * 3, brightness);

    }
    WheelPos -= 170;
    return pixelFromRGBL(WheelPos * 3, 255 - WheelPos * 3, 0, brightness);

}


void http_server(void *clients){
    for (;;){
        if (client.connect(serverIP, serverPort)){

            client.printf("{\"TYPE\":\"HS\",\"CID\":\"%04X-%08X\"}",(uint16_t)(chipid>>32),(uint32_t)chipid);
            while (client.connected() || client.available()){
                if (client.available()){
                    String line = client.readStringUntil('\n'); //读取数据到换行符
                    JsonObject& data = jsonBuffer.parseObject(line);
                    Serial.println(line);
                    if(data.containsKey("TYPE")){
                        if(data["TYPE"] == "CDATA"){
                            if(data.containsKey("color")){
                                color = atoi(data["color"]);
                            }
                            if(data.containsKey("state")){
                                state = atoi(data["state"]);
                            }
                            if(data.containsKey("relay")){
                                relay = atoi(data["relay"]);
                                if(relay==1){
                                    digitalWrite(relay_gpio, LOW);
                                }else{
                                    digitalWrite(relay_gpio, HIGH);
                                }
                            }
                            if(data.containsKey("type")){
                                type = atoi(data["type"]);
                            }
                            if(data.containsKey("lednum")){
                                lednum = atoi(data["lednum"]);
                                if(old_lednum != lednum){
                                    old_lednum = lednum;
                                    reset=1;
                                }
                                if(lednum>240){
                                    lednum=240;
                                }
                            }
                            if(data.containsKey("delays")){
                                delays = atoi(data["delays"]);
                            }
                            if(data.containsKey("brightness")){
                                brightness = atoi(data["brightness"]);
                                if(brightness>255){
                                    brightness=255;
                                }
                            }
                            client.printf("{\"TYPE\":\"STATE\",\"relay\":\"%d\",\"state\":\"%d\",\"type\":\"%d\",\"lednum\":\"%d\",\"delays\":\"%d\",\"brightness\":\"%d\",\"color\":\"%d\"}",relay,state,type,lednum,delays,brightness,color);
                        }
                    }

                }
            }
            client.stop();
        }
        else{
            client.stop();
        }
        Serial.println("lose connect,reconnecting...");
        delay(5000);
    }
    
}

void hb_server(void *clients){
    for (;;){
        delay(5000);
        client.printf("{\"TYPE\":\"STATE\",\"relay\":\"%d\",\"state\":\"%d\",\"type\":\"%d\",\"lednum\":\"%d\",\"delays\":\"%d\",\"brightness\":\"%d\",\"color\":\"%d\"}",relay,state,type,lednum,delays,brightness,color);
        Serial.printf("{\"TYPE\":\"STATE\",\"relay\":\"%d\",\"state\":\"%d\",\"type\":\"%d\",\"lednum\":\"%d\",\"delays\":\"%d\",\"brightness\":\"%d\",\"color\":\"%d\"}\r\n",relay,state,type,lednum,delays,brightness,color);
    }
}
