#include <math.h>
#include <string.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

/* schematik-debug: trace calibration command validation and NVS read-back */
static const char *TAG = "airride";

typedef struct { bool active,pulse,settled; float goal; uint32_t start,pulseAt,next,settledAt; } Job;
typedef struct __attribute__((packed)) {
  uint32_t magic; uint32_t format; uint32_t generation;
  float zero[5];
  uint16_t wheelLimitDbar[4]; uint16_t tankLimitDbar;
  uint16_t compressorOnDbar; uint16_t targetToleranceDbar;
  uint16_t autoTimeoutSec; uint16_t compressorMaxRunSec;
  uint16_t manualTimeoutMs; uint16_t calibrationReferenceDbar[5];
  uint8_t calibratedMask; uint32_t check;
} ControllerConfig;
typedef struct __attribute__((packed)) { uint32_t magic; uint32_t revision; float zero[5]; uint32_t check; } LegacyCalibrationStore;
typedef struct __attribute__((packed)) {
  uint8_t marker,version; uint32_t generation;
  uint16_t wheelLimitDbar[4]; uint16_t tankLimitDbar;
  uint16_t compressorOnDbar; uint16_t targetToleranceDbar;
  uint16_t autoTimeoutSec; uint16_t compressorMaxRunSec; uint16_t manualTimeoutMs;
  uint8_t calibratedMask; uint16_t calibrationReferenceDbar[5]; int16_t zeroMilliVolts[5];
  uint32_t check;
} SettingsFrame;
typedef struct __attribute__((packed)) { uint8_t marker,opcode,status; uint32_t generation; } AckFrame;
typedef struct { uint8_t len; uint8_t data[32]; } Command;

static const gpio_num_t fill[4]={GPIO_NUM_4,GPIO_NUM_14,GPIO_NUM_17,GPIO_NUM_19};
static const gpio_num_t dump[4]={GPIO_NUM_13,GPIO_NUM_16,GPIO_NUM_18,GPIO_NUM_21};
static const adc_channel_t channel[5]={ADC_CHANNEL_4,ADC_CHANNEL_5,ADC_CHANNEL_6,ADC_CHANNEL_7,ADC_CHANNEL_0};
static const uint8_t pmk[16]={0x41,0x69,0x72,0x52,0x69,0x64,0x65,0x2D,0x32,0x30,0x32,0x36,0x2D,0x4B,0x31,0x21};
static const uint8_t lmk[16]={0x41,0x69,0x72,0x52,0x69,0x64,0x65,0x2D,0x32,0x30,0x32,0x36,0x2D,0x4B,0x31,0x21};
static adc_oneshot_unit_handle_t adc; static Job jobs[4]; static ControllerConfig config;
static uint32_t manual[4],lastBeat,compressorAt,lastTele; static bool linked,peer,encrypted,compressor,lockout,teleNow,settingsPending;
static uint8_t peerMac[6],pendingPeerMac[6]; static uint32_t pendingEncryptionAt; static bool pendingEncryption; static portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
static QueueHandle_t commandQueue;
static const float sensorFullScalePsi[5]={150.0f,150.0f,150.0f,150.0f,300.0f};
#define CFG_MAGIC 0x41524944UL
#define CFG_FORMAT 2UL
#define CFG_KEY "cfg_v2"
#define LEGACY_CAL_MAGIC 0x41524944UL
#define LEGACY_CAL_REVISION 1UL
#define ABS_WHEEL_LIMIT_DBAR 100
#define ABS_TANK_LIMIT_DBAR 150

typedef struct __attribute__((packed)) { uint8_t marker; float bar[5]; uint8_t flags; } Telemetry;
typedef struct __attribute__((packed)) {
  uint8_t marker;
  uint8_t validMask;
  uint16_t bar10[5];
} CalibrationState;
static uint16_t calBar10[5]={0,0,0,0,0};
static uint8_t calValidMask=0;
static uint32_t now(void){return xTaskGetTickCount()*portTICK_PERIOD_MS;}
static bool elapsed(uint32_t n,uint32_t a,uint32_t t){return (uint32_t)(n-a)>=t;}
static uint32_t checkBytes(const uint8_t *b,size_t len){uint32_t v=0x6D2B79F5UL;for(size_t i=0;i<len;i++){v=(v<<5)|(v>>27);v^=b[i];}return v;}
static uint32_t configCheck(const ControllerConfig *s){return checkBytes((const uint8_t*)s,offsetof(ControllerConfig,check));}
static uint32_t legacyCheck(const LegacyCalibrationStore *s){return checkBytes((const uint8_t*)s,offsetof(LegacyCalibrationStore,check));}
static bool validConfig(const ControllerConfig *c){
  if(c->magic!=CFG_MAGIC||c->format!=CFG_FORMAT||c->check!=configCheck(c))return false;
  for(uint8_t i=0;i<5;i++)if(!isfinite(c->zero[i])||c->zero[i]<.2f||c->zero[i]>.8f)return false;
  for(uint8_t i=0;i<4;i++)if(c->wheelLimitDbar[i]<1||c->wheelLimitDbar[i]>ABS_WHEEL_LIMIT_DBAR)return false;
  if(c->tankLimitDbar<1||c->tankLimitDbar>ABS_TANK_LIMIT_DBAR)return false;
  if(c->compressorOnDbar>=c->tankLimitDbar||c->targetToleranceDbar<1||c->targetToleranceDbar>20)return false;
  if(c->autoTimeoutSec<1||c->autoTimeoutSec>30||c->compressorMaxRunSec<1||c->compressorMaxRunSec>180)return false;
  if(c->manualTimeoutMs<250||c->manualTimeoutMs>2000||(c->calibratedMask&0xE0))return false;
  for(uint8_t i=0;i<5;i++)if(c->calibrationReferenceDbar[i]>(i==4?150:100))return false;
  return true;
}
static void defaultConfig(ControllerConfig *c){
  memset(c,0,sizeof(*c));c->magic=CFG_MAGIC;c->format=CFG_FORMAT;
  for(uint8_t i=0;i<5;i++)c->zero[i]=.5f;
  for(uint8_t i=0;i<4;i++)c->wheelLimitDbar[i]=100;
  c->tankLimitDbar=150;c->compressorOnDbar=120;c->targetToleranceDbar=2;
  c->autoTimeoutSec=30;c->compressorMaxRunSec=180;c->manualTimeoutMs=2000;
}
static void syncCalibrationStateFromConfig(void){
  calValidMask=(uint8_t)(config.calibratedMask&0x1FU);
  for(uint8_t i=0;i<5;i++)calBar10[i]=(calValidMask&(1U<<i))?config.calibrationReferenceDbar[i]:0;
}
static bool persistConfig(ControllerConfig next){
  next.magic=CFG_MAGIC;next.format=CFG_FORMAT;next.generation=config.generation+1;next.check=configCheck(&next);
  if(!validConfig(&next))return false;
  nvs_handle_t h;esp_err_t e=nvs_open("airride",NVS_READWRITE,&h);
  if(e!=ESP_OK){ESP_LOGE(TAG,"NVS open failed: %s",esp_err_to_name(e));return false;}
  e=nvs_set_blob(h,CFG_KEY,&next,sizeof(next));
  if(e==ESP_OK)e=nvs_commit(h);
  if(e!=ESP_OK)ESP_LOGE(TAG,"NVS write/commit failed: %s",esp_err_to_name(e));
  ControllerConfig verify={0};size_t bytes=sizeof(verify);
  if(e==ESP_OK)e=nvs_get_blob(h,CFG_KEY,&verify,&bytes);
  bool ok=e==ESP_OK&&bytes==sizeof(verify)&&validConfig(&verify)&&memcmp(&next,&verify,sizeof(next))==0;
  if(!ok)ESP_LOGE(TAG,"NVS read-back failed: err=%s bytes=%u valid=%d",esp_err_to_name(e),(unsigned)bytes,validConfig(&verify));
  nvs_close(h);if(ok){config=verify;ESP_LOGI(TAG,"NVS config verified generation=%lu",(unsigned long)config.generation);}return ok;
}
static void loadConfig(void){
  defaultConfig(&config);calValidMask=0;memset(calBar10,0,sizeof(calBar10));nvs_handle_t h;
  if(nvs_open("airride",NVS_READWRITE,&h)!=ESP_OK)return;
  ControllerConfig saved={0};size_t bytes=sizeof(saved);
  if(nvs_get_blob(h,CFG_KEY,&saved,&bytes)==ESP_OK&&bytes==sizeof(saved)&&validConfig(&saved)){config=saved;syncCalibrationStateFromConfig();nvs_close(h);return;}
  LegacyCalibrationStore old={0};bytes=sizeof(old);
  if(nvs_get_blob(h,"cal_v1",&old,&bytes)==ESP_OK&&bytes==sizeof(old)&&old.magic==LEGACY_CAL_MAGIC&&old.revision==LEGACY_CAL_REVISION&&old.check==legacyCheck(&old)){
    bool good=true;for(uint8_t i=0;i<5;i++)good=good&&isfinite(old.zero[i])&&old.zero[i]>=.2f&&old.zero[i]<=.8f;
    if(good)memcpy(config.zero,old.zero,sizeof(config.zero));
  }
  nvs_close(h);syncCalibrationStateFromConfig();ControllerConfig initial=config;persistConfig(initial);
}
static bool good(uint8_t s,float x){return s<5&&isfinite(x)&&x>=-5.0f&&x<=sensorFullScalePsi[s];}
static float wheelLimitPsi(uint8_t w){return ((float)config.wheelLimitDbar[w]/10.0f)/.0689476f;}
static float tankLimitPsi(void){return ((float)config.tankLimitDbar/10.0f)/.0689476f;}
static void set(gpio_num_t p,bool v){gpio_set_level(p,v?1:0);}
static void compressorSet(bool v){if(compressor==v)return;set(GPIO_NUM_22,v);compressor=v;if(v)compressorAt=now();}
static void closeWheel(uint8_t w){set(fill[w],0);set(dump[w],0);manual[w]=0;}
static void cancel(uint8_t w){closeWheel(w);memset(&jobs[w],0,sizeof(Job));}
static void stopAll(void){for(uint8_t w=0;w<4;w++)cancel(w);compressorSet(0);}
static float pressure(uint8_t s){int raw;if(adc_oneshot_read(adc,channel[s],&raw)!=ESP_OK)return NAN;return (((float)raw*3.3f/4095.0f)*(28.0f/18.0f)-config.zero[s])*(sensorFullScalePsi[s]/4.0f);}
static bool addPeer(const uint8_t *mac,bool crypt){if(peer)esp_now_del_peer(peerMac);esp_now_peer_info_t p={0};memcpy(p.peer_addr,mac,6);p.channel=1;p.ifidx=WIFI_IF_STA;p.encrypt=crypt;if(crypt)memcpy(p.lmk,lmk,16);if(esp_now_add_peer(&p)!=ESP_OK)return false;memcpy(peerMac,mac,6);peer=true;encrypted=crypt;return true;}
static void sendCalibrationState(void){
  if(!linked||!peer||!encrypted)return;
  CalibrationState state={.marker=0xA2,.validMask=calValidMask};
  memcpy(state.bar10,calBar10,sizeof(calBar10));
  esp_now_send(peerMac,(const uint8_t *)&state,sizeof(state));
}
static void sendSettings(void){
  if(!linked||!encrypted){
    return;
  }
  SettingsFrame f={0};f.marker=0xA2;f.version=1;f.generation=config.generation;
  memcpy(f.wheelLimitDbar,config.wheelLimitDbar,sizeof(f.wheelLimitDbar));f.tankLimitDbar=config.tankLimitDbar;
  f.compressorOnDbar=config.compressorOnDbar;f.targetToleranceDbar=config.targetToleranceDbar;
  f.autoTimeoutSec=config.autoTimeoutSec;f.compressorMaxRunSec=config.compressorMaxRunSec;f.manualTimeoutMs=config.manualTimeoutMs;
  f.calibratedMask=config.calibratedMask;memcpy(f.calibrationReferenceDbar,config.calibrationReferenceDbar,sizeof(f.calibrationReferenceDbar));
  for(uint8_t i=0;i<5;i++)f.zeroMilliVolts[i]=(int16_t)lroundf(config.zero[i]*1000.0f);
  f.check=checkBytes((const uint8_t*)&f,offsetof(SettingsFrame,check));esp_now_send(peerMac,(const uint8_t*)&f,sizeof(f));
}
static void sendAck(uint8_t opcode,bool ok){if(!linked||!encrypted)return;AckFrame a={.marker=0xA3,.opcode=opcode,.status=(uint8_t)(ok?1:0),.generation=config.generation};esp_now_send(peerMac,(const uint8_t*)&a,sizeof(a));}
static void rx(const esp_now_recv_info_t *info,const uint8_t *data,int len){
  if(!info||!data||len<2)return;
  ESP_LOGI(TAG,"schematik-debug RX len=%d src=%02X:%02X:%02X:%02X:%02X:%02X data=%02X %02X",len,info->src_addr[0],info->src_addr[1],info->src_addr[2],info->src_addr[3],info->src_addr[4],info->src_addr[5],data[0],data[1]);
  if(len==2&&data[0]==0xE1&&data[1]==1&&!linked){
    if(!addPeer(info->src_addr,false)){ESP_LOGE(TAG,"schematik-debug discovery: open peer add failed");return;}
    uint8_t r[2]={0xE1,0x81};esp_err_t sent=esp_now_send(peerMac,r,sizeof(r));
    if(sent==ESP_OK){
      portENTER_CRITICAL(&mux);memcpy(pendingPeerMac,info->src_addr,6);pendingEncryptionAt=now();pendingEncryption=true;portEXIT_CRITICAL(&mux);
      ESP_LOGI(TAG,"schematik-debug discovery reply queued; defer encrypted peer switch");
    }else ESP_LOGE(TAG,"schematik-debug discovery reply send failed: %s",esp_err_to_name(sent));
    return;
  }
  if(len==2&&data[0]==0xE1&&data[1]==2&&peer&&encrypted&&!memcmp(peerMac,info->src_addr,6)){
    if(!linked){linked=true;settingsPending=true;}lastBeat=now();teleNow=true;return;
  }
  if(!linked||!peer||!encrypted||memcmp(peerMac,info->src_addr,6)||data[0]!=0xB1||len>32)return;
  lastBeat=now();
  if(len==2&&data[0]==0xB1&&data[1]==0x43){sendCalibrationState();return;}
  Command c={.len=(uint8_t)len};memcpy(c.data,data,(size_t)len);
  BaseType_t queued=commandQueue?xQueueSend(commandQueue,&c,0):pdFALSE;
  ESP_LOGI(TAG,"schematik-debug B1 command queue=%s opcode=%02X len=%d",queued==pdTRUE?"ok":"FULL_OR_UNREADY",data[1],len);
}
static bool calibrate(uint8_t s,uint16_t bar10){
  if(s>4||bar10>(s==4?150:100)){
    return false;
  }
  ESP_LOGI(TAG,"Calibration request sensor=%u reference_dbar=%u",(unsigned)s,(unsigned)bar10);
  stopAll();float sum=0;
  for(uint8_t i=0;i<32;i++){int r;esp_err_t e=adc_oneshot_read(adc,channel[s],&r);if(e!=ESP_OK){ESP_LOGE(TAG,"Calibration ADC read failed: %s",esp_err_to_name(e));return false;}sum+=(float)r*3.3f/4095.0f*(28.0f/18.0f);vTaskDelay(pdMS_TO_TICKS(2));}
  const float knownPsi=((float)bar10/10.0f)/.0689476f;
  const float z=sum/32.0f-knownPsi/(sensorFullScalePsi[s]/4.0f);
  if(!isfinite(z)||z<.2f||z>.8f){ESP_LOGE(TAG,"Calibration rejected: computed offset %.3f V outside 0.2..0.8 V",(double)z);return false;}
  ControllerConfig next=config;next.zero[s]=z;next.calibrationReferenceDbar[s]=bar10;next.calibratedMask|=(uint8_t)(1U<<s);
  ESP_LOGI(TAG,"Calibration computed offset=%.3f V",(double)z);
  bool saved=persistConfig(next);ESP_LOGI(TAG,"Calibration save result=%s",saved?"verified":"FAILED");
  if(!saved)return false;
  calBar10[s]=bar10;calValidMask|=(uint8_t)(1U<<s);
  return true;
}
static void manualStart(uint8_t w,bool up,float p[5]){if(w>3||!good(w,p[w])||(up&&(!good(4,p[4])||p[w]>=wheelLimitPsi(w))))return;cancel(w);set(up?fill[w]:dump[w],1);manual[w]=now();}
static void targetStart(uint8_t w,uint16_t b){if(w>3||b>config.wheelLimitDbar[w])return;cancel(w);jobs[w].active=true;jobs[w].goal=((float)b/10.0f)/.0689476f;jobs[w].start=jobs[w].next=now();}
static uint16_t readLe16(const uint8_t *p){return (uint16_t)p[0]|((uint16_t)p[1]<<8);}
static uint32_t readLe32(const uint8_t *p){return (uint32_t)p[0]|((uint32_t)p[1]<<8)|((uint32_t)p[2]<<16)|((uint32_t)p[3]<<24);}
static bool changeSettings(const uint8_t *c,uint8_t len){
  /* B1 43 + 20 bytes of settings + 32-bit checksum over the 22-byte header/payload. */
  if(len!=26||readLe32(&c[22])!=checkBytes(c,22))return false;
  ControllerConfig next=config;size_t at=2;
  for(uint8_t i=0;i<4;i++,at+=2)next.wheelLimitDbar[i]=readLe16(&c[at]);
  next.tankLimitDbar=readLe16(&c[at]);at+=2;next.compressorOnDbar=readLe16(&c[at]);at+=2;
  next.targetToleranceDbar=readLe16(&c[at]);at+=2;next.autoTimeoutSec=readLe16(&c[at]);at+=2;
  next.compressorMaxRunSec=readLe16(&c[at]);at+=2;next.manualTimeoutMs=readLe16(&c[at]);
  return persistConfig(next);
}
static void runJob(uint8_t w,float p[5]){
  Job *j=&jobs[w];if(!j->active)return;uint32_t n=now();
  if(!good(w,p[w])||p[w]>=wheelLimitPsi(w)||elapsed(n,j->start,(uint32_t)config.autoTimeoutSec*1000U)){cancel(w);return;}
  if(j->pulse){if(!elapsed(n,j->pulseAt,500))return;closeWheel(w);j->pulse=false;j->next=n+150;return;}
  if((int32_t)(n-j->next)<0){
    return;
  }
  float e=j->goal-p[w];
  float tolerance=((float)config.targetToleranceDbar/10.0f)/.0689476f;
  if(fabsf(e)<=tolerance){closeWheel(w);if(!j->settled){j->settled=true;j->settledAt=n;}else if(elapsed(n,j->settledAt,2000))j->active=false;return;}
  j->settled=false;if(e>0){if(!good(4,p[4])||p[4]>=tankLimitPsi()){cancel(w);return;}set(fill[w],1);}else set(dump[w],1);
  j->pulse=true;j->pulseAt=n;
}
static void process(const uint8_t *c,uint8_t len,float p[5]){
  if(len<2||c[0]!=0xB1)return;
  if(len==2&&c[1]==0x42){sendSettings();return;}
  if(c[1]==0x43){bool ok=changeSettings(c,len);sendAck(0x43,ok);sendSettings();return;}
  if(len==5&&c[1]==0x40){uint16_t v=readLe16(&c[3]);targetStart(c[2],v);return;}
  if(len==5&&c[1]==0x41){bool ok=calibrate(c[2],readLe16(&c[3]));sendAck(0x41,ok);sendSettings();if(ok)sendCalibrationState();return;}
  if(len==2&&c[1]==0x2F){stopAll();return;}
  if(len==2&&c[1]>=0x20&&c[1]<=0x27){manualStart((c[1]-0x20)/2,!(c[1]&1),p);return;}
  if(len==2&&c[1]>=0x30&&c[1]<=0x33)cancel(c[1]-0x30);
}
static void update(float p[5]){
  uint32_t n=now();
  for(uint8_t w=0;w<4;w++)if(manual[w]&&(elapsed(n,manual[w],config.manualTimeoutMs)||p[w]>=wheelLimitPsi(w)))closeWheel(w);
  if(!good(4,p[4])||p[4]>=tankLimitPsi()||(compressor&&elapsed(n,compressorAt,(uint32_t)config.compressorMaxRunSec*1000U))){compressorSet(0);lockout=true;}
  else if(!lockout&&p[4]<=((float)config.compressorOnDbar/10.0f)/.0689476f)compressorSet(1);
  else if(p[4]>=tankLimitPsi())compressorSet(0);
}
static void telemetry(float p[5]){if(!linked||!encrypted)return;Telemetry t={.marker=0xA1};bool ok=true;for(uint8_t i=0;i<5;i++){ok&=good(i,p[i]);t.bar[i]=good(i,p[i])?p[i]*.0689476f:NAN;}t.flags=(ok?1:0)|(compressor?2:0)|(lockout?4:0);esp_now_send(peerMac,(uint8_t*)&t,sizeof(t));}
static void resetLink(void){stopAll();linked=false;teleNow=false;settingsPending=false;portENTER_CRITICAL(&mux);pendingEncryption=false;memset(pendingPeerMac,0,sizeof(pendingPeerMac));portEXIT_CRITICAL(&mux);if(peer)esp_now_del_peer(peerMac);peer=false;encrypted=false;memset(peerMac,0,sizeof(peerMac));}
void app_main(void){
  esp_err_t nvsResult=nvs_flash_init();
  if(nvsResult==ESP_ERR_NVS_NO_FREE_PAGES||nvsResult==ESP_ERR_NVS_NEW_VERSION_FOUND){
    ESP_LOGW(TAG,"NVS needs recovery; erasing the damaged or incompatible settings partition");
    nvsResult=nvs_flash_erase();
    if(nvsResult==ESP_OK)nvsResult=nvs_flash_init();
  }
  if(nvsResult==ESP_OK){
    loadConfig();
  }else{
    ESP_LOGE(TAG,"NVS initialization failed: %s; using safe defaults, calibration cannot be saved",esp_err_to_name(nvsResult));
    defaultConfig(&config);
  }
  gpio_config_t o={.pin_bit_mask=(1ULL<<4)|(1ULL<<13)|(1ULL<<14)|(1ULL<<16)|(1ULL<<17)|(1ULL<<18)|(1ULL<<19)|(1ULL<<21)|(1ULL<<22),.mode=GPIO_MODE_OUTPUT};gpio_config(&o);stopAll();
  adc_oneshot_unit_init_cfg_t u={.unit_id=ADC_UNIT_1};adc_oneshot_new_unit(&u,&adc);adc_oneshot_chan_cfg_t a={.bitwidth=ADC_BITWIDTH_12,.atten=ADC_ATTEN_DB_12};for(uint8_t i=0;i<5;i++)adc_oneshot_config_channel(adc,channel[i],&a);
  commandQueue=xQueueCreate(12,sizeof(Command));wifi_init_config_t w=WIFI_INIT_CONFIG_DEFAULT();esp_wifi_init(&w);esp_wifi_set_mode(WIFI_MODE_STA);esp_wifi_start();esp_wifi_set_channel(1,WIFI_SECOND_CHAN_NONE);esp_now_init();esp_now_set_pmk(pmk);esp_now_register_recv_cb(rx);
  while(1){
    uint32_t n=now();if(linked&&elapsed(n,lastBeat,3000))resetLink();
    bool switchPeer=false;uint8_t switchMac[6];
    portENTER_CRITICAL(&mux);
    if(pendingEncryption&&elapsed(n,pendingEncryptionAt,150)){memcpy(switchMac,pendingPeerMac,6);pendingEncryption=false;switchPeer=true;}
    portEXIT_CRITICAL(&mux);
    if(switchPeer&&!linked&&peer&&!memcmp(peerMac,switchMac,6)){
      if(addPeer(switchMac,true))ESP_LOGI(TAG,"schematik-debug discovery response window elapsed; encrypted peer ready");
      else ESP_LOGE(TAG,"schematik-debug encrypted peer add failed");
    }
    if(linked){float p[5];for(uint8_t i=0;i<5;i++)p[i]=pressure(i);
      if(settingsPending){sendSettings();sendCalibrationState();settingsPending=false;}
      Command c;while(commandQueue&&xQueueReceive(commandQueue,&c,0)==pdTRUE)process(c.data,c.len,p);
      update(p);for(uint8_t i=0;i<4;i++)runJob(i,p);
      if(teleNow||elapsed(n,lastTele,250)){telemetry(p);teleNow=false;lastTele=n;}
    }else stopAll();
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}