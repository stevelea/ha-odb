// ESPHome external component: OBD-II BLE client
// Discovery via esp32_ble_tracker, GATT via NimBLE-Arduino 1.4.1
#include "obd_ble.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include <cstdlib>
#include <cstring>
#include <algorithm>

namespace esphome {
namespace obd_ble {

static const char* TAG = "obd_ble";
static const NimBLEUUID SVC_UUID("0000fff0-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_W_UUID("0000fff1-0000-1000-8000-00805f9b34fb");
static const NimBLEUUID CHAR_N_UUID("0000fff2-0000-1000-8000-00805f9b34fb");
static const char ELM_PROMPT='>'; static const uint32_t CMD_TO=5000;

static const OBDPidDef G6_BMS[]={
  {"SOC","221109","[B4:B5]/10",true,"704"},{"SOH","22110A","[B4:B5]/10",false,"704"},
  {"HV Voltage","221101","[B4:B5]/10",true,"704"},{"HV Current","221103","[B4:B5]/2-1600",true,"704"},
  {"Max Cell Voltage","221105","[B4:B5]/1000",false,"704"},{"Min Cell Voltage","221106","[B4:B5]/1000",false,"704"},
  {"Max Battery Temp","221107","B4-40",true,"704"},{"Min Battery Temp","221108","B4-40",false,"704"},
  {"CLTC Range","221118","[B4:B5]",false,"704"},
  {"Cumulative Charge","221120","A<<24+B<<16+C<<8+D",false,"704"},
  {"Cumulative Dischg","221121","A<<24+B<<16+C<<8+D",false,"704"},
  {"Charge Status","22112D","B4",true,"704"},{"Charge Limit","221130","[B4:B5]-10",false,"704"},
  {"Odometer","220101","[B4:B6]",false,"704"},{"12V Battery","220102","B4/10",true,"704"},
};
static const OBDPidDef G6_VCU[]={
  {"Vehicle Speed","220104","[B4:B5]/100",true,"7E0"},
  {"Accelerator Pedal","220313","B4/2",false,"7E0"},
  {"Front Motor RPM","220317","[B4:B5]-16000",false,"7E0"},
  {"Rear Motor RPM","220318","[B4:B5]-16000",false,"7E0"},
  {"Front Motor Torque","220319","[B4:B5]/4-500",false,"7E0"},
  {"Rear Motor Torque","22031A","[B4:B5]/4-500",false,"7E0"},
  {"Charging HVIL","22031D","B4",true,"7E0"},{"VCU SoC","22031E","[B4:B5]/10",false,"7E0"},
  {"DC Charge Current","22031F","[B4:B5]/10-1200",true,"7E0"},
  {"DC Charge Voltage","220320","[B4:B5]",true,"7E0"},
  {"Brake Pressure","220321","[B4:B5]/5",false,"7E0"},
  {"Fast Charge Temp 1","220322","B4-40",false,"7E0"},{"Fast Charge Temp 2","220323","B4-40",false,"7E0"},
  {"Slow Charge Temp 1","220324","B4-40",false,"7E0"},{"Slow Charge Temp 2","220325","B4-40",false,"7E0"},
  {"Slow Charge Temp 3","220326","B4-40",false,"7E0"},
  {"Motor Temp","220327","B4/2-40",false,"7E0"},{"Coolant Temp","220328","B4/2-40",false,"7E0"},
};
static const int G6_BC=sizeof(G6_BMS)/sizeof(OBDPidDef),G6_VC=sizeof(G6_VCU)/sizeof(OBDPidDef);
static const char* I_CMD[]={"AT Z","AT E0","AT L0","AT S0","AT H1","AT SP6","AT M0","AT AT1","AT FCSM1",
  "AT SH 704","AT CRA 784","AT FCSH 704"};static const int I_CNT=sizeof(I_CMD)/sizeof(char*);

static std::vector<int> h2b(const std::string& h){std::vector<int> b;
  for(size_t i=0;i+1<h.length();i+=2){int hi=h[i],lo=h[i+1];
    hi=(hi>='0'&&hi<='9')?hi-'0':(hi>='A'&&hi<='F')?hi-'A'+10:(hi>='a'&&hi<='f')?hi-'a'+10:0;
    lo=(lo>='0'&&lo<='9')?lo-'0':(lo>='A'&&lo<='F')?lo-'A'+10:(lo>='a'&&lo<='f')?lo-'a'+10:0;
    b.push_back((hi<<4)|lo);}return b;}

// ════════════════════════════════════════════════════

void OBDComponent::setup(){
  ESP_LOGI(TAG,"OBD: MAC=%s profile=%s",mac_address_.c_str(),profile_.c_str());
  NimBLEDevice::init("esp32-obd");
  if(profile_=="xpeng_g6"){for(int i=0;i<G6_BC;i++)pids_.push_back(G6_BMS[i]);for(int i=0;i<G6_VC;i++)pids_.push_back(G6_VCU[i]);}
  ESP_LOGI(TAG,"%d PIDs loaded, waiting for device via BLE tracker...",pids_.size());
}
void OBDComponent::dump_config(){
  ESP_LOGCONFIG(TAG,"OBD-II BLE MAC=%s profile=%s PIDs=%d",mac_address_.c_str(),profile_.c_str(),pids_.size());
}

// ── Check BLE tracker for target device ──────────────────────────────────

bool OBDComponent::check_tracker(){
  if(client_&&client_->isConnected())return false;
  // Parse target MAC
  uint8_t t[6];int j=0;
  std::string m=mac_address_;m.erase(std::remove(m.begin(),m.end(),':'),m.end());
  m.erase(std::remove(m.begin(),m.end(),'-'),m.end());
  for(size_t i=0;i<m.length()&&j<6;i+=2){int hi=(m[i]>='a'?m[i]-'a'+10:m[i]>='A'?m[i]-'A'+10:m[i]-'0');
    int lo=(m[i+1]>='a'?m[i+1]-'a'+10:m[i+1]>='A'?m[i+1]-'A'+10:m[i+1]-'0');t[j++]=(hi<<4)|lo;}

  // Check tracker's discovered devices
  auto* tracker=esp32_ble_tracker::global_esp32_ble_tracker;
  if(!tracker)return false;
  auto devices=tracker->get_discovered();
  for(auto& d:devices){
    auto a=d.address_64();
    if(memcmp(a,t,6)==0){
      ESP_LOGI(TAG,"*** VEEPEAK found! RSSI=%d, connecting...",d.get_rssi());
      start_connect();
      return true;
    }
  }
  return false;
}

// ── Connection ──────────────────────────────────────────────────────────

void OBDComponent::start_connect(){
  if(state_!=PollState::IDLE)return;
  auto* scan=NimBLEDevice::getScan();if(scan->isScanning())scan->stop();
  NimBLEAddress addr(mac_address_,BLE_ADDR_PUBLIC);
  if(connect_ble(addr)){state_=PollState::DISCOVERING;state_start_ms_=millis();}
  else{ESP_LOGW(TAG,"Connect failed");}
}

bool OBDComponent::connect_ble(const NimBLEAddress& addr){
  if(client_)NimBLEDevice::deleteClient(client_);
  client_=NimBLEDevice::createClient();client_->setConnectTimeout(15);
  if(!client_->connect(addr)){ESP_LOGW(TAG,"BLE connect failed");NimBLEDevice::deleteClient(client_);client_=nullptr;return false;}
  ESP_LOGI(TAG,"Connected MTU=%d",client_->getMTU());return true;
}

void OBDComponent::discover_services(){
  if(!client_)return;
  auto* svc=client_->getService(SVC_UUID);
  if(svc){char_write_=svc->getCharacteristic(CHAR_W_UUID);char_notify_=svc->getCharacteristic(CHAR_N_UUID);
    if(char_notify_)char_notify_->subscribe(true,[this](NimBLERemoteCharacteristic*,uint8_t* d,size_t l,bool n){if(n)on_notify(d,l);},true);
    ESP_LOGI(TAG,"GATT ready: write=%p notify=%p",char_write_,char_notify_);}
}

// ── update / loop ──────────────────────────────────────────────────────

void OBDComponent::update(){
  if(state_==PollState::IDLE&&client_&&client_->isConnected()&&char_notify_){
    poll_cycle_++;current_pid_index_=0;bms_done_=false;state_=PollState::POLL_BMS;state_start_ms_=millis();}
}

void OBDComponent::loop(){
  uint32_t now=millis();
  switch(state_){
    case PollState::IDLE:
      if(now-state_start_ms_>5000){check_tracker();state_start_ms_=now;}break;
    case PollState::CONNECTING:if(now-state_start_ms_>15000){state_=PollState::IDLE;}break;
    case PollState::DISCOVERING:
      discover_services();
      if(char_write_&&char_notify_){init_cmd_index_=0;state_=PollState::INIT_ELM;ESP_LOGI(TAG,"Starting ELM327 init");}
      else if(now-state_start_ms_>10000){ESP_LOGW(TAG,"GATT discovery timeout");state_=PollState::IDLE;}
      break;
    case PollState::INIT_ELM:
      if(init_cmd_index_>=I_CNT){ESP_LOGI(TAG,"ELM327 ready");current_ecu_="bms";state_=PollState::IDLE;break;}
      send_at_command(I_CMD[init_cmd_index_]);current_command_=I_CMD[init_cmd_index_];
      state_=PollState::INIT_WAIT;state_start_ms_=now;break;
    case PollState::INIT_WAIT:if(now-state_start_ms_>CMD_TO){init_cmd_index_++;state_=PollState::INIT_ELM;}break;
    case PollState::POLL_BMS:case PollState::POLL_VCU:{
      bool bms=(state_==PollState::POLL_BMS);size_t tot=pids_.size();
      while(current_pid_index_<tot){auto&p=pids_[current_pid_index_];
        bool pb=p.can_header&&!strcmp(p.can_header,"704"),pv=p.can_header&&!strcmp(p.can_header,"7E0");
        if((bms&&pb)||(!bms&&pv)){if(!p.high_priority&&(poll_cycle_%12!=1)){current_pid_index_++;continue;}break;}
        current_pid_index_++;}
      if(bms&&current_pid_index_>=tot){current_pid_index_=0;bms_done_=true;}
      if(!bms&&current_pid_index_>=tot){state_=PollState::IDLE;break;}
      const char* te=bms?"bms":"vcu";if(current_ecu_!=te){switch_ecu_header(te);current_ecu_=te;}
      send_obd_query(pids_[current_pid_index_].pid_hex);current_command_=pids_[current_pid_index_].pid_hex;
      state_=PollState::WAIT_RESPONSE;state_start_ms_=now;break;}
    case PollState::WAIT_RESPONSE:
      if(now-state_start_ms_>CMD_TO){current_pid_index_++;
        state_=(bms_done_&&current_pid_index_>=pids_.size())?PollState::IDLE:(bms_done_?PollState::POLL_VCU:PollState::POLL_BMS);}break;
  }
}

// ── ELM327 ──────────────────────────────────────────────────────────────

bool OBDComponent::send_at_command(const std::string& cmd){if(!char_write_)return false;std::string p=cmd+"\r";return char_write_->writeValue((uint8_t*)p.c_str(),p.length(),false);}
bool OBDComponent::send_obd_query(const std::string& pid){if(!char_write_)return false;std::string p="22 "+pid+"\r";return char_write_->writeValue((uint8_t*)p.c_str(),p.length(),false);}
void OBDComponent::switch_ecu_header(const std::string& ecu){
  if(ecu=="bms"){send_at_command("AT SH 704");delay(60);send_at_command("AT CRA 784");delay(60);send_at_command("AT FCSH 704");delay(60);}
  else{send_at_command("AT SH 7E0");delay(60);send_at_command("AT CRA 7E8");delay(60);send_at_command("AT FCSH 7E0");delay(60);}
}

// ── Notify ──────────────────────────────────────────────────────────────

void OBDComponent::on_notify(uint8_t* data,size_t len){
  for(size_t i=0;i<len;i++){
    if(data[i]==ELM_PROMPT){std::string c;for(char ch:rx_buffer_)if(ch!=' '&&ch!='\r'&&ch!='\n'&&ch!='\t'&&ch!='>')c+=ch;rx_buffer_.clear();if(c.empty())return;
      if(state_==PollState::INIT_WAIT){init_cmd_index_++;state_=PollState::INIT_ELM;}
      else if(state_==PollState::WAIT_RESPONSE&&current_pid_index_<pids_.size()){
        auto&p=pids_[current_pid_index_];float v=parse_response(c,p.formula);
        if(!std::isnan(v)&&current_pid_index_<sensors_.size()){publish_sensor(current_pid_index_,v);ESP_LOGD(TAG,"%s=%.2f",p.name,v);}
        current_pid_index_++;state_=(bms_done_&&current_pid_index_>=pids_.size())?PollState::IDLE:(bms_done_?PollState::POLL_VCU:PollState::POLL_BMS);}}
    else rx_buffer_+=(char)data[i];
  }
}

// ── Parser ──────────────────────────────────────────────────────────────

float OBDComponent::parse_response(const std::string& r,const std::string& f){
  if(r.find("ERROR")!=std::string::npos||r.find("NO DATA")!=std::string::npos||r.find("SEARCHING")!=std::string::npos||r.find("UNABLE")!=std::string::npos||r.find("STOPPED")!=std::string::npos)return NAN;
  std::string c=r;if(c.length()>=3&&c[0]=='7')c=c.substr(3);if(c.length()<6)return NAN;
  auto b=h2b(c);if(b.size()<2)return NAN;if(b.size()>=3&&b[1]==0x7F)return NAN;
  std::string e=f;size_t p;
  while((p=e.find("[B"))!=std::string::npos){size_t cl=e.find(':',p),co=e.find(']',cl);if(cl==std::string::npos||co==std::string::npos)break;
    int st=atoi(e.c_str()+p+2),en=atoi(e.c_str()+cl+2);int64_t v=0;
    for(int i=st;i<=en&&i<(int)b.size();i++)v=(v<<8)|b[i];e.replace(p,co-p+1,std::to_string(v));}
  while((p=e.find('B'))!=std::string::npos){int x=0;size_t i=p+1;while(i<e.length()&&isdigit(e[i]))x=x*10+(e[i++]-'0');
    int bv=(x<(int)b.size())?b[x]:0;e.replace(p,i-p,std::to_string(bv));}
  if(e.find('A')!=std::string::npos){int ds=0;for(size_t i=0;i+1<b.size();i++)if(b[i]==0x62){ds=i+1;if(ds+2<(int)b.size())ds+=2;break;}
    for(size_t i=0;i<e.length();i++)if(e[i]>='A'&&e[i]<='D'){int bv=(ds+e[i]-'A'<(int)b.size())?b[ds+e[i]-'A']:0;auto s=std::to_string(bv);e.replace(i,1,s);i+=s.length()-1;}}
  auto tk=[](const std::string& s,size_t& pp)->float{while(pp<s.length()&&isspace(s[pp]))pp++;char* en;float v=strtof(s.c_str()+pp,&en);pp=en-s.c_str();return v;};
  size_t pp=0;float rv=tk(e,pp);
  while(pp<e.length()){while(pp<e.length()&&isspace(e[pp]))pp++;if(pp>=e.length())break;char op=e[pp++];float rh=tk(e,pp);
    if(op=='+')rv+=rh;else if(op=='-')rv-=rh;else if(op=='*')rv*=rh;else if(op=='/')rv=(rh!=0)?rv/rh:0;
    else if(op=='<'&&pp<e.length()&&e[pp]=='<'){pp++;rv=(int)rv<<(int)rh;}else if(op=='>'&&pp<e.length()&&e[pp]=='>'){pp++;rv=(int)rv>>(int)rh;}}
  return rv;
}

void OBDComponent::publish_sensor(size_t idx,float v){if(idx<sensors_.size()){last_values_[idx]=v;sensors_[idx]->publish_state(v);}}

}  // namespace obd_ble
}  // namespace esphome
