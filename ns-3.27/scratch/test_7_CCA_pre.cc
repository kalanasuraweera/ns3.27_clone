/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <vector>
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <vector> 
#include <thread>
#include <random>

#include <iostream>
#include <mutex>
#include <condition_variable>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <unistd.h>
#include <cctype>
#include <locale>
#include <fstream>

#include <ctime>
#include <stdlib.h>
#include <time.h>

#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/olsr-helper.h"
#include <ns3/buildings-module.h>
#include "ns3/stats-module.h"
#include "ns3/spectrum-module.h"

#include <nlohmann/json.hpp>

#define HOST "127.0.0.1"
#define WRITE_PORT 3874
#define READ_PORT 8874

// for convenience
using json = nlohmann::json;

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("ThirdScriptExample");

bool enableInterference = true;
bool activateWalls = false;

int startTime = 2;
int stopTime = 4;

int maxDevicesPerNode = 2;

int nStas = 5;
int nHidden = 4;
int iter=0;

int payloadSize = 972;

bool is_ready = false;
std::mutex m;
std::condition_variable cv;

string command_read;
json json_to_send;

bool is_paused = false;
std::mutex m_pause;
std::condition_variable cv_pause;

vector< vector<double> > allPacketsSent;
vector< vector<double> > allPacketsSentTime;

vector< vector< vector<double> > > allPacketsSentSta;
vector< vector< vector<double> > > allPacketsSentStaTime;

vector< vector<double> > allPacketsReceived;
vector< vector<double> > allPacketsReceivedTime;

vector< vector< vector<double> > > allPacketsReceivedSta;
vector< vector< vector<double> > > allPacketsReceivedStaTime;

vector< double > allTxRates;
vector< double > allRxRates;

vector< vector<double> > allTxRatesSta;
vector< vector<double> > allRxRatesSta;

vector< vector< vector<double> > > allRSSIdB;
vector< vector<double> > allDevicesRSSI;

vector< vector<double> > allPacketsReSent;
vector< vector<double> > allPacketsReSentTime;

vector< vector< vector<double> > > allPacketsReSentSta;
vector< vector< vector<double> > > allPacketsReSentStaTime;

vector< vector<double> > allPacketsSentOk;
vector< vector<double> > allPacketsSentOkTime;

vector< vector<double> > allPacketsFailed;
vector< vector<double> > allPacketsFailedTime;

vector< vector< vector<double> > > allPacketsFailedSta;
vector< vector< vector<double> > > allPacketsFailedStaTime;

vector< vector<double> > allPacketsOverhead;
vector< vector<double> > allPacketsOverheadTime;

vector< vector<double> > allPacketsError;
vector< vector<double> > allPacketsErrorTime;

vector< vector<double> > allDevicesChannelTimes;

vector<int> currentChannels;

vector<int> staAssociations;

vector<double> allThroughputs;
vector<double> allDelays;

vector<pair<double, double> > locations;
vector<vector<int> > ext_locations = {
        {6, 3},
{8, 6},
{6, 3},
{7, 7},
{14, 1},
{13, 4},
{13, 7},
{15, 3},
{15, 7},
{13, 2},
{6, 7},
{20, 6},
{7, 6},
{15, 10},
{15, 3},
{17, 5},
{12, 7},
{9, 4},
{15, 3},
{7, 10},
{1, 10},
{11, 5},
{8, 8},
{3, 1},
{6, 7},
{6, 6},
{2, 4},
{8, 8},
{14, 6},
{18, 1},
{3, 10},
{3, 3},
{20, 2},
{8, 2},
{17, 3},
{1, 4},
{18, 1},
{1, 7},
{1, 5},
{18, 5},
{8, 4},
{8, 5},
{3, 3},
{20, 9},
{10, 7},
{14, 8},
{16, 10},
{7, 2},
{2, 6},
{18, 1}
};

bool stop_simul = false;

void stopHandler();

void StateCallback (std::string, Time, Time, enum WifiPhy::State);

void
PhyRxOkTrace (string, Ptr<const Packet>, double, WifiMode, enum WifiPreamble);

void
PhyRxErrorTrace (std::string, Ptr<const Packet>, double);

void ReTxTrace (std::string,Mac48Address, Ptr<const Packet>);

void
MonitorSniffRx (string, Ptr<const Packet>,
                     uint16_t,
                     WifiTxVector,
                     MpduInfo,
                     SignalNoiseDbm);

// void
// AssocTrace(string context, Mac48Address address){

//   int nWifis = nStas + 4;

//   uint16_t deviceID = getDeviceID(context);



//   int assocID = getDeviceIDByAddress(address);

//   cerr<<Simulator::Now().GetSeconds()<<" deviceID: "<<deviceID<<" assocID: "<<assocID<<endl;

//   if(deviceID >=4 && deviceID<nWifis){
//     NS_ASSERT(assocID==1 || assocID==3);
//     staAssociations[deviceID-4]=assocID;
//   }
  
// }

void
MacRxOkTrace (string, Ptr<const Packet>);


void
PhyTxTrace (std::string, Ptr<const Packet>, WifiMode, WifiPreamble, uint8_t);

void
TxDataFailedTrace (std::string, Mac48Address);

//std::thread read_thread, write_thread;

// trim from start (in place)
static inline void ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
        return !std::isspace(ch);
    }));
}

// trim from end (in place)
static inline void rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}

// trim from both ends (in place)
static inline void trim(std::string &s) {
    ltrim(s);
    rtrim(s);
}

void error(const char *msg)
{
    perror(msg);
    exit(1);
}

void pauseSimulation(){
  if(!is_paused){
    cerr<<Simulator::Now().GetSeconds()<<" Pausing Simulation"<<endl;
    Simulator::Pause();
    std::unique_lock<std::mutex> lk_pause(m_pause);
    is_paused = true;
    cv_pause.notify_one();
  }
}

string
getStringMacAddress(Mac48Address address){
  std::stringstream buffer;
  buffer << address;
  return buffer.str();
}

string
getStringIpv4Address(Ipv4Address address){
  std::stringstream buffer;
  buffer << address;
  return buffer.str();
}

uint16_t
getDeviceID(string context){

  stringstream ss(context);
  string item;

  uint16_t ret = -1;

  uint16_t nodeID = - 1;
  uint16_t deviceID = -1;
  int c=0;
  std::string delimiter = "/";

  size_t pos = 0;
  string s = context;
  std::string token;
  while ((pos = s.find(delimiter)) != std::string::npos) {
    c++;
    token = s.substr(0, pos);
    //cerr<<"token: "<<token<<endl;
    if(c==3){
      nodeID = atoi (token.c_str());
      // if(deviceID==1){
      //   cerr<<context<<endl;
      //   cerr<<"deviceID: "<<deviceID<<endl;
      // }
    }
    if(c==5){
      deviceID = atoi (token.c_str());
      if(deviceID==1){
        //cerr<<context<<endl;
      }
      break;
    }
    s.erase(0, pos + delimiter.length());
  }

  // if(deviceID != -1 && deviceID != -1){
  //   return deviceID*maxDevicesPerNode + deviceID;
  // }

  // cerr<<context<<endl;
  
  //cerr<<"deviceID: "<<deviceID<<" deviceID: "<<deviceID<<endl;

  if(nodeID >= 2 && nodeID < 100){
    ret= nodeID + deviceID + 2;
  }
  else if (nodeID==0){
    ret= nodeID + deviceID;
  }
  else if (nodeID==1){
    ret= nodeID + deviceID + 1;
  }


  // if(nodeID == 7){
  //   cerr<<context<<endl;
  //   cerr<<"deviceID: "<<ret<<endl;
  // }

  // if(ret == 5){
  //   cerr<<"FIVE"<<endl;
  //   exit(0);
  // }

  if(ret == 65535)
    cerr << "Device ID Error" << endl;

  return ret;
}

void calcAverageRSSIs(){
  for (uint16_t i = 0;  i < nStas + 4; ++ i)
  {
    
    for (uint16_t j = 0; j < nStas + 4; j++){

      double rssiSum = 0.0;

      for(uint16_t k=0;k< allRSSIdB[i][j].size(); k++) rssiSum += allRSSIdB[i][j][k];

      //cerr<<"rssiSum: "<<rssiSum<<"size: "<<allRSSIdB[i][j].size()<<" "<<i<<" "<<j<<endl;

      allDevicesRSSI[i][j] = rssiSum/allRSSIdB[i][j].size();

    }

  }
}


void calcRates(){
  for (uint16_t i = 0;  i < nStas + 4; ++ i)
  {

    double rxTotal =0.0, txTotal=0.0;

    int k;

    for(k = allPacketsReceivedTime[i].size() - 1; k >=0 ; k--){
      if(allPacketsReceivedTime[i][k] > allPacketsReceivedTime[i].back() - 1000000000) rxTotal+=allPacketsReceived[i][k];
      else{
        break;
      }
    }

    cerr<<i<<" rxTotal: "<<rxTotal<<endl;

    if(rxTotal > 0 && k+1!=allPacketsReceivedTime[i].size() - 1 ){
      allRxRates[i]=(rxTotal*1000000000)/(allPacketsReceivedTime[i].back()-allPacketsReceivedTime[i][k+1]);
      allRxRates[i]=allRxRates[i]*8/1000000;
    }

    for(k = allPacketsSentTime[i].size() - 1; k >=0 ; k--){
      if(allPacketsSentTime[i][k] > allPacketsSentTime[i].back() - 1000000000) txTotal+=allPacketsSent[i][k];
      else{
        break;
      }
    }

    if(txTotal > 0 && k+1!=allPacketsSentTime[i].size() - 1){
      allTxRates[i]=(txTotal*1000000000)/(allPacketsSentTime[i].back()-allPacketsSentTime[i][k+1]);
      allTxRates[i]=allTxRates[i]*8/1000000;
    }

    if(i != 1 && i!=3) continue;

    for (uint16_t j = 4; j < nStas + 4; j++){

      if(staAssociations[j-4]!=i) continue;

      double rxTotal =0.0, txTotal=0.0;

      int k;

      for(k = allPacketsReceivedStaTime[i][j].size() - 1; k >=0 ; k--){
        if(allPacketsReceivedStaTime[i][j][k] > allPacketsReceivedStaTime[i][j].back() - 1000000000) rxTotal+=allPacketsReceivedSta[i][j][k];
        else{
          //cerr<<i<<" "<<j<<" rxTotal: "<<rxTotal<<endl;
          break;
        }
      }

      //cerr<<"k: "<<k<<endl;
      if(rxTotal > 0 && k+1!=allPacketsReceivedStaTime[i][j].size() - 1 ){
        allRxRatesSta[i][j]=(rxTotal*1000000000)/(allPacketsReceivedStaTime[i][j].back()-allPacketsReceivedStaTime[i][j][k+1]);
        allRxRatesSta[i][j]=(allRxRatesSta[i][j]*8.0/1000000);
      }

      for(k = allPacketsSentStaTime[i][j].size() - 1; k >=0 ; k--){
        if(allPacketsSentStaTime[i][j][k] > allPacketsSentStaTime[i][j].back() - 1000000000) txTotal+=allPacketsSentSta[i][j][k];
        else{
          //cerr<<i<<" "<<j<<" txTotal: "<<txTotal<<endl;
          break;
        }
      }

      //cerr<<"k: "<<k<<endl;
      if(txTotal > 0 && k+1!=allPacketsSentStaTime[i][j].size() - 1){
        allTxRatesSta[i][j]=(txTotal*1000000000)/(allPacketsSentStaTime[i][j].back()-allPacketsSentStaTime[i][j][k+1]);
        allTxRatesSta[i][j]=(allTxRatesSta[i][j]*8.0/1000000);
        //cerr<<i<<" "<<j<<" rxTotal: "<<rxTotal<<" txTotal: "<<txTotal<<endl;
      }

    }

  }
}

void calcThroughputPeriodic()
{
  for (uint16_t i = 0;  i < nStas + 4; ++ i)
  {
    double minTThr_t=allPacketsReceivedTime[i].at(0)/1000000000;
    int thrMinT=(int)minTThr_t+1;

    int timeSize=allPacketsReceivedTime[i].size()-1;
    double thrMaxT_t=allPacketsReceivedTime[i].at(timeSize)/1000000000;
    int thrMaxT=(int)thrMaxT_t+1;


    //cerr<<"thrMinT: "<<thrMinT<<" thrMaxT: "<<thrMaxT<<endl;

    int t = thrMinT, tt = 0;
    while ( t <= thrMaxT )
    {
      double totR=0;
      while ( tt<(int)allPacketsReceivedTime[i].size() )
      {
        double timeDouble = allPacketsReceivedTime[i].at(tt)/(1000000000.0f);
        if( timeDouble < (double)t)
        {
          totR += allPacketsReceived[i].at(tt)*8;
          tt++;
        }
        else{
          break;
        }
      }
      if(t > startTime){
        allThroughputs[i] = totR/1000000;
        //cerr <<  "Throughput at " << i << " at time "<< (int)t << ": "<< totR/1000000 << endl;
      }
      t++;
    }
  }
}



class Experiment{
  public:
  std::string errorModelType = "ns3::NistErrorRateModel";
  uint32_t playStepMilliShort = 200;
  uint32_t playStepMilliLong = 1000;
  uint32_t thread_id = 423452534;

  double x_min = 0.0;
  double x_max = 20.0;
  double y_min = 0.0;
  double y_max = 10.0;
  double z_min = 0.0;
  double z_max = 3.0;
  double NFloors = 1;
  double NRoomsX = 4;
  double NRoomsY = 2;

  int ext_x;
  int ext_y;

  int ch_bh_init = 1, ch_fh_init = 6;
  vector<int> ch_intf_init={3,7,1,9};

  //int ch_bh_init = 3, ch_fh_init = 7;
  //vector<int> ch_intf_init={3,7,1,9};
  //int ch_bh_next =  3, ch_fh_next = 6;

  int ch_bh_to_change =  ch_bh_init, ch_fh_to_change = ch_fh_init;

  int backhaul_signal = 0;
  int backhaul_signal_avg = 0;
  int backhaul_signal_samples = 0;

  NodeContainer apNode, extNode, staNodes, hiddenAPs, hiddenSTAs;

  NetDeviceContainer apDeviceBackhaul, apDeviceFronthaul , extDeviceBackhaul, extDeviceFronthaul, staDevices, hiddenAPDevices, hiddenSTADevices;

  Ipv4InterfaceContainer apFronthaulInterface, apBackhaulInterface, staInterfaces, extFronthaulInterface, extBackhaulInterface, hiddenAPInterfaces, hiddenSTAInterfaces;

  Experiment (){
    int nWifis = nStas + 4  + 5;

    srand (time(NULL));

    std::random_device rd;     // only used once to initialise (seed) engine
    std::mt19937 rng(rd());    // random-number engine used (Mersenne-Twister in this case)

    ext_x = ext_locations[iter][0];
    ext_y = ext_locations[iter][1];

    std::ofstream outfile_channels;

    std::uniform_int_distribution<int> uni(1,11); // guaranteed unbiased
    ch_bh_init = uni(rng);

    // std::uniform_int_distribution<int> unif(1, 11);
    // ch_fh_init = unif(rng);

    // while(ch_bh_init == ch_fh_init){
    //   ch_fh_init = unif(rng);
    // }
    
    //ch_fh_init = ch_bh_init - 5;

    if(ch_bh_init > 6){
      std::uniform_int_distribution<int> unif(ch_bh_init - 5, ch_bh_init - 3);
      ch_fh_init = unif(rng);
      //ch_fh_init = ch_bh_init - 5;
    }
    else{
      std::uniform_int_distribution<int> unif(ch_bh_init + 3, ch_bh_init + 5);
      ch_fh_init = unif(rng);
      //ch_fh_init = ch_bh_init + 5;
    }

    outfile_channels.open("CCA/channels_pre.txt", std::ios_base::app);
    outfile_channels<<iter<<" "<<ch_bh_init<<" "<<ch_fh_init<<endl;
    cerr<< "Channels: "<<ch_bh_init<<" "<<ch_fh_init<<endl;

    // ext_x = 16;
    // ext_y = 10;

    is_paused = false;

    stop_simul = false;

    //std::ofstream outfile;

    // outfile.open("ext_locations.txt", std::ios_base::app);
    // outfile << iter<<" "<<ext_x<<" "<< ext_y<<endl;

    //ext_locations.push_back(make_pair(ext_x, ext_y));

    currentChannels.resize(nWifis);

    allPacketsReceived.resize(nWifis);
    allPacketsReceivedTime.resize(nWifis);

    allPacketsReceivedSta.resize(nWifis);
    allPacketsReceivedStaTime.resize(nWifis);

    allRSSIdB.resize(nWifis);
    allDevicesRSSI.resize(nWifis);

    allPacketsReSent.resize(nWifis);
    allPacketsReSentTime.resize(nWifis);

    allPacketsReSentSta.resize(nWifis);
    allPacketsReSentStaTime.resize(nWifis);

    allPacketsSent.resize(nWifis);
    allPacketsSentTime.resize(nWifis);

    allTxRates.resize(nWifis);
    allRxRates.resize(nWifis);

    allTxRatesSta.resize(nWifis);
    allRxRatesSta.resize(nWifis);

    allPacketsSentOk.resize(nWifis);
    allPacketsSentOkTime.resize(nWifis);

    allPacketsSentSta.resize(nWifis);
    allPacketsSentStaTime.resize(nWifis);

    allPacketsError.resize(nWifis);
    allPacketsErrorTime.resize(nWifis);

    allPacketsFailed.resize(nWifis);
    allPacketsFailedTime.resize(nWifis);

    allPacketsFailedSta.resize(nWifis);
    allPacketsFailedStaTime.resize(nWifis);

    allPacketsOverhead.resize(nWifis);
    allPacketsOverheadTime.resize(nWifis);

    allDevicesChannelTimes.resize(nWifis);

    staAssociations.resize(nWifis);

    locations.resize(nWifis);

    allThroughputs.resize(nWifis);
    allDelays.resize(nWifis);

    for(uint64_t i=0;i<nWifis;i++){
      allPacketsReceived[i].resize(stopTime);
      allPacketsReceivedTime[i].resize(stopTime);

      allPacketsReceivedSta[i].resize(nWifis);
      allPacketsReceivedStaTime[i].resize(nWifis);

      allPacketsSentSta[i].resize(nWifis);
      allPacketsSentStaTime[i].resize(nWifis);

      allTxRatesSta[i].resize(nWifis);
      allRxRatesSta[i].resize(nWifis);

      allPacketsReSentSta[i].resize(nWifis);
      allPacketsReSentStaTime[i].resize(nWifis);

      allPacketsFailedSta[i].resize(nWifis);
      allPacketsFailedStaTime[i].resize(nWifis);

      allRSSIdB[i].resize(nWifis);
      allDevicesRSSI[i].resize(nWifis);

      allDevicesChannelTimes[i] = {0.0, 0.0, 0.0, 0.0, 0.0};// active start time, busy duration, idle duration, tx duration, rx duration
    }
  }

  uint16_t getDeviceIDByAddress(Mac48Address address){

    uint16_t ret = -1;

    if(apDeviceBackhaul.Get(0)->GetAddress() == address) ret = 0;
    if(apDeviceFronthaul.Get(0)->GetAddress() == address) ret = 1;

    if(extDeviceBackhaul.Get(0)->GetAddress() == address) ret = 2;
    if(extDeviceFronthaul.Get(0)->GetAddress() == address) ret = 3;

    for(int i=0;i < nStas;i++){
      if(staDevices.Get(i)->GetAddress() == address) ret = 4 + i;
    }


    //if(ret == 65535) cerr<<address<<endl;
    //exit(0);
    

    return ret;

  }

  

  double
  getVectorSum(vector<double>& array){
    double ret = 0.0;
    for(uint16_t i=0;i<array.size(); i++){
      ret+=array[i];
    }
    return ret;
  }

  bool
  isContained(vector<Mac48Address> & arr, Mac48Address address){

    bool ret;

    if ( std::find(arr.begin(), arr.end(), address) != arr.end() ) ret = true;
    else ret = false;

    //cerr<<ret<<endl;
    return ret;
  }

  json
  sensing(int a1 ,int a2, int a3, int a4, int a5){
  /*sensing shell scripts accepts five arguments which take values 1 or 0
  the first argument specify  whether you want to obtain the associated
  stations for main AP (value 1) or not (value 0)
  the second argument specify  whether you want to obtain the neighboring
  APs for main AP (value 1) or not (value 0)
  the third argument specify  whether you want to obtain the associated
  stations for extenders (value 1) or not (value 0)
  the fourth argument specify  whether you want to obtain the neighboring
  APs for extenders (value 1) or not (value 0)
  the fifth argument specify  whether you want to test download and upload
  speed of backhaul link AP-to-EXT (value 1) or not (value 0)*/

    cerr<<a1<<" "<<a2<<" "<<a3<<" "<<a4<<" "<<a5<<endl;

    bool get_associated_stas_for_ap;
    bool get_neighboring_aps_for_ap;
    bool get_associated_stas_for_exts;
    bool get_neighboring_aps_for_exts;
    bool get_speeds;

    vector<json> empty_dummy;

    calcRates();

    calcAverageRSSIs();

    double currentTime = Simulator::Now().GetMilliSeconds();

    json ret;

    ret["timestamp"] = Simulator::Now().GetSeconds() - startTime;

    vector<json> ap_interfaces;

    Ptr<AdhocWifiMac> apBackhaulMac = DynamicCast<AdhocWifiMac> (DynamicCast<WifiNetDevice> (apDeviceBackhaul.Get(0))->GetMac () );

    {
      Ptr<AdhocWifiMac> mapMac = DynamicCast<AdhocWifiMac> (DynamicCast<WifiNetDevice> (apDeviceBackhaul.Get(0))->GetMac () );
      Ptr<WifiNetDevice> mapBackhaulWifiNetDevice = DynamicCast<WifiNetDevice> (apDeviceBackhaul.Get(0));
      json map_iface;
      map_iface["channel"] = currentChannels[0];
      map_iface["macAddr"] = getStringMacAddress(mapMac->GetAddress());
      map_iface["ap_stats"]["statistics"]["rx_bytes"] = getVectorSum(allPacketsReceived[0]);
      map_iface["ap_stats"]["statistics"]["tx_bytes"] = getVectorSum(allPacketsReceived[0]);

      json map_channel_survey;

      map_channel_survey["channel_active_ms"] = allDevicesChannelTimes[0][2] + allDevicesChannelTimes[0][3] + allDevicesChannelTimes[0][4];
      map_channel_survey["channel_busy_ms"] = allDevicesChannelTimes[0][3] + allDevicesChannelTimes[0][4];
      map_channel_survey["channel_transmit_ms"] = allDevicesChannelTimes[0][3];

      map_iface["stations"] = empty_dummy;

      map_iface["channel_survey"] = {map_channel_survey};

      ap_interfaces.push_back(map_iface);
      
    }

    get_associated_stas_for_ap = a1;
    get_neighboring_aps_for_ap = a2;
    get_associated_stas_for_exts = a3;
    get_neighboring_aps_for_exts = a4;
    get_speeds = a5;

    {
      Ptr<AdhocWifiMac> mapMac = DynamicCast<AdhocWifiMac> (DynamicCast<WifiNetDevice> (apDeviceFronthaul.Get(0))->GetMac () );
      Ptr<WifiNetDevice> mapFronthaulWifiNetDevice = DynamicCast<WifiNetDevice> (apDeviceFronthaul.Get(0));
      json map_iface;
      map_iface["channel"] = currentChannels[1];
      map_iface["macAddr"] = getStringMacAddress(mapMac->GetAddress());
      map_iface["ap_stats"]["statistics"]["rx_bytes"] = getVectorSum(allPacketsReceived[1]);
      map_iface["ap_stats"]["statistics"]["tx_bytes"] = getVectorSum(allPacketsReceived[1]);

      json map_channel_survey;

      map_channel_survey["channel_active_ms"] = allDevicesChannelTimes[1][2] + allDevicesChannelTimes[1][3] + allDevicesChannelTimes[1][4];
      map_channel_survey["channel_busy_ms"] = allDevicesChannelTimes[1][3] + allDevicesChannelTimes[1][4];
      map_channel_survey["channel_transmit_ms"] = allDevicesChannelTimes[1][3];

      map_iface["channel_survey"] = {map_channel_survey};

      map_iface["stations"] = empty_dummy;

      if(get_associated_stas_for_ap){
        vector<int> stas;

        for(int j=0;j<nStas;j++){
          if(staAssociations[j]==1) stas.push_back(j);
        }

        vector<json> stations;

        for(int i=0;i<stas.size();i++){

          int staDeviceID = stas[i] + 4;

          Ptr<AdhocWifiMac> staMac = DynamicCast<AdhocWifiMac> (DynamicCast<WifiNetDevice> (staDevices.Get(stas[i]))->GetMac () );

          json station;
          station["macAddr"] = getStringMacAddress(staMac->GetAddress());
          station["signal"] = allRSSIdB[staDeviceID].back();
          station["signal_avg"]= allDevicesRSSI[1][staDeviceID]; //change later
          station["rx_bytes"]= getVectorSum(allPacketsReceived[staDeviceID]);
          station["tx_bytes"]= getVectorSum(allPacketsSentSta[1][staDeviceID]);
          station["tx_packets"]= allPacketsSentSta[1][staDeviceID].size();
          station["tx_retries"]= allPacketsReSentSta[1][staDeviceID].size();
          station["tx_failed"]= allPacketsFailedSta[1][staDeviceID].size();
          station["tx_bitrate"]= to_string(allTxRatesSta[1][staDeviceID]);
          station["rx_bitrate"]= to_string(allRxRatesSta[1][staDeviceID]);
          stations.push_back(station);
        }

        map_iface["stations"] = stations;
        
        ap_interfaces.insert(ap_interfaces.begin(), map_iface);

      }
      
    }

    if(get_neighboring_aps_for_ap){

    }

    ret["interfaces"]= ap_interfaces;

    Ptr<AdhocWifiMac> extFronthaulMac = DynamicCast<AdhocWifiMac> (DynamicCast<WifiNetDevice> (extDeviceFronthaul.Get(0))->GetMac () );
    Ptr<AdhocWifiMac> extBackhaulMac = DynamicCast<AdhocWifiMac> (DynamicCast<WifiNetDevice> (extDeviceBackhaul.Get(0))->GetMac () );

    Ptr<WifiNetDevice> extBackhaulWifiNetDevice = DynamicCast<WifiNetDevice> (extDeviceBackhaul.Get(0));
    Ptr<WifiNetDevice> extFronthaulWifiNetDevice = DynamicCast<WifiNetDevice> (extDeviceFronthaul.Get(0));

    json ext;

    ext["id"]= getStringIpv4Address(extBackhaulInterface.GetAddress(0));
    ext["timestamp"] = Simulator::Now().GetSeconds() - startTime;

    json downlink_wds;

    downlink_wds["macAddr"] = getStringMacAddress(apBackhaulMac->GetAddress());
    downlink_wds["channel"] = currentChannels[0];
    downlink_wds["signal"] = allRSSIdB[0][2].back();
    downlink_wds["signal_avg"]= allDevicesRSSI[0][2];
    downlink_wds["rx_bytes"]= getVectorSum(allPacketsReceived[0]);
    downlink_wds["tx_bytes"]= getVectorSum(allPacketsSent[0]);
    downlink_wds["tx_packets"]= allPacketsSent[0].size();
    downlink_wds["tx_retries"]= allPacketsReSent[0].size();
    downlink_wds["tx_failed"]= allPacketsFailed[0].size();
    downlink_wds["tx_bitrate"]= to_string(allTxRates[0]);
    downlink_wds["rx_bitrate"]= to_string(allRxRates[0]);
    vector<json> ext_interfaces;


    {
      json ext_iface;

      ext_iface["channel"]= currentChannels[2];
      ext_iface["ap_stats"]["statistics"]["rx_bytes"] = getVectorSum(allPacketsReceived[2]);
      ext_iface["ap_stats"]["statistics"]["tx_bytes"] = getVectorSum(allPacketsSent[2]);

      json ext_channel_survey;

      ext_channel_survey["channel_active_ms"] = allDevicesChannelTimes[2][2] + allDevicesChannelTimes[2][3] + allDevicesChannelTimes[2][4];
      ext_channel_survey["channel_busy_ms"] = allDevicesChannelTimes[2][3] + allDevicesChannelTimes[2][4];
      ext_channel_survey["channel_transmit_ms"] = allDevicesChannelTimes[2][3];

      ext_iface["channel_survey"] = {ext_channel_survey};

      ext_iface["stations"] = empty_dummy;

      ext_interfaces.push_back(ext_iface);

    }

    json uplink_wds;

    uplink_wds["macAddr"] = getStringMacAddress(extBackhaulMac->GetAddress());
    uplink_wds["channel"] = currentChannels[2];
    uplink_wds["signal"] = allRSSIdB[0][2].back();
    uplink_wds["signal_avg"]= allDevicesRSSI[0][2];
    uplink_wds["rx_bytes"]= getVectorSum(allPacketsReceived[2]);
    uplink_wds["tx_bytes"]= getVectorSum(allPacketsSent[2]);
    uplink_wds["tx_packets"]= allPacketsSent[2].size();
    uplink_wds["tx_retries"]= allPacketsReSent[2].size();
    uplink_wds["tx_failed"]= allPacketsFailed[2].size();
    uplink_wds["tx_bitrate"]= to_string(allTxRates[2]);
    uplink_wds["rx_bitrate"]= to_string(allRxRates[2]);

    json channel_survey_bh;

    channel_survey_bh["channel_active_ms"]= allDevicesChannelTimes[2][2] + allDevicesChannelTimes[2][3] + allDevicesChannelTimes[2][4];
    channel_survey_bh["channel_busy_ms"]= allDevicesChannelTimes[2][3] + allDevicesChannelTimes[2][4];
    channel_survey_bh["channel_transmit_ms"]= allDevicesChannelTimes[2][3];

    ext["downlink_wds"] = downlink_wds;
    ext["uplink_wds"] = uplink_wds;
    ext["channel_survey_bh"] = channel_survey_bh;

    {
      json ext_iface;

      ext_iface["channel"]= currentChannels[3];
      ext_iface["ap_stats"]["statistics"]["rx_bytes"] = getVectorSum(allPacketsReceived[3]);
      ext_iface["ap_stats"]["statistics"]["tx_bytes"] = getVectorSum(allPacketsSent[3]);

      json ext_channel_survey;

      ext_channel_survey["channel_active_ms"] = allDevicesChannelTimes[3][2] + allDevicesChannelTimes[3][3] + allDevicesChannelTimes[3][4];
      ext_channel_survey["channel_busy_ms"] = allDevicesChannelTimes[3][3] + allDevicesChannelTimes[3][4];
      ext_channel_survey["channel_transmit_ms"] = allDevicesChannelTimes[3][3];

      ext_iface["channel_survey"] = {ext_channel_survey};

      ext_iface["stations"] = empty_dummy;

      if(get_associated_stas_for_exts){

        vector<int> stas;

        for(int j=0;j<nStas;j++){
          if(staAssociations[j]==3) stas.push_back(j);
        }

        vector<json> stations;

        for(int i=0;i<stas.size();i++){

          int staDeviceID = stas[i] + 4;

          Ptr<AdhocWifiMac> staMac = DynamicCast<AdhocWifiMac> (DynamicCast<WifiNetDevice> (staDevices.Get(stas[i]))->GetMac () );

          DynamicCast<WifiNetDevice> (staDevices.Get(stas[i]));

          json station;
          station["macAddr"] = getStringMacAddress(staMac->GetAddress());
          station["channel"] = currentChannels[3];
          station["signal"] = allRSSIdB[3][staDeviceID].back();
          station["signal_avg"]= allDevicesRSSI[3][staDeviceID];
          station["rx_bytes"]= getVectorSum(allPacketsReceived[staDeviceID]);
          station["tx_bytes"]= getVectorSum(allPacketsSentSta[3][staDeviceID]);
          station["tx_packets"]= allPacketsSentSta[3][staDeviceID].size();
          station["tx_retries"]= allPacketsReSentSta[3][staDeviceID].size();
          station["tx_failed"]= allPacketsFailedSta[3][staDeviceID].size();
          station["tx_bitrate"]= to_string(allTxRatesSta[3][staDeviceID]);
          station["rx_bitrate"]= to_string(allRxRatesSta[3][staDeviceID]);
          //cerr<<"rx_bitrate "<<allRxRatesSta[3][staDeviceID]<<endl;
          stations.push_back(station);
        }

        ext_iface["stations"] = stations;

      }

      ext_interfaces.insert(ext_interfaces.begin(), ext_iface);

    }

    ext["interfaces"]=ext_interfaces;

    if(get_neighboring_aps_for_exts){

    }

    ret["extenders"] = {ext};  

    if(get_speeds){

    }

    

    return ret;

  }

  Mac48Address
  getMacAddress(int index){
    Ptr<AdhocWifiMac> staMac = DynamicCast<AdhocWifiMac> (DynamicCast<WifiNetDevice> (staDevices.Get(index))->GetMac () );
    return staMac->GetAddress();
  }

  void
  setupRouting(){

    Ipv4StaticRoutingHelper ipv4RoutingHelper;

    for(int i=0;i<2;i++){
      staAssociations[i]=1;
    }

    for(int i=2;i<nStas;i++){
      staAssociations[i]=3;
    }

    Ptr<Ipv4> ipv4ap = (apNode.Get(0) )->GetObject<Ipv4> ();
    Ptr<Ipv4StaticRouting> staticRoutingMap = ipv4RoutingHelper.GetStaticRouting (ipv4ap);

    staticRoutingMap->AddHostRouteTo (extBackhaulInterface.GetAddress(0), apBackhaulInterface.GetAddress(0), 1);
    staticRoutingMap->AddHostRouteTo (extBackhaulInterface.GetAddress(0), extBackhaulInterface.GetAddress(0), 1);

    for(int i=0;i<nStas;i++){
      if(staAssociations[i]==1){
        //staticRoutingMap->AddHostRouteTo (staInterfaces.GetAddress(i), apFronthaulInterface.GetAddress(0), 2);
        staticRoutingMap->AddHostRouteTo (staInterfaces.GetAddress(i), staInterfaces.GetAddress(i), 2);
      }
      else if (staAssociations[i]==3) {
        staticRoutingMap->AddHostRouteTo (staInterfaces.GetAddress(i), extBackhaulInterface.GetAddress(0), 1);
        //staticRoutingMap->AddHostRouteTo (staInterfaces.GetAddress(i), staInterfaces.GetAddress(i), 1);
      }
    }

    Ptr<Ipv4> ipv4ext = (extNode.Get(0) )->GetObject<Ipv4> ();
    Ptr<Ipv4StaticRouting> staticRoutingExt = ipv4RoutingHelper.GetStaticRouting (ipv4ext);
    for(int i=0;i<nStas;i++){
      if (staAssociations[i]==3){
        //cerr<<"setting up routing"<<endl;
        //staticRoutingExt->AddHostRouteTo (staInterfaces.GetAddress(i), extFronthaulInterface.GetAddress(0), 2);
        //staticRoutingExt->AddHostRouteTo (staInterfaces.GetAddress(i), staInterfaces.GetAddress(i), 1);
        staticRoutingExt->AddHostRouteTo (staInterfaces.GetAddress(i), staInterfaces.GetAddress(i), 2);
      }
    }
    
    for(int i=0;i<nStas;i++){
      Ptr<Ipv4> ipv4sta = (staNodes.Get(i) )->GetObject<Ipv4> ();
      Ptr<Ipv4StaticRouting> staticRoutingSta = ipv4RoutingHelper.GetStaticRouting (ipv4sta);

      for(int i=0;i<nStas;i++){
        if(staAssociations[i]==1){
          staticRoutingSta->AddHostRouteTo (apFronthaulInterface.GetAddress(0), apFronthaulInterface.GetAddress(0), 1);
        }
        else if (staAssociations[i]==3){
          staticRoutingSta->AddHostRouteTo (apFronthaulInterface.GetAddress(0), extFronthaulInterface.GetAddress(0), 1);
        }
      }
    }

    if(enableInterference){
      for(int i=0;i<nHidden;i++){
        Ptr<Ipv4> ipv4ap = (hiddenAPs.Get(i) )->GetObject<Ipv4> ();
        Ptr<Ipv4StaticRouting> staticRoutingMap = ipv4RoutingHelper.GetStaticRouting (ipv4ap);
        staticRoutingMap->AddHostRouteTo (hiddenSTAInterfaces.GetAddress(i), hiddenSTAInterfaces.GetAddress(i), 1);
      }
    }

  }


  // void
  // pauseSimulation(){
  //   cerr<<"Pausing Simulation"<<endl;
  //   Simulator::Pause();
  //   sleep(5);
  //   cerr<<"Unpausing Simulation"<<endl;
  //   Simulator::Play();
  // }


  void changeExtLocation(bool nearer){

    //whether the extender should be placed nearer or further away from the master ap

    Ptr<MobilityModel> extMobility = extNode.Get(0)->GetObject<MobilityModel>();

    pair<double, double> new_location;

    if(nearer){
      new_location = make_pair((locations[0].first + locations[1].first)/2, (locations[0].second + locations[1].second)/2);
    }
    else{

      pair<double, double> sta_centroid =make_pair(0.0, 0.0);

      for(int i=0;i<nStas;i++){
        sta_centroid.first += locations[2+i].first;
        sta_centroid.second += locations[2+i].second;
      }

      sta_centroid.first /= nStas;
      sta_centroid.second /= nStas;

      new_location = make_pair((locations[1].first + sta_centroid.first)/2, (locations[1].second + sta_centroid.second)/2);
    }

    cerr<<"x: "<<new_location.first<<", y: "<<new_location.second<<endl;

    locations[1]=make_pair(new_location.first, new_location.second);

    extMobility->SetPosition(Vector(new_location.first, new_location.second, 0));
  }

  void relocateExt(bool nearer){
    calcAverageRSSIs();
    cerr<<"Backhaul RSSI: "<< allDevicesRSSI[0][2] << endl;
    //cerr<<sensing(1, 1, 1, 1, 1).dump()<<endl;
    changeExtLocation(nearer);
  }

  void changeChannelsAll(int channelBH, int channelFH){
    // cerr<<"change11"<<endl;
    // apDeviceBackhaul.Get(0)->GetObject<WifiNetDevice>()->GetPhy()->SetChannelNumber(channelBH);
    // currentChannels[0] = channelBH;
    // cerr<<"change12"<<endl;
    // extDeviceBackhaul.Get(0)->GetObject<WifiNetDevice>()->GetPhy()->SetChannelNumber(channelBH);
    // currentChannels[2] = channelBH;
    // cerr<<"change13"<<endl;
    // apDeviceFronthaul.Get(0)->GetObject<WifiNetDevice>()->GetPhy()->SetChannelNumber(channelFH);
    // currentChannels[1] = channelFH;
    // cerr<<"change14"<<endl;
    // extDeviceFronthaul.Get(0)->GetObject<WifiNetDevice>()->GetPhy()->SetChannelNumber(channelFH);
    // currentChannels[3] = channelFH;
    // cerr<<"change15"<<endl;
    // for(int i=0;i<nStas;i++){
    //   cerr<<"change "<<i<<endl;
    //   currentChannels[4+i] = channelFH;
    //   staDevices.Get(i)->GetObject<WifiNetDevice>()->GetPhy()->SetChannelNumber(channelFH);
    // }
    ch_bh_to_change = channelBH;
    ch_fh_to_change = channelFH;
  }

  void changeChannelRecover(int channelBHNew, int channelFHNew, int channelBHOld, int channelFHOld){
    // changeChannelsAll(channelBHNew, channelFHNew);
    ch_bh_to_change = channelBHNew;
    ch_fh_to_change = channelFHNew;
  }

  Ptr<SpectrumWifiPhy>
  GetSpectrumWifiPhyPtr (const NetDeviceContainer &nc)
  {
    Ptr<WifiNetDevice> wnd = nc.Get (0)->GetObject<WifiNetDevice> ();
    Ptr<WifiPhy> wp = wnd->GetPhy ();
    return wp->GetObject<SpectrumWifiPhy> ();
  }

  void peekChannels(){
    cerr<<"channel ap backhaul: "<<GetSpectrumWifiPhyPtr(apDeviceBackhaul.Get(0))->GetChannelNumber()<<endl;
    cerr<<"channel ext backhaul: "<<GetSpectrumWifiPhyPtr(extDeviceBackhaul.Get(0))->GetChannelNumber()<<endl;
    cerr<<"channel ap fronthaul: "<<GetSpectrumWifiPhyPtr(apDeviceFronthaul.Get(0))->GetChannelNumber()<<endl;
    cerr<<"channel ext fronthaul: "<<GetSpectrumWifiPhyPtr(extDeviceFronthaul.Get(0))->GetChannelNumber()<<endl;
    for(int i=0;i<nStas;i++){
      cerr<<"channel sta "<<i<<": "<<GetSpectrumWifiPhyPtr(staDevices.Get(i))->GetChannelNumber()<<endl;
    }
  }



  void changeChannelBackhaul(int channelBH){
    // apDeviceBackhaul.Get(0)->GetObject<WifiNetDevice>()->GetPhy()->SetChannelNumber(channelBH);
    // extDeviceBackhaul.Get(0)->GetObject<WifiNetDevice>()->GetPhy()->SetChannelNumber(channelBH);
    ch_bh_to_change = channelBH;
  }



  void channelChangeHandler(){
    //cerr<<"ch_bh_to_change: "<<ch_bh_to_change<<" ch_fh_to_change: "<<ch_fh_to_change<<endl;
    if(ch_bh_to_change != currentChannels[0]){
      apDeviceBackhaul.Get(0)->GetObject<WifiNetDevice>()->GetPhy()->SetChannelNumber(ch_bh_to_change);
      currentChannels[0] = ch_bh_to_change;
    }
    if(ch_bh_to_change != currentChannels[2]){
      extDeviceBackhaul.Get(0)->GetObject<WifiNetDevice>()->GetPhy()->SetChannelNumber(ch_bh_to_change);
      currentChannels[2] = ch_bh_to_change;
    }
    if(ch_fh_to_change != currentChannels[1]){
      apDeviceFronthaul.Get(0)->GetObject<WifiNetDevice>()->GetPhy()->SetChannelNumber(ch_fh_to_change);
      currentChannels[1] = ch_fh_to_change;
    }
    if(ch_fh_to_change != currentChannels[3]){
      extDeviceFronthaul.Get(0)->GetObject<WifiNetDevice>()->GetPhy()->SetChannelNumber(ch_fh_to_change);
      currentChannels[3] = ch_fh_to_change;
    }
    for(int i=0;i<nStas;i++){
      if(ch_fh_to_change != currentChannels[4+i]){
        cerr<<"change "<<i<<endl;
        cerr<<"ch_fh_to_change: "<<ch_fh_to_change<<" currentChannel: "<<currentChannels[4+i]<<endl;
        currentChannels[4+i] = ch_fh_to_change;
        staDevices.Get(i)->GetObject<WifiNetDevice>()->GetPhy()->SetChannelNumber(ch_fh_to_change);
      }
    }
    
    Simulator::Schedule(Seconds(0.2), &Experiment::channelChangeHandler, this);
  }

  void
  getSensingSample(int a1 ,int a2, int a3, int a4, int a5){
    cerr<<"FSDFSSD"<<endl;
    json sample = sensing(1, 1, 1, 1, 1);
    cerr<< sample.dump(4) << endl;
  }

  void playFor(uint32_t delay_in_milli){
    if(is_paused){
      cerr<<"IS paused"<<endl;
      Simulator::Play();
      is_paused = false;
    }

    int randomInt = rand() % 1032234 + 1;

    cerr<<"scheduling with context..."<<endl;
    Simulator::ScheduleWithContext(randomInt, Seconds(delay_in_milli/1000.0), &pauseSimulation);
  }

  void switchChannel(int bh_next, int fh_next){
    //Simulator::Pause();
    changeChannelRecover(bh_next, fh_next, currentChannels[0], currentChannels[1]);
    //playFor(playStepMilliLong);
    //int randomInt = rand() % 1032234 + 1;
    //Simulator::ScheduleWithContext(randomInt, Seconds(4.9), &peekChannels);
  }

  bool executeCommand(string command){
    istringstream iss(command);
    string s;

    int idx=0;

    bool continueToWrite = false;

    string command_keyword;

    vector<int> args;

    while ( getline( iss, s, ' ' ) ) {
      if(idx==0){
        command_keyword = s;
      }
      if(idx>=1){
        args.push_back(stoi(s));
      }
      idx++;
    }

    if(command_keyword == "sensing"){
      cerr<<"command sensing"<<endl;
      json_to_send = sensing(args[0], args[1], args[2], args[3], args[4]);
      cerr<<"sensing finished"<<endl;
      continueToWrite = true;
      pauseSimulation();
    }
    else if(command_keyword == "notify-location"){
      cerr<<"command notify location"<<endl;
      changeExtLocation(args[0]);
      playFor(playStepMilliLong);  
    }
    else if(command_keyword == "change-location-all"){
      cerr<<"command change location all"<<endl;
      changeChannelsAll(args[0], args[2]);
      playFor(playStepMilliLong); 
    }
    else if(command_keyword == "change-channel-recover"){
      cerr<<"command channel recover"<<endl;
      changeChannelRecover(args[0], args[1], args[2], args[3]);
      playFor(playStepMilliLong); 
    }
    else if(command_keyword == "change-channel-backhaul"){
      cerr<<"command change channel backhaul"<<endl;
      changeChannelBackhaul(args[0]);
      playFor(playStepMilliLong); 
    }
    else if(command_keyword == "no-action"){
      cerr<<"command no action"<<endl;
      playFor(playStepMilliShort); 
    }

    return continueToWrite;

  }


  void
  onStart(){
    cerr<<"onStart"<<endl;
    playFor(playStepMilliShort);
  }

  void 
  runExperiment (){

    // Packet::EnablePrinting ();
    // Packet::EnableChecking ();  

    //GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));

    //Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));

    

    LogLevel logLevel = (LogLevel) (LOG_LEVEL_DEBUG | LOG_PREFIX_TIME);
    //LogComponentEnable ("AdhocWifiMac", logLevel);
    //LogComponentEnable ("AdhocWifiMac", logLevel);

    apNode.Create(1);
    extNode.Create(1);
    staNodes.Create(nStas);
    
    if(enableInterference){
      hiddenAPs.Create(nHidden);
      hiddenSTAs.Create(nHidden);
    }

    SpectrumWifiPhyHelper spectrumPhy = SpectrumWifiPhyHelper::Default ();
    //Bug 2460: CcaMode1Threshold default should be set to -62 dBm when using Spectrum
    Config::SetDefault ("ns3::WifiPhy::CcaMode1Threshold", DoubleValue (-62.0));

    Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel> ();

    Ptr<LogDistancePropagationLossModel> lossModel = CreateObject<LogDistancePropagationLossModel> ();
    lossModel->SetPathLossExponent (2.273459);//4.273459
    spectrumChannel->AddPropagationLossModel (lossModel);

    Ptr<ConstantSpeedPropagationDelayModel> delayModel
      = CreateObject<ConstantSpeedPropagationDelayModel> ();
    spectrumChannel->SetPropagationDelayModel (delayModel);

    spectrumPhy.SetChannel (spectrumChannel);
    spectrumPhy.SetErrorRateModel (errorModelType);
    //spectrumPhy.Set ("Frequency", UintegerValue (5180));
    spectrumPhy.Set ("TxPowerStart", DoubleValue (8)); // dBm  (1.26 mW)
    spectrumPhy.Set ("TxPowerEnd", DoubleValue (8));

    //spectrumPhy.Set ("ShortGuardEnabled", BooleanValue (true));
    spectrumPhy.Set ("ChannelWidth", UintegerValue (22));

    WifiHelper wifi;
    wifi.SetStandard (WIFI_PHY_STANDARD_80211b);

    wifi.SetRemoteStationManager ("ns3::IdealWifiManager");

    Ssid ssid = Ssid ("ns-3-ssid");
    Ssid ssid1 = Ssid ("ns-3-ssid-hidden");
    Ssid ssid2 = Ssid ("ns-3-ssid-1");

    WifiMacHelper macSTA, macMAPBackhaul, macMAPFronthaul, macEXTFronthaul, macEXTBackhaul, macHiddenAP, macHiddenSTA;

    macMAPBackhaul.SetType ("ns3::AdhocWifiMac");

    macMAPFronthaul.SetType ("ns3::AdhocWifiMac");

    macEXTBackhaul.SetType ("ns3::AdhocWifiMac");
    
    macEXTFronthaul.SetType ("ns3::AdhocWifiMac");

    macSTA.SetType ("ns3::AdhocWifiMac");


    macHiddenAP.SetType ("ns3::AdhocWifiMac");
    macHiddenSTA.SetType("ns3::AdhocWifiMac");

    spectrumPhy.Set("ChannelNumber", UintegerValue(ch_bh_init));

    apDeviceBackhaul = wifi.Install (spectrumPhy, macMAPBackhaul, apNode);
    extDeviceBackhaul = wifi.Install (spectrumPhy, macEXTBackhaul, extNode);
    currentChannels[0]= ch_bh_init;
    currentChannels[2]= ch_bh_init;

    spectrumPhy.Set("ChannelNumber", UintegerValue(ch_fh_init));

    apDeviceFronthaul = wifi.Install (spectrumPhy, macMAPFronthaul, apNode);
    currentChannels[1]= ch_fh_init;

    extDeviceFronthaul = wifi.Install (spectrumPhy, macEXTFronthaul, extNode);
    staDevices = wifi.Install (spectrumPhy, macSTA, staNodes);
    currentChannels[3]= ch_fh_init;

    for(int i=0;i<nStas;i++){
      currentChannels[4+i] = ch_fh_init;
    } 

    if(enableInterference){
      hiddenAPDevices = wifi.Install (spectrumPhy, macHiddenAP, hiddenAPs);
      hiddenSTADevices = wifi.Install (spectrumPhy, macHiddenSTA, hiddenSTAs);
      for(int i=0;i<nHidden;i++){
        hiddenAPDevices.Get(i)->GetObject<WifiNetDevice>()->GetPhy()->SetChannelNumber(ch_intf_init[i]);
        hiddenSTADevices.Get(i)->GetObject<WifiNetDevice>()->GetPhy()->SetChannelNumber(ch_intf_init[i]);
      }
    }

    MobilityHelper mobility;

    Ptr<ListPositionAllocator> positionAlloc = CreateObject <ListPositionAllocator>();
    locations[0]=make_pair(0, 0);
    locations[1]=make_pair(ext_x, ext_y);
    positionAlloc ->Add(Vector(0, 0, 0)); // node0 (AP)
    positionAlloc ->Add(Vector(ext_x, ext_y, 0)); // node1 (EXT)

    // for(int i=0;i<nStas;i++){
    //   if(i<=nStas/2){
    //     positionAlloc ->Add(Vector(5, i*5, 0)); // node2 (STA node)
    //     locations[2+i]=make_pair(5, i*5);
    //   }
    //   else{
    //     positionAlloc ->Add(Vector(5, i*(-5), 0)); // node2 (STA node)
    //     locations[2+i]=make_pair(5, i*(-5));
    //   }
    // }

    positionAlloc ->Add(Vector(3, 2.5, 0));
    positionAlloc ->Add(Vector(4, 5, 0));

    positionAlloc ->Add(Vector(12, 2.5, 0));
    positionAlloc ->Add(Vector(12, 0, 0));
    positionAlloc ->Add(Vector(12, 1, 0));

    locations[2]=make_pair(3, 2.5);
    locations[3]=make_pair(4, 5);

    locations[4]=make_pair(12, 2.5);
    locations[5]=make_pair(12, 0.5);
    locations[6]=make_pair(12, 1);  

    if(enableInterference){
      positionAlloc ->Add(Vector(6, -2, 0)); // node3 (Hidden)
      positionAlloc ->Add(Vector(-2, 7, 0));
      positionAlloc ->Add(Vector(18, 7, 0));
      positionAlloc ->Add(Vector(7, 13, 0));

      positionAlloc ->Add(Vector(6, 4, 0)); // node4 (Hidden)
      positionAlloc ->Add(Vector(1, 4, 0));
      positionAlloc ->Add(Vector(16, 7, 0));
      positionAlloc ->Add(Vector(7, 9.5, 0));
    }

    mobility.SetPositionAllocator(positionAlloc);

    if(activateWalls){
      Ptr<Building> b = CreateObject <Building> ();
      b->SetBoundaries (Box (x_min, x_max, y_min, y_max, z_min, z_max));
      b->SetBuildingType (Building::Residential);
      b->SetExtWallsType (Building::ConcreteWithWindows);
      b->SetNFloors (NFloors);
      b->SetNRoomsX (NRoomsX);
      b->SetNRoomsY (NRoomsY);
    }
    
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    mobility.Install (apNode);
    mobility.Install (extNode);
    mobility.Install (staNodes);

    if(enableInterference){
      mobility.Install (hiddenAPs);
      mobility.Install (hiddenSTAs);
    }

    if(activateWalls){
      BuildingsHelper::Install (apNode);
      BuildingsHelper::Install (extNode);
      BuildingsHelper::Install (staNodes);
      BuildingsHelper::Install (hiddenAPs);
      BuildingsHelper::Install (hiddenSTAs);
      BuildingsHelper::MakeMobilityModelConsistent ();
    }

    InternetStackHelper internet;

    // Ipv4ListRoutingHelper listRouting;
    // listRouting.Add (ipv4RoutingHelper, 0);

    internet.Install (apNode);
    internet.Install (extNode);
    internet.Install (staNodes);
    

    if(enableInterference){
      internet.Install (hiddenAPs);
      internet.Install (hiddenSTAs);
    }

    Ipv4AddressHelper address;

    address.SetBase ("192.168.0.0", "255.255.255.0");
    apBackhaulInterface = address.Assign (apDeviceBackhaul);
    apFronthaulInterface = address.Assign (apDeviceFronthaul);
    extBackhaulInterface = address.Assign (extDeviceBackhaul);
    extFronthaulInterface = address.Assign(extDeviceFronthaul);
    staInterfaces = address.Assign(staDevices);

    if(enableInterference){
      hiddenAPInterfaces = address.Assign(hiddenAPDevices);
      hiddenSTAInterfaces = address.Assign(hiddenSTADevices);
    }

    

    for(int i=0;i<nStas;i++){

      ApplicationContainer cbrMain;
      int randomInt = rand() % 1000 + 1;

      
      uint16_t cbrPort = 9;
      OnOffHelper onOffHelper ("ns3::UdpSocketFactory", InetSocketAddress (staInterfaces.GetAddress(i), cbrPort));
      onOffHelper.SetAttribute ("PacketSize", UintegerValue (payloadSize));
      onOffHelper.SetAttribute ("OnTime",  StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
      onOffHelper.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));

      // flow 1:  node 0 -> node 1
      onOffHelper.SetAttribute ("DataRate", StringValue ("20Mb/s"));
      onOffHelper.SetAttribute ("StartTime", TimeValue (Seconds (startTime+0.001*i)));
      cbrMain.Add (onOffHelper.Install (apNode.Get (0))); 

    }

    if(enableInterference){
      ApplicationContainer cbrInterference;
      uint16_t cbrPort = 9;
      OnOffHelper onOffHelper ("ns3::UdpSocketFactory", InetSocketAddress (hiddenSTAInterfaces.GetAddress(0), cbrPort));
      onOffHelper.SetAttribute ("PacketSize", UintegerValue (payloadSize));
      onOffHelper.SetAttribute ("OnTime",  StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
      onOffHelper.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));

      // flow 1:  node 0 -> node 1
      onOffHelper.SetAttribute ("DataRate", StringValue ("20Mb/s"));
      onOffHelper.SetAttribute ("StartTime", TimeValue (Seconds (startTime+0.004)));
      cbrInterference.Add (onOffHelper.Install (hiddenAPs.Get (0)));
    }


    //Ptr<Socket> srcSocket1 = Socket::CreateSocket (apNode.Get(0), TypeId::LookupByName ("ns3::TcpSocketFactory"));
    //Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::StaWifiMac/Assoc", MakeCallback (&AssocTrace));

    //Simulator::Schedule(Seconds(startTime), &Experiment::onStart, this);
    setupRouting();
    // Simulator::Schedule(Seconds(0.1), stopHandler);
    // Simulator::Schedule(Seconds(0.2 + 0.001), &Experiment::channelChangeHandler, this);

    //Simulator::Schedule(Seconds(4), &switchChannel);
    //Simulator::Schedule(Seconds(3), &relocateExt, 0);
    //Simulator::Schedule(Seconds(4), &switchChannel, 11, 1);
    //Simulator::Schedule(Seconds(5), &switchChannel, 1, 6);
    //Simulator::Schedule(Seconds(3), &peekChannels);
    //Simulator::Schedule(Seconds(8.8), &pauseSimulation);
    //Simulator::Schedule(Seconds(8.5), &getSensingSample, 1, 1, 1, 1, 1);
    

    Config::Connect ("/NodeList/*/DeviceList/*/Mac/MacRx", MakeCallback (&MacRxOkTrace));
    Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/RxOk", MakeCallback (&PhyRxOkTrace));
    Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/RxError", MakeCallback (&PhyRxErrorTrace));
    Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/MacReTxRequired", MakeCallback (&ReTxTrace));
    Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/Tx", MakeCallback (&PhyTxTrace));
    Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::SpectrumWifiPhy/State/State", MakeCallback (&StateCallback));
    Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/RemoteStationManager/MacTxDataFailed", MakeCallback (&TxDataFailedTrace));
    Config::Connect ("/NodeList/*/DeviceList/*/Phy/MonitorSnifferRx", MakeCallback (&MonitorSniffRx));

    Simulator::Stop (Seconds(stopTime));

    // FlowMonitorHelper flowmon;
    // Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
    
    Simulator::Run ();

    //calcThroughputPeriodic();

    // monitor->CheckForLostPackets ();
    // Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
    // FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();
    // for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
    // {
    //   // first 2 FlowIds are for ECHO apps, we don't want to display them
    //   //
    //   // Duration for throughput measurement is 9.0 seconds, since 
    //   //   StartTime of the OnOffApplication is at about "second 1"
    //   // and 
    //   //   Simulator::Stops at "second 10".
    //   Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
    //   //std::cerr << "Flow " << i->first<< " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
    //   for(int j=0;j<nStas;j++){
    //     //cerr<<staInterfaces.GetAddress(j)<<endl;
    //     if(t.destinationAddress == staInterfaces.GetAddress(j)){
    //       double mean_delay = i->second.delaySum.GetSeconds () / i->second.rxPackets;
    //       //cerr<<"mean delay: "<<mean_delay<<endl;
    //       //cerr<<staInterfaces.GetAddress(j)<<endl;
    //       allDelays[4+j] = mean_delay;
    //     }
    //   }
    // }

    Simulator::Destroy ();

  }
};

Experiment experi;

void stopHandler(){
  if(stop_simul){
    stop_simul = false;
    cerr<<"STOPPING SIMULATION"<<endl;
    Simulator::Stop();
  }
  Simulator::Schedule(Seconds(0.1), stopHandler);
}


void
StateCallback (std::string context, Time init, Time duration, enum WifiPhy::State state)
{

//cerr<<"State change"<<endl;
int nWifis = nStas + 4;

uint16_t deviceID = getDeviceID(context);

uint64_t startMilliSeconds = startTime * 1000;

if(Simulator::Now().GetSeconds() < startTime || deviceID >= nWifis) return;

if (state == WifiPhy::CCA_BUSY)
  {
    //cerr<<"BUSY: "<<deviceID<<" "<<init.GetMilliSeconds ()<<endl;
    allDevicesChannelTimes[deviceID][1] += (init.GetMilliSeconds() < startMilliSeconds) ? duration.GetMilliSeconds () - (startMilliSeconds-init.GetMilliSeconds()): duration.GetMilliSeconds ();
  }
else if (state == WifiPhy::IDLE)
  {
    allDevicesChannelTimes[deviceID][2] += (init.GetMilliSeconds() < startMilliSeconds) ? duration.GetMilliSeconds () - (startMilliSeconds-init.GetMilliSeconds()): duration.GetMilliSeconds ();
  }
else if (state == WifiPhy::TX)
  {
    allDevicesChannelTimes[deviceID][3] += (init.GetMilliSeconds() < startMilliSeconds) ? duration.GetMilliSeconds () - (startMilliSeconds-init.GetMilliSeconds()): duration.GetMilliSeconds ();
  }
else if (state == WifiPhy::RX)
  {
    allDevicesChannelTimes[deviceID][4] += (init.GetMilliSeconds() < startMilliSeconds) ? duration.GetMilliSeconds () - (startMilliSeconds-init.GetMilliSeconds()): duration.GetMilliSeconds ();
  }
}

void
PhyRxOkTrace (string context, Ptr<const Packet> packet, double snr, WifiMode mode, enum WifiPreamble preamble)
{

  int nWifis = nStas + 4 + 2;

  uint16_t deviceID = getDeviceID(context);

  if(deviceID == 1 || deviceID == 3){

    uint16_t StaID =0;

    WifiMacHeader head;

    if(packet->PeekHeader (head)){
      Mac48Address dest = head.GetAddr2 ();

      //packet->Print(std::cerr);

      int senderID = experi.getDeviceIDByAddress(dest);

      

      if(senderID == 65535) return;

      if(senderID >= 4 && senderID<nWifis){

        //cerr<<"deviceID: "<<deviceID<<" senderID: "<<senderID<<endl;

        StaID = senderID;
        allPacketsReceivedSta[deviceID][StaID].push_back(packet->GetSize());
        allPacketsReceivedStaTime[deviceID][StaID].push_back(Simulator::Now().GetNanoSeconds());
      }

    }
  }

  if(packet->GetSize() < 100 && deviceID<nWifis){
    allPacketsOverhead[deviceID].push_back(packet->GetSize());
    allPacketsOverheadTime[deviceID].push_back(Simulator::Now().GetNanoSeconds());
  }
}

void
PhyRxErrorTrace (std::string context, Ptr<const Packet> packet, double snr)
{

  int nWifis = nStas + 4;

  uint16_t deviceID = getDeviceID(context);

  //cerr<<"Error deviceID: "<<deviceID<<endl;

  if(deviceID >= 0 && nWifis<nWifis){
    //errorSTA++;
    allPacketsError[deviceID].push_back( packet->GetSize() );
    allPacketsErrorTime[deviceID].push_back( Simulator::Now().GetNanoSeconds()  );
    //cerr<<"Node ID: "<<deviceID<< " Packet Size: " << packet->GetSize() << endl;
  }

  // if(packet->GetSize()<30)
  // {
  //   PhyTag tag;
  //   double rssi = 1;
  //   double senderID;

  //   if (packet->PeekPacketTag(tag)){
  //       rssi = tag.GetRSSI();
  //       senderID = tag.GetSender();
  //   }
   
  //   if(deviceID>=backboneNodes*2)
  //   deviceID-=backboneNodes;
  //   if(senderID>=backboneNodes*2)
  //   senderID-=backboneNodes;

  //   //cerr << allRSSIdB[deviceID].at(senderID) << "," << rssi << endl;
  //   if(deviceID == 4) NS_LOG_UNCOND("Time: "<<Simulator::Now().GetSeconds()<<" RSSI: "<<rssi);
  //   allRSSIdB[deviceID].at(senderID)=rssi;
  // }
}

void ReTxTrace (std::string context,Mac48Address address, Ptr<const Packet> packet)
{
    //cerr << context << endl;
    //cerr << address << endl;
    //cerr << packet->GetSize() << endl;

  int nWifis = nStas + 4;

  uint16_t deviceID = getDeviceID(context);

  //cerr<<"Resent deviceID: "<<deviceID<<" Address: "<<address<<endl;

  if(deviceID == 1 || deviceID == 3){

    uint16_t StaID =0;

    WifiMacHeader head;

    int senderID = experi.getDeviceIDByAddress(address);

    //cerr<<"Resent deviceID: "<<deviceID<<" senderID: "<<senderID<<endl;

    if(senderID < nWifis && senderID >= 4){
        StaID = senderID;
        allPacketsReSentSta[deviceID][StaID].push_back(packet->GetSize());
        allPacketsReSentStaTime[deviceID][StaID].push_back(Simulator::Now().GetNanoSeconds());
    }
  }
  
  //cerr << "deviceID = " << (int)deviceID << endl;
  //std::cerr << "Retransmission is needed" << ", AP = " << deviceID << ", DEST = " << address << std::endl;
  //NS_LOG_UNCOND( Simulator::Now().GetSeconds() << " Retransmission is needed" << ", AP = " << deviceID << ", DEST = " << address);

  //retransTotals[deviceID]++;

  if(deviceID>=0 && deviceID<nWifis){
    allPacketsReSent[deviceID].push_back(packet->GetSize());
    allPacketsReSentTime[deviceID].push_back(Simulator::Now().GetNanoSeconds());
  }

}

void
MonitorSniffRx (string context, Ptr<const Packet> packet,
                     uint16_t channelFreqMhz,
                     WifiTxVector txVector,
                     MpduInfo aMpdu,
                     SignalNoiseDbm signalNoise)
{

  //cerr<<"rssi"<<endl;
  int nWifis = nStas + 4;

  uint16_t deviceID = getDeviceID(context);

  double rssi = signalNoise.signal;

  if(deviceID<nWifis){
    uint16_t StaID =0;

    WifiMacHeader head;

    if(packet->PeekHeader (head)){
      Mac48Address dest = head.GetAddr2 ();

      int senderID = experi.getDeviceIDByAddress(dest);

      if(senderID == 65535) return;

      if(senderID<nWifis){
        allRSSIdB[senderID][deviceID].push_back(rssi);
      }
    }    
    
  }

  // g_samples++;
  // g_signalDbmAvg += ((signalNoise.signal - g_signalDbmAvg) / g_samples);
  // g_noiseDbmAvg += ((signalNoise.noise - g_noiseDbmAvg) / g_samples);
}

// void
// AssocTrace(string context, Mac48Address address){

//   int nWifis = nStas + 4;

//   uint16_t deviceID = getDeviceID(context);



//   int assocID = getDeviceIDByAddress(address);

//   cerr<<Simulator::Now().GetSeconds()<<" deviceID: "<<deviceID<<" assocID: "<<assocID<<endl;

//   if(deviceID >=4 && deviceID<nWifis){
//     NS_ASSERT(assocID==1 || assocID==3);
//     staAssociations[deviceID-4]=assocID;
//   }
  
// }

void
MacRxOkTrace (string context, Ptr<const Packet> packet)
{
  if(Simulator::Now().GetSeconds() < startTime) return;
  uint16_t deviceID = getDeviceID(context);

  int nWifis = nStas + 4;

  //cerr<<deviceID<<endl;

  if(currentChannels[0]==1 && currentChannels[1]==6 && deviceID>=4 && deviceID<nWifis){
    //cerr<<deviceID<<endl;
  }

  if(deviceID>=0 && packet->GetSize()>=1000 && deviceID < nWifis)
  {
    //cerr<<packet->GetSize()<<endl;
    WifiMacHeader hdr;
    //cerr<<packet->GetSize()-packet->PeekHeader(hdr)<<endl;
    allPacketsReceived[deviceID].push_back(packet->GetSize()-packet->PeekHeader(hdr));
    allPacketsReceivedTime[deviceID].push_back(Simulator::Now().GetNanoSeconds());

  }

}


void
PhyTxTrace (std::string context, Ptr<const Packet> packet, WifiMode mode, WifiPreamble preamble, uint8_t txPower)
{
    //std::cerr << Simulator::Now().GetSeconds() << ": PHYTX mode=" << mode << ", " << (int)packet->GetSize() << std::endl;
    //cerr << "context: " << context << endl;
    uint16_t deviceID = getDeviceID(context);

    int nWifis = nStas + 4;

    //cerr << "nodeID = " << (int)nodeID << endl;

    if(deviceID<nWifis && packet->GetSize()>=1000)
    {
      allPacketsSent[deviceID].push_back(packet->GetSize());
      allPacketsSentTime[deviceID].push_back(Simulator::Now().GetNanoSeconds());

      // WifiMacHeader hdr;
      // packet->PeekHeader(hdr);

      if(deviceID == 1 || deviceID == 3){

        uint16_t StaID =0;

        WifiMacHeader head;

        //packet->Print(std::cerr);
        
        if(packet->PeekHeader (head)){
          Mac48Address dest = head.GetAddr1 ();

          

          //packet->Print(std::cerr);
          
          //exit(0);

          int senderID = experi.getDeviceIDByAddress(dest);

          //cerr<<"Mac48Address: "<<dest<< " senderID: " << senderID <<endl;

          if(senderID>=4 && senderID<nWifis){
            StaID = senderID;
            allPacketsSentSta[deviceID][StaID].push_back(packet->GetSize());
            allPacketsSentStaTime[deviceID][StaID].push_back(Simulator::Now().GetNanoSeconds());
          }

        }

      }

      //cerr << "Transmit to IP: "<<hdr.GetAddr1()<<endl;
      //cerr << "Transmit  IP 2: "<<hdr.GetAddr2()<<endl;
      //cerr << "Transmit by IP: "<<hdr.GetAddr3()<<endl;

    }
}

void
TxDataFailedTrace (std::string context, Mac48Address address)
{
  uint16_t deviceID = getDeviceID(context);

  //cerr<<"Failed deviceID: "<<deviceID<<" Address: "<<address<<endl;

  int nWifis = nStas + 4;

  if(deviceID<nWifis)
  {
    allPacketsFailed[deviceID].push_back(1);
    allPacketsFailedTime[deviceID].push_back(Simulator::Now().GetNanoSeconds());
  }

  if(deviceID == 1 || deviceID == 3){

    uint16_t StaID =0;

    WifiMacHeader head;

    int senderID = experi.getDeviceIDByAddress(address);

    //cerr<<"Resent deviceID: "<<deviceID<<" senderID: "<<senderID<<endl;

    if(senderID < nWifis && senderID >= 4){
        StaID = senderID;
        allPacketsFailedSta[deviceID][StaID].push_back(1);
        allPacketsFailedStaTime[deviceID][StaID].push_back(Simulator::Now().GetNanoSeconds());
    }
  }


}

void write_to_matlab(){
      int sockfd, n, portno = WRITE_PORT;
      struct sockaddr_in serv_addr;
      struct hostent *server;

      bool first = true;

      char buffer[256];
      sockfd = socket(AF_INET, SOCK_STREAM, 0);
      if (sockfd < 0) 
          error("ERROR opening socket");
      //else cerr<<"socket opened"<<endl;
      server = gethostbyname(HOST);
      if (server == NULL) {
          fprintf(stderr,"ERROR, no such host\n");
          exit(0);
      }
      //else cerr<<"host exists"<<endl;
      bzero((char *) &serv_addr, sizeof(serv_addr));
      serv_addr.sin_family = AF_INET;
      bcopy((char *)server->h_addr, 
           (char *)&serv_addr.sin_addr.s_addr,  
           server->h_length);
      serv_addr.sin_port = htons(portno);
      
      //else cerr<<"connected"<<endl;

      while(1){
          is_ready = false;
          usleep(100000);
          std::unique_lock<std::mutex> lk(m);
          while(!is_ready){
              cerr<<"waiting to write"<<endl;
              cv.wait(lk);
              if (!is_ready)
                  std::cerr << "Spurious wake up!\n";
              cerr<<"woke up"<<endl;
          }
          if(first){
              if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
              error("ERROR connecting");
              first = false;
          }
          
          string json_to_send_string = json_to_send.dump();
          cerr<<"json_to_send_string: "<<json_to_send_string<<endl;
          n = write(sockfd,(json_to_send_string+ "\r").c_str(), json_to_send_string.size() + 1);
          
          if (n < 0) 
               error("ERROR writing to socket");
          cerr<<"wrote to socket"<<endl;
          cerr<<"TIMESTAMP: "<<json_to_send.at("timestamp").get<double>()<<endl;
          if(json_to_send.at("timestamp").get<double>() > stopTime-startTime){
            cerr<<"STOP SIMULATION"<<endl;
            stop_simul = true;
          }
      }
      close(sockfd);
  }

  void read_from_matlab(){
      int sockfd, newsockfd, portno = READ_PORT;
      socklen_t clilen;
      char buffer[256];
      struct sockaddr_in serv_addr, cli_addr;
      int n;
      sockfd = socket(AF_INET, SOCK_STREAM, 0);
      if (sockfd < 0) 
      error("ERROR opening socket");
      //else cerr<<"opened socket"<<endl;

      bzero((char *) &serv_addr, sizeof(serv_addr));
      serv_addr.sin_family = AF_INET;
      serv_addr.sin_addr.s_addr = INADDR_ANY;
      serv_addr.sin_port = htons(portno);
      if (bind(sockfd, (struct sockaddr *) &serv_addr,
            sizeof(serv_addr)) < 0) 
            error("ERROR on binding");
      //else cerr<<"successful binding"<<endl;
      listen(sockfd,5);
      clilen = sizeof(cli_addr);
      cerr<<"waiting to accept"<<endl;
      newsockfd = accept(sockfd, 
           (struct sockaddr *) &cli_addr, 
           &clilen);
      if (newsockfd < 0) 
        error("ERROR on accept");
      //else cerr<<"successful accept"<<endl;
      while(1){
          if(iter>1){
            cerr<<"iter>1"<<endl;
            cerr<<is_paused<<endl;
          }
          bzero(buffer,256);
          std::unique_lock<std::mutex> lk_pause(m_pause);
          while(!is_paused){
              cerr<<"waiting for simulation to pause"<<endl;
              cv_pause.wait(lk_pause);
              if (!is_paused)
                  std::cerr << "Spurious wake up!\n";
              cerr<<"simulation paused... going ahead"<<endl;
          }
          cerr<<"waiting for matlab"<<endl;
          n = read(newsockfd,buffer,255);
          if (n < 0) error("ERROR reading from socket");
          //else cerr<<"success reading from socket"<<endl;
          //printf("%s\n",buffer);

          command_read = buffer;
          trim(command_read);
          //if(arr.size() == 0) exit(1);
          cerr<<"Message from Matlab: "<<endl;
          cerr<<command_read<<endl;

          bool continueToWrite = experi.executeCommand(command_read);

          usleep(500000);

          if(continueToWrite){
            std::unique_lock<std::mutex> lk(m);
            is_ready = true;
            cv.notify_one();
            cerr<<"notifying"<<endl;
          }
      }
      //n = write(newsockfd,"I got your message",18);
      //if (n < 0) error("ERROR writing to socket");
      close(newsockfd);
      close(sockfd);
  }


int 
main (int argc, char *argv[]){

  int nWifis = nStas + 4  + 5;

  NS_ASSERT_MSG(argc == 2, "WRONG INPUT");

  iter = stoi(string(argv[1]));

  std::ofstream outfile, outfile_delay;

  outfile.open("CCA/throughput_CCA_pre.txt", std::ios_base::app);
  outfile_delay.open("CCA/delay_CCA_pre.txt", std::ios_base::app);

  allPacketsReceived.resize(nWifis);
  allPacketsReceivedTime.resize(nWifis);

  fill(allThroughputs.begin(), allThroughputs.end(),0.0);
  fill(allDelays.begin(), allDelays.end(),0.0);

  for(uint64_t i=0;i<nWifis;i++){
    allPacketsReceived[i].resize(stopTime);
    fill(allPacketsReceived[i].begin(), allPacketsReceived[i].end(),0.0);
    allPacketsReceivedTime[i].resize(stopTime);
    fill(allPacketsReceivedTime[i].begin(), allPacketsReceivedTime[i].end(),0.0);
  }
  experi = Experiment();
  experi.runExperiment();
  calcThroughputPeriodic();
  cerr<<"runExperiment returned"<<endl;
  //ssleep(20);

  double sm = 0.0;

  for(int k=0;k<nStas;k++){
    sm+=allThroughputs[4+k];
  }

  cerr<<"Throughput for "<<iter<<": "<<sm<<endl;
  outfile<<iter+1<<" "<<sm<<endl;

  // double average_delay = 0.0;

  // for(int k=0;k<nStas;k++){
  //   average_delay+=allDelays[4+k];
  // }

  // average_delay /= nStas;

  // cerr<<"Delay for "<<iter<<": "<<average_delay<<endl;
  // outfile_delay<<iter+1<<" "<<average_delay<<endl;
  //outfile<<iter+1<<","<<sm<<endl;
  

  // for(int i=0;i<ext_locations.size();i++){
  //   cerr<<ext_locations[i].first<<","<<ext_locations[i].second<<endl;
  // }

  return 0;
}