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

#include "ns3/seq-ts-header.h"


#include <nlohmann/json.hpp>

#define HOST "127.0.0.1"
#define WRITE_PORT 3093
#define READ_PORT 8093

// for convenience
using json = nlohmann::json;

using namespace ns3;
using namespace std;

NS_LOG_COMPONENT_DEFINE ("ThirdScriptExample");

bool enableInterference = true;
bool activateWalls = false;

int startTime = 2;
int stopTime = 120;

int maxDevicesPerNode = 2;

int nExts = 3;
int nStas = 4;
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

vector<double> allDelays;
vector<double> allDelaySums;
vector<double> allPacketSums;

vector<pair<double, double> > locations;

bool stop_simul = false;

double alpha = 0.3;
double beta = 0.4;
double temp = 0.0;
double omega = 0.0;
double phi = 0.0;
double sigma = 0.0;
double psi = 0.0;
double S = 0.0;//packet size

double rho = 0.0;
double eff_h = 0.0;
double eff_o = 0.0;



bool isAdmissionBlocked = false; //is admission of clients blocked
bool isSelfOptimizationBlocked = false;//is self-optimization blocked



vector< vector< vector<int>>> P; //set of all paths from an EXT to the mAP for each EXT
vector<double> measured_intf_thrs = {};%throughput at each interface 

vector<double> max_link_thrs = {};//link throughput when that link is saturated
vector<double> measured_link_thrs = {};//current total throughput of links
vector<double> max_assoc_thr = {};//max throughput that can be expected if only a single STA is assosciated to EXT j
vector<double> utils = {};//(i,j,j) channel utilization of the channel associated with link i,j meas at j
vector<double> sinrs = {};//(i,j) SINR of i measured at j
vector<double> retr = {};//(j) retries ratio of intf j

vector<double> eds_vector = {};//for each path, the most recently calculated value

vector<int> kappas = {};//(j) kappa of STA j

vector<double> thr_vector = {};//(j) current measured throughput at STA j
vector<double> del_vector = {};//(j) current measured delay at STA j


void stopHandler();

void
RxCallback (std::string, Ptr<const Packet>, const Address &);

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

void
MacRxOkTrace (string, Ptr<const Packet>);


void
PhyTxTrace (std::string, Ptr<const Packet>, WifiMode, WifiPreamble, uint8_t);

void
TxDataFailedTrace (std::string, Mac48Address);


class Graph
{
  int V;
  list<int> *adj;

  void printAllPathsUtil(int , int , bool [], int [], int &, vector< vector<int>>&);

public:
  Graph(int V);
  void addEdge(int u, int v);
  vector< vector<int>> printAllPaths(int s, int d);
};

Graph::Graph(int V)
{
  this->V = V;
  adj = new list<int>[V];
}

void Graph::addEdge(int u, int v)
{
  adj[u].push_back(v);
}

vector< vector<int>> Graph::printAllPaths(int s, int d)
{
  bool *visited = new bool[V];

  int *path = new int[V];
  int path_index = 0;

  for (int i = 0; i < V; i++)
    visited[i] = false;

    vector< vector<int>> paths;

  printAllPathsUtil(s, d, visited, path, path_index, paths);
  
  return paths;
}

void Graph::printAllPathsUtil(int u, int d, bool visited[],
              int path[], int &path_index, vector< vector<int>>& paths)
{
  visited[u] = true;
  path[path_index] = u;
  path_index++;

  if (u == d)
  {
      vector<int> vec;
    for (int i = 0; i<path_index; i++)
      vec.push_back(path[i]);
    paths.push_back(vec);
  }
  else
  {
    list<int>::iterator i;
    for (i = adj[u].begin(); i != adj[u].end(); ++i)
      if (!visited[*i])
        printAllPathsUtil(*i, d, visited, path, path_index, paths);
  }

  path_index--;
  visited[u] = false;
}


Graph connectivity_graph(nExts+1);

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
    cout<<Simulator::Now().GetSeconds()<<" Pausing Simulation"<<endl;
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
    //cout<<"token: "<<token<<endl;
    if(c==3){
      nodeID = atoi (token.c_str());
      // if(deviceID==1){
      //   cout<<context<<endl;
      //   cout<<"deviceID: "<<deviceID<<endl;
      // }
    }
    if(c==5){
      deviceID = atoi (token.c_str());
      if(deviceID==1){
        //cout<<context<<endl;
      }
      break;
    }
    s.erase(0, pos + delimiter.length());
  }

  // if(deviceID != -1 && deviceID != -1){
  //   return deviceID*maxDevicesPerNode + deviceID;
  // }

  // cout<<context<<endl;
  
  //cout<<"deviceID: "<<deviceID<<" deviceID: "<<deviceID<<endl;

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
  //   cout<<context<<endl;
  //   cout<<"deviceID: "<<ret<<endl;
  // }

  // if(ret == 5){
  //   cout<<"FIVE"<<endl;
  //   exit(0);
  // }

  if(ret == 65535)
    cout << "Device ID Error" << endl;

  return ret;
}

void calcAverageRSSIs(){
  for (uint16_t i = 0;  i < nStas + 4; ++ i)
  {
    
    for (uint16_t j = 0; j < nStas + 4; j++){

      double rssiSum = 0.0;

      for(uint16_t k=0;k< allRSSIdB[i][j].size(); k++) rssiSum += allRSSIdB[i][j][k];

      //cout<<"rssiSum: "<<rssiSum<<"size: "<<allRSSIdB[i][j].size()<<" "<<i<<" "<<j<<endl;

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

    cout<<i<<" rxTotal: "<<rxTotal<<endl;

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
          //cout<<i<<" "<<j<<" rxTotal: "<<rxTotal<<endl;
          break;
        }
      }

      //cout<<"k: "<<k<<endl;
      if(rxTotal > 0 && k+1!=allPacketsReceivedStaTime[i][j].size() - 1 ){
        allRxRatesSta[i][j]=(rxTotal*1000000000)/(allPacketsReceivedStaTime[i][j].back()-allPacketsReceivedStaTime[i][j][k+1]);
        allRxRatesSta[i][j]=(allRxRatesSta[i][j]*8.0/1000000);
      }

      for(k = allPacketsSentStaTime[i][j].size() - 1; k >=0 ; k--){
        if(allPacketsSentStaTime[i][j][k] > allPacketsSentStaTime[i][j].back() - 1000000000) txTotal+=allPacketsSentSta[i][j][k];
        else{
          //cout<<i<<" "<<j<<" txTotal: "<<txTotal<<endl;
          break;
        }
      }

      //cout<<"k: "<<k<<endl;
      if(txTotal > 0 && k+1!=allPacketsSentStaTime[i][j].size() - 1){
        allTxRatesSta[i][j]=(txTotal*1000000000)/(allPacketsSentStaTime[i][j].back()-allPacketsSentStaTime[i][j][k+1]);
        allTxRatesSta[i][j]=(allTxRatesSta[i][j]*8.0/1000000);
        //cout<<i<<" "<<j<<" rxTotal: "<<rxTotal<<" txTotal: "<<txTotal<<endl;
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


    //cout<<"thrMinT: "<<thrMinT<<" thrMaxT: "<<thrMaxT<<endl;

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
      if(t > startTime) cout <<  "Throughput at " << i << " at time "<< (int)t << ": "<< totR/1000000 << endl;
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

  double ext_x;
  double ext_y;

  // int ch_bh_init = 2, ch_fh_init = 5;
  // vector<int> ch_intf_init={1,2,4,8};

  int ch_bh_init = 3, ch_fh_init = 7;
  vector<int> ch_intf_init={3,7,1,9};

  int ch_bh_to_change =  ch_bh_init, ch_fh_to_change = ch_fh_init;

  int backhaul_signal = 0;
  int backhaul_signal_avg = 0;
  int backhaul_signal_samples = 0;

  NodeContainer apNode, extNodes, staNodes, hiddenAPs, hiddenSTAs;

  NetDeviceContainer apDeviceBackhaul, apDeviceFronthaul, extDeviceBackhauls, extDeviceFronthauls, staDevices, hiddenAPDevices, hiddenSTADevices;

  Ipv4InterfaceContainer apFronthaulInterface, apBackhaulInterface, staInterfaces, extFronthaulInterfaces, extBackhaulInterfaces, hiddenAPInterfaces, hiddenSTAInterfaces;

  Experiment (){
    int nWifis = nStas + 4  + 5;

    srand (time(NULL));

    // ext_x = rand() % 20 + 1;
    // ext_y = rand() % 10 + 1;

    ext_x = ext_locations[iter][0];
    ext_y = ext_locations[iter][1];

    is_paused = false;

    stop_simul = false;

    std::ofstream outfile;

    // outfile.open("ext_locations.txt", std::ios_base::app);
    // outfile << iter<<" "<<ext_x<<" "<< ext_y<<endl;

    //ext_locations.push_back(make_pair(ext_x, ext_y));


    currentChannels.resize(nWifis);

    allPacketsReceived.clear();
    allPacketsReceived.resize(nWifis);
    allPacketsReceivedTime.clear();
    allPacketsReceivedTime.resize(nWifis);

    allPacketsReceivedSta.clear();
    allPacketsReceivedSta.resize(nWifis);
    allPacketsReceivedStaTime.clear();
    allPacketsReceivedStaTime.resize(nWifis);

    allRSSIdB.clear();
    allRSSIdB.resize(nWifis);
    allDevicesRSSI.clear();
    allDevicesRSSI.resize(nWifis);

    allPacketsReSent.clear();
    allPacketsReSent.resize(nWifis);
    allPacketsReSentTime.clear();
    allPacketsReSentTime.resize(nWifis);

    allPacketsReSentSta.clear();
    allPacketsReSentSta.resize(nWifis);
    allPacketsReSentStaTime.clear();
    allPacketsReSentStaTime.resize(nWifis);

    allPacketsSent.clear();
    allPacketsSent.resize(nWifis);
    allPacketsSentTime.clear();
    allPacketsSentTime.resize(nWifis);

    allTxRates.clear();
    allTxRates.resize(nWifis);
    allRxRates.clear();
    allRxRates.resize(nWifis);

    allTxRatesSta.clear();
    allTxRatesSta.resize(nWifis);
    allRxRatesSta.clear();
    allRxRatesSta.resize(nWifis);

    allPacketsSentOk.clear();
    allPacketsSentOk.resize(nWifis);
    allPacketsSentOkTime.clear();
    allPacketsSentOkTime.resize(nWifis);

    allPacketsSentSta.clear();
    allPacketsSentSta.resize(nWifis);
    allPacketsSentStaTime.clear();
    allPacketsSentStaTime.resize(nWifis);

    allPacketsError.clear();
    allPacketsError.resize(nWifis);
    allPacketsErrorTime.clear();
    allPacketsErrorTime.resize(nWifis);

    allPacketsFailed.clear();
    allPacketsFailed.resize(nWifis);
    allPacketsFailedTime.clear();
    allPacketsFailedTime.resize(nWifis);

    allPacketsFailedSta.clear();
    allPacketsFailedSta.resize(nWifis);
    allPacketsFailedStaTime.clear();
    allPacketsFailedStaTime.resize(nWifis);

    allPacketsOverhead.clear();
    allPacketsOverhead.resize(nWifis);
    allPacketsOverheadTime.clear();
    allPacketsOverheadTime.resize(nWifis);

    allDevicesChannelTimes.clear();
    allDevicesChannelTimes.resize(nWifis);

    staAssociations.clear();
    staAssociations.resize(nWifis);

    locations.clear();
    locations.resize(nWifis);

    allDelays.clear();
    allDelays.resize(nWifis);

    allDelaySums.clear();
    allDelaySums.resize(nWifis);

    allPacketSums.clear();
    allPacketSums.resize(nWifis);

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


    //if(ret == 65535) cout<<address<<endl;
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

    //cout<<ret<<endl;
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

    cout<<a1<<" "<<a2<<" "<<a3<<" "<<a4<<" "<<a5<<endl;

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
          station["signal"] = 0.0;//allRSSIdB[staDeviceID].back();
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
    downlink_wds["signal"] = 0.0;//allRSSIdB[0][2].back();
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
    uplink_wds["signal"] = 0.0;//allRSSIdB[0][2].back();
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
          station["signal"] = 0.0;//allRSSIdB[3][staDeviceID].back();
          station["signal_avg"]= allDevicesRSSI[3][staDeviceID];
          station["rx_bytes"]= getVectorSum(allPacketsReceived[staDeviceID]);
          station["tx_bytes"]= getVectorSum(allPacketsSentSta[3][staDeviceID]);
          station["tx_packets"]= allPacketsSentSta[3][staDeviceID].size();
          station["tx_retries"]= allPacketsReSentSta[3][staDeviceID].size();
          station["tx_failed"]= allPacketsFailedSta[3][staDeviceID].size();
          station["tx_bitrate"]= to_string(allTxRatesSta[3][staDeviceID]);
          station["rx_bitrate"]= to_string(allRxRatesSta[3][staDeviceID]);
          //cout<<"rx_bitrate "<<allRxRatesSta[3][staDeviceID]<<endl;
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
        cout<<"setting up routing"<<endl;
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
  //   cout<<"Pausing Simulation"<<endl;
  //   Simulator::Pause();
  //   sleep(5);
  //   cout<<"Unpausing Simulation"<<endl;
  //   Simulator::Play();
  // }


  void relocateExt(bool nearer){
    calcAverageRSSIs();
    cout<<"Backhaul RSSI: "<< allDevicesRSSI[0][2] << endl;
    //cout<<sensing(1, 1, 1, 1, 1).dump()<<endl;
    changeExtLocation(nearer);
  }

  Ptr<SpectrumWifiPhy>
  GetSpectrumWifiPhyPtr (const NetDeviceContainer &nc)
  {
    Ptr<WifiNetDevice> wnd = nc.Get (0)->GetObject<WifiNetDevice> ();
    Ptr<WifiPhy> wp = wnd->GetPhy ();
    return wp->GetObject<SpectrumWifiPhy> ();
  }



  void
  getSensingSample(int a1 ,int a2, int a3, int a4, int a5){
    cout<<"FSDFSSD"<<endl;
    json sample = sensing(1, 1, 1, 1, 1);
    cout<< sample.dump(4) << endl;
  }

  void playFor(uint32_t delay_in_milli){
    if(is_paused){
      cout<<"IS paused"<<endl;
      Simulator::Play();
      is_paused = false;
    }

    int randomInt = rand() % 1032234 + 1;

    //cout<<"scheduling with context..."<<endl;
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
      cout<<"command sensing"<<endl;
      json_to_send = sensing(args[0], args[1], args[2], args[3], args[4]);
      cout<<"sensing finished"<<endl;
      continueToWrite = true;
      pauseSimulation();
    }
    else if(command_keyword == "notify-location"){
      cout<<"command notify location"<<endl;
      changeExtLocation(args[0]);
      playFor(playStepMilliLong);  
    }
    else if(command_keyword == "change-location-all"){
      cout<<"command change location all"<<endl;
      changeChannelsAll(args[0], args[2]);
      playFor(playStepMilliLong); 
    }
    else if(command_keyword == "change-channel-recover"){
      cout<<"command channel recover"<<endl;
      changeChannelRecover(args[0], args[1], args[2], args[3]);
      playFor(playStepMilliLong); 
    }
    else if(command_keyword == "change-channel-backhaul"){
      cout<<"command change channel backhaul"<<endl;
      changeChannelBackhaul(args[0]);
      playFor(playStepMilliLong); 
    }
    else if(command_keyword == "no-action"){
      cout<<"command no action"<<endl;
      playFor(playStepMilliShort); 
    }

    return continueToWrite;

  }


  void
  onStart(){
    cout<<"onStart"<<endl;
    playFor(playStepMilliShort);
  }

  // void
  // delayCalc(){
  //   cout<<"here"<<endl;
  //   monitor->CheckForLostPackets ();
  //   FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();
  //   double mean_delay = 0.0;
  //   for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
  //   {
  //     // first 2 FlowIds are for ECHO apps, we don't want to display them
  //     //
  //     // Duration for throughput measurement is 9.0 seconds, since 
  //     //   StartTime of the OnOffApplication is at about "second 1"
  //     // and 
  //     //   Simulator::Stops at "second 10".
  //     Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
  //     //std::cerr << "Flow " << i->first<< " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
  //     for(int j=0;j<nStas;j++){
  //       //cerr<<staInterfaces.GetAddress(j)<<endl;
  //       if(t.destinationAddress == staInterfaces.GetAddress(j)){

  //         //cout<<allDelaySums[4+j]<<" "<<i->second.delaySum.GetSeconds ()<<endl;
  //         //cout<<allPacketSums[4+j]<<" "<<i->second.rxPackets <<endl;

  //         double currentDelaySum = i->second.delaySum.GetSeconds ();
  //         double currentPacketSum = i->second.rxPackets;

  //         double sm = currentDelaySum;// - allDelaySums[4+j];
  //         double packet_sm = currentPacketSum;// - allPacketSums[4+j];

  //         allDelaySums[4+j] = currentDelaySum;
  //         allPacketSums[4+j] = currentPacketSum;

  //         double delay = sm / packet_sm;
  //         cerr<<"delay: "<<delay<<endl;
  //         //cerr<<staInterfaces.GetAddress(j)<<endl;

  //         allDelays[4+j] = delay;
  //         mean_delay += delay;
  //       }
  //     }
  //   }
  //   cout<<"mean_delay: "<<mean_delay/nStas<<endl;
  //   monitor->DoDispose();
  //   monitor->StartRightNow();
  //   FlowMonitorHelper flowmon;
  //   cout<<"here1"<<endl;
  //   monitor = flowmon.InstallAll ();
  //   cout<<"here2"<<endl;
  //   classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  //   cout<<"here3"<<endl;
  //   Simulator::Schedule(Seconds(1.0), &Experiment::delayCalc, this);
  // }

  void
  timeNote(){
    cout<<Simulator::Now().GetSeconds()<<endl;
    Simulator::Schedule(Seconds(1.0), &Experiment::timeNote, this);
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

    macMAPFronthaul.SetType ("ns3::ApWifiMac");

    macEXTBackhaul.SetType ("ns3::AdhocWifiMac");
    
    macEXTFronthaul.SetType ("ns3::ApWifiMac");

    macSTA.SetType ("ns3::StaWifiMac");


    macHiddenAP.SetType ("ns3::AdhocWifiMac");
    macHiddenSTA.SetType("ns3::AdhocWifiMac");

    spectrumPhy.Set("ChannelNumber", UintegerValue(ch_bh_init));

    apDeviceBackhaul = wifi.Install (spectrumPhy, macMAPBackhaul, apNode);
    extDeviceBackhauls = wifi.Install (spectrumPhy, macEXTBackhaul, extNode);
    currentChannels[0]= ch_bh_init;
    currentChannels[2]= ch_bh_init;

    spectrumPhy.Set("ChannelNumber", UintegerValue(ch_fh_init));

    apDeviceFronthaul = wifi.Install (spectrumPhy, macMAPFronthaul, apNode);
    currentChannels[1]= ch_fh_init;

    extDeviceFronthauls = wifi.Install (spectrumPhy, macEXTFronthaul, extNode);
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

      uint16_t cbrPort = 9;

      PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (staInterfaces.GetAddress(i), cbrPort));
      ApplicationContainer apps_sink = sink.Install (staNodes.Get (i));

      OnOffHelper onOffHelper ("ns3::UdpSocketFactory", InetSocketAddress (staInterfaces.GetAddress(i), cbrPort));
      onOffHelper.SetAttribute ("PacketSize", UintegerValue (payloadSize));
      onOffHelper.SetAttribute ("OnTime",  StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
      onOffHelper.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));

      // flow 1:  node 0 -> node 1
      onOffHelper.SetAttribute ("DataRate", StringValue ("20Mb/s"));
      onOffHelper.SetAttribute ("StartTime", TimeValue (Seconds (startTime+0.001*i)));
      onOffHelper.Install (apNode.Get (0));

      apps_sink.Start (Seconds (startTime));
      apps_sink.Stop (Seconds (stopTime));

    }

    if(enableInterference){
      for(int i=0;i<nHidden;i++){
        ApplicationContainer cbrInterference;
        uint16_t cbrPort = 9;
        OnOffHelper onOffHelper ("ns3::UdpSocketFactory", InetSocketAddress (hiddenSTAInterfaces.GetAddress(i), cbrPort));
        onOffHelper.SetAttribute ("PacketSize", UintegerValue (payloadSize));
        onOffHelper.SetAttribute ("OnTime",  StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
        onOffHelper.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));

        // flow 1:  node 0 -> node 1
        onOffHelper.SetAttribute ("DataRate", StringValue ("20Mb/s"));
        onOffHelper.SetAttribute ("StartTime", TimeValue (Seconds (startTime+0.004+0.0001*i)));
        cbrInterference.Add (onOffHelper.Install (hiddenAPs.Get (i))); 
      }
    }


    //Ptr<Socket> srcSocket1 = Socket::CreateSocket (apNode.Get(0), TypeId::LookupByName ("ns3::TcpSocketFactory"));
    //Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::StaWifiMac/Assoc", MakeCallback (&AssocTrace));

    Simulator::Schedule(Seconds(startTime), &Experiment::onStart, this);
    setupRouting();
    Simulator::Schedule(Seconds(0.1), stopHandler);
    Simulator::Schedule(Seconds(0.2 + 0.001), &Experiment::channelChangeHandler, this);

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
    Config::Connect ("/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx", MakeCallback (&RxCallback));

    //Simulator::Schedule(Seconds(stopTime - 1.0), &Experiment::delayCalc, this);
    
    //Simulator::Schedule(Seconds(1.0), &Experiment::timeNote, this);

    //Simulator::Stop(Seconds(stopTime));

    // FlowMonitorHelper flowmon;
    // Ptr<FlowMonitor> monitor = flowmon.InstallAll ();

    Simulator::Run ();

    double delay_sum = 0.0;
    double packet_sum = 0.0;

    for(int j=0;j<nStas;j++){
        delay_sum += allDelaySums[4+j];
        packet_sum += allPacketSums[4+j];
    }

    double mean_delay = delay_sum/packet_sum;

    std::ofstream outfile;

    outfile.open("ICALO_delays_1.txt", std::ios_base::app);

    cout<<"Mean delay: "<<mean_delay<<endl;
    outfile<<iter+1<<" "<<mean_delay<<endl;

    // monitor->CheckForLostPackets ();
    // Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
    // FlowMonitor::FlowStatsContainer stats = monitor->GetFlowStats ();
    // double mean_delay = 0.0;
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

    //       //cout<<allDelaySums[4+j]<<" "<<i->second.delaySum.GetSeconds ()<<endl;
    //       //cout<<allPacketSums[4+j]<<" "<<i->second.rxPackets <<endl;

    //       double currentDelaySum = i->second.delaySum.GetSeconds ();
    //       double currentPacketSum = i->second.rxPackets;

    //       double sm = currentDelaySum;// - allDelaySums[4+j];
    //       double packet_sm = currentPacketSum;// - allPacketSums[4+j];

    //       allDelaySums[4+j] = currentDelaySum;
    //       allPacketSums[4+j] = currentPacketSum;

    //       double delay = sm / packet_sm;
    //       cerr<<"delay: "<<delay<<endl;
    //       //cerr<<staInterfaces.GetAddress(j)<<endl;

    //       allDelays[4+j] = delay;
    //       mean_delay += delay;
    //     }
    //   }
    // }

    //calcThroughputPeriodic();
    Simulator::Destroy ();
  }
};

Experiment experi;

void stopHandler(){
  if(stop_simul){
    stop_simul = false;
    cout<<"STOPPING SIMULATION"<<endl;
    Simulator::Stop();
  }
  Simulator::Schedule(Seconds(0.1), stopHandler);
}


void
RxCallback (std::string context, Ptr<const Packet> packet, const Address &from){

  if(Simulator::Now().GetSeconds() < (stopTime - 2.0)) return;

  uint16_t deviceID = getDeviceID(context);

  TimestampTag timestamp;
  if (packet->FindFirstMatchingByteTag (timestamp)) {
    Time tx = timestamp.GetTimestamp ();
    if(deviceID >= 4 && deviceID<=8 ){
      //cout<<deviceID<<" "<<(Simulator::Now().GetSeconds() - tx.GetSeconds()) * 1000.0 <<endl;
      allDelaySums[deviceID]+=(Simulator::Now().GetSeconds() - tx.GetSeconds()) * 1000.0;
      allPacketSums[deviceID]+=1;
    }
  }
}

void
StateCallback (std::string context, Time init, Time duration, enum WifiPhy::State state)
{

//cout<<"State change"<<endl;
int nWifis = nStas + 4;

uint16_t deviceID = getDeviceID(context);

uint64_t startMilliSeconds = startTime * 1000;

if(Simulator::Now().GetSeconds() < startTime || deviceID >= nWifis) return;

if (state == WifiPhy::CCA_BUSY)
  {
    //cout<<"BUSY: "<<deviceID<<" "<<init.GetMilliSeconds ()<<endl;
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

  //int nWifis = nStas + 4 + 2;

  uint16_t deviceID = getDeviceID(context);

  if(deviceID < 65535){

    uint16_t StaID =0;

    WifiMacHeader head;

    if(packet->PeekHeader (head)){
      Mac48Address dest = head.GetAddr2 ();

      //packet->Print(std::cout);

      int senderID = experi.getDeviceIDByAddress(dest);  

      if(senderID == 65535) return;

      

    }
  }

  
}

void
PhyRxErrorTrace (std::string context, Ptr<const Packet> packet, double snr)
{

  int nWifis = nStas + 4;

  uint16_t deviceID = getDeviceID(context);

  //cout<<"Error deviceID: "<<deviceID<<endl;

  if(deviceID >= 0 && nWifis<nWifis){
    //errorSTA++;
    allPacketsError[deviceID].push_back( packet->GetSize() );
    allPacketsErrorTime[deviceID].push_back( Simulator::Now().GetNanoSeconds()  );
    //cout<<"Node ID: "<<deviceID<< " Packet Size: " << packet->GetSize() << endl;
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

  //   //cout << allRSSIdB[deviceID].at(senderID) << "," << rssi << endl;
  //   if(deviceID == 4) NS_LOG_UNCOND("Time: "<<Simulator::Now().GetSeconds()<<" RSSI: "<<rssi);
  //   allRSSIdB[deviceID].at(senderID)=rssi;
  // }
}

void ReTxTrace (std::string context,Mac48Address address, Ptr<const Packet> packet)
{
    //cout << context << endl;
    //cout << address << endl;
    //cout << packet->GetSize() << endl;

  int nWifis = nStas + 4;

  uint16_t deviceID = getDeviceID(context);

  //cout<<"Resent deviceID: "<<deviceID<<" Address: "<<address<<endl;

  if(deviceID == 1 || deviceID == 3){

    uint16_t StaID =0;

    WifiMacHeader head;

    int senderID = experi.getDeviceIDByAddress(address);

    //cout<<"Resent deviceID: "<<deviceID<<" senderID: "<<senderID<<endl;

    if(senderID < nWifis && senderID >= 4){
        StaID = senderID;
        allPacketsReSentSta[deviceID][StaID].push_back(packet->GetSize());
        allPacketsReSentStaTime[deviceID][StaID].push_back(Simulator::Now().GetNanoSeconds());
    }
  }
  
  //cout << "deviceID = " << (int)deviceID << endl;
  //std::cout << "Retransmission is needed" << ", AP = " << deviceID << ", DEST = " << address << std::endl;
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

//   cout<<Simulator::Now().GetSeconds()<<" deviceID: "<<deviceID<<" assocID: "<<assocID<<endl;

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

  // //cout<<deviceID<<endl;

  // if(currentChannels[0]==1 && currentChannels[1]==6 && deviceID>=4 && deviceID<nWifis){
  //   //cout<<deviceID<<endl;
  // }

  // if(deviceID>=0 && packet->GetSize()>=1000 && deviceID < nWifis)
  // {
  //   //cout<<packet->GetSize()<<endl;
  //   WifiMacHeader hdr;
  //   //cout<<packet->GetSize()-packet->PeekHeader(hdr)<<endl;
  //   allPacketsReceived[deviceID].push_back(packet->GetSize()-packet->PeekHeader(hdr));
  //   allPacketsReceivedTime[deviceID].push_back(Simulator::Now().GetNanoSeconds());

  // }

  WifiMacHeader head;
        
  if(packet->PeekHeader (head)){
    Mac48Address dest = head.GetAddr1 ();

    int senderID = experi.getDeviceIDByAddress(dest);

    connectivity_graph.addEdge(deviceID, senderID);
    connectivity_graph.addEdge(senderID, deviceID);

  }



}


void
PhyTxTrace (std::string context, Ptr<const Packet> packet, WifiMode mode, WifiPreamble preamble, uint8_t txPower)
{
    //std::cout << Simulator::Now().GetSeconds() << ": PHYTX mode=" << mode << ", " << (int)packet->GetSize() << std::endl;
    //cout << "context: " << context << endl;

    uint16_t deviceID = getDeviceID(context);

    int nWifis = nStas + 4;

    //cout << "nodeID = " << (int)nodeID << endl;

    if(deviceID<nWifis && packet->GetSize()>=1000)
    {
      allPacketsSent[deviceID].push_back(packet->GetSize());
      allPacketsSentTime[deviceID].push_back(Simulator::Now().GetNanoSeconds());

      // WifiMacHeader hdr;
      // packet->PeekHeader(hdr);

      TimestampTag timestamp;
      if (packet->FindFirstMatchingByteTag (timestamp)) {
        Time tx = timestamp.GetTimestamp ();

        //cout<<Simulator::Now().GetSeconds()<<" "<<tx.GetSeconds()<<endl;
      }

      if(deviceID == 1 || deviceID == 3){

        uint16_t StaID =0;

        WifiMacHeader head;

        //packet->Print(std::cout);
        
        if(packet->PeekHeader (head)){
          Mac48Address dest = head.GetAddr1 ();

          

          //packet->Print(std::cout);
          
          //exit(0);

          int senderID = experi.getDeviceIDByAddress(dest);

          //cout<<"Mac48Address: "<<dest<< " senderID: " << senderID <<endl;

          if(senderID>=4 && senderID<nWifis){
            StaID = senderID;
            allPacketsSentSta[deviceID][StaID].push_back(packet->GetSize());
            allPacketsSentStaTime[deviceID][StaID].push_back(Simulator::Now().GetNanoSeconds());
          }

        }

      }

      //cout << "Transmit to IP: "<<hdr.GetAddr1()<<endl;
      //cout << "Transmit  IP 2: "<<hdr.GetAddr2()<<endl;
      //cout << "Transmit by IP: "<<hdr.GetAddr3()<<endl;

    }
}

void
TxDataFailedTrace (std::string context, Mac48Address address)
{
  uint16_t deviceID = getDeviceID(context);

  //cout<<"Failed deviceID: "<<deviceID<<" Address: "<<address<<endl;

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

    //cout<<"Resent deviceID: "<<deviceID<<" senderID: "<<senderID<<endl;

    if(senderID < nWifis && senderID >= 4){
        StaID = senderID;
        allPacketsFailedSta[deviceID][StaID].push_back(1);
        allPacketsFailedStaTime[deviceID][StaID].push_back(Simulator::Now().GetNanoSeconds());
    }
  }


}


void
probeTrace (string context, Mac48Address address){
  uint16_t deviceID = getDeviceID(context);



  int probID = getDeviceIDByAddress(address);


  if(associated_map.find(probID)==associated_map.end()){
    unassociated_stas.insert(probID);
    unassociated_sta_candidates[probID].insert(deviceID);
  }



}

void
AssocTrace(string context, Mac48Address address){

  int nWifis = nStas + 4;

  uint16_t deviceID = getDeviceID(context);

  int assocID = getDeviceIDByAddress(address);

  cout<<Simulator::Now().GetSeconds()<<" deviceID: "<<deviceID<<" assocID: "<<assocID<<endl;

  if(deviceID >=4 && deviceID<nWifis){
    NS_ASSERT(assocID==1 || assocID==3);
    staAssociations[deviceID-4]=assocID;
  }
  
}


double
calc_mat(path){
    thrs = zeros(length(path), 1);

    for (int i = 0; i < paths.size(); ++i){
      thrs(i) = max_link_thrs(i) - measured_link_thrs(i);
    }

    return min(thrs);
}

double
get_tau_max(int j){
    paths = P(j);
    mats = zeros(length(paths), 1);

    for (int i = 0; i < paths.size(); ++i){
      link = paths(:, :, i);
      mats(i)=calc_mat(link);
    }

    return max(mats);
}

double
get_eeta_min(j){
    paths = P(j);
    eds = zeros(length(paths), 1);

    for (int i = 0; i < paths.size(); ++i){
      link = paths(:, :, i);
      eds(i)=eds_vector();
    }

    return min(eds);
}

double
get_thr_exp(i, j){
    return (1-utils(i, j, j)) * (min(get_tau_max(), ), measured_intf_thrs(j)) * log2(1+sinrs(i,j));
}

double
get_del_exp(){
    return S/get_thr_exp(i, j) * 1/(1-retr(j)) + get_eeta_min(j); 
}

double
get_measured_obj_func(){
    double sm_eta=0.0, sm_tau=0.0;
    int n_eta=0, n_tau=0;
    for (int i = 0; i < nStas; ++i){
      if(kappas[i]){
        sm_tau+=alpha*thr_vector[i];
        n_tau++;
      }
      else{
        sm_eta+=(1-alpha)*del_vector[i];
        n_eta++;
      }
    }
    return sm_tau/n_tau+sm_eta/n_eta;
}


void
buildConnectivityGraph(){

  uint16_t cbrPort = 9;

  for (int i = 0; i < nExts; ++i){
    OnOffHelper onOffHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address ("255.255.255.255"), cbrPort));
    onOffHelper.SetAttribute ("PacketSize", UintegerValue (payloadSize));
    onOffHelper.SetAttribute ("OnTime",  StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
    onOffHelper.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));

    // flow 1:  node 0 -> node 1
    onOffHelper.SetAttribute ("DataRate", StringValue ("20Mb/s"));
    onOffHelper.SetAttribute ("StartTime", TimeValue (Seconds (startTime+0.001)));
    onOffHelper.Install (extNodes.Get (i)); 
  }

  for (int i = 0; i < nExts; ++i){
    PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (extBackhaulInterfaces.GetAddress(i), cbrPort));
    ApplicationContainer apps_sink = sink.Install (extNodes.Get (i));

    apps_sink.Start (Seconds (startTime));
    apps_sink.Stop (Seconds (stopTime));
  }

  //pause for a short time

  vector< vector<int>> temp;

  P.push_back(temp);

  for (int i = 0; i < nExts; ++i){
    P.push_back(connectivity_graph.printAllPaths(i+1, 0));
  }
  


}


void
runAdmission(){

  while(1){
    if(isAdmissionBlocked){
      continue;
    }
    if(!unassociated_stas.empty()){
      continue;
    }

    isSelfOptimizationBlocked = true;

    int firstStaDeviceId = unassociated_stas.erase(unassociated_stas.begin());



    isSelfOptimizationBlocked = false;
  }


}

void
runSelfOptimization(){

  while(1){
    if(isSelfOptimizationBlocked){
      continue;
    }

    isAdmissionBlocked = true;


    isAdmissionBlocked = false;
  }

}

int
main (int argc, char *argv[]){

  std::thread read_thread(read_from_matlab);
  std::thread write_thread(write_to_matlab);

  read_thread.detach();
  write_thread.detach();
  cout<<"created threads"<<endl;

  vector<int> iter_vector = {4,6,12,14,16,18,20,22,24,26,29,31,34,35,38,40,49};

  for(int i=0;i<iter_vector.size();i++){
    iter = iter_vector[i] - 1;
    experi = Experiment();
    experi.runExperiment();
    cout<<"runExperiment returned"<<endl;
    //ssleep(20);
  }

  // for(int i=0;i<ext_locations.size();i++){
  //   cout<<ext_locations[i].first<<","<<ext_locations[i].second<<endl;
  // }

  return 0;
}