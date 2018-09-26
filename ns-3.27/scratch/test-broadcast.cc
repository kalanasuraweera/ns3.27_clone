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
#include "ns3/core-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/mobility-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/spectrum-module.h"
#include "matplotlibcpp.h"

// Network Topology

/*
*  m_xSize * step
*  |<--------->|
*   step
*  |<--->|      (ap1)
*  * --- * --- * <---Ping sink  _
*  | \   |   / |                ^
*  |   \ | /   |                |
*  * --- * --- * m_ySize * step |
*  |   / | \   |                |
*  | /   |   \ |                |
*  * --- * --- *                |
  (cli)  (ap0)
*  ^ Ping source
*/


using namespace ns3;
using namespace std;

namespace plt = matplotlibcpp;

NS_LOG_COMPONENT_DEFINE ("ThirdScriptExample");

void
RxCallback (std::string context, Ptr<const Packet> packet, const Address &from)
{

cout<<context<<endl;

// SeqTsHeader seqTs;
// packet->PeekHeader (seqTs);
// cout<<"delay: "<< Simulator::Now () - seqTs.GetTs ()<<endl;

Ptr<Packet> pkt = packet->Copy();

WifiMacHeader head;
WifiMacTrailer fcs;

std::cout<<"Packet Size:"<<pkt->GetSize()<<std::endl;

pkt->RemoveHeader(head);
//pkt->RemoveAllPacketTags ();
//pkt->RemoveAllByteTags ();

pkt->RemoveTrailer (fcs);

// std::cout<<"Packet Size:"<<pkt->GetSize()<<std::endl;
// uint8_t *buffer = new uint8_t[pkt->GetSize()];
// pkt->CopyData (buffer, pkt->GetSize());
// cout<<buffer[0]<<endl;


std::cout<<"Packet Size:"<<pkt->GetSize()<<std::endl;
uint8_t *buffer = new uint8_t[pkt->GetSize()];
pkt->CopyData (buffer, pkt->GetSize());
//string receivedData(buffer, buffer+p->GetSize());
//printf("%s\n", buffer);
std::string receivedData(buffer, buffer+ pkt->GetSize ());

std::cout<<"String Size:"<<receivedData.size()<<std::endl;

std::cout<<"Received:"<<receivedData.c_str()<<std::endl;
}

void
MacRxOkTrace (string context, Ptr<const Packet> packet)
{

//cout<<context<<endl;


// cout<<"delay: "<< Simulator::Now () - seqTs.GetTs ()<<endl;

Ptr<Packet> pkt = packet->Copy();

WifiMacHeader head;
WifiMacTrailer fcs;

//std::cout<<"Packet Size:"<<pkt->GetSize()<<std::endl;

// while(true){
//   cout<<pkt->PeekHeader(head)<<endl;
//   cout<<pkt->RemoveHeader(head)<<endl;
//   cout<<pkt->PeekHeader(head)<<endl;
//   if(!pkt->PeekHeader(head)) break;
// }

pkt->RemoveHeader(head);

//SeqTsHeader seqTs;
//pkt->RemoveHeader (seqTs);


pkt->RemoveHeader(head);
//cout<<pkt->GetSerializedSize()<<endl;
//pkt->RemoveAllPacketTags ();
//pkt->RemoveAllByteTags ();

//pkt->RemoveTrailer (fcs);

// std::cout<<"Packet Size:"<<pkt->GetSize()<<std::endl;
// uint8_t *buffer = new uint8_t[pkt->GetSize()];
// pkt->CopyData (buffer, pkt->GetSize());
// cout<<buffer[0]<<endl;


//std::cout<<"Packet Size:"<<pkt->GetSize()<<std::endl;
uint8_t *buffer = new uint8_t[pkt->GetSize()];
pkt->CopyData (buffer, pkt->GetSize());
//string receivedData(buffer, buffer+p->GetSize());
//printf("%s\n", buffer);
std::string receivedData(buffer, buffer+ pkt->GetSize ());
//std::cout<<"String Size:"<<receivedData.size()<<std::endl;

//for(int i=0;i<receivedData.size();i++) printf("%d\n", receivedData[i]);

std::string arr = receivedData.substr(0, receivedData.find(":"));
double timestamp = std::stod(receivedData.substr(receivedData.find(":")+1, receivedData.size()), NULL);

std::vector<uint32_t> vect;

std::stringstream ss(arr);

int i;

while (ss >> i)
{
    vect.push_back(i);

    if (ss.peek() == ',')
        ss.ignore();
}

for (i=0; i< vect.size(); i++)
        //std::cout << vect.at(i)<<std::endl;

//std::cout<<"Received timestamp: "<<timestamp<<std::endl;
if(Simulator::Now().GetSeconds() > 4) cout<<"Timestamp: "<<timestamp<<" Delay: "<<Simulator::Now().GetSeconds() - timestamp<<endl;

}


void
PhyRxOkTrace (string context, Ptr<const Packet> packet, double snr, WifiMode mode, enum WifiPreamble preamble)
{

  cout<<context<<endl;

  // SeqTsHeader seqTs;
  // packet->PeekHeader (seqTs);
  // cout<<"delay: "<< Simulator::Now () - seqTs.GetTs ()<<endl;

  Ptr<Packet> pkt = packet->Copy();

  WifiMacHeader head;
  WifiMacTrailer fcs;

  std::cout<<"Packet Size:"<<pkt->GetSize()<<std::endl;

  pkt->RemoveHeader(head);
  pkt->RemoveAllPacketTags ();
  pkt->RemoveAllByteTags ();

  pkt->RemoveTrailer (fcs);

  // std::cout<<"Packet Size:"<<pkt->GetSize()<<std::endl;
  // uint8_t *buffer = new uint8_t[pkt->GetSize()];
  // pkt->CopyData (buffer, pkt->GetSize());
  // cout<<buffer[0]<<endl;


  std::cout<<"Packet Size:"<<pkt->GetSize()<<std::endl;
  uint8_t *buffer = new uint8_t[pkt->GetSize()];
  pkt->CopyData (buffer, pkt->GetSize());
  //string receivedData(buffer, buffer+p->GetSize());
  std::string receivedData(buffer, buffer+pkt->GetSize ());
  std::cout<<"Received:"<<receivedData.c_str()<<std::endl;


  // if(pkt->PeekHeader (head)){
  //   head.Print(cout);
  //   cout<<endl;
  // }

  // int nWifis = nStas + 4 + 2;

  // uint16_t deviceID = getDeviceID(context);

  // if(deviceID == 1 || deviceID == 3){

  //   uint16_t StaID =0;

  //   WifiMacHeader head;

  //   if(packet->PeekHeader (head)){
  //     Mac48Address dest = head.GetAddr2 ();

  //     //packet->Print(std::cout);

  //     int senderID = experi.getDeviceIDByAddress(dest);

      

  //     if(senderID == 65535) return;

  //     if(senderID >= 4 && senderID<nWifis){

  //       //cout<<"deviceID: "<<deviceID<<" senderID: "<<senderID<<endl;

  //       StaID = senderID;
  //       allPacketsReceivedSta[deviceID][StaID].push_back(packet->GetSize());
  //       allPacketsReceivedStaTime[deviceID][StaID].push_back(Simulator::Now().GetNanoSeconds());
  //     }

  //   }
  // }
}

vector< vector<int>> P; 


class Graph
{
  int V;
  list<int> *adj;

  void printAllPathsUtil(int , int , bool [], int [], int &);

public:
  Graph(int V);
  void addEdge(int u, int v);
  void printAllPaths(int s, int d);
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

void Graph::printAllPaths(int s, int d)
{
  bool *visited = new bool[V];

  int *path = new int[V];
  int path_index = 0;

  for (int i = 0; i < V; i++)
    visited[i] = false;

  printAllPathsUtil(s, d, visited, path, path_index);
}

void Graph::printAllPathsUtil(int u, int d, bool visited[],
              int path[], int &path_index)
{
  visited[u] = true;
  path[path_index] = u;
  path_index++;

  if (u == d)
  {
      vector<int> vec;
    for (int i = 0; i<path_index; i++)
      vec.push_back(path[i]);
    P.push_back(vec);
  }
  else
  {
    list<int>::iterator i;
    for (i = adj[u].begin(); i != adj[u].end(); ++i)
      if (!visited[*i])
        printAllPathsUtil(*i, d, visited, path, path_index);
  }

  path_index--;
  visited[u] = false;
}




int 
main (int argc, char *argv[])
{
  bool verbose = true;
  uint32_t nCsma = 3;
  uint32_t nWifi = 3;
  bool tracing = true;

  int payloadSize = 972;

  int startTime = 0;
  int stopTime = 5;

  std::string errorModelType = "ns3::NistErrorRateModel";
  uint32_t nAP = 4;
  uint32_t whitelistIndex = 1;

  CommandLine cmd;
  cmd.AddValue ("nCsma", "Number of \"extra\" CSMA nodes/devices", nCsma);
  cmd.AddValue ("nWifi", "Number of wifi STA devices", nWifi);
  cmd.AddValue ("verbose", "Tell echo applications to log if true", verbose);
  cmd.AddValue ("tracing", "Enable pcap tracing", tracing);

  cmd.Parse (argc,argv);


  LogLevel logLevel = (LogLevel) (LOG_LEVEL_DEBUG | LOG_PREFIX_TIME);
  LogComponentEnable ("ApWifiMac", logLevel);
  LogComponentEnable ("StaWifiMac", logLevel);

  // The underlying restriction of 18 is due to the grid position
  // allocator's configuration; the grid layout will exceed the
  // bounding box if more than 18 nodes are provided.
  if (nWifi > 18)
    {
      std::cout << "nWifi should be 18 or less; otherwise grid layout exceeds the bounding box" << std::endl;
      return 1;
    }

  if (verbose)
    {
      //LogComponentEnable ("UdpEchoClientApplication", LOG_LEVEL_INFO);
      //LogComponentEnable ("UdpEchoServerApplication", LOG_LEVEL_INFO);
      LogLevel logLevel = (LogLevel) (LOG_LEVEL_DEBUG | LOG_PREFIX_TIME | LOG_LEVEL_FUNCTION);
      //LogComponentEnable ("ApWifiMac", logLevel);
      LogComponentEnable ("StaWifiMac", logLevel);
    }


  NodeContainer wifiApNodes;
  wifiApNodes.Create (nAP);

  NodeContainer wifiStaNodes;
  wifiStaNodes.Create (1);




  SpectrumWifiPhyHelper spectrumPhy = SpectrumWifiPhyHelper::Default ();
  //Bug 2460: CcaMode1Threshold default should be set to -62 dBm when using Spectrum
  Config::SetDefault ("ns3::WifiPhy::CcaMode1Threshold", DoubleValue (-62.0));

  Ptr<MultiModelSpectrumChannel> spectrumChannel = CreateObject<MultiModelSpectrumChannel> ();

  Ptr<LogDistancePropagationLossModel> lossModel = CreateObject<LogDistancePropagationLossModel> ();
  lossModel->SetPathLossExponent (4.273459);//4.273459
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

  WifiMacHelper mac;
  mac.SetType ("ns3::AdhocWifiMac");

  NetDeviceContainer staDevices;
  staDevices = wifi.Install (spectrumPhy, mac, wifiStaNodes);


  NetDeviceContainer apDevices;
  apDevices = wifi.Install (spectrumPhy, mac, wifiApNodes);


  //Ptr<WifiNetDevice> device = (Ptr<WifiNetDevice>) apDevices.Get(0);

  /*
  Ptr<ApWifiMac> apMac1 = DynamicCast<ApWifiMac> (DynamicCast<WifiNetDevice> (apDevices.Get(0))->GetMac () );
  Ptr<ApWifiMac> apMac2 = DynamicCast<ApWifiMac> (DynamicCast<WifiNetDevice> (apDevices.Get(1))->GetMac () );
  Ptr<StaWifiMac> staMac = DynamicCast<StaWifiMac> (DynamicCast<WifiNetDevice> (staDevices.Get(0))->GetMac () );

  
  std::cout << "The MAC of "<< 0 <<" is " << apMac1->GetAddress() << std::endl;
  std::cout << "The MAC of "<< 1 <<" is " << apMac2->GetAddress() << std::endl;

  std::cout << "The MAC of client is " << staMac->GetAddress() << std::endl;
  */

  MobilityHelper mobility;

  /*
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (1.0),
                                 "DeltaY", DoubleValue (1.0),
                                 "GridWidth", UintegerValue (3),
                                 "LayoutType", StringValue ("RowFirst"));
  */

  
  Ptr<ListPositionAllocator> positionAlloc = CreateObject <ListPositionAllocator>();
  positionAlloc ->Add(Vector(0, 0, 0)); // node0 (sta)
  positionAlloc ->Add(Vector(34, 4, 0)); // node1 (ap1)
  positionAlloc ->Add(Vector(3, 2, 0)); // node2 (ap2)
  positionAlloc ->Add(Vector(1, 2, 0)); // node3 (ap3)
  positionAlloc ->Add(Vector(1, 0, 0)); // node4 (ap4)
  mobility.SetPositionAllocator(positionAlloc);
  


  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifiStaNodes);
  mobility.Install (wifiApNodes);
  
  


  InternetStackHelper stack;
  stack.Install (wifiApNodes);
  stack.Install (wifiStaNodes);

  Ipv4AddressHelper address;

  address.SetBase ("192.168.1.0", "255.255.255.0");
  Ipv4InterfaceContainer staNodeInterfaces;
  Ipv4InterfaceContainer apNodeInterfaces;
  staNodeInterfaces  = address.Assign (staDevices);
  apNodeInterfaces = address.Assign (apDevices);

  /*
  UdpEchoServerHelper echoServer (9);

  ApplicationContainer serverApps = echoServer.Install (wifiApNodes.Get(0));
  serverApps.Start (Seconds (1.0));
  serverApps.Stop (Seconds (10.0));
  
  UdpEchoClientHelper echoClient (apNodeInterfaces.GetAddress(0), 9);
  echoClient.SetAttribute ("MaxPackets", UintegerValue (1));
  echoClient.SetAttribute ("Interval", TimeValue (Seconds (1.0)));
  echoClient.SetAttribute ("PacketSize", UintegerValue (1024));

  ApplicationContainer clientApps = echoClient.Install (wifiStaNodes.Get (0));
  clientApps.Start (Seconds (2.0));
  clientApps.Stop (Seconds (10.0));
  */

  //Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  
  

  Simulator::Stop (Seconds (stopTime));

  //phy.EnablePcap ("test", apDevices.Get (0), true);


  uint16_t cbrPort = 9;


  OnOffHelper onOffHelper ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address ("255.255.255.255"), cbrPort));
  onOffHelper.SetAttribute ("PacketSize", UintegerValue (payloadSize));
  onOffHelper.SetAttribute ("OnTime",  StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  onOffHelper.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));

  // flow 1:  node 0 -> node 1
  onOffHelper.SetAttribute ("DataRate", StringValue ("20Mb/s"));
  onOffHelper.SetAttribute ("StartTime", TimeValue (Seconds (startTime+0.001)));
  

  ApplicationContainer apps_source = onOffHelper.Install (wifiStaNodes.Get (0));

  Ptr<OnOffApplication> onOff = DynamicCast<OnOffApplication> (apps_source.Get ( apps_source.GetN()-1 ));

  //onOff->SetMaxBytes(1000000000);

  vector<uint32_t> vec = {1, 2, 3, 4};

  onOff->SetPath(vec);
  onOff->SetIsTimeStamp(true);

  for (int i = 0; i < nAP; ++i){
    PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (apNodeInterfaces.GetAddress(i), cbrPort));
    ApplicationContainer apps_sink = sink.Install (wifiApNodes.Get (i));

    apps_sink.Start (Seconds (startTime));
    apps_sink.Stop (Seconds (stopTime));
  }

  
  // try{
  //   //Ptr<ApWifiMac> apMac1 = DynamicCast<ApWifiMac> (DynamicCast<WifiNetDevice> (apDevices.Get(0))->GetMac () );
  //   //Ptr<ApWifiMac> apMac2 = DynamicCast<ApWifiMac> (DynamicCast<WifiNetDevice> (apDevices.Get(1))->GetMac () );
  //   Ptr<StaWifiMac> staMac = DynamicCast<StaWifiMac> (DynamicCast<WifiNetDevice> (staDevices.Get(0))->GetMac () );

  //   std::vector<Ptr<ApWifiMac> > v;

  //   for(uint32_t i = 0; i<nAP; i++){
  //     v.push_back( DynamicCast<ApWifiMac> (DynamicCast<WifiNetDevice> (apDevices.Get(i))->GetMac () ) );
  //     if( i != whitelistIndex) v[i]->addToBlacklist(staMac->GetAddress());
  //     std::cout << "The MAC of "<< i <<" is " << v[i]->GetAddress() << std::endl;
  //   }
  //   //std::cout << "Whitelisting STA at "<< v[whitelistIndex]->GetAddress() << std::endl;
  // }
  // catch (const std::exception& e) { // reference to the base of a polymorphic object
  //    std::cout << e.what(); // information from length_error printed
  // }


  // plt::plot({1,3,2,4});
  // plt::show();

  Config::Connect ("/NodeList/*/DeviceList/*/Mac/MacRx", MakeCallback (&MacRxOkTrace));
  //Config::Connect ("/NodeList/*/DeviceList/*/Phy/State/RxOk", MakeCallback (&PhyRxOkTrace));
  //Config::Connect ("/NodeList/*/ApplicationList/*/$ns3::PacketSink/Rx", MakeCallback (&RxCallback));
  

  Simulator::Run ();
  Simulator::Destroy ();
  return 0; 
}
