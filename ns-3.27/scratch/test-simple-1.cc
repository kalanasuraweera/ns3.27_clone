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

namespace plt = matplotlibcpp;

NS_LOG_COMPONENT_DEFINE ("ThirdScriptExample");


int 
main (int argc, char *argv[])
{
  bool verbose = true;
  uint32_t nCsma = 3;
  uint32_t nWifi = 3;
  bool tracing = true;

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
  Ssid ssid = Ssid ("ns-3-ssid");
  mac.SetType ("ns3::StaWifiMac",
               "Ssid", SsidValue (ssid),
               "ActiveProbing", BooleanValue (true));

  NetDeviceContainer staDevices;
  staDevices = wifi.Install (spectrumPhy, mac, wifiStaNodes);

  mac.SetType ("ns3::ApWifiMac",
               "Ssid", SsidValue (ssid));

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
  positionAlloc ->Add(Vector(4, 4, 0)); // node1 (ap1)
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

  Simulator::Stop (Seconds (5));

  //phy.EnablePcap ("test", apDevices.Get (0), true);

  
  try{
    //Ptr<ApWifiMac> apMac1 = DynamicCast<ApWifiMac> (DynamicCast<WifiNetDevice> (apDevices.Get(0))->GetMac () );
    //Ptr<ApWifiMac> apMac2 = DynamicCast<ApWifiMac> (DynamicCast<WifiNetDevice> (apDevices.Get(1))->GetMac () );
    Ptr<StaWifiMac> staMac = DynamicCast<StaWifiMac> (DynamicCast<WifiNetDevice> (staDevices.Get(0))->GetMac () );

    std::vector<Ptr<ApWifiMac> > v;

    for(uint32_t i = 0; i<nAP; i++){
      v.push_back( DynamicCast<ApWifiMac> (DynamicCast<WifiNetDevice> (apDevices.Get(i))->GetMac () ) );
      if( i != whitelistIndex) v[i]->addToBlacklist(staMac->GetAddress());
      std::cout << "The MAC of "<< i <<" is " << v[i]->GetAddress() << std::endl;
    }
    //std::cout << "Whitelisting STA at "<< v[whitelistIndex]->GetAddress() << std::endl;
  }
  catch (const std::exception& e) { // reference to the base of a polymorphic object
     std::cout << e.what(); // information from length_error printed
  }


  plt::plot({1,3,2,4});
  plt::show();

  

  Simulator::Run ();
  Simulator::Destroy ();
  return 0; 
}
