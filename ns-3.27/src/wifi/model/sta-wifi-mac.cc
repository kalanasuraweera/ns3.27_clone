/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
     /*
     * Copyright (c) 2006, 2009 INRIA
     * Copyright (c) 2009 MIRKO BANCHI
     *
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
    *
     * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
    * Author: Mirko Banchi <mk.banchi@gmail.com>
    */
 #include "sta-wifi-mac.h"

 #include "ns3/log.h"
 #include "ns3/simulator.h"
 #include "ns3/string.h"
 #include "ns3/pointer.h"
 #include "ns3/boolean.h"
 #include "ns3/trace-source-accessor.h"

 #include "mac-low.h"
 #include "dcf-manager.h"
 #include "snr-tag.h"
 #include "mac-rx-middle.h"
 #include "mac-tx-middle.h"
 #include "wifi-mac-header.h"
 #include "msdu-aggregator.h"
 #include "amsdu-subframe-header.h"
 #include "mgt-headers.h"
 #include "ht-capabilities.h"

 /*
  * The state machine for this STA is:
  --------------                                          -----------
  | Associated |   <--------------------      ------->    | Refused |
  --------------                        \    /            -----------
     \                    --------       \  /
      \    ------------   | Wait |   -----------------------------
       \-> | Scanning |-->|Beacon|-->| Wait Association Response |
           ------------   --------   -----------------------------
            ^ |  \                       ^
            | |   \                      |
             -     \    -----------------------
                    \-> | Wait Probe Response |
                        -----------------------

   Unassociated (not depicted) is a transient state between Scanning and either
   WaitBeacon or WaitProbeResponse
  */
using namespace std;
 namespace ns3 {

 NS_LOG_COMPONENT_DEFINE ("StaWifiMac");

 NS_OBJECT_ENSURE_REGISTERED (StaWifiMac);

 TypeId
 StaWifiMac::GetTypeId (void)
 {
   static TypeId tid = TypeId ("ns3::StaWifiMac")
     .SetParent<RegularWifiMac> ()
     .SetGroupName ("Wifi")
     .AddConstructor<StaWifiMac> ()
     .AddAttribute ("ProbeRequestTimeout", "The interval between two consecutive probe request attempts.",
                    TimeValue (Seconds (0.05)),
                    MakeTimeAccessor (&StaWifiMac::m_probeRequestTimeout),
                    MakeTimeChecker ())
     .AddAttribute ("AssocRequestTimeout", "The interval between two consecutive assoc request attempts.",
                    TimeValue (Seconds (0.1)),
                    MakeTimeAccessor (&StaWifiMac::m_assocRequestTimeout),
                    MakeTimeChecker ())
     .AddAttribute ("ScanningTimeout", "The interval to dwell on a channel while scanning",
                    TimeValue (MilliSeconds (30)),
                    MakeTimeAccessor (&StaWifiMac::m_scanningTimeout),
                    MakeTimeChecker ())
	.AddAttribute ("ScanningTimeoutHandover", "The interval to dwell on a channel while scanning",
					TimeValue (MilliSeconds (30)),
					MakeTimeAccessor (&StaWifiMac::m_scanningHOTimeout),
					MakeTimeChecker ())
     .AddAttribute ("MaxMissedBeacons",
                    "Number of beacons which much be consecutively missed before "
                    "we attempt to restart association.",
                    UintegerValue (100000000),
                    MakeUintegerAccessor (&StaWifiMac::m_maxMissedBeacons),
                    MakeUintegerChecker<uint32_t> ())
     .AddAttribute ("ActiveProbing", "If true, we send probe requests. If false, we don't. NOTE: if more than one STA in your simulation is using active probing, you should enable it at a different simulation time for each STA, otherwise all the STAs will start sending probes at the same time resulting in collisions. See bug 1060 for more info.",
                    BooleanValue (false),
                    MakeBooleanAccessor (&StaWifiMac::SetActiveProbing, &StaWifiMac::GetActiveProbing),
                    MakeBooleanChecker ())
     .AddTraceSource ("Assoc", "Associated with an access point.",
                      MakeTraceSourceAccessor (&StaWifiMac::m_assocLogger),
                      "ns3::Mac48Address::TracedCallback")
     .AddTraceSource ("DeAssoc", "Association with an access point lost.",
                      MakeTraceSourceAccessor (&StaWifiMac::m_deAssocLogger),
                      "ns3::Mac48Address::TracedCallback")
   ;
   return tid;
 }

 StaWifiMac::StaWifiMac ()
   : m_state (SCANNING),
     m_probeRequestEvent (),
     m_assocRequestEvent (),
     m_waitBeaconEvent (),
     m_beaconWatchdogEnd (Seconds (0.0))
 {
   NS_LOG_FUNCTION (this);

   // Let the lower layers know that we are acting as a non-AP STA in
   // an infrastructure BSS.
   SetTypeOfStation (STA);
 }

 StaWifiMac::~StaWifiMac ()
 {
   NS_LOG_FUNCTION (this);
 }

 void
 StaWifiMac::DoInitialize (void)
 {
	 m_probeRequestEvent = Simulator::Schedule (Seconds(4),
	                                               &StaWifiMac::StartScanning, this);
	 //StartScanning();
 }

 void
 StaWifiMac::SetMaxMissedBeacons (uint32_t missed)
 {
   NS_LOG_FUNCTION (this << missed);
   m_maxMissedBeacons = missed;
 }

 void
 StaWifiMac::SetProbeRequestTimeout (Time timeout)
 {
   NS_LOG_FUNCTION (this << timeout);
   m_probeRequestTimeout = timeout;
 }

 void
 StaWifiMac::SetAssocRequestTimeout (Time timeout)
 {

   NS_LOG_FUNCTION (this << timeout);
   m_assocRequestTimeout = timeout;
 }

 void
 StaWifiMac::StartActiveAssociation (void)
 {
	 cout << "StartActiveAssociation" <<endl;
   NS_LOG_FUNCTION (this);
   TryToEnsureAssociated ();
 }

 void
 StaWifiMac::SetActiveProbing (bool enable)
 {
   NS_LOG_FUNCTION (this << enable);
   if (enable)
     {
       Simulator::ScheduleNow (&StaWifiMac::TryToEnsureAssociated, this);
     }
   else
     {
       m_probeRequestEvent.Cancel ();
     }
   m_activeProbing = enable;
 }

 bool StaWifiMac::GetActiveProbing (void) const
 {
   return m_activeProbing;
 }

 void
 StaWifiMac::SendProbeRequest (void)
 {
     cout << "SendProbeRequest= "  <<endl;
   NS_LOG_FUNCTION (this);
   WifiMacHeader hdr;
   hdr.SetProbeReq ();
   hdr.SetAddr1 (Mac48Address::GetBroadcast ());
   hdr.SetAddr2 (GetAddress ());
   hdr.SetAddr3 (Mac48Address::GetBroadcast ());
   hdr.SetDsNotFrom ();
   hdr.SetDsNotTo ();
   Ptr<Packet> packet = Create<Packet> ();
   MgtProbeRequestHeader probe;
   probe.SetSsid (GetSsid ());
   probe.SetSupportedRates (GetSupportedRates ());
   if (m_htSupported)
     {
       probe.SetHtCapabilities (GetHtCapabilities());
       hdr.SetNoOrder();
     }

   packet->AddHeader (probe);

   // The standard is not clear on the correct queue for management
   // frames if we are a QoS AP. The approach taken here is to always
   // use the DCF for these regardless of whether we have a QoS
   // association or not.
   m_dca->Queue (packet, hdr);

   if (m_probeRequestEvent.IsRunning ())
     {
       m_probeRequestEvent.Cancel ();
     }
   if(m_state!=SCANNING_HO)
   m_probeRequestEvent = Simulator::Schedule (m_probeRequestTimeout,
                                              &StaWifiMac::ProbeRequestTimeout, this);
   //if(m_state==SCANNING_HO)
   ////cout << "Probe Request is Sent" << endl;
 }

 void
 StaWifiMac::SendAssociationRequest (void)
 {
   NS_LOG_FUNCTION (this << GetBssid ());
   WifiMacHeader hdr;
   hdr.SetAssocReq ();
   hdr.SetAddr1 (GetBssid ());
   hdr.SetAddr2 (GetAddress ());
   hdr.SetAddr3 (GetBssid ());
   hdr.SetDsNotFrom ();
   hdr.SetDsNotTo ();
   Ptr<Packet> packet = Create<Packet> ();
   MgtAssocRequestHeader assoc;
   assoc.SetSsid (GetSsid ());
   assoc.SetSupportedRates (GetSupportedRates ());
   if (m_htSupported)
     {
       assoc.SetHtCapabilities (GetHtCapabilities());
       hdr.SetNoOrder();
     }

   packet->AddHeader (assoc);

   // The standard is not clear on the correct queue for management
   // frames if we are a QoS AP. The approach taken here is to always
   // use the DCF for these regardless of whether we have a QoS
   // association or not.
   m_dca->Queue (packet, hdr);

   if (m_assocRequestEvent.IsRunning ())
     {
       m_assocRequestEvent.Cancel ();
     }
   m_assocRequestEvent = Simulator::Schedule (m_assocRequestTimeout,
                                              &StaWifiMac::AssocRequestTimeout, this);
 }

 void
 StaWifiMac::TryToEnsureAssociated (void)
 {
   cout << "TryToEnsureAssociated = " << m_state << endl;
   NS_LOG_FUNCTION (this);
   switch (m_state)
     {
     case ASSOCIATED:
       return;
       break;
     case WAIT_PROBE_RESP:
       /* we have sent a probe request earlier so we
          do not need to re-send a probe request immediately.
          We just need to wait until probe-request-timeout
          or until we get a probe response
        */
       break;
     case UNASSOCIATED:
       /* we were associated but we missed a bunch of beacons
        * so we should assume we are not associated anymore.
        * We try to initiate a probe request now.
        */
       m_linkDown ();
       if (m_activeProbing)
         {
           SetState (WAIT_PROBE_RESP);
           m_bestBeaconObserved.Clear ();
           clearBssidQueue();
           SendProbeRequest ();
         }
       else
         {
           if (m_waitBeaconEvent.IsRunning ())
             {
               m_waitBeaconEvent.Cancel ();
             }
           m_bestBeaconObserved.Clear ();
           clearBssidQueue();
           m_waitBeaconEvent = Simulator::Schedule (m_scanningTimeout,
                                        &StaWifiMac::WaitBeaconTimeout, this);
           cout << "m_scanningTimeout = " << m_scanningTimeout << endl;
           m_bestBeaconObserved.Clear ();
           SetState (WAIT_BEACON);
         }
       break;
     case WAIT_BEACON:
       /* Continue to wait and gather beacons */
       break;
     case WAIT_ASSOC_RESP:
       /* we have sent an assoc request so we do not need to
          re-send an assoc request right now. We just need to
          wait until either assoc-request-timeout or until
          we get an assoc response.
        */
       break;
     case REFUSED:
       /* we have sent an assoc request and received a negative
          assoc resp. We wait until someone restarts an
          association with a given ssid.
        */
    	 ////cout << "Refused" << endl;
       StartScanning ();
       break;
     case SCANNING:
     case SCANNING_HO:
       break;
     }
 }

 void
 StaWifiMac::AssocRequestTimeout (void)
 {
   NS_LOG_FUNCTION (this);
   SetState (WAIT_ASSOC_RESP);
   //SendAssociationRequest ();
   PreSendAssociationRequest(); //newly added
 }

 void
StaWifiMac::SelectAPForAssociation(void){
  NS_LOG_FUNCTION(this);
  /*
  if(bssidQueue.size() == 0){
    NS_LOG_DEBUG("Loading backup");
    bssidQueue = std::priority_queue<BeaconInfo, std::vector<BeaconInfo>, CompareBeaconInfo>(backupBssids.begin(), backupBssids.end() );
    backupBssids.clear();
  }
  */
  /*
  if(bestBeacon.m_bssid == m_lastRequestedBssid){
    if(bssidQueue.size() == 0) return;
    bestBeacon = popBssidQueue();
  }
  */
  if(bssidQueue.size() == 0){
    m_bestBeaconObserved.Clear();
    return;
  }
  BeaconInfo bestBeacon = popBssidQueue();
  //NS_LOG_DEBUG ("Resetting the BSSID to " << bestBeacon.m_bssid << " : " << bestBeacon.m_snr);
  //NS_LOG_DEBUG("Setting BSSID 4");
  //SetBssid(bestBeacon.m_bssid);
  m_bestBeaconObserved.m_channelNumber = bestBeacon.m_channelNumber;
  m_bestBeaconObserved.m_snr = bestBeacon.m_snr;
  m_bestBeaconObserved.m_bssid = bestBeacon.m_bssid;
  //m_bestBeaconObserved.m_capabilities = bestBeacon.m_capabilities;
  m_bestBeaconObserved.m_probeResp = bestBeacon.m_probeResp;
}

void
StaWifiMac::PreSendAssociationRequest (void)
{
  NS_LOG_FUNCTION (this);
  SelectAPForAssociation(); //newly added
  if (m_bestBeaconObserved.m_snr > 0)
   {
     NS_LOG_DEBUG ("one or more ProbeResponse received; selecting " << m_bestBeaconObserved.m_bssid);
     SupportedRates rates = m_bestBeaconObserved.m_probeResp.GetSupportedRates ();
     for (uint32_t i = 0; i < m_phy->GetNBssMembershipSelectors (); i++)
       {
        uint32_t selector = m_phy->GetBssMembershipSelector (i);
        if (!rates.IsSupportedRate (selector))
          {
            return;
          }
       }
     SetBssid (m_bestBeaconObserved.m_bssid);
     Time delay = MicroSeconds (m_bestBeaconObserved.m_probeResp.GetBeaconIntervalUs () * m_maxMissedBeacons);
     RestartBeaconWatchdog (delay);
     if (m_probeRequestEvent.IsRunning ())
       {
         m_probeRequestEvent.Cancel ();
       }
      SetState (WAIT_ASSOC_RESP);
      SendAssociationRequest ();
   }
   else{
      SetState (SCANNING);
      m_probeRequestEvent = Simulator::Schedule (Seconds(4),
                                                 &StaWifiMac::StartScanning, this);
      TryToEnsureAssociated();
      return;
   }
}

 void
 StaWifiMac::ProbeRequestTimeout (void)
 {
   NS_LOG_FUNCTION (this);
   SetState (WAIT_PROBE_RESP);
   SelectAPForAssociation(); //newly added
   if (m_bestBeaconObserved.m_snr > 0)
     {
       NS_LOG_DEBUG ("one or more ProbeResponse received; selecting " << m_bestBeaconObserved.m_bssid);
       SupportedRates rates = m_bestBeaconObserved.m_probeResp.GetSupportedRates ();
       for (uint32_t i = 0; i < m_phy->GetNBssMembershipSelectors (); i++)
         {
          uint32_t selector = m_phy->GetBssMembershipSelector (i);
          if (!rates.IsSupportedRate (selector))
            {
              return;
            }
         }
       SetBssid (m_bestBeaconObserved.m_bssid);
       Time delay = MicroSeconds (m_bestBeaconObserved.m_probeResp.GetBeaconIntervalUs () * m_maxMissedBeacons);
       RestartBeaconWatchdog (delay);
       if (m_probeRequestEvent.IsRunning ())
         {
           m_probeRequestEvent.Cancel ();
         }
        SetState (WAIT_ASSOC_RESP);
        SendAssociationRequest ();
     }
   else
     {
       NS_LOG_DEBUG ("no probe responses received; resend request");
       SendProbeRequest ();
     }
 }

 void
 StaWifiMac::WaitBeaconTimeout (void)
 {
   NS_LOG_FUNCTION (this);
   ////cout << "WaitBeaconTimeout" << endl;
   ////cout << "m_bestBeaconObserved.m_snr = " << m_bestBeaconObserved.m_snr << endl;
   //if (m_bestBeaconObserved.m_snr > -1000)
   if (0==0)
     {
       NS_LOG_DEBUG ("Beacon found, selecting " << m_bestBeaconObserved.m_bssid);
       cout << "Beacon found, selecting " << m_bestBeaconObserved.m_bssid << endl;
       SetBssid (m_bestBeaconObserved.m_bssid);
       SetState (WAIT_ASSOC_RESP);
       //SendAssociationRequest ();
       PreSendAssociationRequest(); //newly added
     }
   else
     {
       NS_LOG_DEBUG ("no beacons received; restart scanning");
      ////cout << "no beacons received; restart scanning" <<endl;
       //StartScanning ();
     }
 }

 void
 StaWifiMac::MissedBeacons (void)
 {
	 //cout << "watchdog : " << m_beaconWatchdogEnd.GetSeconds() <<endl;
   NS_LOG_FUNCTION (this);
   if (m_beaconWatchdogEnd > Simulator::Now ())
     {
       if (m_beaconWatchdog.IsRunning ())
         {
           m_beaconWatchdog.Cancel ();
         }
       m_beaconWatchdog = Simulator::Schedule (m_beaconWatchdogEnd - Simulator::Now (),
                                               &StaWifiMac::MissedBeacons, this);
       return;
     }
   NS_LOG_DEBUG ("beacon watchdog expired; starting to scan again");
   cout << "beacon watchdog expired; starting to scan again" << endl;
   StartScanning ();
 }

 void
 StaWifiMac::RestartBeaconWatchdog (Time delay)
 {
   NS_LOG_FUNCTION (this << delay);
   //cout << "RestartBeaconWatchdog, Delay = " << delay << endl;
   m_beaconWatchdogEnd = std::max (Simulator::Now () + delay, m_beaconWatchdogEnd);
   if (Simulator::GetDelayLeft (m_beaconWatchdog) < delay
       && m_beaconWatchdog.IsExpired ())
     {
       NS_LOG_DEBUG ("really restart watchdog.");
       m_beaconWatchdog = Simulator::Schedule (delay, &StaWifiMac::MissedBeacons, this);
     }
 }

 bool
 StaWifiMac::IsAssociated (void) const
 {
   return m_state == ASSOCIATED;
 }

 bool
 StaWifiMac::IsWaitAssocResp (void) const
 {
   return m_state == WAIT_ASSOC_RESP;
 }

 void
 StaWifiMac::Enqueue (Ptr<const Packet> packet, Mac48Address to)
 {
	 //cout << "Enqueue: IsAssociated ? " <<  IsAssociated() <<endl;
   NS_LOG_FUNCTION (this << packet << to);
   if (!IsAssociated ())
     {
       NotifyTxDrop (packet);
       TryToEnsureAssociated ();
       return;
     }
   WifiMacHeader hdr;

   // If we are not a QoS AP then we definitely want to use AC_BE to
   // transmit the packet. A TID of zero will map to AC_BE (through \c
   // QosUtilsMapTidToAc()), so we use that as our default here.
   uint8_t tid = 0;

   // For now, an AP that supports QoS does not support non-QoS
   // associations, and vice versa. In future the AP model should
   // support simultaneously associated QoS and non-QoS STAs, at which
   // point there will need to be per-association QoS state maintained
   // by the association state machine, and consulted here.
   if (m_qosSupported)
     {
       hdr.SetType (WIFI_MAC_QOSDATA);
       hdr.SetQosAckPolicy (WifiMacHeader::NORMAL_ACK);
       hdr.SetQosNoEosp ();
       hdr.SetQosNoAmsdu ();
       // Transmission of multiple frames in the same TXOP is not
       // supported for now
       hdr.SetQosTxopLimit (0);

       // Fill in the QoS control field in the MAC header
       tid = QosUtilsGetTidForPacket (packet);
       // Any value greater than 7 is invalid and likely indicates that
       // the packet had no QoS tag, so we revert to zero, which'll
       // mean that AC_BE is used.
       if (tid >= 7)
         {
           tid = 0;
         }
       hdr.SetQosTid (tid);
     }
   else
     {
       hdr.SetTypeData ();
     }
 if (m_htSupported)
     {
       hdr.SetNoOrder();
     }

   hdr.SetAddr1 (GetBssid ());
   hdr.SetAddr2 (m_low->GetAddress ());
   hdr.SetAddr3 (to);
   hdr.SetDsNotFrom ();
   hdr.SetDsTo ();

   if (m_qosSupported)
     {
       // Sanity check that the TID is valid
       NS_ASSERT (tid < 8);
       m_edca[QosUtilsMapTidToAc (tid)]->Queue (packet, hdr);
     }
   else
     {
       m_dca->Queue (packet, hdr);
     }
 }

 void
 StaWifiMac::StartScanning (void)
 {
   NS_LOG_FUNCTION (this);
   SetState (SCANNING);
   
   m_candidateChannels = m_phy->GetOperationalChannelList ();
   //m_candidateChannels = {1};

   cout << "Number of All Available Channels = " << m_candidateChannels.size () << endl;
   if (0==0)
     {
       NS_LOG_DEBUG ("No need to scan; only one channel possible");
       std::cout << "No need to scan; only one channel possible" <<std::endl;
       //m_channelNumber =m_candidateChannels.back ();
       //m_candidateChannels.clear ();
       SetState (UNASSOCIATED);
       TryToEnsureAssociated ();
       return;
     }
   // Keep track of whether we find any good beacons, so that if we do
   // not, we restart scanning
   m_bestBeaconObserved.Clear ();
   clearBssidQueue();
   uint32_t nextChannel = m_candidateChannels.back ();
   m_candidateChannels.pop_back ();
   NS_LOG_DEBUG ("Scanning channel " << nextChannel);
   //cout << "Starting by = " << nextChannel << endl;
   m_phy->SetChannelNumber(nextChannel);
   Simulator::Schedule (m_scanningTimeout, &StaWifiMac::ScanningTimeout, this);
 }

 void
 StaWifiMac::StartScanningHandover (void)
 {
   NS_LOG_FUNCTION (this);
   SetState (SCANNING_HO);
   //cout << Simulator::Now().GetSeconds() << ": New State= " << SCANNING_HO << endl;
   //cout << "StartScanningHandover"<<endl;
   
   m_candidateChannels = m_phy->GetOperationalChannelList ();
   //m_candidateChannels = {1};

   //cout << "Number of All Available Channels = " << m_candidateChannels.size () << endl;
   // Keep track of whether we find any good beacons, so that if we do
   // not, we restart scanning
   m_bestBeaconObserved.Clear ();
   //cout << "Current AP MAC = " << GetBssid() << endl;
 //  m_bestBeaconObserved.m_bssid = GetBssid();

   uint32_t nextChannel = m_candidateChannels.back ();
   m_candidateChannels.pop_back ();
   NS_LOG_DEBUG ("Scanning channel " << nextChannel);
   //cout << "Starting by = " << nextChannel << endl;
   m_phy->SetChannelNumber(nextChannel);
   Simulator::Schedule (m_scanningHOTimeout,&StaWifiMac::ScanningHandoverTimeout, this);
 }

 void
 StaWifiMac::ScanningTimeout (void)
 {
   NS_LOG_FUNCTION (this);
   //cout << Simulator::Now().GetSeconds() << ": ScanningTimeout " << endl;

   //cout << "Number of Remaining Channels to Scan = " << m_candidateChannels.size () << endl;
   if (m_candidateChannels.size () == 0)
     {
       if (m_bestBeaconObserved.m_channelNumber == 0)
         {
           NS_LOG_DEBUG ("No beacons found when scanning; restart scanning");
           //cout << "No beacons found when scanning; restart scanning" << endl;
           StartScanning ();
           return;
         }
       NS_LOG_DEBUG ("Stopping scanning; best beacon found on channel " << m_bestBeaconObserved.m_channelNumber);
       //cout << "Stopping scanning; best beacon found on channel " << m_bestBeaconObserved.m_channelNumber<<endl;
       m_phy->SetChannelNumber (m_bestBeaconObserved.m_channelNumber);
       m_channelNumber=m_bestBeaconObserved.m_channelNumber;
       SetState (UNASSOCIATED);
       TryToEnsureAssociated ();
       return;
     }
   uint32_t nextChannel = m_candidateChannels.back ();
   m_candidateChannels.pop_back ();
   NS_LOG_DEBUG ("Scanning channel " << nextChannel);
   //cout << "Scanning channel " << nextChannel << endl;
   m_phy->SetChannelNumber(nextChannel);
   Simulator::Schedule (m_scanningTimeout, &StaWifiMac::ScanningTimeout, this);
 }

 void
 StaWifiMac::ScanningHandoverTimeout (void)
  {
    NS_LOG_FUNCTION (this);
    //cout << Simulator::Now().GetSeconds() << ": ScanningHandoverTimeout " << endl;
    //cout << "Number of Remaining Channels to Scan = " << m_candidateChannels.size () << endl;
    if (m_candidateChannels.size () == 0)
      {
        	if(m_bestBeaconObserved.m_bssid!=GetBssid())
        	{
        		//StartScanning();
        		m_phy->SetChannelNumber(m_bestBeaconObserved.m_channelNumber);

        		//cout << "Better AP is found" << endl;
        		//cout << "Old AP MAC = " << GetBssid()  << ", Old Ch: " << m_channelNumber<< endl;
        		//cout << "New AP MAC = " << m_bestBeaconObserved.m_bssid << ", New Ch: " << m_bestBeaconObserved.m_channelNumber<< endl;
        		SetState(UNASSOCIATED);
        	    TryToEnsureAssociated ();
        	}
        	else
        	{
        	//cout << "Old AP MAC = " << GetBssid()  << ", Old Ch: " << m_channelNumber<< endl;
    		//cout << "New AP MAC = " << m_bestBeaconObserved.m_bssid << ", New Ch: " << m_bestBeaconObserved.m_channelNumber<< endl;

        		m_phy->SetChannelNumber(m_channelNumber);
        		//cout << "No handover" << endl;
        		SetState(ASSOCIATED);
        	}

    	    return;
      }
    uint32_t nextChannel = m_candidateChannels.back ();
    m_candidateChannels.pop_back ();
    NS_LOG_DEBUG ("Scanning channel " << nextChannel);
    //cout << "Scanning channel " << nextChannel << endl;
    m_phy->SetChannelNumber(nextChannel);
    // Send Handover Probe Packet
    //SendProbeRequest();
    Simulator::Schedule (m_scanningHOTimeout, &StaWifiMac::ScanningHandoverTimeout, this);
  }


 void
 StaWifiMac::Receive (Ptr<Packet> packet, const WifiMacHeader *hdr)
 {
//cout << "IsAssociated" << IsAssociated() << endl;//
//cout << "hdr->IsAssocResp () " << ", WAIT_ASSOC_RESP?" << (m_state == WAIT_ASSOC_RESP) << endl;
   NS_LOG_FUNCTION (this << packet << hdr);
   NS_ASSERT (!hdr->IsCtl ());
   if (hdr->GetAddr3 () == GetAddress ())
     {
       NS_LOG_LOGIC ("packet sent by us.");
       return;
     }
   else if (hdr->GetAddr1 () != GetAddress ()
            && !hdr->GetAddr1 ().IsGroup ())
     {
       NS_LOG_LOGIC ("packet is not for us");
       NotifyRxDrop (packet);
       return;
     }
   else if (hdr->IsData ())
     {
       if (!IsAssociated ())
         {
           NS_LOG_LOGIC ("Received data frame while not associated: ignore");
           NotifyRxDrop (packet);
           return;
         }
       if (!(hdr->IsFromDs () && !hdr->IsToDs ()))
         {
           NS_LOG_LOGIC ("Received data frame not from the DS: ignore");
           NotifyRxDrop (packet);
           return;
         }
       if (hdr->GetAddr2 () != GetBssid ())
         {
           NS_LOG_LOGIC ("Received data frame not from the BSS we are associated with: ignore");
           NotifyRxDrop (packet);
           return;
         }

       if (hdr->IsQosData ())
         {
           if (hdr->IsQosAmsdu ())
             {
               NS_ASSERT (hdr->GetAddr3 () == GetBssid ());
               DeaggregateAmsduAndForward (packet, hdr);
               packet = 0;
             }
           else
             {
               ForwardUp (packet, hdr->GetAddr3 (), hdr->GetAddr1 ());
             }
         }
       else
         {
           ForwardUp (packet, hdr->GetAddr3 (), hdr->GetAddr1 ());
         }
       return;
     }
   else if (hdr->IsProbeReq ()
            || hdr->IsAssocReq ())
     {
       // This is a frame aimed at an AP, so we can safely ignore it.
       NotifyRxDrop (packet);
       return;
     }
   else if (hdr->IsBeacon ())
     {
       MgtBeaconHeader beacon;
       packet->RemoveHeader (beacon);
       bool goodBeacon = false;
       SnrTag tag;
       //PhyTag tag;
       bool removed = packet->RemovePacketTag (tag);
       NS_ASSERT (removed);
       //NS_LOG_DEBUG ("SnrTag value: " << tag.GetSNR());
       //cout << "Node Mac = " << GetAddress() << endl;
       //cout << "AP Mac = " << hdr->GetAddr2 () << endl;
       //cout << "Beacon Received, SnrTag value: " << tag.GetSNR() << endl;

       if (GetSsid ().IsBroadcast ()
           || beacon.GetSsid ().IsEqual (GetSsid ()))
         {
           goodBeacon = true;
         }
       SupportedRates rates = beacon.GetSupportedRates ();
       for (uint32_t i = 0; i < m_phy->GetNBssMembershipSelectors (); i++)
         {
            uint32_t selector = m_phy->GetBssMembershipSelector (i);
            if (!rates.IsSupportedRate (selector))
              {
                 goodBeacon = false;
              }
          }
       if ((IsWaitAssocResp () || IsAssociated ()) && hdr->GetAddr3 () != GetBssid ())
         {
           // Waiting for a different response or beacon
           goodBeacon = false;
         }

       if (goodBeacon && hdr->GetAddr3 () == m_bestBeaconObserved.m_bssid)
       //if (goodBeacon)
         {
       		//cout << "STA " << GetAddress() << ", Received Beacon while " << m_state << endl;
           Time delay = MicroSeconds (beacon.GetBeaconIntervalUs () * m_maxMissedBeacons);
           RestartBeaconWatchdog (delay);
           //cout << "New BSSID = " << hdr->GetAddr3 () << ", New Watchdog " << endl;
           SetBssid (hdr->GetAddr3 ());
         }
       if (goodBeacon && m_state == UNASSOCIATED)
         {
    	   //cout << "Received Beacon while UNASSOCIATED" << endl;
           SetState (WAIT_ASSOC_RESP);
           //SendAssociationRequest ();
           PreSendAssociationRequest(); //newly added
         }
       if (goodBeacon && (m_state == SCANNING || m_state == WAIT_BEACON))
         {
           NS_LOG_DEBUG ("Beacon received while scanning");
           //cout << "Beacon received while scanning channel " << m_phy->GetChannelNumber () << endl;
            BeaconInfo beaconTemp;
            beaconTemp.m_channelNumber = m_phy->GetChannelNumber ();
            beaconTemp.m_snr = tag.Get ();
            beaconTemp.m_bssid = hdr->GetAddr3 ();
            //beaconTemp.m_capabilities = capabilities;
            NS_LOG_DEBUG("Adding to queue BSSID " << beaconTemp.m_bssid);
            if (! m_activeProbing){
              addToBssidQueue(beaconTemp);
            }
           if (m_bestBeaconObserved.m_snr < tag.Get ())
             {
               NS_LOG_DEBUG ("Beacon has highest SNR so far: " << tag.Get ());
               //cout << "Beacon has highest SNR so far: " << tag.GetSNR () << endl;
               m_bestBeaconObserved.m_channelNumber = m_phy->GetChannelNumber ();
               m_bestBeaconObserved.m_snr = tag.Get ();
               m_bestBeaconObserved.m_bssid = hdr->GetAddr3 ();
             }
         }

       else{
    	   if (goodBeacon &&m_state == SCANNING_HO && tag.Get () > m_bestBeaconObserved.m_snr)
    	   {
               m_bestBeaconObserved.m_channelNumber = m_phy->GetChannelNumber ();
               m_bestBeaconObserved.m_snr = tag.Get ();
               m_bestBeaconObserved.m_bssid = hdr->GetAddr3 ();
               //cout << "Scanning HO, Best AP MAC = " << m_bestBeaconObserved.m_bssid << endl;
    	   }
       }

       return;
     }
   else if (hdr->IsProbeResp ())
     {
	   cout << "Probe Response on Channel " << m_phy->GetChannelNumber() << endl;
	            MgtProbeResponseHeader probeResp;
	            packet->RemoveHeader (probeResp);
	            if (!probeResp.GetSsid ().IsEqual (GetSsid ()))
	              {
	                //not a probe resp for our ssid.
	                return;
	              }
	            else{
					SnrTag tag;
					//PhyTag tag;
					bool removed = packet->RemovePacketTag (tag);
					NS_ASSERT (removed);
					NS_LOG_DEBUG ("SnrTag value: " << tag.Get());
          BeaconInfo beaconTemp;
          beaconTemp.m_channelNumber = m_phy->GetChannelNumber ();
          beaconTemp.m_snr = tag.Get ();
          beaconTemp.m_bssid = hdr->GetAddr3 ();
          //beaconTemp.m_capabilities = capabilities;
          beaconTemp.m_probeResp = probeResp;
          NS_LOG_DEBUG("Adding to queue BSSID " << beaconTemp.m_bssid);
          addToBssidQueue(beaconTemp);
				       if (m_state == WAIT_PROBE_RESP)
				         {
				           if (tag.Get () > m_bestBeaconObserved.m_snr)
				             {
				               NS_LOG_DEBUG ("Save the Probe Response as a candidate");
				               m_bestBeaconObserved.m_channelNumber = m_phy->GetChannelNumber ();
				               m_bestBeaconObserved.m_snr = tag.Get ();
				               m_bestBeaconObserved.m_bssid = hdr->GetAddr3 ();
				               m_bestBeaconObserved.m_probeResp = probeResp;
				             }
				         }
	            }

       return;
     }
   else if (hdr->IsAssocResp ())
     {
       if (m_state == WAIT_ASSOC_RESP)
         {
           MgtAssocResponseHeader assocResp;
           packet->RemoveHeader (assocResp);
           if (m_assocRequestEvent.IsRunning ())
             {
               m_assocRequestEvent.Cancel ();
             }
           if (assocResp.GetStatusCode ().IsSuccess ())
           {
       	     //cout << "Associ resp. status code = " << assocResp.GetStatusCode ().IsSuccess () <<endl;
             SetState (ASSOCIATED);
             NS_LOG_DEBUG ("assoc completed");
             cout << "assoc completed" << endl;
             SupportedRates rates = assocResp.GetSupportedRates ();
             cout << "m_htSupported = " << m_htSupported << endl;
             cout << "m_vhtSupported = " << m_vhtSupported << endl;
             //cout << "Time = " << Simulator::Now().GetSeconds() << endl;
             if (m_htSupported)
               {
                 HtCapabilities htcapabilities = assocResp.GetHtCapabilities ();
                 m_stationManager->AddStationHtCapabilities (hdr->GetAddr2 (),htcapabilities);
               }
             if (m_vhtSupported)
               {
                 VhtCapabilities vhtcapabilities = assocResp.GetVhtCapabilities ();
                 m_stationManager->AddStationVhtCapabilities (hdr->GetAddr2 (), vhtcapabilities);
               }

             for (uint32_t i = 0; i < m_phy->GetNModes (); i++)
               {
                 WifiMode mode = m_phy->GetMode (i);
                 if (rates.IsSupportedRate (mode.GetDataRate (m_phy->GetChannelWidth (), false, 1)))
                   {
                     m_stationManager->AddSupportedMode (hdr->GetAddr2 (), mode);
                     if (rates.IsBasicRate (mode.GetDataRate (m_phy->GetChannelWidth (), false, 1)))
                       {
                         m_stationManager->AddBasicMode (mode);
                       }
                   }
               }
             if (m_htSupported)
               {
                 HtCapabilities htcapabilities = assocResp.GetHtCapabilities ();
                 for (uint32_t i = 0; i < m_phy->GetNMcs (); i++)
                   {
                     WifiMode mcs = m_phy->GetMcs (i);
                     if (mcs.GetModulationClass () == WIFI_MOD_CLASS_HT && htcapabilities.IsSupportedMcs (mcs.GetMcsValue ()))
                       {
                         m_stationManager->AddSupportedMcs (hdr->GetAddr2 (), mcs);
                         m_stationManager->AddSupportedMode (hdr->GetAddr2 (), mcs);
                         //here should add a control to add basic MCS when it is implemented
                       }
                   }
               }
             if (m_vhtSupported)
               {
                 VhtCapabilities vhtcapabilities = assocResp.GetVhtCapabilities ();
                 for (uint32_t i = 0; i < m_phy->GetNMcs (); i++)
                   {
                     WifiMode mcs = m_phy->GetMcs (i);
                     if (mcs.GetModulationClass () == WIFI_MOD_CLASS_VHT && vhtcapabilities.IsSupportedTxMcs (mcs.GetMcsValue ()))
                       {
                         m_stationManager->AddSupportedMcs (hdr->GetAddr2 (), mcs);
                         m_stationManager->AddSupportedMode (hdr->GetAddr2 (), mcs);
                         //here should add a control to add basic MCS when it is implemented
                       }
                   }
               }
             if (!m_linkUp.IsNull ())
               {
                 m_linkUp ();
               }
             }
           else
             {
               NS_LOG_DEBUG ("assoc refused");
               SetState (REFUSED);
               PreSendAssociationRequest(); //newly added
             }
         }
       return;
     }

   // Invoke the receive handler of our parent class to deal with any
   // other frames. Specifically, this will handle Block Ack-related
   // Management Action frames.
   RegularWifiMac::Receive (packet, hdr);
 }

 SupportedRates
 StaWifiMac::GetSupportedRates (void) const
 {
   SupportedRates rates;
   if (m_htSupported || m_vhtSupported)
     {
       for (uint32_t i = 0; i < m_phy->GetNBssMembershipSelectors (); i++)
         {
           rates.SetBasicRate (m_phy->GetBssMembershipSelector (i));
         }
     }
   for (uint32_t i = 0; i < m_phy->GetNModes (); i++)
     {
       WifiMode mode = m_phy->GetMode (i);
       rates.AddSupportedRate (mode.GetDataRate (m_phy->GetChannelWidth (), false, 1));
     }
   return rates;
 }

 HtCapabilities
 StaWifiMac::GetHtCapabilities (void) const
 {
	 std::cout << "GetHtCapabilities" << ": m_phy->GetChannelWidth () = " << m_phy->GetChannelWidth ()  << endl;
   HtCapabilities capabilities;
   capabilities.SetHtSupported (1);
   if (m_htSupported)
     {
       capabilities.SetLdpc (m_phy->GetLdpc ());
       capabilities.SetSupportedChannelWidth (m_phy->GetChannelWidth () == 40);
       capabilities.SetShortGuardInterval20 (m_phy->GetShortGuardInterval ());
       capabilities.SetShortGuardInterval40 (m_phy->GetChannelWidth () == 40 && m_phy->GetShortGuardInterval ());
       capabilities.SetGreenfield (m_phy->GetGreenfield ());
       capabilities.SetMaxAmsduLength (1); //hardcoded for now (TBD)
       capabilities.SetLSigProtectionSupport (!m_phy->GetGreenfield ());
       capabilities.SetMaxAmpduLength (3); //hardcoded for now (TBD)
       uint64_t maxSupportedRate = 0; //in bit/s
       for (uint8_t i = 0; i < m_phy->GetNMcs (); i++)
         {
           WifiMode mcs = m_phy->GetMcs (i);
           capabilities.SetRxMcsBitmask (mcs.GetMcsValue ());
           if (mcs.GetDataRate (m_phy->GetChannelWidth (), m_phy->GetGuardInterval ().GetNanoSeconds(), 1) > maxSupportedRate)
             {
               maxSupportedRate = mcs.GetDataRate (m_phy->GetChannelWidth (), m_phy->GetGuardInterval ().GetNanoSeconds(), 1);
             }
         }
       capabilities.SetRxHighestSupportedDataRate (maxSupportedRate / 1e6); //in Mbit/s
       capabilities.SetTxMcsSetDefined (m_phy->GetNMcs () > 0);
       capabilities.SetTxMaxNSpatialStreams (m_phy->GetNumberOfAntennas ());
     }
   return capabilities;
 }

 VhtCapabilities
 StaWifiMac::GetVhtCapabilities (void) const
 {
   std::cout << "GetVhtCapabilities" << ": m_phy->GetChannelWidth () = " << m_phy->GetChannelWidth ()  << endl;
   VhtCapabilities capabilities;
   capabilities.SetVhtSupported (1);
   if (m_vhtSupported)
     {
       if (m_phy->GetChannelWidth () == 160)
         {
           capabilities.SetSupportedChannelWidthSet (1);
         }
       else
         {
           capabilities.SetSupportedChannelWidthSet (0);
         }
       capabilities.SetMaxMpduLength (2); //hardcoded for now (TBD)
       capabilities.SetRxLdpc (m_phy->GetLdpc ());
       capabilities.SetShortGuardIntervalFor80Mhz ((m_phy->GetChannelWidth () == 80) && m_phy->GetShortGuardInterval ());
       capabilities.SetShortGuardIntervalFor160Mhz ((m_phy->GetChannelWidth () == 160) && m_phy->GetShortGuardInterval ());
       capabilities.SetMaxAmpduLengthExponent (7); //hardcoded for now (TBD)
       uint8_t maxMcs = 0;
       for (uint8_t i = 0; i < m_phy->GetNMcs (); i++)
         {
           WifiMode mcs = m_phy->GetMcs (i);
           if (mcs.GetMcsValue () > maxMcs)
             {
               maxMcs = mcs.GetMcsValue ();
             }
         }
       capabilities.SetRxMcsMap (maxMcs, 1); //Only 1 SS is currently supported
       capabilities.SetTxMcsMap (maxMcs, 1); //Only 1 SS is currently supported
     }
   return capabilities;
 }

 void
 StaWifiMac::SetState (MacState value)
 {
   if (value == ASSOCIATED
       && m_state != ASSOCIATED)
     {
       m_assocLogger (GetBssid ());
       NS_LOG_DEBUG ("Associating to " << GetBssid ());
       //cout << "Associating to " << GetBssid () <<endl;
       //cout << " GetAddress ()= " <<  GetAddress () << endl;
     }
   else if (value != ASSOCIATED
            && m_state == ASSOCIATED)
     {
       m_deAssocLogger (GetBssid ());
     }
   m_state = value;
 }

 void
StaWifiMac::addToBssidQueue(BeaconInfo beaconInfo){
  //std::cout<<"Adding to queue"<<std::endl;
  if(queuedBssids.find(beaconInfo.m_bssid) == queuedBssids.end()){
    bssidQueue.push(beaconInfo);
    queuedBssids[beaconInfo.m_bssid] = true;
  }
}

 BeaconInfo
StaWifiMac::popBssidQueue(){
  BeaconInfo ret = bssidQueue.top();
  queuedBssids.erase(ret.m_bssid);
  bssidQueue.pop();
  backupBssids.push_back(ret);
  return ret;
}

 void
StaWifiMac::clearBssidQueue(){
  NS_LOG_DEBUG("CLEARING");
  bssidQueue = std::priority_queue <BeaconInfo, std::vector<BeaconInfo>, CompareBeaconInfo>();
  backupBssids.clear();
  queuedBssids.clear();
}

 } //namespace ns3
