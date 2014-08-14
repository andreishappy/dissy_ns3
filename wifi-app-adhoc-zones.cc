/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2009 The Boeing Company
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
 */

// 
// This script configures two nodes on an 802.11b physical layer, with
// 802.11b NICs in adhoc mode, and by default, sends one packet of 1000 
// (application) bytes to the other node.  The physical layer is configured
// to receive at a fixed RSS (regardless of the distance and transmit
// power); therefore, changing position of the nodes has no effect. 
//
// There are a number of command-line options available to control
// the default behavior.  The list of available command-line options
// can be listed with the following command:
// ./waf --run "wifi-simple-adhoc --help"
//
// For instance, for this configuration, the physical layer will
// stop successfully receiving packets when rss drops below -97 dBm.
// To see this effect, try running:
//
// ./waf --run "wifi-simple-adhoc --rss=-97 --numPackets=20"
// ./waf --run "wifi-simple-adhoc --rss=-98 --numPackets=20"
// ./waf --run "wifi-simple-adhoc --rss=-99 --numPackets=20"
//
// Note that all ns-3 attributes (not just the ones exposed in the below
// script) can be changed at command line; see the documentation.
//
// This script can also be helpful to put the Wifi layer into verbose
// logging mode; this command will turn on all wifi logging:
// 
// ./waf --run "wifi-simple-adhoc --verbose=1"
//
// When you are done, you will notice two pcap trace files in your directory.
// If you have tcpdump installed, you can try this:
//
// tcpdump -r wifi-simple-adhoc-0-0.pcap -nn -tt
//

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/config-store-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include <ns3/event-id.h>
#include "ns3/olsr-helper.h"
#include "ns3/olsr-routing-protocol.h"
#include "ns3/config.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <algorithm>

#define DATA 1

NS_LOG_COMPONENT_DEFINE ("WifiSimpleAdhoc");

using namespace ns3;
using namespace ns3::Config;
using namespace ns3::olsr;
using namespace std;


//used to distinguish between control plane and data plane
std::map<uint64_t,int> messageTypeMap;
//DATA PLANE
std::map<uint64_t,std::vector<int64_t> > sentTimestampMap;
std::map<uint64_t,int> sourceMap;

std::map<int,Ipv4Address> nodeIdToAddress;
std::map<Ipv4Address,int> AddressToNodeId;


std::ofstream fileLog;
std::ofstream debugLog;
Time DEFAULT_SEND_P = MilliSeconds(3000);
int NR_NODES = 50; //global used by all nodes
Time DATA_GENERATION_RATE = Seconds(1); //seconds
int BACKOFF_PERIOD = 20;




template<typename T>
void AddVectorToStream(std::ostream &ss, T toAdd) {
	ss << "[";
	for (typename T::iterator it = toAdd.begin(); it != toAdd.end(); ++it) {
				ss << *it;
				if (it + 1 < toAdd.end()) {
					ss << ", ";
				}
			}
	ss << "]";
}

Time GetRandomPerturbation(Time toPerturb) {
	return toPerturb + MilliSeconds(rand()%BACKOFF_PERIOD);
}

class WifiAppAdhoc : public Application {
private:
	Ptr<Socket> m_recvSocket;
	Ptr<Socket> m_sendSocket;
	Ptr<Node> m_node;
	EventId m_nextSend, m_nextTimestamp;
	std::vector<int64_t> m_lastTimestampSynched;
	int m_nodeId;
	Time m_synch_frequency;

public:
	WifiAppAdhoc(Ptr<Node> n, int nodeId, int nrNodes) {
		m_node = n;
		m_nodeId = nodeId;
		m_lastTimestampSynched = std::vector<int64_t>(nrNodes,0);
		m_synch_frequency = GetRandomPerturbation(DEFAULT_SEND_P);

		Ptr<MobilityModel> mobility = m_node->GetObject<MobilityModel>();
		Vector pos = mobility->GetPosition();
		std::ostringstream ss;
		ss << "Node: "<< m_nodeId<< " x=" << pos.x << " y=" << pos.y;
		NS_LOG_UNCOND(ss.str().c_str());

	}

	void PrintNeighbours() {
		Ptr<RoutingProtocol> 					routing = m_node->GetObject<RoutingProtocol>();
	    vector<RoutingTableEntry> 				entry = routing->GetRoutingTableEntries();
	    vector<RoutingTableEntry>::iterator 	it;

	    ostringstream ss;
	    ss << "Node: " << m_nodeId << " Neighbours: \n";

	    for (it = entry.begin(); it!=entry.end(); it++) {
	    	ss << AddressToNodeId.find(it->destAddr)->second << " -> " << it->distance <<"\n";
	    }

	    NS_LOG_UNCOND(ss.str().c_str());
	    Simulator::Schedule(Seconds(1),&WifiAppAdhoc::PrintNeighbours,this);
	}

	virtual void StartApplication(void) {
		TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");

		//Set up receiving socket
	    m_recvSocket = Socket::CreateSocket (m_node, tid);
	    InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
		m_recvSocket->Bind (local);
		m_recvSocket->SetRecvCallback (MakeCallback (&WifiAppAdhoc::ReceivePacket,this));

		//Set up sending socket
		m_sendSocket = Socket::CreateSocket (m_node, tid);
		InetSocketAddress remote = InetSocketAddress (Ipv4Address ("255.255.255.255"), 80);
		m_sendSocket->SetAllowBroadcast (true);
		m_sendSocket->Connect(remote);

		//m_nextSend = Simulator::Schedule (GetRandomPerturbation(m_synch_frequency), &WifiAppAdhoc::GenerateTraffic,this);
        m_nextTimestamp = Simulator::Schedule(DATA_GENERATION_RATE,&WifiAppAdhoc::UpdateLocalTimestamp,this);

        Simulator::Schedule(Seconds(1),&WifiAppAdhoc::PrintNeighbours,this);

		std::ostringstream ss;
		ss << "Started @" << Simulator::Now().GetMilliSeconds();

		Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4>();
		Ipv4InterfaceAddress iaddr = ipv4->GetAddress (1,0);
		Ipv4Address addri = iaddr.GetLocal ();

		std::pair<int, Ipv4Address> nodeToAddress (m_nodeId,addri);
		std::pair<Ipv4Address,int> addressToNode (addri,m_nodeId);
		nodeIdToAddress.insert(nodeToAddress);
		AddressToNodeId.insert(addressToNode);

		ss << "With Address: " << addri;

		NS_LOG_UNCOND(ss.str().c_str());


	}

	void UpdateLocalTimestamp() {
		m_lastTimestampSynched[m_nodeId] = Simulator::Now().GetMilliSeconds();
		m_nextTimestamp = Simulator::Schedule(DATA_GENERATION_RATE,&WifiAppAdhoc::UpdateLocalTimestamp,this);
	}

	void GenerateTraffic ()
	{
		NS_LOG_UNCOND ("Entered Generate Traffic");

		Ptr<Packet> packet = Create<Packet>(10);
		uint64_t packetUID = packet->GetUid();

		std::pair<uint64_t,int> type (packetUID, DATA);
		messageTypeMap.insert(type);
		std::pair<uint64_t,std::vector<int64_t> > entry(packetUID,m_lastTimestampSynched);
		sentTimestampMap.insert(entry);
		std::pair<uint64_t,int> sourceEntry(packetUID,m_nodeId);
	    sourceMap.insert(sourceEntry);

		if (m_sendSocket) {
	      m_sendSocket->Send(packet);
	      std::ostringstream ss;
	      ss << "Node : " << m_nodeId << " packet sent UID: " << packet->GetUid() << " @" << (Simulator::Now().GetMilliSeconds()) << "\n";
	      AddVectorToStream(ss,m_lastTimestampSynched);
	      NS_LOG_UNCOND (ss.str().c_str());
		}

    	m_nextSend = Simulator::Schedule (GetRandomPerturbation(m_synch_frequency), &WifiAppAdhoc::GenerateTraffic,this);
	}
	


	void ReceivePacket(Ptr<Socket> socket) {
		Ptr<Packet> packet = socket->Recv();
		uint64_t packetUid = packet->GetUid();

	    int messageType = messageTypeMap.find(packetUid)->second;
		switch(messageType) {
		case DATA:
			ReceiveDataPacket(packetUid);
			break;
		// OTHER type handlers here
		}
	}

	void ReceiveDataPacket(uint64_t packetUid) {
		std::vector<int64_t> timestampVectorReceived = sentTimestampMap.find(packetUid)->second;
		int sourceId = sourceMap.find(packetUid)->second;

		std::ostringstream ss;
		ss << "Node: " << m_nodeId << " OO Received Packet From: " << sourceId << "Vector: ";
		AddVectorToStream(ss,timestampVectorReceived);

		NS_LOG_UNCOND (ss.str().c_str());

		ss.str(""); ss.clear();
		ss<< "Time: " << Simulator::Now().GetMilliSeconds() << " Node: " << m_nodeId << "\n" << "BEFORE: ";
		AddVectorToStream(ss,m_lastTimestampSynched);
		NS_LOG_UNCOND(ss.str().c_str());

		for (int i = 0; i < NR_NODES; ++i) {
		if (m_lastTimestampSynched[i] < timestampVectorReceived[i]) {
			m_lastTimestampSynched[i] = timestampVectorReceived[i];
//			ss.str("");
//			ss.clear();
//			ss << "Node: " << m_nodeId << " New synch: " << timestampVectorReceived[i];
//			NS_LOG_UNCOND(ss.str().c_str());
		}
		}

		ss.str("");
		ss.clear();
		ss << "AFTER:  ";
		AddVectorToStream(ss,m_lastTimestampSynched);
		NS_LOG_UNCOND(ss.str().c_str());


		fileLog << Simulator::Now().GetMilliSeconds() << ", " << packetUid << "\n";
	}
	
};

int main (int argc, char *argv[])
{

  fileLog.open("example.txt");
  fileLog << "time, packetId\n";

  debugLog.open("debug.txt");

  std::string phyMode ("DsssRate1Mbps");
  double rss = -80;  // -dBm
  uint32_t packetSize = 10; // bytes
  uint32_t numPackets = 20;
  double interval = 1.0; // seconds
  bool verbose = false;

  CommandLine cmd;

  cmd.AddValue ("phyMode", "Wifi Phy mode", phyMode);
  cmd.AddValue ("rss", "received signal strength", rss);
  cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
  cmd.AddValue ("numPackets", "number of packets generated", numPackets);
  cmd.AddValue ("interval", "interval (seconds) between packets", interval);
  cmd.AddValue ("verbose", "turn on all WifiNetDevice log components", verbose);

  cmd.Parse (argc, argv);
  // Convert to time object

  // disable fragmentation for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
  // turn off RTS/CTS for frames below 2200 bytes
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));
  // Fix non-unicast data rate to be the same as that of unicast
  Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode", 
                      StringValue (phyMode));

  NodeContainer c;
  c.Create (NR_NODES);

  // The below set of helpers will help us to put together the wifi NICs we want
  WifiHelper wifi;
  if (verbose)
    {
      wifi.EnableLogComponents ();  // Turn on all Wifi logging
    }
  wifi.SetStandard (WIFI_PHY_STANDARD_80211b);

  YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
  // This is one parameter that matters when using FixedRssLossModel
  // set it to zero; otherwise, gain will be added
  wifiPhy.Set ("RxGain", DoubleValue (0) ); 
  // ns-3 supports RadioTap and Prism tracing extensions for 802.11b
  wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO); 

  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  // The below FixedRssLossModel will cause the rss to be fixed regardless
  // of the distance between the two stations, and the transmit power
  //wifiChannel.AddPropagationLoss ("ns3::FixedRssLossModel","Rss",DoubleValue (rss));
  wifiChannel.AddPropagationLoss ("ns3::RangePropagationLossModel",
                                   "MaxRange", DoubleValue (140.0));
  wifiPhy.SetChannel (wifiChannel.Create ());

  // Add a non-QoS upper mac, and disable rate control
//  NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
  QosWifiMacHelper wifiMac = QosWifiMacHelper::Default ();
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode",StringValue (phyMode),
                                "ControlMode",StringValue (phyMode));
  // Set it to adhoc mode
  wifiMac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, c);

  // Note that with FixedRssLossModel, the positions below are not 
  // used for received signal strength. 
  MobilityHelper mobility;

  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                                 "MinX", DoubleValue (0.0),
                                 "MinY", DoubleValue (0.0),
                                 "DeltaX", DoubleValue (100),
                                 "DeltaY", DoubleValue (100),
                                 "GridWidth", UintegerValue (7),
                                 "LayoutType", StringValue ("RowFirst"));

//  mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
//                             "Bounds", RectangleValue (Rectangle (0, 900, 0, 900)));


//  mobility.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
//                                "Bounds", RectangleValue (Rectangle (0, 800, 0, 800)),
//                                "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=1.5]"),
//                                "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=20]"));

  mobility.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
                                  "Bounds", RectangleValue (Rectangle (0, 800, 0, 800)),
                                  "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=1.5]"),
                                  "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=20]"));
//  mobility.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
//  									   "Bounds", RectangleValue (Rectangle (0, 900, 0, 900)),
//  									   "Speed", RandomVariableValue (ConstantVariable (1.5)),
//  									   "Pause", RandomVariableValue (ConstantVariable (10)));

//  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
//  positionAlloc->Add (Vector (70, 50, 0.0));
//  positionAlloc->Add (Vector (50, 50, 0.0));
//  positionAlloc->Add (Vector (400, 10, 0.0));
//
//  mobility.SetPositionAllocator (positionAlloc);

// Constant position mobility
//  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (c);

  OlsrHelper olsr;
  Ipv4ListRoutingHelper list;
  list.Add(olsr,0);
  InternetStackHelper internet;
  internet.SetRoutingHelper(list);
  internet.Install (c);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);

//  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
//  Ptr<Socket> recvSink = Socket::CreateSocket (c.Get (0), tid);
//  InetSocketAddress local = InetSocketAddress (Ipv4Address::GetAny (), 80);
//  recvSink->Bind (local);
//  recvSink->SetRecvCallback (MakeCallback (&ReceivePacket));

/*
  Ptr<Socket> recvSink2 = Socket::CreateSocket (c.Get (2), tid);
  recvSink2->Bind (local);
  recvSink2->SetRecvCallback (MakeCallback (&ReceivePacket));
*/

  for (int i; i<NR_NODES; ++i) {
	  Ptr<WifiAppAdhoc> app = CreateObject<WifiAppAdhoc>(c.Get(i),i,NR_NODES);
	  c.Get(i)->AddApplication(app);
	  app->SetStartTime(Time(0));



  }
	  //app->SetStopTime(Time(10000.0));
//
//  Ptr<WifiAppAdhoc> app2 = CreateObject<WifiAppAdhoc>(c.Get(1),1,NR_NODES);
//  c.Get(1)->AddApplication(app2);
//  app2->SetStartTime(MilliSeconds(1));
//  //app2->SetStopTime(Time(10000.0));

  // Tracing
  wifiPhy.EnablePcap ("wifi-simple-adhoc", devices);

  // Output what we are doing
  NS_LOG_UNCOND ("Testing " << numPackets  << " packets sent with receiver rss " << rss );

  Simulator::Stop(Seconds(600));
  Simulator::Run ();
  Simulator::Destroy ();

  return 0;
}

