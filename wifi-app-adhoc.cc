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

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <algorithm>

//Message Type definitions
#define DATA 1
#define RREQ 2
#define RACK 3

NS_LOG_COMPONENT_DEFINE ("WifiSimpleAdhoc");

using namespace ns3;

//used to distinguish between control plane and data plane
std::map<uint64_t,int> messageTypeMap;
//DATA PLANE
std::map<uint64_t,std::vector<int64_t> > sentTimestampMap;
std::map<uint64_t,int> sourceMap;
//CONTROL PLANCE
//   RREQ
std::map<uint64_t,std::vector<int> > pathMap;
std::map<uint64_t,int > destinationMap;
//   RACK
std::map<uint64_t,std::vector<int> > partialPathMap;
std::map<uint64_t,std::vector<int> > completePathMap;

std::ofstream fileLog;
std::ofstream debugLog;
Time DEFAULT_SEND_P = MilliSeconds(3000);
int NR_NODES = 20; //global used by all nodes
Time DATA_GENERATION_RATE = Seconds(1); //seconds
Time BACKOFF_PERIOD = MilliSeconds(20);
Time FAST = MilliSeconds(300);
int PATH_DISCOVERY_BACKOFF = 20;


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

bool NoLoopsInPath(std::vector<int> path) {
//	std::ostringstream ss;
//	ss << "Checking for loops ";
//	AddVectorToStream(ss,path);
//	NS_LOG_UNCOND(ss.str().c_str());
	std::map<int,bool> seen;
	for (std::vector<int>::iterator it = path.begin(); it != path.end(); ++it) {
		if (seen.find(*it) != seen.end() ) return false;
		else seen.insert(std::pair<int,bool> (*it,true));
	}
	return true;
}


class WifiAppAdhoc : public Application {
private:
	Ptr<Socket> m_recvSocket;
	Ptr<Socket> m_sendSocket;
	Ptr<Node> m_node;
	EventId m_nextSend, m_nextTimestamp;
	std::vector<int64_t> m_lastTimestampSynched;
	int m_nodeId;

public:
	WifiAppAdhoc(Ptr<Node> n, int nodeId, int nrNodes) {
		m_node = n;
		m_nodeId = nodeId;
		m_lastTimestampSynched = std::vector<int64_t>(nrNodes,0);

		Ptr<MobilityModel> mobility = m_node->GetObject<MobilityModel>();
		Vector pos = mobility->GetPosition();
		std::ostringstream ss;
		ss << "Node: "<< m_nodeId<< " x=" << pos.x << " y=" << pos.y;
		NS_LOG_UNCOND(ss.str().c_str());

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

		//Schedule first send
		if (m_nodeId == 1) {
			Simulator::Schedule(MilliSeconds(300),&WifiAppAdhoc::RequestPath,this);
		}

		m_nextSend = Simulator::Schedule (DEFAULT_SEND_P + MilliSeconds(rand() % 30 + 1), &WifiAppAdhoc::GenerateTraffic,this);

        m_nextTimestamp = Simulator::Schedule(DATA_GENERATION_RATE + MilliSeconds(rand() %50 + 1),&WifiAppAdhoc::UpdateLocalTimestamp,this);
		std::ostringstream ss;
		ss << "Started @" << Simulator::Now().GetMilliSeconds();
		NS_LOG_UNCOND(ss.str().c_str());
	}

	void RequestPath() {
		Ptr<Packet> packet = Create<Packet>(0);
		uint64_t packetUid = packet->GetUid();
		std::pair<uint64_t,int> type (packetUid, RREQ);
		messageTypeMap.insert(type);

		std::vector<int> pathVector;
		pathVector.push_back(m_nodeId);
		std::pair<uint64_t,std::vector<int> > path (packet->GetUid(),pathVector);
		pathMap.insert(path);

		std::pair<uint64_t,int> destination (packet->GetUid(),4);
		destinationMap.insert(destination);

		m_sendSocket->Send(packet);
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

    	m_nextSend = Simulator::Schedule (DEFAULT_SEND_P + MilliSeconds(rand() % 50 + 1), &WifiAppAdhoc::GenerateTraffic,this);


	      //NS_LOG_UNCOND (Simulator::GetDelayLeft(m_nextSend).GetMilliSeconds());
	}
	


	void ReceivePacket(Ptr<Socket> socket) {
		Ptr<Packet> packet = socket->Recv();
		uint64_t packetUid = packet->GetUid();

	    int messageType = messageTypeMap.find(packetUid)->second;
		switch(messageType) {
		case DATA:
			ReceiveDataPacket(packetUid);
			break;
		case RREQ:
			ReceiveRREQ(packetUid);
			break;
		case RACK:
			RelayRACK(packetUid);
			break;
		// OTHER type handlers here
		}
	}

	void ReceiveRREQ(uint64_t packetUid) {

		//Add myself to path
		std::vector<int> path = pathMap.find(packetUid)->second;
		path.push_back(m_nodeId);

		//Find out if I am destination
		int destination = destinationMap.find(packetUid)->second;
		if (m_nodeId == destination) {
//			std::ostringstream ss;
//			ss << "Node: " << m_nodeId << "Found path: ";
//			AddVectorToStream(ss,path);
//		    NS_LOG_UNCOND(ss.str().c_str());
//
//		    debugLog << "Node: " << m_nodeId << "Found path: ";
//		    AddVectorToStream(debugLog,path); debugLog << "\n";
//		    debugLog.flush();

		    RelayRACK2(path,path);
		} else if (NoLoopsInPath(path)) {
			Simulator::Schedule(MilliSeconds(rand()%PATH_DISCOVERY_BACKOFF+1),&WifiAppAdhoc::RelayRREQ, this, path,destination);
//			std::ostringstream ss;
//		    ss << "Node: " << m_nodeId << " ReceivedRReq";
//			NS_LOG_UNCOND(ss.str().c_str());
//
//			debugLog << "Node: " << m_nodeId << " ReceivedRReq \n";
//			debugLog.flush();

		}
	}

	void RelayRREQ(std::vector<int> path, int destination) {
		Ptr<Packet> packet = Create<Packet>(10);
		std::pair<uint64_t,int> type (packet->GetUid(), RREQ);
		messageTypeMap.insert(type);
		std::pair<uint64_t,std::vector<int> > p (packet->GetUid(),path);
		pathMap.insert(p);
		std::pair<uint64_t,int> destinationPair (packet->GetUid(),destination);
		destinationMap.insert(destinationPair);
		m_sendSocket->Send(packet);
	}

	void RelayRACK(uint64_t packetUid) {

		std::vector<int> completePath = completePathMap.find(packetUid)->second;
		std::vector<int> partialPath = partialPathMap.find(packetUid)->second;

		std::ostringstream ss;
//	    ss << "Node:  " << m_nodeId << " Undirected RACK";
//	    AddVectorToStream(ss,completePath);
//	    AddVectorToStream(ss,partialPath);
//	    NS_LOG_UNCOND(ss.str().c_str());

//	    debugLog << "Node " << m_nodeId << "Undirected RACK";
//	    AddVectorToStream(debugLog,completePath); debugLog << "\n";
//	    AddVectorToStream(debugLog,partialPath); debugLog << "\n";
//        debugLog.flush();

        int ownId = partialPath.back();
        partialPath.pop_back();
        if (ownId == m_nodeId && partialPath.size() == 0) {
        	std::ostringstream ss;
        	ss << "Node: " << m_nodeId << " Found path: ";
        	AddVectorToStream(ss,completePath);
        	NS_LOG_UNCOND(ss.str().c_str());

        	debugLog << "Node: " << m_nodeId << " Found path: ";
        	AddVectorToStream(debugLog,completePath); debugLog << "\n";
        	debugLog.flush();
        } else {
            Simulator::Schedule(MilliSeconds(rand()%PATH_DISCOVERY_BACKOFF + 1),&WifiAppAdhoc::RelayRACK2,this, completePathMap.find(packetUid)->second, partialPathMap.find(packetUid)->second);
        }
		//RelayRACK2(completePathMap.find(packetUid)->second, partialPathMap.find(packetUid)->second);
	}

	void RelayRACK2(std::vector<int> completePath, std::vector<int> partialPath) {
		//pull off own id

		int ownId = partialPath.back();
		partialPath.pop_back();
        if (ownId == m_nodeId) {
//        std::ostringstream ss;
//        ss << "Node: " << m_nodeId << "ReceivedRACK \nComplete: ";
//        AddVectorToStream(ss,completePath);
//        ss << "Partial";
//        AddVectorToStream(ss,partialPath);
//        NS_LOG_UNCOND(ss.str().c_str());
//
//        debugLog << "Node: " << m_nodeId << "ReceivedRACK \nComplete: ";
//        AddVectorToStream(debugLog,completePath); debugLog << "\n";
//        debugLog << "Partial";
//        AddVectorToStream(debugLog,partialPath); debugLog << "\n";
//        debugLog.flush();

		//Not final destination of RACK
		NS_ASSERT(partialPath.size() > 0);

		Ptr<Packet> packet = Create<Packet>(0);
		std::pair<uint64_t,int> type (packet->GetUid(), RACK);
		messageTypeMap.insert(type);

		std::pair<uint64_t,std::vector<int> > partialPathEntry (packet->GetUid(),partialPath);
		partialPathMap.insert(partialPathEntry);

		std::pair<uint64_t,std::vector<int> > completePathEntry (packet->GetUid(),completePath);
		completePathMap.insert(completePathEntry);

		m_sendSocket->Send(packet);
//        ss.str(""); ss.clear();
//        ss << "Node: " << m_nodeId << "\n Partial";
//        AddVectorToStream(ss,partialPath);
//        ss << "\n Complete";
//        AddVectorToStream(ss,completePath);
//        NS_LOG_UNCOND(ss.str().c_str());



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




//void ReceivePacket (Ptr<Socket> socket)
//{
//	Ptr<Packet> packet = socket->Recv();
//	uint64_t packetUid = packet->GetUid();
//
//	int64_t timestamp = sentTimestampMap.find(packetUid)->second;
//
//
//
//	std::ostringstream ss;
//	ss << "Received Packet UID:" << packetUid << " Timestamp: " << timestamp;
//	NS_LOG_UNCOND (ss.str().c_str());
//
//	fileLog << Simulator::Now().GetMilliSeconds() << ", " << packetUid << "\n";
//}

/*
static void GenerateTraffic (Ptr<Socket> socket, uint32_t pktSize, 
                             uint32_t pktCount, Time pktInterval )
{
  if (pktCount > 0)
    {
	  Ptr<Packet> packet = Create<Packet>(pktSize);
	  uint64_t packetUID = packet->GetUid();

	  std::pair<uint64_t,int64_t> entry(packetUID,Simulator::Now().GetMilliSeconds());
	  sentTimestamps.insert(entry);


      socket->Send (packet);
      EventId nextEvent =  Simulator::Schedule (pktInterval, &GenerateTraffic,
                                                socket, pktSize,pktCount-1, pktInterval);

      NS_LOG_UNCOND (Simulator::GetDelayLeft(nextEvent).GetMilliSeconds());
    }
  else
    {
      socket->Close ();
    }
}
*/


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
                                 "GridWidth", UintegerValue (3),
                                 "LayoutType", StringValue ("RowFirst"));

//  mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
//                             "Bounds", RectangleValue (Rectangle (0, 1000, 0, 1000)));

//  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
//  positionAlloc->Add (Vector (70, 50, 0.0));
//  positionAlloc->Add (Vector (50, 50, 0.0));
//  positionAlloc->Add (Vector (400, 10, 0.0));
//
//  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (c);

  InternetStackHelper internet;
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
	  app->SetStartTime(MilliSeconds(i));
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

