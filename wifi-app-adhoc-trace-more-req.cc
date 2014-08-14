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
#include <math.h>


#define DATA 1

NS_LOG_COMPONENT_DEFINE ("WifiSimpleAdhoc");

using namespace ns3;
using namespace ns3::Config;
using namespace ns3::olsr;
using namespace std;

ostringstream ss;


//used to distinguish between control plane and data plane
std::map<uint64_t,int> messageTypeMap;
//DATA PLANE
std::map<uint64_t,std::vector<int64_t> > sentTimestampMap;
std::map<uint64_t,int> sourceMap;

std::map<int,Ipv4Address> nodeIdToAddress;
std::map<Ipv4Address,int> AddressToNodeId;



std::ofstream packetLog;
std::ofstream communicationLog;
std::ofstream stateLog;

int DEFAULT_SEND_P = 90000; //in milliseconds
Time DEFAULT_FREQUENCY = MilliSeconds(DEFAULT_SEND_P);
int NR_NODES = 50; //global used by all nodes
Time DATA_GENERATION_RATE = Seconds(1); //seconds
int BACKOFF_PERIOD = 100;

vector<int> producers;
vector<int> consumers;
vector<Time> requirements;
//Zone algorithm
vector<int> shortestPathLengths;
int i;

float ZONE_THRESHOLD = 1;
float SPEED_UP = 1;

uint64_t packetUid;




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
	return toPerturb - MilliSeconds(BACKOFF_PERIOD/2) + MilliSeconds(rand()%BACKOFF_PERIOD + 1);
}

void Log(ostringstream &ss) {
	NS_LOG_UNCOND(ss.str().c_str());
	ss.str("");
	ss.clear();
}


class WifiAppAdhoc : public Application {
private:
	Ptr<Socket> m_recvSocket;
	Ptr<Socket> m_sendSocket;
	Ptr<Node> m_node;
	EventId m_nextSend, m_nextTimestamp;
	std::vector<int64_t> m_lastTimestampSynched;
	int m_nodeId;
	Time m_synchFrequency;
	float m_zoneValue;
	int m_consumerToProducerDistance;
	int m_stretch;
	int m_shortestPathLength;
	Time m_lastSend;

public:
	WifiAppAdhoc(Ptr<Node> n, int nodeId, int nrNodes) {
		m_node = n;
		m_nodeId = nodeId;
		m_lastTimestampSynched = std::vector<int64_t>(nrNodes,0);
		m_synchFrequency = DEFAULT_FREQUENCY;
		m_zoneValue = 0;
		m_stretch = 1000;
		m_consumerToProducerDistance = 0;
		m_shortestPathLength = 0;
		Ptr<MobilityModel> mobility = m_node->GetObject<MobilityModel>();
		m_lastSend = Time(0);

		Vector pos = mobility->GetPosition();
		ss << "Node: "<< m_nodeId<< " x=" << pos.x << " y=" << pos.y;
		Log(ss);

	}


	bool IsConsumer() {
		for (vector<int>::iterator it = consumers.begin(); it != consumers.end(); it++) {
			if (m_nodeId == *it) {
				ss << "Node: " << m_nodeId << " is consumer";
				Log(ss);
				return true;
			}
		}
		return false;
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


			m_nextSend = Simulator::Schedule (GetRandomPerturbation(m_synchFrequency), &WifiAppAdhoc::SendData,this);

	        m_nextTimestamp = Simulator::Schedule(DATA_GENERATION_RATE,&WifiAppAdhoc::UpdateLocalTimestamp,this);

	        if (IsConsumer()) {
	        	Simulator::Schedule(Seconds(1),&WifiAppAdhoc::UpdateShortestPath,this);
	        }

	        Simulator::Schedule(Seconds(1) + MilliSeconds(1),&WifiAppAdhoc::Reconfigure,this);



			Ptr<Ipv4> ipv4 = GetNode()->GetObject<Ipv4>();
			Ipv4InterfaceAddress iaddr = ipv4->GetAddress (1,0);
			Ipv4Address addri = iaddr.GetLocal ();

			std::pair<int, Ipv4Address> nodeToAddress (m_nodeId,addri);
			std::pair<Ipv4Address,int> addressToNode (addri,m_nodeId);
			nodeIdToAddress.insert(nodeToAddress);
			AddressToNodeId.insert(addressToNode);

			ss << "Started @" << Simulator::Now().GetMilliSeconds();
			ss << "With Address: " << addri;

			Log(ss);


	}


	void UpdateShortestPath() {
		vector<int> producersICareAbout;

		for (i = 0; i < (int)consumers.size(); i++) {
			if (consumers[i] == m_nodeId) {
				shortestPathLengths[i] = FindDistanceToNode(producers[i]);
				ss << "Time :" << Simulator::Now().GetMilliSeconds() << "SP " << m_nodeId << " -> " << producers[i] << " = " << shortestPathLengths[i] << "\n";
			    Log(ss);
			}
		}




	    Simulator::Schedule(Seconds(1),&WifiAppAdhoc::UpdateShortestPath,this);
	}

	int FindDistanceToNode(int nodeId) {
		Ptr<RoutingProtocol> 					routing = m_node->GetObject<RoutingProtocol>();
		vector<RoutingTableEntry> 				entry = routing->GetRoutingTableEntries();
	    vector<RoutingTableEntry>::iterator 	it;

		for (it = entry.begin(); it!=entry.end(); it++) {
			   	if (AddressToNodeId.find(it->destAddr)->second == nodeId) {
			    	   		return it->distance;
			    }
		}

		return 0;
	}


	void SetNextSendTimer(Time new_m_synchFrequency) {

		if (m_synchFrequency != new_m_synchFrequency) {
			m_synchFrequency = new_m_synchFrequency;

			Simulator::Cancel(m_nextSend);


			Time timeSinceLastSend = Simulator::Now() - m_lastSend;
			ss << "Node: " << m_nodeId << " Reconfigured to: " << m_synchFrequency.GetMilliSeconds() << "\n";
			ss << "Node: " << m_nodeId << " Last send:       " << timeSinceLastSend.GetMilliSeconds() << "\n";

			if (timeSinceLastSend > m_synchFrequency) {
				ss<< "      SendData IMMEDIATELY \n";
				SendData();
			} else {
				ss<< "      " << "Setting timer to " << (m_synchFrequency - (Simulator::Now() - m_lastSend)).GetMilliSeconds() << "\n";
				m_nextSend = Simulator::Schedule (m_synchFrequency - (Simulator::Now() - m_lastSend), &WifiAppAdhoc::SendData,this);
			}


			Log(ss);

		}
	}

	void Reconfigure() {
		Time new_m_synchFrequency = DEFAULT_FREQUENCY;
		Time test_new_m_synchFrequency;

	    ss << "Node: " << m_nodeId << "\n";
	    ss << "DEFAULT FREQ: " << DEFAULT_FREQUENCY.GetMilliSeconds() << "\n";

		for (i = 0; i < (int)producers.size(); i++) {
			if (shortestPathLengths[i] != 0) {
				//CHECK if I am partitioned
				int ownDistanceToProducer = FindDistanceToNode(producers[i]);
				int ownDistanceToConsumer = FindDistanceToNode(consumers[i]);

				if (m_nodeId == producers[i]) {
					if (ownDistanceToConsumer != 0) {
						test_new_m_synchFrequency = MilliSeconds(requirements[i].GetMilliSeconds() / SPEED_UP / shortestPathLengths[i]);

						ss << "PROD TEST FREQ: " << producers[i] << " -> " << consumers[i] << ": "
						   << test_new_m_synchFrequency.GetMilliSeconds() << "\n";

						if (test_new_m_synchFrequency < new_m_synchFrequency) {
							new_m_synchFrequency = test_new_m_synchFrequency;
						}

					}
					continue;
				}

				//Consumer does not change his frequency
				if (m_nodeId == consumers[i]) {
					continue;
				}

				if ( ownDistanceToConsumer != 0 && ownDistanceToProducer != 0)
				{
//				    m_consumerToProducerDistance = ownDistanceToConsumer + ownDistanceToProducer;
//				    m_stretch = m_consumerToProducerDistance - shortestPathLength;
					// if I'm not then calculate my stretch
				    if ( ownDistanceToConsumer + ownDistanceToProducer <= shortestPathLengths[i] + ZONE_THRESHOLD )
				    {
				    	// if the stretch is within the threshold then calculate the frequency
				    	test_new_m_synchFrequency = MilliSeconds(requirements[i].GetMilliSeconds() / SPEED_UP / shortestPathLengths[i]);

				    	ss << "INTER TEST FREQ: " << producers[i] << " -> " << consumers[i] << ": "
				    	                          << test_new_m_synchFrequency.GetMilliSeconds() << "\n";

				    	// if the frequency is lower than the one I have stored, then reset it
				    	if (test_new_m_synchFrequency < new_m_synchFrequency) {
							new_m_synchFrequency = test_new_m_synchFrequency;
						}
				    }
				}
			}
		}

		ss << "FINALLY RECONF TO: " << new_m_synchFrequency.GetMilliSeconds();
		Log(ss);

		SetNextSendTimer(new_m_synchFrequency);
		//finally do the reconfiguration

		Simulator::Schedule(Seconds(1),&WifiAppAdhoc::Reconfigure,this);

	}








	void UpdateLocalTimestamp() {
		m_lastTimestampSynched[m_nodeId] = Simulator::Now().GetMilliSeconds();
		m_nextTimestamp = Simulator::Schedule(DATA_GENERATION_RATE,&WifiAppAdhoc::UpdateLocalTimestamp,this);
	}

	void SendData ()
	{
		Ptr<Packet> packet = Create<Packet>(10240);
		uint64_t packetUID = packet->GetUid();

		std::pair<uint64_t,int> type (packetUID, DATA);
		messageTypeMap.insert(type);
		std::pair<uint64_t,std::vector<int64_t> > entry(packetUID,m_lastTimestampSynched);
		sentTimestampMap.insert(entry);
		std::pair<uint64_t,int> sourceEntry(packetUID,m_nodeId);
	    sourceMap.insert(sourceEntry);

	    NS_ASSERT(m_sendSocket);
	    m_sendSocket->Send(packet);

	    m_lastSend = Simulator::Now();


	    // (After Warm-up)
	    if (Simulator::Now() > Seconds(300)) {
			for (i = 0; i<NR_NODES; ++i) {
			   packetLog << packet->GetUid() << ", "
						<< i << ", "
						<< m_lastTimestampSynched[i] << '\r' << '\n';;
			}

			packetLog.flush();

			communicationLog << packet->GetUid() << ", "
							 << m_nodeId << ", "
							 << "-1, "
							 << Simulator::Now().GetMilliSeconds() << ", "
							 << "1, "
							 << m_shortestPathLength << ", "
							 << m_consumerToProducerDistance << ", "
							 << m_synchFrequency.GetMilliSeconds() << '\r' << '\n';;

			communicationLog.flush();

			m_nextSend = Simulator::Schedule (GetRandomPerturbation(m_synchFrequency), &WifiAppAdhoc::SendData,this);
	    }
	}
	


	void ReceivePacket(Ptr<Socket> socket) {
		Ptr<Packet> packet = socket->Recv();

		packetUid = packet->GetUid();

	    int messageType = messageTypeMap.find(packetUid)->second;
		switch(messageType) {
		case DATA:
			ReceiveDataPacket(packet);
			break;
		// OTHER type handlers here
		}
	}

	void ReceiveDataPacket(Ptr<Packet> packet) {
		packetUid = packet->GetUid();
		std::vector<int64_t> timestampVectorReceived = sentTimestampMap.find(packetUid)->second;


		for (i = 0; i < NR_NODES; ++i) {
		   if (m_lastTimestampSynched[i] < timestampVectorReceived[i]) {
		      m_lastTimestampSynched[i] = timestampVectorReceived[i];
		   }
		}

		// (After Warm-up)
		if (Simulator::Now() > Seconds(300)) {

			communicationLog << packetUid << ", "
					         << sourceMap.find(packetUid)->second << ", "
					         << m_nodeId << ", "
					         << Simulator::Now().GetMilliSeconds() << ", "
					         << "2, "
					         << m_shortestPathLength << ", "
					         << m_consumerToProducerDistance << ", "
					         << m_synchFrequency.GetMilliSeconds() << '\r' << '\n';

			communicationLog.flush();


			for (i = 0; i < NR_NODES; ++i) {
				stateLog << packetUid << ", "
						 << m_nodeId << ", "
						 << Simulator::Now().GetMilliSeconds() << ", "
						 << i << ", "
						 << m_lastTimestampSynched[i] << '\r' << '\n';
			}
			stateLog.flush();
		}
	}
	
};

int main (int argc, char *argv[])
{

  producers.push_back(12);
  producers.push_back(13);

  consumers.push_back(42);
  consumers.push_back(43);

  shortestPathLengths.push_back(0);
  shortestPathLengths.push_back(0);

  requirements.push_back(Seconds(60));
  requirements.push_back(Seconds(90));

  NS_LOG_UNCOND((MilliSeconds(100) - MilliSeconds(20)).GetMilliSeconds());

  std::string phyMode ("DsssRate1Mbps");
  bool verbose = false;

  std::string traceFile;
//  float zoneThreshold;


  CommandLine cmd;
  traceFile = "scratch/scenarioExp.ns_movements";
//  cmd.AddValue("traceFile","NS2 movement trace file",traceFile);
//  cmd.AddValue("commFile","File to save communication log",communicationFile);
//  cmd.AddValue("stateFile","File to save the state of nodes",stateFile);
//  cmd.AddValue("producer","ProducerId",producer);
//  cmd.AddValue("consumer","ConsumerId",consumer);
//  cmd.AddValue("zoneThreshold","ZoneThreshold",zoneThreshold);

  cmd.Parse (argc, argv);

  // Convert to time object

//  if(traceFile.empty() || communicationFile.empty() || stateFile.empty() || ZONE_THRESHOLD == 0) {
//  		std::cout << "Usage of " << argv[0] << ":\n" << "./waf --run \"script\n"
//  				  << "   --traceFile=/path/to/tracefile\"\n"
////  				  << "   --communicationFile=name of com file \n"
////  				  << "   --stateFile=name of state file\n"
////  				  << "   --zoneThreshold=\n"
//  				  ;
//  		return 1;
//  	}


//  ZONE_THRESHOLD = zoneThreshold;
  ss << "packetLog-zone-" << ZONE_THRESHOLD << "-speed-" << SPEED_UP <<".log";
  packetLog.open(ss.str().c_str());
  ss.str("");
  ss.clear();
  packetLog << "packetUid, timeSeriesId, timestamp" << '\r' << '\n';
  packetLog.flush();

  ss << "commLog-zone-" << ZONE_THRESHOLD << "-speed-" << SPEED_UP <<".log";
  communicationLog.open(ss.str().c_str());
  ss.str("");
  ss.clear();

  communicationLog << "packetUid, source, destination, timestamp, send/receive, shortestPathLength, stretchedPathLength, synchFrequency" << '\r' << '\n';
  communicationLog.flush();

  ss << "stateLog-zone-" << ZONE_THRESHOLD << "-speed-" << SPEED_UP <<".log";
  stateLog.open(ss.str().c_str());
  ss.str("");
  ss.clear();

  stateLog << "packetUid, nodeId, timestamp, timeSeriesId, latestTimestamp" << '\r' << '\n';
  stateLog.flush();

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
//  MobilityHelper mobility;

//  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
//                                 "MinX", DoubleValue (0.0),
//                                 "MinY", DoubleValue (0.0),
//                                 "DeltaX", DoubleValue (100),
//                                 "DeltaY", DoubleValue (100),
//                                 "GridWidth", UintegerValue (7),
//                                 "LayoutType", StringValue ("RowFirst"));

//  mobility.SetMobilityModel ("ns3::RandomWalk2dMobilityModel",
//                             "Bounds", RectangleValue (Rectangle (0, 900, 0, 900)));


//  mobility.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
//                                "Bounds", RectangleValue (Rectangle (0, 800, 0, 800)),
//                                "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=1.5]"),
//                                "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=20]"));

//  mobility.SetMobilityModel ("ns3::RandomWaypointMobilityModel",
//                                  "Bounds", RectangleValue (Rectangle (0, 800, 0, 800)),
//                                  "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=1.5]"),
//                                  "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=20]"));
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

  Ns2MobilityHelper ns2mobility = Ns2MobilityHelper(traceFile);


  ns2mobility.Install();

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

  NS_LOG_UNCOND("Starting Nodes");
  for (int i; i<NR_NODES; ++i) {
	  Ptr<WifiAppAdhoc> app = CreateObject<WifiAppAdhoc>(c.Get(i),i,NR_NODES);
	  c.Get(i)->AddApplication(app);
	  app->SetStartTime(Time(0));

  }

  NS_LOG_UNCOND("Started Nodes");

	  //app->SetStopTime(Time(10000.0));
//
//  Ptr<WifiAppAdhoc> app2 = CreateObject<WifiAppAdhoc>(c.Get(1),1,NR_NODES);
//  c.Get(1)->AddApplication(app2);
//  app2->SetStartTime(MilliSeconds(1));
//  //app2->SetStopTime(Time(10000.0));

  // Tracing
//  wifiPhy.EnablePcap ("wifi-simple-adhoc", devices);

  Simulator::Stop(Seconds(2100));
  Simulator::Run ();

  NS_LOG_UNCOND("Started the simulation");

  Simulator::Destroy ();


  ss << "Finished: \n         Speed: " << SPEED_UP << " Zone: " << ZONE_THRESHOLD;
  Log(ss);

  return 0;
}

