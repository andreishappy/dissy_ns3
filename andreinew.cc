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

#include <fstream>
#include <iostream>
#include <sstream>
#include <map>
#include <stdio.h>
#include <sys/time.h>
#include <climits>
#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/config.h"
#include "ns3/olsr-helper.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/olsr-routing-protocol.h"


using namespace ns3;
using namespace std;
using namespace ns3::Config;
using namespace ns3::olsr;

NS_LOG_COMPONENT_DEFINE ("FirstScriptExample");

int
main (int argc, char *argv[])
{

	NodeContainer m_mobileNodes;

	uint32_t numberOfMobileNodes = 10;
	uint32_t gridXLength = 100;
	uint32_t gridYLength = 100;
	double mobilitySpeed = 0;
	RandomVariable mobilityPause =  UniformVariable (200.0, 210.0);

	m_mobileNodes.Create(numberOfMobileNodes);

	StringValue phyMode = StringValue ("DsssRate11Mbps");

	Config::SetDefault
			("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue
			 ("2200"));
	Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold",
								StringValue ("2200"));
	Config::SetDefault ("ns3::WifiRemoteStationManager::NonUnicastMode",
								phyMode);

	WifiHelper wifi;
	wifi.SetStandard (WIFI_PHY_STANDARD_80211b);

	YansWifiPhyHelper wifiPhy =  YansWifiPhyHelper::Default ();
	wifiPhy.Set ("RxGain", DoubleValue (0) );

	YansWifiChannelHelper wifiChannel;
	wifiChannel.SetPropagationDelay
			("ns3::ConstantSpeedPropagationDelayModel");
	NS_LOG_UNCOND("	Propagation delay model: ConstantSpeedPropagationDelayModel");
	NS_LOG_UNCOND("	Propagation loss model: LogDistancePropagationLossModel -exp 2");
	wifiChannel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel",
											"Exponent", DoubleValue(2));

	wifiPhy.SetChannel (wifiChannel.Create ());

	NqosWifiMacHelper wifiMac = NqosWifiMacHelper::Default ();
			wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
										  "DataMode", phyMode,
										  "ControlMode", phyMode);


	MobilityHelper mobility;

	NS_LOG_UNCOND("	Mobile nodes ...");
	NS_LOG_UNCOND("	Position allocator: RandomRectanglePositionAllocator");

	/*mobility.SetPositionAllocator ("ns3::RandomRectanglePositionAllocator",
										   "X", RandomVariableValue(UniformVariable (0, gridXLength)),
										   "Y", RandomVariableValue(UniformVariable (0, gridYLength)));
    */

	  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
	                                 "MinX", DoubleValue (0.0),
	                                 "MinY", DoubleValue (0.0),
	                                 "DeltaX", DoubleValue (5.0),
	                                 "DeltaY", DoubleValue (10.0),
	                                 "GridWidth", UintegerValue (3),
	                                 "LayoutType", StringValue ("RowFirst"));


	if (mobilitySpeed == 0)
		{
			mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

			NS_LOG_UNCOND("	Mobility model: ConstantPositionMobilityModel");
		}
	else
		{
			mobility.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
										   "Bounds", RectangleValue (Rectangle (0, gridXLength, 0,
																				gridYLength)),
										   "Speed", RandomVariableValue (ConstantVariable (mobilitySpeed)),
										   "Pause", RandomVariableValue (mobilityPause));

			NS_LOG_UNCOND("	Mobility model: RandomDirection2dMobilityModel");
			NS_LOG_UNCOND("		Speed: " << mobilitySpeed);
			NS_LOG_UNCOND("		Pause: " << mobilityPause);
			NS_LOG_UNCOND("		Bounds: " << Rectangle (0, gridXLength, 0,
															gridYLength));
		}

		mobility.Install (m_mobileNodes);

		//trace files for mobility
		//std::string testMobilityFilePath = CreateTempDirFilename("mobility-trace.mob");



		wifiMac.SetType ("ns3::AdhocWifiMac");
		NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, m_mobileNodes);

		OlsrHelper olsr;
		Ipv4StaticRoutingHelper staticRouting;
		Ipv4ListRoutingHelper list;
		//list.Add (staticRouting, 0);
		list.Add (olsr, 0);

		InternetStackHelper internet;
		internet.SetRoutingHelper(list);
		internet.Install (m_mobileNodes);

		Ipv4AddressHelper ipv4;
		ipv4.SetBase ("10.1.1.0", "255.255.255.0");
		Ipv4InterfaceContainer i = ipv4.Assign (devices);

		Simulator::Stop (Seconds (1.0));


		Simulator::Run ();
		Simulator::Destroy ();
	    return 0;
}
