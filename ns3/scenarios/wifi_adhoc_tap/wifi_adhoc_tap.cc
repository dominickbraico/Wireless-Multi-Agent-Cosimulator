// N-agent 802.11 ad hoc scenario with one ghost node + one TapBridge per agent.
// All parameters come from the experiment YAML (no magic numbers here).
// Real packets enter/leave through TAP devices created by the orchestrator.
//
//   wifi_adhoc_tap --config=<experiment.yaml> [--log-dir=<dir>] [--duration=<s>]
// network.wifi.standard: 80211a (default), 80211g, 80211n, 80211ac or 80211ax; optional network.wifi.band: 5GHZ | 2_4GHZ.

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/realtime-simulator-impl.h"
#include "ns3/tap-bridge-module.h"
#include "ns3/wifi-module.h"

#include <yaml-cpp/yaml.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <csignal>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <thread>

using namespace ns3;

namespace
{

struct AgentCfg
{
    std::string name;
    std::string tap;
    Vector position; // initial ghost-node position (world frame, meters)
};

struct LossCfg
{
    double exponent;
    double refDistance;
    double refLoss;
};

struct Cfg
{
    bool poseEnabled = false;
    std::string poseAddr;
    int posePort = 0;
    std::string name;
    double durationS;
    std::string logDir;
    std::string syncMode;
    double hardLimitS;
    double maxLagMs;
    std::string dataMode;
    std::string ctrlMode;
    std::string standard = "80211a";
    std::string band = "5GHZ"; // 5GHZ | 2_4GHZ
    double txPowerDbm;
    int retryLimit; // <0 = ns-3 default
    double rxNoiseFigureDb = 7.0;
    double preambleMinRssiDbm = -82.0;
    double preambleSnrDb = 4.0;
    double ccaEdDbm = -62.0;
    double rxSensDbm = -101.0;
    std::string errorModel = "TableBased";
    std::vector<LossCfg> loss;
    std::vector<AgentCfg> agents;
};

std::string
Replace(std::string s, const std::string& from, const std::string& to)
{
    for (size_t p = s.find(from); p != std::string::npos; p = s.find(from, p + to.size()))
    {
        s.replace(p, from.size(), to);
    }
    return s;
}

std::string
Timestamp()
{
    std::time_t t = std::time(nullptr);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", std::localtime(&t));
    return buf;
}

// network.wifi.standard -> ns-3 standard. The rate strings (data_mode, control_mode) are passed through to ns-3 as
// given, so they must belong to the chosen standard (OfdmRate12Mbps, ErpOfdmRate12Mbps, HtMcs4, VhtMcs4, HeMcs4, ...).
WifiStandard
StandardOf(const std::string& name)
{
    static const std::map<std::string, WifiStandard> known{{"80211a", WIFI_STANDARD_80211a},
                                                           {"80211g", WIFI_STANDARD_80211g},
                                                           {"80211n", WIFI_STANDARD_80211n},
                                                           {"80211ac", WIFI_STANDARD_80211ac},
                                                           {"80211ax", WIFI_STANDARD_80211ax}};
    auto it = known.find(name);
    return it == known.end() ? WIFI_STANDARD_UNSPECIFIED : it->second;
}

Cfg
LoadConfig(const std::string& path)
{
    YAML::Node y = YAML::LoadFile(path);
    Cfg c;
    c.name = y["experiment"]["name"].as<std::string>();
    c.durationS = y["experiment"]["duration_s"].as<double>();
    c.logDir = Replace(Replace(y["experiment"]["log_dir"].as<std::string>(), "${name}", c.name),
                       "${timestamp}",
                       Timestamp());
    c.syncMode = y["time"]["ns3_sync_mode"].as<std::string>();
    c.hardLimitS = y["time"]["ns3_hard_limit_s"].as<double>();
    c.maxLagMs = y["time"]["max_lag_ms"].as<double>();

    auto w = y["network"]["wifi"];
    c.standard = w["standard"].as<std::string>();
    NS_ABORT_MSG_IF(StandardOf(c.standard) == WIFI_STANDARD_UNSPECIFIED,
                    "unsupported network.wifi.standard '" << c.standard
                                                          << "' (supported: 80211a, 80211g, 80211n, 80211ac, 80211ax)");
    c.band = w["band"].as<std::string>(c.standard == "80211g" ? "2_4GHZ" : "5GHZ");
    NS_ABORT_MSG_IF(c.band != "5GHZ" && c.band != "2_4GHZ", "network.wifi.band must be 5GHZ or 2_4GHZ");
    NS_ABORT_MSG_IF(w["mac"].as<std::string>() != "adhoc", "only adhoc MAC supported");
    NS_ABORT_MSG_IF(w["rate_manager"].as<std::string>() != "ConstantRate",
                    "only ConstantRate supported so far");
    c.errorModel = w["error_rate_model"].as<std::string>(c.errorModel);
    c.dataMode = w["data_mode"].as<std::string>();
    c.ctrlMode = w["control_mode"].as<std::string>();
    c.txPowerDbm = w["tx_power_dbm"].as<double>();
    c.retryLimit = (w["retry_limit"] && !w["retry_limit"].IsNull()) ? w["retry_limit"].as<int>() : -1;

    if (auto ph = w["phy"])
    {
        c.rxNoiseFigureDb = ph["rx_noise_figure_db"].as<double>(c.rxNoiseFigureDb);
        c.preambleMinRssiDbm = ph["preamble_detection_min_rssi_dbm"].as<double>(c.preambleMinRssiDbm);
        c.preambleSnrDb = ph["preamble_detection_snr_db"].as<double>(c.preambleSnrDb);
        c.ccaEdDbm = ph["cca_ed_threshold_dbm"].as<double>(c.ccaEdDbm);
        c.rxSensDbm = ph["rx_sensitivity_dbm"].as<double>(c.rxSensDbm);
    }

    for (const auto& l : y["network"]["channel"]["loss"])
    {
        NS_ABORT_MSG_IF(l["type"].as<std::string>() != "LogDistance",
                        "only LogDistance loss supported so far");
        c.loss.push_back({l["exponent"].as<double>(),
                          l["ref_distance_m"].as<double>(),
                          l["ref_loss_db"].as<double>()});
    }
    NS_ABORT_MSG_IF(y["network"]["channel"]["delay"].as<std::string>() != "ConstantSpeed",
                    "only ConstantSpeed delay supported");

    if (y["network"]["pose"] && y["network"]["pose"]["port"])
    {
        auto pz = y["network"]["pose"];
        NS_ABORT_MSG_IF(pz["transport"].as<std::string>() != "udp", "only udp pose transport");
        c.poseEnabled = true;
        c.poseAddr = pz["listen_addr"].as<std::string>("127.0.0.1");
        c.posePort = pz["port"].as<int>();
    }

    for (const auto& a : y["agents"])
    {
        // Explicit initial_position wins (static tests); otherwise spawn_pose xyz + antenna_offset, so a
        // flight config states where each vehicle starts only once.
        std::vector<double> p;
        if (a["initial_position"])
        {
            p = a["initial_position"].as<std::vector<double>>();
            NS_ABORT_MSG_IF(p.size() != 3, "initial_position needs 3 values");
        }
        else
        {
            NS_ABORT_MSG_IF(!a["spawn_pose"],
                            "agent " << a["name"].as<std::string>()
                                     << ": needs initial_position or spawn_pose");
            auto sp = a["spawn_pose"].as<std::vector<double>>();
            NS_ABORT_MSG_IF(sp.size() < 3, "spawn_pose needs at least x y z");
            std::vector<double> off{0.0, 0.0, 0.0};
            if (a["antenna_offset"])
            {
                off = a["antenna_offset"].as<std::vector<double>>();
                NS_ABORT_MSG_IF(off.size() != 3, "antenna_offset needs 3 values");
            }
            p = {sp[0] + off[0], sp[1] + off[1], sp[2] + off[2]};
        }
        c.agents.push_back({a["name"].as<std::string>(),
                            a["tap"].as<std::string>(),
                            Vector(p[0], p[1], p[2])});
    }
    NS_ABORT_MSG_IF(c.agents.empty(), "no agents configured");
    return c;
}

extern std::ofstream g_poseCsv;
extern std::ofstream g_rssiCsv;

// ---- time health ----
double g_maxLagMs = 0;
double g_sumLagMs = 0;
uint64_t g_lagSamples = 0;
std::ofstream g_lagCsv;

void
LagTick(Ptr<RealtimeSimulatorImpl> impl)
{
    double lagMs = (impl->RealtimeNow() - Simulator::Now()).GetSeconds() * 1e3;
    g_lagCsv << Simulator::Now().GetSeconds() << "," << lagMs << "\n";
    g_lagCsv.flush();
    g_rssiCsv.flush();
    if (g_poseCsv.is_open())
    {
        g_poseCsv.flush();
    }
    g_maxLagMs = std::max(g_maxLagMs, lagMs);
    g_sumLagMs += lagMs;
    ++g_lagSamples;
    Simulator::Schedule(Seconds(1), &LagTick, impl);
}

double WallNow();

// ---- RSSI trace ----
std::ofstream g_rssiCsv;

void
SnifferRx(uint32_t nodeId,
          Ptr<const Packet> pkt,
          uint16_t freq,
          WifiTxVector txv,
          MpduInfo mpdu,
          SignalNoiseDbm sn,
          uint16_t staId)
{
    g_rssiCsv << std::fixed << std::setprecision(6) << Simulator::Now().GetSeconds() << "," << nodeId
              << "," << sn.signal << "," << sn.noise << "," << pkt->GetSize() << ","
              << txv.GetMode().GetUniqueName() << "," << WallNow() << "\n";
}

// ---- graceful shutdown: SIGTERM/SIGINT -> stop the simulation so logs and summary.json are written ----
std::atomic<bool> g_terminate{false};

void
OnSignal(int)
{
    g_terminate = true; // only set a flag; a watcher thread does the actual (non-signal-safe) stop
}

void
StopNow()
{
    Simulator::Stop();
}

// ---- live pose feed (simulator -> ns-3) ----
// A listener thread receives UDP poses and may ONLY call Simulator::ScheduleWithContext
// (thread-safe in RealtimeSimulatorImpl); every ns-3 object is touched in the event.
struct PoseRec
{
    uint64_t seq;
    double tSim;
    double tWallSend;
    double tWallRecv;
    Vector pos;
};

std::ofstream g_poseCsv;
uint64_t g_poseApplied = 0;
std::atomic<uint64_t> g_poseDatagrams{0};
std::atomic<uint64_t> g_poseBadDatagrams{0};
double g_maxPoseStaleMs = 0;

double
WallNow()
{
    using namespace std::chrono;
    return duration<double>(system_clock::now().time_since_epoch()).count();
}

// Runs on the simulator thread. Receives a node ID (plain integer), never a Ptr: ns-3's Ptr
// reference counts are not atomic, so a Ptr copied on the listener thread races with the
// simulator thread (observed as heap corruption at shutdown).
void
ApplyPose(uint32_t nodeId, PoseRec r)
{
    Ptr<Node> node = NodeList::GetNode(nodeId);
    node->GetObject<MobilityModel>()->SetPosition(r.pos);
    double staleMs = (WallNow() - r.tWallSend) * 1e3; // sim-side send -> applied in ns-3
    g_maxPoseStaleMs = std::max(g_maxPoseStaleMs, staleMs);
    ++g_poseApplied;
    g_poseCsv << std::fixed << std::setprecision(6) << Simulator::Now().GetSeconds() << ","
              << node->GetId() << "," << r.seq << "," << r.tSim << "," << r.pos.x << ","
              << r.pos.y << "," << r.pos.z << "," << std::setprecision(3) << staleMs << ","
              << std::setprecision(6) << WallNow() << "\n";
}

void
PoseListener(const Cfg& cfg, std::vector<uint32_t> nodeIds, std::atomic<bool>& stop)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    NS_ABORT_MSG_IF(fd < 0, "pose socket");
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(cfg.posePort);
    NS_ABORT_MSG_IF(inet_pton(AF_INET, cfg.poseAddr.c_str(), &addr.sin_addr) != 1, "pose addr");
    NS_ABORT_MSG_IF(bind(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0,
                    "cannot bind pose socket " << cfg.poseAddr << ":" << cfg.posePort);
    timeval tv{0, 200000}; // wake to check the stop flag
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    std::map<std::string, uint32_t> byName;
    for (uint32_t i = 0; i < cfg.agents.size(); ++i)
    {
        byName[cfg.agents[i].name] = i;
    }
    char buf[4096];
    while (!stop)
    {
        ssize_t n = recv(fd, buf, sizeof(buf) - 1, 0);
        if (n <= 0)
        {
            continue;
        }
        double tRecv = WallNow();
        ++g_poseDatagrams;
        try
        {
            YAML::Node j = YAML::Load(std::string(buf, n)); // JSON is valid YAML flow syntax
            uint64_t seq = j["seq"].as<uint64_t>();
            double tSim = j["t_sim"].as<double>();
            double tSend = j["t_wall"].as<double>();
            for (const auto& kv : j["agents"])
            {
                auto it = byName.find(kv.first.as<std::string>());
                if (it == byName.end())
                {
                    continue;
                }
                auto p = kv.second.as<std::vector<double>>();
                PoseRec r{seq, tSim, tSend, tRecv, Vector(p.at(0), p.at(1), p.at(2))};
                const uint32_t nodeId = nodeIds.at(it->second); // plain integer only: no ns-3 objects here
                Simulator::ScheduleWithContext(nodeId, Seconds(0), &ApplyPose, nodeId, r);
            }
        }
        catch (const std::exception&)
        {
            ++g_poseBadDatagrams;
        }
    }
    close(fd);
}

} // namespace

int
main(int argc, char* argv[])
{
    // Hand-parsed: ns-3's CommandLine reads strings with operator>> and so
    // truncates any path containing a space.
    std::string configPath;
    std::string logDirOverride;
    double durationOverride = -1;
    for (int i = 1; i < argc; ++i)
    {
        std::string a = argv[i];
        auto val = [&](const std::string& key, std::string& out) {
            if (a.rfind(key + "=", 0) == 0)
            {
                out = a.substr(key.size() + 1);
                return true;
            }
            if (a == key && i + 1 < argc)
            {
                out = argv[++i];
                return true;
            }
            return false;
        };
        std::string dur;
        if (!val("--config", configPath) && !val("--log-dir", logDirOverride) &&
            val("--duration", dur))
        {
            durationOverride = std::stod(dur);
        }
    }
    NS_ABORT_MSG_IF(configPath.empty(),
                    "usage: wifi_adhoc_tap --config=<yaml> [--log-dir=<dir>] [--duration=<s>]");

    Cfg cfg = LoadConfig(configPath);
    if (!logDirOverride.empty())
    {
        cfg.logDir = logDirOverride;
    }
    if (durationOverride > 0)
    {
        cfg.durationS = durationOverride;
    }
    std::filesystem::create_directories(cfg.logDir);
    std::filesystem::copy_file(configPath,
                               cfg.logDir + "/experiment.yaml",
                               std::filesystem::copy_options::overwrite_existing);

    // Real packets cross this simulator: real-time scheduler + checksums are mandatory.
    GlobalValue::Bind("SimulatorImplementationType", StringValue("ns3::RealtimeSimulatorImpl"));
    GlobalValue::Bind("ChecksumEnabled", BooleanValue(true));
    Config::SetDefault("ns3::RealtimeSimulatorImpl::SynchronizationMode", StringValue(cfg.syncMode));
    Config::SetDefault("ns3::RealtimeSimulatorImpl::HardLimit",
                       TimeValue(Seconds(cfg.hardLimitS)));
    if (cfg.retryLimit >= 0)
    {
        Config::SetDefault("ns3::WifiRemoteStationManager::MaxSsrc", UintegerValue(cfg.retryLimit));
        Config::SetDefault("ns3::WifiRemoteStationManager::MaxSlrc", UintegerValue(cfg.retryLimit));
    }

    const uint32_t n = cfg.agents.size();
    NodeContainer ghosts;
    ghosts.Create(n);

    // ONE shared channel object for all agents (shared medium).
    YansWifiChannelHelper channel;
    channel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    for (const auto& l : cfg.loss)
    {
        channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                                   "Exponent",
                                   DoubleValue(l.exponent),
                                   "ReferenceDistance",
                                   DoubleValue(l.refDistance),
                                   "ReferenceLoss",
                                   DoubleValue(l.refLoss));
    }
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    phy.Set("TxPowerStart", DoubleValue(cfg.txPowerDbm));
    phy.Set("TxPowerEnd", DoubleValue(cfg.txPowerDbm));
    phy.SetErrorRateModel("ns3::" + cfg.errorModel + "ErrorRateModel");
    phy.Set("RxNoiseFigure", DoubleValue(cfg.rxNoiseFigureDb));
    phy.Set("CcaEdThreshold", DoubleValue(cfg.ccaEdDbm));
    phy.Set("RxSensitivity", DoubleValue(cfg.rxSensDbm));
    phy.SetPcapDataLinkType(WifiPhyHelper::DLT_IEEE802_11_RADIO);
    // channel number 0 and width 0 select the standard's default channel in the chosen band
    if (cfg.standard != "80211a")
    {
        phy.Set("ChannelSettings", StringValue("{0, 0, BAND_" + cfg.band + ", 0}"));
    }

    WifiHelper wifi;
    wifi.SetStandard(StandardOf(cfg.standard));
    wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                 "DataMode",
                                 StringValue(cfg.dataMode),
                                 "ControlMode",
                                 StringValue(cfg.ctrlMode));
    phy.SetPreambleDetectionModel("ns3::ThresholdPreambleDetectionModel",
                                  "Threshold",
                                  DoubleValue(cfg.preambleSnrDb),
                                  "MinimumRssi",
                                  DoubleValue(cfg.preambleMinRssiDbm));
    WifiMacHelper mac;
    mac.SetType("ns3::AdhocWifiMac");
    NetDeviceContainer devs = wifi.Install(phy, mac, ghosts);

    Ptr<ListPositionAllocator> alloc = CreateObject<ListPositionAllocator>();
    for (const auto& a : cfg.agents)
    {
        alloc->Add(a.position);
    }
    MobilityHelper mob;
    mob.SetPositionAllocator(alloc);
    mob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mob.Install(ghosts);

    // Ghost nodes get NO internet stack; IPs live on the Linux side (radio0).
    // One TapBridge per node, UseLocal (Wi-Fi devices cannot be promiscuous).
    TapBridgeHelper tap;
    tap.SetAttribute("Mode", StringValue("UseLocal"));
    for (uint32_t i = 0; i < n; ++i)
    {
        tap.SetAttribute("DeviceName", StringValue(cfg.agents[i].tap));
        tap.Install(ghosts.Get(i), devs.Get(i));
    }

    phy.EnablePcap(cfg.logDir + "/wifi", devs);

    g_rssiCsv.open(cfg.logDir + "/rssi.csv");
    g_rssiCsv << "t_s,rx_node,signal_dbm,noise_dbm,bytes,mode,t_wall_s\n";
    for (uint32_t i = 0; i < n; ++i)
    {
        std::ostringstream path;
        path << "/NodeList/" << ghosts.Get(i)->GetId()
             << "/DeviceList/0/$ns3::WifiNetDevice/Phys/0/MonitorSnifferRx";
        Config::ConnectWithoutContext(
            path.str(),
            MakeBoundCallback(&SnifferRx, ghosts.Get(i)->GetId()));
    }

    g_lagCsv.open(cfg.logDir + "/lag.csv");
    g_lagCsv << "t_s,lag_ms\n";
    Ptr<RealtimeSimulatorImpl> rt = DynamicCast<RealtimeSimulatorImpl>(Simulator::GetImplementation());
    NS_ABORT_MSG_IF(!rt, "RealtimeSimulatorImpl not active");
    Simulator::Schedule(Seconds(1), &LagTick, rt);

    std::atomic<bool> stopPose{false};
    std::thread poseThread;
    if (cfg.poseEnabled)
    {
        g_poseCsv.open(cfg.logDir + "/pose_applied.csv");
        g_poseCsv << "t_ns3_s,node,seq,t_sim_s,x,y,z,stale_ms,t_wall_apply_s\n";
        std::vector<uint32_t> nodeIds;
        for (uint32_t i = 0; i < n; ++i)
        {
            nodeIds.push_back(ghosts.Get(i)->GetId());
        }
        poseThread = std::thread(PoseListener, std::cref(cfg), nodeIds, std::ref(stopPose));
    }

    std::signal(SIGTERM, OnSignal);
    std::signal(SIGINT, OnSignal);
    std::atomic<bool> stopWatch{false};
    std::thread watcher([&]() {
        while (!stopWatch && !g_terminate)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (g_terminate)
        {
            Simulator::ScheduleWithContext(0, Seconds(0), &StopNow); // plain function, no ns-3 objects
        }
    });

    Simulator::Stop(Seconds(cfg.durationS));
    Simulator::Run();
    stopWatch = true;
    watcher.join();
    stopPose = true;
    if (poseThread.joinable())
    {
        poseThread.join();
    }
    Simulator::Destroy();
    g_poseCsv.close();

    g_lagCsv.close();
    g_rssiCsv.close();

    bool lagValid = g_maxLagMs <= cfg.maxLagMs;
    std::ofstream sum(cfg.logDir + "/summary.json");
    sum << std::fixed << std::setprecision(3) << "{\n"
        << "  \"experiment\": \"" << cfg.name << "\",\n"
        << "  \"duration_s\": " << cfg.durationS << ",\n"
        << "  \"agents\": " << n << ",\n"
        << "  \"wifi_standard\": \"" << cfg.standard << "\",\n"
        << "  \"wifi_band\": \"" << cfg.band << "\",\n"
        << "  \"wifi_data_mode\": \"" << cfg.dataMode << "\",\n"
        << "  \"ns3_max_lag_ms\": " << g_maxLagMs << ",\n"
        << "  \"ns3_mean_lag_ms\": " << (g_lagSamples ? g_sumLagMs / g_lagSamples : 0) << ",\n"
        << "  \"max_lag_limit_ms\": " << cfg.maxLagMs << ",\n"
        << "  \"pose_datagrams\": " << g_poseDatagrams << ",\n"
        << "  \"pose_bad_datagrams\": " << g_poseBadDatagrams << ",\n"
        << "  \"pose_applied\": " << g_poseApplied << ",\n"
        << "  \"pose_max_stale_ms\": " << g_maxPoseStaleMs << ",\n"
        << "  \"valid_ns3_lag\": " << (lagValid ? "true" : "false") << "\n}\n";
    std::cout << "run " << (lagValid ? "VALID" : "INVALID (ns-3 lag over limit)")
              << ", max lag " << g_maxLagMs << " ms, logs: " << cfg.logDir << std::endl;
    return 0;
}
