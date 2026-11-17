// radio_relay: copies the topics listed for an experiment between an agent's local DDS domain and the radio domain.
//
// One process, two ROS 2 contexts:
//   local  (default domain 0, discovery range LOCALHOST): where PX4's topics and the user's nodes live
//   radio  (default domain 42, bound to radio0 by CYCLONEDDS_URI): the only DDS domain allowed onto the emulated channel
// For every listed topic the relay subscribes in each context and republishes the serialized bytes, unchanged, in the
// other one, with the same topic name, type and QoS. Nothing is added to a message, so the load on the radio is the
// load of the user's own messages (plus DDS discovery of the listed topics only).
//
// Echo suppression: each subscription sets ignore_local_publications, so a message the relay itself published into a
// domain is not delivered back to the relay's subscription in that domain (rmw_cyclonedds_cpp implements it per
// participant, and a context has one participant). tests/custom_relay.sh (R2, R2b, R4b) checks it.
//
// Parameters:  topics_file  one topic per line: <name> <type> <reliability> <durability> <depth>   ('#' starts a comment)
//              agent_name   used in log lines
//              local_domain (0), radio_domain (42)
//              log_csv      optional; one row per relayed message: t_s,direction,topic,bytes,hash
//                           t_s = wall clock (CLOCK_REALTIME, seconds); direction tx = local -> radio, rx = radio -> local;
//                           hash = FNV-1a 64 of the serialized payload (tools/analyze_radio.py matches tx and rx rows by it)
// Exit code 2 with a message on stderr for an unreadable topics file, an unknown message type or a bad QoS value.
#include <cinttypes>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace
{
struct Topic
{
  std::string name, type, reliability, durability;
  int depth = 10;
};

[[noreturn]] void Die(const std::string & why)
{
  std::fprintf(stderr, "radio_relay: %s\n", why.c_str());
  std::exit(2);
}

std::vector<Topic> ReadTopics(const std::string & path)
{
  std::ifstream in(path);
  if (!in) {
    Die("cannot read topics_file '" + path + "'");
  }
  std::vector<Topic> topics;
  std::string line;
  while (std::getline(in, line)) {
    const auto hash = line.find('#');
    if (hash != std::string::npos) {
      line.erase(hash);
    }
    std::istringstream fields(line);
    Topic t;
    if (!(fields >> t.name)) {
      continue;  // blank line
    }
    if (!(fields >> t.type >> t.reliability >> t.durability >> t.depth)) {
      Die("topics_file: expected '<name> <type> <reliability> <durability> <depth>', got '" + line + "'");
    }
    if (t.reliability != "best_effort" && t.reliability != "reliable") {
      Die(t.name + ": reliability must be best_effort or reliable, got '" + t.reliability + "'");
    }
    if (t.durability != "volatile" && t.durability != "transient_local") {
      Die(t.name + ": durability must be volatile or transient_local, got '" + t.durability + "'");
    }
    if (t.depth < 1) {
      Die(t.name + ": depth must be an integer >= 1");
    }
    topics.push_back(t);
  }
  if (topics.empty()) {
    Die("topics_file '" + path + "' lists no topic");
  }
  return topics;
}

rclcpp::QoS MakeQos(const Topic & t)
{
  rclcpp::QoS qos{rclcpp::KeepLast(static_cast<size_t>(t.depth))};
  t.reliability == "reliable" ? qos.reliable() : qos.best_effort();
  t.durability == "transient_local" ? qos.transient_local() : qos.durability_volatile();
  return qos;
}

uint64_t Fnv1a64(const uint8_t * data, size_t n)
{
  uint64_t h = 14695981039346656037ULL;
  for (size_t i = 0; i < n; ++i) {
    h = (h ^ data[i]) * 1099511628211ULL;
  }
  return h;
}

class Log
{
public:
  explicit Log(const std::string & path)
  {
    if (path.empty()) {
      return;
    }
    file_ = std::fopen(path.c_str(), "w");
    if (!file_) {
      Die("cannot write log_csv '" + path + "'");
    }
    std::setvbuf(file_, nullptr, _IOLBF, 0);  // line-buffered: rows survive a kill, and a live reader sees them
    std::fputs("t_s,direction,topic,bytes,hash\n", file_);
  }
  void Row(const char * direction, const std::string & topic, const rcl_serialized_message_t & m)
  {
    if (!file_) {
      return;
    }
    timespec now{};
    clock_gettime(CLOCK_REALTIME, &now);
    std::lock_guard<std::mutex> lock(mutex_);
    std::fprintf(
      file_, "%lld.%06ld,%s,%s,%zu,%016" PRIx64 "\n", static_cast<long long>(now.tv_sec), now.tv_nsec / 1000,
      direction, topic.c_str(), m.buffer_length, Fnv1a64(m.buffer, m.buffer_length));
  }

private:
  std::FILE * file_ = nullptr;
  std::mutex mutex_;
};

const char * Name(rclcpp::ReliabilityPolicy p) {return p == rclcpp::ReliabilityPolicy::Reliable ? "RELIABLE" : "BEST_EFFORT";}
const char * Name(rclcpp::DurabilityPolicy p) {return p == rclcpp::DurabilityPolicy::TransientLocal ? "TRANSIENT_LOCAL" : "VOLATILE";}
}  // namespace

int main(int argc, char ** argv)
{
  // The local context must not discover beyond this host. The discovery range is read from the environment when a
  // context is initialised, so it is set for the local context and put back for the radio one.
  const char * previous = std::getenv("ROS_AUTOMATIC_DISCOVERY_RANGE");
  const std::string saved = previous ? previous : "";

  // Domain ids are needed before the nodes exist, so they are read from argv here (-p local_domain:=N -p radio_domain:=N).
  int local_domain = 0, radio_domain = 42;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a.rfind("local_domain:=", 0) == 0) {local_domain = std::atoi(a.c_str() + 14);}
    if (a.rfind("radio_domain:=", 0) == 0) {radio_domain = std::atoi(a.c_str() + 14);}
  }

  rclcpp::install_signal_handlers();
  setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST", 1);
  auto local_ctx = std::make_shared<rclcpp::Context>();
  rclcpp::InitOptions local_init;
  local_init.set_domain_id(static_cast<size_t>(local_domain));
  local_ctx->init(argc, argv, local_init);
  previous ? setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", saved.c_str(), 1) : unsetenv("ROS_AUTOMATIC_DISCOVERY_RANGE");

  auto radio_ctx = std::make_shared<rclcpp::Context>();
  rclcpp::InitOptions radio_init;
  radio_init.set_domain_id(static_cast<size_t>(radio_domain));
  radio_init.auto_initialize_logging(false);  // logging is process-wide and was initialised by the local context
  radio_ctx->init(argc, argv, radio_init);

  auto local = std::make_shared<rclcpp::Node>("radio_relay", rclcpp::NodeOptions().context(local_ctx));
  // The radio-side node carries no parameter services, parameter events or /rosout: each would add endpoints, and so
  // discovery traffic, to the emulated channel.
  auto radio = std::make_shared<rclcpp::Node>(
    "radio_relay", rclcpp::NodeOptions().context(radio_ctx).start_parameter_services(false)
    .start_parameter_event_publisher(false).enable_rosout(false));

  local->declare_parameter("local_domain", local_domain);
  local->declare_parameter("radio_domain", radio_domain);
  const std::string topics_file = local->declare_parameter("topics_file", std::string());
  const std::string agent = local->declare_parameter("agent_name", std::string("agent"));
  Log log(local->declare_parameter("log_csv", std::string()));
  if (topics_file.empty()) {
    Die("parameter topics_file is required");
  }

  rclcpp::SubscriptionOptions sub_options;
  sub_options.ignore_local_publications = true;  // echo suppression, see the file header

  std::vector<rclcpp::GenericPublisher::SharedPtr> publishers;
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions;
  for (const Topic & t : ReadTopics(topics_file)) {
    const rclcpp::QoS qos = MakeQos(t);
    try {
      auto to_radio = radio->create_generic_publisher(t.name, t.type, qos);
      auto to_local = local->create_generic_publisher(t.name, t.type, qos);
      subscriptions.push_back(
        local->create_generic_subscription(
          t.name, t.type, qos,
          [to_radio, &log, name = t.name](std::shared_ptr<rclcpp::SerializedMessage> m) {
            log.Row("tx", name, m->get_rcl_serialized_message());
            to_radio->publish(*m);
          }, sub_options));
      subscriptions.push_back(
        radio->create_generic_subscription(
          t.name, t.type, qos,
          [to_local, &log, name = t.name](std::shared_ptr<rclcpp::SerializedMessage> m) {
            log.Row("rx", name, m->get_rcl_serialized_message());
            to_local->publish(*m);
          }, sub_options));
      const rclcpp::QoS actual = to_radio->get_actual_qos();
      std::printf(
        "relay %s: %s %s %s %s KEEP_LAST depth=%zu\n", agent.c_str(), t.name.c_str(), t.type.c_str(),
        Name(actual.reliability()), Name(actual.durability()), actual.depth());
      publishers.push_back(to_radio);
      publishers.push_back(to_local);
    } catch (const std::exception & e) {
      Die(t.name + ": cannot use message type '" + t.type + "' (" + e.what() + ")");
    }
  }
  std::printf("relay %s: local domain %d <-> radio domain %d, %zu topic(s)\n", agent.c_str(), local_domain, radio_domain, publishers.size() / 2);
  std::fflush(stdout);

  rclcpp::ExecutorOptions radio_exec_options;
  radio_exec_options.context = radio_ctx;
  rclcpp::executors::SingleThreadedExecutor radio_exec(radio_exec_options);
  radio_exec.add_node(radio);
  std::thread radio_thread([&radio_exec]() {radio_exec.spin();});

  rclcpp::ExecutorOptions local_exec_options;
  local_exec_options.context = local_ctx;
  rclcpp::executors::SingleThreadedExecutor local_exec(local_exec_options);
  local_exec.add_node(local);
  local_exec.spin();  // returns when a signal shuts the contexts down

  radio_exec.cancel();
  radio_thread.join();
  rclcpp::shutdown(local_ctx);
  rclcpp::shutdown(radio_ctx);
  return 0;
}
