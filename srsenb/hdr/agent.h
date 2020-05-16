#ifndef EMPOWER_AGENT_H
#define EMPOWER_AGENT_H

#include <memory>
#include <iostream>
#include <map>
#include <mutex>
#include <vector>
#include <chrono>
#include <thread>

#include <empoweragentproto/empoweragentproto.hh>

#include "srslte/common/log.h"
#include "srslte/common/logger_file.h"
#include "srslte/common/log_filter.h"

#include "srslte/interfaces/enb_interfaces.h"

// Forward declaration of the struct holding the configuration read
// from the cfg file or from the command line.
namespace srsenb {
struct all_args_t;
struct rrc_cfg_t;
}

namespace Empower {
namespace Agent {

const int MAX_CELLS = 4;

enum user_status {
    AGENT_USER_STATUS_CONNECTED = 1,
    AGENT_USER_STATUS_DISCONNECTED = 2
};

typedef struct {
    uint16_t rnti;
    uint8_t meas_id;
    asn1::rrc::report_interv_e interval;
    asn1::rrc::report_cfg_eutra_s::report_amount_e_ amount;
} meas_cfg_t;

typedef struct {
    uint16_t pci;
    uint32_t dl_earfcn;
    uint32_t ul_earfcn;
    uint8_t n_prb;
    uint32_t dl_prbs_counter;
    uint32_t ul_prbs_counter;
} cell_t;

typedef struct {
  uint64_t imsi;
  uint32_t tmsi;
  uint16_t rnti;
  cell_t *cell;
  user_status status;
  uint8_t next_meas_id;
  std::map<uint8_t, meas_cfg_t> meas;
} user_t;

// Forward declaration
class CommonHeaderEncoder;

class agent : public srsenb::agent_interface_rrc,
              public srsenb::agent_interface_mac
{
public:
  agent(srslte::logger* logger_);
  ~agent();

  // Initialize the agent
  bool init(const srsenb::all_args_t& all_args, const srsenb::rrc_cfg_t& rrc_cfg, srsenb::stack_interface_agent * stack_);

  // Start the agent threads.
  bool start();

  void add_user(uint64_t imsi, uint32_t tmsi, uint16_t rnti);
  void rem_user(uint16_t rnti);
  void handle_ue_meas_report(uint16_t rnti, const asn1::rrc::meas_report_s& msg);

  void update_dl_mac_prb_utilization_report(srsenb::sched_interface::dl_sched_res_t * sched);
  void update_ul_mac_prb_utilization_report(srsenb::sched_interface::ul_sched_res_t * sched);

  // The logger
  srslte::logger* logger = nullptr;
  srslte::log_filter agent_log;

private:

  // Thread helper methods
  static void* hello_loop_helper(void* arg);
  static void* main_loop_helper(void* arg);

  // The agent main loop.
  void main_loop();

  // The hello loop.
  void hello_loop();

  // Send CAP response.
  void send_hello_request(uint32_t xid);

  // Send CAPs response.
  void send_caps_response(uint32_t xid);

  // Send UE Reports
  void send_ue_reports_response(uint32_t xid);
  void send_ue_reports_response(uint32_t xid, uint16_t rnti);

  // Send MAC PRBs utilization reports
  void send_mac_prb_utilization_report(uint32_t xid);

  // Send the id of the just created measurement
  void send_meas_id(uint16_t xid, uint16_t rnti, uint8_t meas_id);

  // Send the the ue measurement report
  void send_meas_report(uint16_t rnti, uint8_t meas_id, uint8_t rsrp, uint8_t rsrq);

  // Add a new measurement
  uint8_t add_meas(uint16_t rnti, uint8_t amount, uint8_t interval);

  // Add a new measurement
  uint8_t rem_meas(uint16_t rnti, uint8_t meas_d);

  // Compute number of DL PRBs used from DCI
  uint32_t dl_prbs_from_dci(srslte_dci_dl_t* dci, uint32_t prbs);

  // Compute number of DL PRBs used from DCI
  uint32_t ul_prbs_from_dci(srslte_dci_ul_t* dci, uint32_t prbs);

  // Compute number of PRBs from bitmask
  uint32_t prbs_from_mask(srslte_ra_type_t ra_format, uint32_t mask, uint32_t prbs);

  // Parse incoming messages
  void handle_incoming_message();

  // The cell identifier
  std::uint16_t pci;

  // The list of cells
  std::map<uint16_t, cell_t> cells;

  /// The User Equipments
  std::map<uint16_t, user_t> users;

  // I/O Socket
  IO io;

  // Main agent thread
  pthread_t agent_thread;

  // Main agent thread
  pthread_t hello_thread;

  // The IPv4 address of the controller (to be contacted by the agent)
  NetworkLib::IPv4Address address;

  // The TCP port of the controller (to be contacted by the agent)
  std::uint16_t port;

  // The hello period
  std::uint32_t delay;

  /// The eNodeB identifier (from enb.enb_id)
  std::uint32_t enb_id;

  // The sequence number
  std::uint32_t sequence;

  // Mutex (general)
  std::mutex mtx;

  // Agent interface to Stack
  srsenb::stack_interface_agent * stack;

  // Fill the header of the messages being sent out
  void fill_header(CommonHeaderEncoder& headerEncoder);

};

} // namespace Agent
} // namespace Empower

#endif
