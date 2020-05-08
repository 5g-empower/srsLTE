#include "srsenb/hdr/agent.h"
#include "srsenb/hdr/enb.h"

#include "srslte/interfaces/enb_rrc_interface_types.h"
#include "srslte/interfaces/sched_interface.h"

#include <empoweragentproto/empoweragentproto.hh>

#include <iostream>
#include <thread>
#include <vector>
#include <chrono>

namespace Empower {
namespace Agent {

agent::agent(srslte::logger* logger_) : logger(logger_) {}

agent::~agent() {}

bool agent::init(const srsenb::all_args_t& all_args, const srsenb::rrc_cfg_t& rrc_cfg, srsenb::stack_interface_agent * stack_)
{

    agent_log.init("AGENT", logger);
    agent_log.set_level(srslte::LOG_LEVEL_INFO);
    agent_log.set_hex_limit(32);

    address = NetworkLib::IPv4Address(all_args.agent.address);
    port    = all_args.agent.port;
    delay   = all_args.agent.delay;

    // Configure the TCP connection destination, and the delay/timeout
    io.address(address).port(port).delay(delay);

    // Take the pci
    pci = all_args.stack.s1ap.cell_id;

    // Take the enb_id
    enb_id = all_args.stack.s1ap.enb_id;

    // srsenb only supports one cell
    std::vector<srsenb::cell_cfg_t>::iterator it;
    srsenb::cell_list_t cell_list = rrc_cfg.cell_list;

    for (it = cell_list.begin(); it != cell_list.end(); it++) {
        cell_t cell;
        cell.pci = it->cell_id;
        cell.n_prb = all_args.enb.n_prb;
        cell.dl_earfcn = it->dl_earfcn;
        cell.dl_prbs_counter = 0;
        cell.ul_earfcn = it->ul_earfcn;
        cell.ul_prbs_counter = 0;
        cells.insert(std::make_pair(pci, cell));
    }

    // Initialize the sequence number to be used when sending messages
    sequence = 1;

    // Set pointer to Stack
    stack = stack_;

    return false;

}

void agent::add_user(uint64_t imsi, uint32_t tmsi, uint16_t rnti) {
    std::map<uint16_t, user_t>::iterator it;
    mtx.lock();
    it = users.find(rnti);
    agent_log.info("Adding new user (imsi: %lu, tmsi: %u, rnti: %u)", imsi, tmsi, rnti);
    if(it == users.end()) {
        user_t user;
        user.cell = &cells[pci];
        user.imsi = imsi;
        user.rnti = rnti;
        user.status = AGENT_USER_STATUS_CONNECTED;
        user.tmsi = tmsi;
        user.next_meas_id = 1;
        users.insert(std::make_pair(rnti, user));
        it = users.find(rnti);
    }
    it->second.status = AGENT_USER_STATUS_CONNECTED;
    send_ue_reports_response(0, it->second.rnti);
    mtx.unlock();
}

void agent::rem_user(uint16_t rnti) {
    std::map<uint16_t, user_t>::iterator it;
    mtx.lock();
    it = users.find(rnti);
    agent_log.info("Removing user (rnti: %u)", rnti);
    if(it != users.end()) {
        it->second.status = AGENT_USER_STATUS_DISCONNECTED;
        send_ue_reports_response(0, it->second.rnti);
        users.erase(it);
    }
    mtx.unlock();
}

void agent::update_dl_mac_prb_utilization_report(srsenb::sched_interface::dl_sched_res_t * sched_result) {

    uint32_t i;
    uint32_t prbs = 0;

    for (i = 0; i < sched_result->nof_bc_elems; i++) {
        prbs += dl_prbs_from_dci(&sched_result->bc[i].dci, cells[pci].n_prb);
    }

    for (i = 0; i < sched_result->nof_rar_elems; i++) {
        prbs += dl_prbs_from_dci(&sched_result->rar[i].dci, cells[pci].n_prb);
    }

    for (i = 0; i < sched_result->nof_data_elems; i++) {
        prbs += dl_prbs_from_dci(&sched_result->data[i].dci, cells[pci].n_prb);
    }

    mtx.lock();
    cells[pci].dl_prbs_counter +=prbs;
    mtx.unlock();

}

uint32_t agent::dl_prbs_from_dci(srslte_dci_dl_t* dci, uint32_t prbs) {

    if (dci->alloc_type == SRSLTE_RA_ALLOC_TYPE0) {
        return prbs_from_mask(dci->alloc_type, dci->type0_alloc.rbg_bitmask, prbs);
    } else if (dci->alloc_type == SRSLTE_RA_ALLOC_TYPE1) {
        return prbs_from_mask(dci->alloc_type, dci->type1_alloc.vrb_bitmask, prbs);
    } else {
        return prbs_from_mask(dci->alloc_type, dci->type2_alloc.riv, prbs);
    }

}

void agent::update_ul_mac_prb_utilization_report(srsenb::sched_interface::ul_sched_res_t * sched_result) {

    uint32_t i;
    uint32_t prbs = 0;

    for (i = 0; i < sched_result->nof_dci_elems; i++) {
        prbs += ul_prbs_from_dci(&sched_result->pusch[i].dci, cells[pci].n_prb);
    }

    mtx.lock();
    cells[pci].ul_prbs_counter +=prbs;
    mtx.unlock();

}

uint32_t agent::ul_prbs_from_dci(srslte_dci_ul_t* dci, uint32_t prbs) {

    return prbs_from_mask(SRSLTE_RA_ALLOC_TYPE2, dci->type2_alloc.riv, prbs);

}

uint32_t agent::prbs_from_mask(srslte_ra_type_t ra_format, uint32_t mask, uint32_t prbs) {

    int ret = 0;
    uint32_t i;

    switch (ra_format) {

    case SRSLTE_RA_ALLOC_TYPE0:
        for (i = 0; i < sizeof(uint32_t) * 8; i++) {
            if (mask & (1 << i)) {
                ret += srslte_ra_type0_P(prbs);
            }
        }
        break;

    case SRSLTE_RA_ALLOC_TYPE1:
        for (i = 0; i < sizeof(uint32_t) * 8; i++) {
            if (mask & (1 << i)) {
                ret += 1;
            }
        }
        break;

    case SRSLTE_RA_ALLOC_TYPE2:
        ret = (int) floor((double) mask / (double) prbs) + 1;
        break;
    }

    return ret;

}

bool agent::start()
{

    pthread_attr_t     attr;
    struct sched_param param;
    param.sched_priority = 0;
    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_OTHER);
    pthread_attr_setschedparam(&attr, &param);

    // Start the agent thread, executing `main_loop()` via helper function `main_loop_helper()`.
    if (pthread_create(&(agent_thread), &attr, main_loop_helper, reinterpret_cast<void*>(this))) {
        std::cerr << "AGENT: *** error starting agent main loop\n";
        return true;
    }

    // Start the hello thread, executing `hello_loop()` via helper function `hello_loop_helper()`.
    if (pthread_create(&(hello_thread), &attr, hello_loop_helper, reinterpret_cast<void*>(this))) {
        std::cerr << "AGENT: *** error starting agent hello loop\n";
        return true;
    }

    // No errors
    return false;

}

void* agent::main_loop_helper(void* arg)
{
  agent* this_instance = reinterpret_cast<agent*>(arg);
  this_instance->main_loop();
  return nullptr;
}

void* agent::hello_loop_helper(void* arg)
{
  agent* this_instance = reinterpret_cast<agent*>(arg);
  this_instance->hello_loop();
  return nullptr;
}

void agent::send_hello_request(uint32_t xid)
{

    try {

        // Allocate a couple of buffers to read and write messages, and obtain
        // a writable view on them.
        NetworkLib::BufferWritableView writeBuffer =
            io.makeMessageBuffer();

        MessageEncoder messageEncoder(writeBuffer);

        fill_header(messageEncoder.header());

        messageEncoder.header()
            .messageClass(MessageClass::REQUEST_SET)
            .entityClass(EntityClass::HELLO_SERVICE)
            .transactionId(xid);

        TLVPeriodicityMs tlvPeriodicity;
        tlvPeriodicity.milliseconds(io.delay());

        // Add the period TLV to the message
        messageEncoder.add(tlvPeriodicity).end();

        // Send the HELLO_SERVICE Request message
        size_t len = io.writeMessage(messageEncoder.data());

        agent_log.debug("Sending hello request message (%lu bytes)", len);

    } catch (std::exception& e) {
        agent_log.error("Error while sending message (%s)", e.what());
    }

}

void agent::send_caps_response(uint32_t xid)
{

    try {

        // Allocate a couple of buffers to read and write messages, and obtain
        // a writable view on them.
        NetworkLib::BufferWritableView writeBuffer =
            io.makeMessageBuffer();

        MessageEncoder messageEncoder(writeBuffer);

        fill_header(messageEncoder.header());
        messageEncoder.header()
            .messageClass(MessageClass::RESPONSE_SUCCESS)
            .entityClass(EntityClass::CAPABILITIES_SERVICE)
            .transactionId(xid);

        std::map<uint16_t, cell_t>::iterator it;

        for (it = cells.begin(); it != cells.end(); it++) {
            // Add the cells TLV to the message
            TLVCell tlvCell;
            tlvCell
              .pci(it->second.pci)
              .nPrb(it->second.n_prb)
              .dlEarfcn(it->second.dl_earfcn)
              .ulEarfcn(it->second.ul_earfcn);
            messageEncoder.add(tlvCell);
        }

        messageEncoder.end();

        // Send the CAP Response massage
        size_t len = io.writeMessage(messageEncoder.data());

        agent_log.debug("Sending cap response message (%lu bytes)", len);

    } catch (std::exception& e) {
        agent_log.error("Error while sending message (%s)", e.what());
    }

}

void agent::send_mac_prb_utilization_report(uint32_t xid)
{

    try {

        // Allocate a couple of buffers to read and write messages, and obtain
        // a writable view on them.
        NetworkLib::BufferWritableView writeBuffer = io.makeMessageBuffer();

        MessageEncoder messageEncoder(writeBuffer);

        fill_header(messageEncoder.header());
        messageEncoder.header()
            .messageClass(MessageClass::RESPONSE_SUCCESS)
            .entityClass(EntityClass::MAC_PRB_UTILIZATION_SERVICE)
            .transactionId(xid);

        std::map<uint16_t, cell_t>::iterator it;

        for (it = cells.begin(); it != cells.end(); it++) {
            // Add the UE Report TLV to the message
            TLVMACPrbReportReport tlvMacPrbsUtilizationReport;
            tlvMacPrbsUtilizationReport
              .pci(it->second.pci)
              .nPrb(it->second.n_prb)
              .dlPrbCounters(it->second.dl_prbs_counter)
              .ulPrbCounters(it->second.ul_prbs_counter);
            messageEncoder.add(tlvMacPrbsUtilizationReport);
        }

        messageEncoder.end();

        // Send the UE_REPORTS_SERVICE Response massage
        size_t len = io.writeMessage(messageEncoder.data());

        // Send the UE Reports message
        agent_log.debug("Sending prb response message(%lu bytes)", len);

    } catch (std::exception& e) {
        agent_log.error("Error while sending message (%s)", e.what());
    }

}

void agent::send_ue_reports_response(uint32_t xid)
{

    try {

        // Allocate a couple of buffers to read and write messages, and obtain
        // a writable view on them.
        NetworkLib::BufferWritableView writeBuffer = io.makeMessageBuffer();

        MessageEncoder messageEncoder(writeBuffer);

        fill_header(messageEncoder.header());
        messageEncoder.header()
            .messageClass(MessageClass::RESPONSE_SUCCESS)
            .entityClass(EntityClass::UE_REPORTS_SERVICE)
            .transactionId(xid);

        std::map<uint16_t, user_t>::iterator it;

        for (it = users.begin(); it != users.end(); it++) {
            // Add the UE Report TLV to the message
            TLVUEReport tlvUeReport;
            tlvUeReport
              .imsi(it->second.imsi)
              .tmsi(it->second.tmsi)
              .rnti(it->second.rnti)
              .pci(it->second.cell->pci)
              .status(it->second.status);
            messageEncoder.add(tlvUeReport);
        }

        messageEncoder.end();

        // Send the UE_REPORTS_SERVICE Response massage
        size_t len = io.writeMessage(messageEncoder.data());

        // Send the UE Reports message
        agent_log.debug("Sending ue reports message (%lu bytes)", len);

    } catch (std::exception& e) {
        agent_log.error("Error while sending message (%s)", e.what());
    }

}

void agent::send_ue_reports_response(uint32_t xid, uint16_t rnti)
{

    try {

        // Allocate a couple of buffers to read and write messages, and obtain
        // a writable view on them.
        NetworkLib::BufferWritableView writeBuffer = io.makeMessageBuffer();

        MessageEncoder messageEncoder(writeBuffer);

        fill_header(messageEncoder.header());
        messageEncoder.header()
            .messageClass(MessageClass::RESPONSE_SUCCESS)
            .entityClass(EntityClass::UE_REPORTS_SERVICE)
            .transactionId(xid);

        std::map<uint16_t, user_t>::iterator it;

        it = users.find(rnti);

        // Add the UE Report TLV to the message
        TLVUEReport tlvUeReport;
        tlvUeReport
          .imsi(it->second.imsi)
          .tmsi(it->second.tmsi)
          .rnti(it->second.rnti)
          .pci(it->second.cell->pci)
          .status(it->second.status);

        messageEncoder.add(tlvUeReport);

        messageEncoder.end();

        // Send the UE_REPORTS_SERVICE Response massage
        size_t len = io.writeMessage(messageEncoder.data());

        // Send the UE Reports message
        agent_log.debug("Sending ue reports message (%lu bytes)", len);

    } catch (std::exception& e) {
        agent_log.error("Error while sending message (%s)", e.what());
    }

}

void agent::send_meas_id(uint16_t xid, uint16_t rnti, uint8_t meas_id)
{

    try {

        // Allocate a couple of buffers to read and write messages, and obtain
        // a writable view on them.
        NetworkLib::BufferWritableView writeBuffer = io.makeMessageBuffer();

        MessageEncoder messageEncoder(writeBuffer);

        MessageClass result = MessageClass::RESPONSE_SUCCESS;

        if (meas_id == 0) {
            result = MessageClass::RESPONSE_FAILURE;
        }

        fill_header(messageEncoder.header());
        messageEncoder.header()
            .messageClass(result)
            .entityClass(EntityClass::UE_MEASUREMENTS_SERVICE)
            .transactionId(xid);

        // Add the UE Report TLV to the message
        TLVUEMeasurementId tlvUeMeasurementId;
        tlvUeMeasurementId
          .rnti(rnti)
          .measId(meas_id);

        messageEncoder.add(tlvUeMeasurementId);

        messageEncoder.end();

        // Send the UE_MEASUREMENTS_SERVICE Response massage
        size_t len = io.writeMessage(messageEncoder.data());

        // Send the UE Reports message
        agent_log.debug("Sending meas id (%lu bytes)", len);

    } catch (std::exception& e) {
        agent_log.error("Error while sending message (%s)", e.what());
    }

}

void agent::send_meas_report(uint16_t rnti, uint8_t meas_id, uint8_t rsrp, uint8_t rsrq)
{

    try {

        // Allocate a couple of buffers to read and write messages, and obtain
        // a writable view on them.
        NetworkLib::BufferWritableView writeBuffer = io.makeMessageBuffer();

        MessageEncoder messageEncoder(writeBuffer);

        MessageClass result = MessageClass::RESPONSE_SUCCESS;

        if (meas_id == 0) {
            result = MessageClass::RESPONSE_FAILURE;
        }

        fill_header(messageEncoder.header());
        messageEncoder.header()
            .messageClass(result)
            .entityClass(EntityClass::UE_MEASUREMENTS_SERVICE)
            .transactionId(0);

        // Add the UE Report TLV to the message
        TLVUEMeasurementReport tlvUeMeasurementReport;
        tlvUeMeasurementReport
          .rnti(rnti)
          .measId(meas_id)
          .rsrp(rsrp)
          .rsrq(rsrq);

        messageEncoder.add(tlvUeMeasurementReport);

        messageEncoder.end();

        // Send the UE_MEASUREMENTS_SERVICE Response massage
        size_t len = io.writeMessage(messageEncoder.data());

        // Send the UE Reports message
        agent_log.debug("Sending meas report (%lu bytes)", len);

    } catch (std::exception& e) {
        agent_log.error("Error while sending message (%s)", e.what());
    }

}

uint8_t agent::add_meas(uint16_t rnti, uint8_t interval, uint8_t amount) {

    agent_log.info("Add new measurement (rnti=%u, interval=%u, amount=%u))", rnti, interval, amount);

    auto user_it = users.find(rnti);

    if (user_it == users.end()) {
        agent_log.error("Unable to find RNTI %u", rnti);
        return 0;
    }

    meas_cfg_t meas_cfg;

    meas_cfg.meas_id = user_it->second.next_meas_id++;
    meas_cfg.rnti = user_it->second.rnti;
    meas_cfg.amount = amount;
    meas_cfg.interval = interval;

    user_it->second.meas.insert(std::make_pair(meas_cfg.meas_id, meas_cfg));

    stack->rrc_meas_config_add(meas_cfg.rnti,
                               meas_cfg.meas_id,
                               user_it->second.cell->pci,
                               user_it->second.cell->dl_earfcn);

    return meas_cfg.meas_id;

}

uint8_t agent::rem_meas(uint16_t rnti, uint8_t meas_id) {

    agent_log.info("Rem measurement (rnti=%u, meas_id=%u))", rnti, meas_id);

    auto user_it = users.find(rnti);

    if (user_it == users.end()) {
        agent_log.error("Unable to find RNTI %u", rnti);
        return 0;
    }

    auto meas_it = user_it->second.meas.find(meas_id);

    if (meas_it == user_it->second.meas.end()) {
        agent_log.error("Unable to find Meas Id %u", meas_id);
        return 0;
    }

    user_it->second.meas.erase(meas_it);

    stack->rrc_meas_config_rem(rnti, meas_id);

    return meas_id;

}

void agent::handle_ue_meas_report(uint16_t rnti, const asn1::rrc::meas_report_s& msg)
{

    const asn1::rrc::meas_results_s& meas_res = msg.crit_exts.c1().meas_report_r8().meas_results;

    uint8_t meas_id = meas_res.meas_id;

    auto user_it = users.find(rnti);

    if (user_it == users.end()) {
        agent_log.error("Unable to find RNTI %u", rnti);
        return;
    }

    auto meas_it = user_it->second.meas.find(meas_id);

    if (meas_it == user_it->second.meas.end()) {
        agent_log.error("Unable to find Meas Id %u", meas_id);
        return;
    }

    uint8_t rsrp = meas_res.meas_result_pcell.rsrp_result;
    uint8_t rsrq = meas_res.meas_result_pcell.rsrq_result;

    agent_log.info("Measurement report (rnti=%u, meas_id=%u))", rnti, meas_id);

    send_meas_report(rnti, meas_id, rsrp, rsrq);

}

void agent::handle_incoming_message() {

    // Allocate a couple of buffers to read and write messages, and obtain
    // a writable view on them.
    NetworkLib::BufferWritableView readBuffer = io.makeMessageBuffer();
    NetworkLib::BufferWritableView writeBuffer = io.makeMessageBuffer();

    // Read a message
    auto messageBuffer = io.readMessage(readBuffer);

    // Buffer is empty
    if (messageBuffer.empty()) {
        return;
    }

    // Decode the message
    MessageDecoder messageDecoder(messageBuffer);

    // Unable to decode the message
    if (messageDecoder.isFailure()) {
        return;
    }

    uint32_t msg = unsigned(messageDecoder.header().entityClass());
    uint32_t seq = messageDecoder.header().sequence();
    uint32_t xid = messageDecoder.header().transactionId();

    agent_log.info("Received message type %u xid %u seq %u", msg, xid, seq);

    switch (messageDecoder.header().entityClass()) {
    case EntityClass::HELLO_SERVICE: {
        break;
    }
    case EntityClass::CAPABILITIES_SERVICE: {
        send_caps_response(xid);
        break;
    }
    case EntityClass::UE_REPORTS_SERVICE: {
        send_ue_reports_response(xid);
        break;
    }
    case EntityClass::MAC_PRB_UTILIZATION_SERVICE: {
        send_mac_prb_utilization_report(xid);
        break;
    }
    case EntityClass::UE_MEASUREMENTS_SERVICE: {
        // We assume that the message can have a single TLV
        if (messageDecoder.header().messageClass() == MessageClass::REQUEST_ADD) {
            TLVUEMeasurementConfig tlv;
            messageDecoder.get(tlv);
            uint8_t id = add_meas(tlv.rnti(), tlv.interval(), tlv.amount());
            send_meas_id(xid, tlv.rnti(), id);
        } else if (messageDecoder.header().messageClass() == MessageClass::REQUEST_DEL) {
            TLVUEMeasurementId tlv;
            messageDecoder.get(tlv);
            uint8_t id = rem_meas(tlv.rnti(), tlv.measId());
            send_meas_id(xid, tlv.rnti(), id);
        }
        break;
    }
    default:
        agent_log.error("Unexpected message class");
        break;
    }

}

void agent::main_loop() {
    try {
        for (;;) {
            if (io.isConnectionClosed()) {
                // Try to open the TCP connection to the controller
                io.openSocket();
            }
            // Now test again if the connection is still closed.
            if (io.isConnectionClosed()) {
                // Connection still closed. Sleep for a while.
                io.sleep();
                // Rinse and repeat
                continue;
            }
            if (io.isDataAvailable()) {
                mtx.lock();
                handle_incoming_message();
                mtx.unlock();
            }
        }
    } catch (std::exception &e) {
        std::cerr << "AGENT: *** caught exception in main agent loop: " << e.what() << '\n';
    }
}

void agent::hello_loop() {
    try {
        for (;;) {
            std::this_thread::sleep_for(std::chrono::milliseconds(delay));
            if (!io.isConnectionClosed()) {
                send_hello_request(0);
            }
        }
    } catch (std::exception &e) {
        std::cerr << "AGENT: *** caught exception in hello loop: " << e.what() << '\n';
    }
}

void agent::fill_header(CommonHeaderEncoder &headerEncoder) {
    headerEncoder.sequence(sequence).elementId(static_cast<std::uint64_t>(enb_id));
    ++sequence;
}

} // namespace Agent
} // namespace Empower
