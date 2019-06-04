#ifndef ESP8266NDN_ETHERNET_TRANSPORT_HPP
#define ESP8266NDN_ETHERNET_TRANSPORT_HPP

#if defined(ESP8266) || defined(ESP32)

#include "transport.hpp"
#include <memory>

extern "C" {
struct netif;
}
class Print;

namespace ndn {

/** \brief Receive queue length of EthernetTransport.
 */
static const int ETHTRANSPORT_RX_QUEUE_LEN = 4;

/** \brief a transport that communicates over Ethernet
 */
class EthernetTransport : public PollModeTransport
{
public:
  /** \brief interpretation of endpointId
   */
  union EndpointId {
    uint64_t endpointId;
    struct {
      uint8_t addr[6];
      bool isMulticast : 1; ///< RX only, ignored on TX
      uint16_t _a : 15;
    };
  };

  static void
  listNetifs(Print& os);

  EthernetTransport();

  ~EthernetTransport();

  /** \brief Start intercepting NDN packets on a network interface.
   *  \return whether success
   */
  bool
  begin(const char ifname[2], uint8_t ifnum);

  /** \brief Start intercepting NDN packets on any station interface.
   *  \return whether success
   */
  bool
  begin();

  void
  end();

  /** \begin receive a packet
   *  \param[out] endpointId identity of remote endpoint and whether packet was multicast
   */
  size_t
  receive(uint8_t* buf, size_t bufSize, uint64_t& endpointId) final;

  /** \begin transmit a packet
   *  \param endpointId identity of remote endpoint, zero for sending to multicast group
   */
  ndn_Error
  send(const uint8_t* pkt, size_t len, uint64_t endpointId) override;

private:
  bool
  begin(netif* netif);

private:
  class Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace ndn

#endif // defined(ESP8266) || defined(ESP32)

#endif // ESP8266NDN_ETHERNET_TRANSPORT_HPP
