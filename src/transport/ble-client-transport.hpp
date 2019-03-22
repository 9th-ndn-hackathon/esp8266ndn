#ifndef ESP8266NDN_BLE_CLIENT_TRANSPORT_HPP
#define ESP8266NDN_BLE_CLIENT_TRANSPORT_HPP

#include "transport.hpp"
#include <memory>

namespace ndn {

class BleClientImpl;

/** \brief a transport that acts as a Bluetooth Low Energy client
 */
class BleClientTransport : public Transport
{
public:
  BleClientTransport();

  ~BleClientTransport();

  bool
  begin();

  size_t
  receive(uint8_t* buf, size_t bufSize, uint64_t& endpointId) final;

  ndn_Error
  send(const uint8_t* pkt, size_t len, uint64_t endpointId) final;

private:
  std::unique_ptr<BleClientImpl> m_impl;
};

} // namespace ndn

#endif // ESP8266NDN_BLE_CLIENT_TRANSPORT_HPP
