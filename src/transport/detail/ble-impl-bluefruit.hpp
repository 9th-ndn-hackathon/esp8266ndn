#ifndef ESP8266NDN_BLE_IMPL_BLUEFRUIT_HPP
#define ESP8266NDN_BLE_IMPL_BLUEFRUIT_HPP

#include "ble-uuid.hpp"
#include <bluefruit.h>
#include "../../core/detail/fix-maxmin.hpp"

namespace ndn {

class BleClientImpl;

class BleDeviceImplClass
{
public:
  int
  init(const char* deviceName, bool enableServer, int nClients)
  {
    Bluefruit.configPrphConn(BLE_GATT_ATT_MTU_MAX, BLE_GAP_EVENT_LENGTH_DEFAULT, BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT);
    Bluefruit.configCentralConn(BLE_GATT_ATT_MTU_MAX, BLE_GAP_EVENT_LENGTH_DEFAULT, BLE_GATTS_HVN_TX_QUEUE_SIZE_DEFAULT, BLE_GATTC_WRITE_CMD_TX_QUEUE_SIZE_DEFAULT);
    VERIFY_STATUS(Bluefruit.begin(static_cast<int>(enableServer), nClients), __LINE__);
    Bluefruit.setName(deviceName);
    Bluefruit.setTxPower(4);
    return 0;
  }

  String
  getAddr()
  {
    uint8_t mac[6]; // reversed
    uint8_t addrType = Bluefruit.Gap.getAddr(mac);
    char addr[18];
    snprintf(addr, sizeof(addr), "%02X:%02X:%02X:%02X:%02X:%02X", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
    if (addrType == BLE_GAP_ADDR_TYPE_RANDOM_STATIC) {
      return String(addr) + " (addr-type=random-static)";
    }
    return String(addr) + " (addr-type=" + static_cast<int>(addrType) + ")";
  }

  int
  startScanConnect(BleClientImpl& client);

private:
  static void
  scanCb(ble_gap_evt_adv_report_t* report);

  static void
  connectCb(uint16_t connHandle);

private:
  BleClientImpl* m_client = nullptr;
};

extern BleDeviceImplClass BleDeviceImpl;

class BleServiceImpl : public BLEService
{
public:
  BleServiceImpl()
    : BLEService(BLE_UUID_SVC)
    , m_rx(BLE_UUID_RX)
    , m_tx(BLE_UUID_TX)
  {
  }

  err_t
  begin() override
  {
    VERIFY_STATUS(this->BLEService::begin(), __LINE__);

    uint16_t mtu = Bluefruit.Gap.getMaxMtuByConnCfg(CONN_CFG_PERIPHERAL);

    m_rx.setProperties(CHR_PROPS_WRITE | CHR_PROPS_NOTIFY);
    m_rx.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    m_rx.setMaxLen(mtu);
    m_rx.setUserDescriptor("NDN-RX");
    VERIFY_STATUS(m_rx.begin(), __LINE__);

    m_tx.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
    m_tx.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
    m_tx.setMaxLen(mtu);
    m_tx.setUserDescriptor("NDN-TX");
    VERIFY_STATUS(m_tx.begin(), __LINE__);

    return 0;
  }

  int
  advertise()
  {
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addService(*this);
    Bluefruit.Advertising.restartOnDisconnect(true);
    if (!Bluefruit.Advertising.start()) {
      return __LINE__;
    }
    return 0;
  }

  size_t
  receive(uint8_t* buf, size_t bufSize)
  {
    uint16_t len = m_rx.read(buf, bufSize);
    m_rx.write("", 0);
    return len;
  }

  bool
  send(const uint8_t* pkt, size_t len)
  {
    m_tx.write(pkt, len);
    m_tx.notify(pkt, len);
    return true;
  }

private:
  BLECharacteristic m_rx;
  BLECharacteristic m_tx;
};

class BleClientImpl : public BLEClientService
{
public:
  BleClientImpl()
    : BLEClientService(BLE_UUID_SVC)
    , m_rx(BLE_UUID_TX)
    , m_tx(BLE_UUID_RX)
  {
  }

  bool
  begin() override
  {
    this->BLEClientService::begin();
    m_rx.begin(this);
    m_tx.begin(this);
    m_rx.setNotifyCallback(&BleClientImpl::rxNotify);
    return true;
  }

  bool
  discover(uint16_t connHandle) override
  {
    VERIFY(this->BLEClientService::discover(connHandle));
    _conn_hdl = BLE_CONN_HANDLE_INVALID;
    VERIFY(2 == Bluefruit.Discovery.discoverCharacteristic(connHandle, m_rx, m_tx));
    VERIFY(m_rx.enableNotify());
    _conn_hdl = connHandle;
    return true;
  }

  size_t
  receive(uint8_t* buf, size_t bufSize)
  {
    uint16_t len = m_rxLen;
    memcpy(buf, m_rxData, std::min<uint16_t>(bufSize, len));
    m_rxLen = 0;
    return len;
  }

  bool
  send(const uint8_t* pkt, size_t len)
  {
    if (!this->discovered()) {
      return false;
    }
    m_tx.write(pkt, len);
    return true;
  }

private:
  static void
  rxNotify(BLEClientCharacteristic* rx, uint8_t* data, uint16_t len)
  {
    BleClientImpl& client = static_cast<BleClientImpl&>(rx->parentService());
    memcpy(client.m_rxData, data, len);
    client.m_rxLen = len;
  }

private:
  BLEClientCharacteristic m_rx;
  BLEClientCharacteristic m_tx;
  uint16_t m_rxLen = 0;
  uint8_t m_rxData[BLE_GATT_ATT_MTU_MAX];
};

inline int
BleDeviceImplClass::startScanConnect(BleClientImpl& client)
{
  if (m_client != nullptr) {
    return __LINE__; // only support one client
  }
  m_client = &client;

  Bluefruit.Central.setConnectCallback(&BleDeviceImplClass::connectCb);
  Bluefruit.Scanner.setRxCallback(&BleDeviceImplClass::scanCb);

  Bluefruit.Scanner.clearFilters();
  Bluefruit.Scanner.restartOnDisconnect(true);
  Bluefruit.Scanner.setInterval(160, 80);
  Bluefruit.Scanner.filterService(client);
  Bluefruit.Scanner.useActiveScan(false);
  Bluefruit.Scanner.start(0);
  return 0;
}

inline void
BleDeviceImplClass::scanCb(ble_gap_evt_adv_report_t* report)
{
  Bluefruit.Central.connect(report);
}

inline void
BleDeviceImplClass::connectCb(uint16_t connHandle)
{
  BleClientImpl* client = BleDeviceImpl.m_client;
  if (client != nullptr && client->discover(connHandle)) {
    return;
  }
  Bluefruit.Central.disconnect(connHandle);
}

} // namespace ndn

#endif // ESP8266NDN_BLE_IMPL_BLUEFRUIT_HPP
