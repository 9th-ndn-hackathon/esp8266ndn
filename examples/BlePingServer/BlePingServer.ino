#ifdef ESP32
// extras/BlePingClient.py is a client that can connect to this ndnping server.

#include <esp8266ndn.h>

char PREFIX[] = "/example/esp32/ble/ping";

ndn::BleServerTransport g_transport;
ndn::Face g_face(g_transport);
ndn::DigestKey g_pvtkey;

ndn_NameComponent g_comps[4];
ndn::NameLite g_prefix(g_comps, 4);
ndn::PingServer g_server(g_face, g_prefix);

void
makePayload(void* arg, const ndn::InterestLite& interest, uint8_t* payloadBuf, size_t* payloadSize)
{
  auto text = reinterpret_cast<const char*>(arg);
  size_t len = strlen(text);
  memcpy(payloadBuf, text, len);
  *payloadSize = len;
}

void
setup()
{
  Serial.begin(115200);
  Serial.println();
  ndn::setLogOutput(Serial);

  // BLE MTU is 512, no need for the default 1500-octet buffer
  {
    ndn::PacketBuffer::Options opt;
    opt.maxSize = 512;
    opt.maxNameComps = 8;
    opt.maxKeyNameComps = 8;
    auto newPb = new ndn::PacketBuffer(opt);
    auto oldPb = g_face.swapPacketBuffer(newPb);
    if (oldPb != nullptr) {
      delete oldPb;
    }
  }
  g_transport.begin("ESP32-BLE-NDN");
  g_face.setSigningKey(g_pvtkey);

  ndn::parseNameFromUri(g_prefix, PREFIX);
  g_server.onProbe(&makePayload, const_cast<void*>(reinterpret_cast<const void*>("PingServer")));
}

void
loop()
{
  g_face.loop();
  delay(100);
}

#else // ESP32

void
setup()
{
}

void
loop()
{
}

#endif // ESP32