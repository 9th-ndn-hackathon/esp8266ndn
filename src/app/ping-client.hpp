#ifndef ESP8266NDN_PING_CLIENT_HPP
#define ESP8266NDN_PING_CLIENT_HPP

#include "../core/face.hpp"

namespace ndn {

/** \brief NDN reachability test tool client side
 */
class PingClient
{
public:
  /** \brief constructor
   *  \param face a face to communication with router
   *  \param interest a prepared Interest for probe packets;
   *         typically its name should end with 'ping', and it should have MustBeFresh selector;
   *         the name may contain an initial sequence number,
   *         otherwise it must have space to add a sequence number
   *  \param pingInterval interval between probes, in millis
   *  \param pingTimeout probe timeout, in millis; default is InterestLifetime;
   *         must be less than \p pingInterval
   */
  PingClient(Face& face, InterestLite& interest, int pingInterval, int pingTimeout = -1);

  /** \brief loop the client
   */
  void
  loop();

  /** \brief process incoming Data
   *  \retval true Data is accepted by this client
   *  \retval false Data should be passed to the next consumer
   */
  bool
  processData(const DataLite& data);

  /** \brief process incoming Nack
   *  \retval true Nack is accepted by this client
   *  \retval false Nack should be passed to the next consumer
   */
  bool
  processNack(const NetworkNackLite& nack, const InterestLite& interest);

  /** \brief send a probe now
   *  \note Response or timeout for previous probe will be ignored.
   */
  bool
  probe();

  enum class Event {
    NONE,
    PROBE,
    RESPONSE,
    TIMEOUT,
    NACK,
  };

  typedef void (*EventCallback)(void* arg, Event evt, uint64_t seq);

  /** \brief set event handler
   *
   *  Only one handler is allowed. This overwrites any previous handler setting.
   */
  void
  onEvent(EventCallback cb, void* cbarg)
  {
    m_evtCb = cb;
    m_evtCbArg = cbarg;
  }

private:
  uint32_t
  getLastSeq() const;

private:
  Face& m_face;
  InterestLite& m_interest;
  uint8_t m_seqBuf[9]; ///< buffer for sequence number component
  const int m_pingInterval;
  const int m_pingTimeout;
  unsigned long m_lastProbe; ///< timestamp of last probe
  bool m_isPending; ///< whether lastProbe is waiting for either response or timeout
  EventCallback m_evtCb;
  void* m_evtCbArg;
};

} // namespace ndn

#endif // ESP8266NDN_PING_CLIENT_HPP