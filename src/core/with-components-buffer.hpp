#ifndef ESP8266NDN_WITH_COMPONENTS_BUFFER_HPP
#define ESP8266NDN_WITH_COMPONENTS_BUFFER_HPP

#include "../ndn-cpp/lite/data-lite.hpp"
#include "../ndn-cpp/lite/interest-lite.hpp"
#include "../ndn-cpp/lite/key-locator-lite.hpp"
#include "../ndn-cpp/lite/name-lite.hpp"
#include "../ndn-cpp/lite/signature-lite.hpp"

#include <array>

namespace ndn {
namespace detail {

template<typename Base, int MaxComps>
class SingleWCB : public Base
{
public:
  SingleWCB()
    : Base(m_comps.data(), m_comps.size())
  {
  }

private:
  std::array<ndn_NameComponent, MaxComps> m_comps;
};

} // namespace detail

/** \brief Name with components buffer.
 */
template<int MaxComps>
using NameWCB = detail::SingleWCB<NameLite, MaxComps>;

/** \brief Name with allocated components buffer.
 */
class NameACB : public NameLite
{
public:
  explicit
  NameACB(size_t maxComps)
    : NameLite(new ndn_NameComponent[maxComps], maxComps)
  {
  }

  ~NameACB()
  {
    delete[] components;
  }
};

/** \brief Signature with components buffer.
 */
template<int MaxComps>
using SignatureWCB = detail::SingleWCB<SignatureLite, MaxComps>;

/** \brief KeyLocator with components buffer.
 */
template<int MaxComps>
using KeyLocatorWCB = detail::SingleWCB<KeyLocatorLite, MaxComps>;

/** \brief Interest with components buffer.
 */
template<int MaxComps, int MaxKeyComps>
class InterestWCB : public InterestLite
{
public:
  InterestWCB()
    : InterestLite(m_comps.data(), m_comps.size(), nullptr, 0, m_keyComps.data(), m_keyComps.size())
  {
  }

private:
  std::array<ndn_NameComponent, MaxComps> m_comps;
  std::array<ndn_NameComponent, MaxKeyComps> m_keyComps;
};

/** \brief Data with components buffer.
 */
template<int MaxComps, int MaxKeyComps>
class DataWCB : public DataLite
{
public:
  DataWCB()
    : DataLite(m_comps.data(), m_comps.size(), m_keyComps.data(), m_keyComps.size())
  {
  }

private:
  std::array<ndn_NameComponent, MaxComps> m_comps;
  std::array<ndn_NameComponent, MaxKeyComps> m_keyComps;
};

/** \brief Data with allocated components buffer.
 */
class DataACB : public DataLite
{
public:
  explicit
  DataACB(size_t maxComps, size_t maxKeyComps)
    : DataLite(new ndn_NameComponent[maxComps], maxComps, new ndn_NameComponent[maxKeyComps], maxKeyComps)
  {
  }

  ~DataACB()
  {
    delete[] getName().components;
    delete[] getSignature().getKeyLocator().getKeyName().components;
  }
};

} // namespace ndn

#endif // ESP8266NDN_WITH_COMPONENTS_BUFFER_HPP
