/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil -*- */
/**
 * Copyright (C) 2015-2019 Regents of the University of California.
 * @author: Jeff Thompson <jefft0@remap.ucla.edu>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version, with the additional exemption that
 * compiling, linking, and/or using OpenSSL is allowed.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * A copy of the GNU Lesser General Public License is in the file COPYING.
 */

#include "../c/control-parameters.h"
#include "../lite/control-parameters-lite.hpp"

namespace ndn {

ControlParametersLite::ControlParametersLite
  (struct ndn_NameComponent *nameComponents, size_t maxNameComponents,
   struct ndn_NameComponent *strategyNameComponents,
   size_t strategyMaxNameComponents)
{
  ndn_ControlParameters_initialize
    (this, nameComponents, maxNameComponents, strategyNameComponents,
     strategyMaxNameComponents);
}

ndn_Error
ControlParametersLite::set(const ControlParametersLite& other)
{
  return ndn_ControlParameters_setFromControlParameters(this, &other);
}

}
