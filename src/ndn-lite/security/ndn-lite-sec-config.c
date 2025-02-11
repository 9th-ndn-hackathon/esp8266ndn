/*
 * Copyright (C) 2018-2019 Zhiyi Zhang, Edward Lu
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v3.0. See the file LICENSE in the top level
 * directory for more details.
 *
 * See AUTHORS.md for complete list of NDN IOT PKG authors and contributors.
 */

#include "ndn-lite-sec-config.h"
#include "ndn-lite-rng.h"
#include "ndn-lite-ecc.h"

void (*platform_security_init)(void) = NULL;

void
register_platform_security_init(void (*init)(void)) {
  platform_security_init = init;
}

void
ndn_security_init(void)
{
  // SHA256 backend
  ndn_lite_default_sha_load_backend();

  // RNG backend
  // doing nothing

  // AES backend
  ndn_lite_default_aes_load_backend();

  // ECC backend
  ndn_lite_default_ecc_load_backend();

  // HMAC backend
  ndn_lite_default_hmac_load_backend();

  if (platform_security_init != NULL) {
    platform_security_init();
  }
  ndn_rng_backend_t* rng_backend = ndn_rng_get_backend();
  if (rng_backend) {
    ndn_ecc_set_rng(rng_backend->rng);
  }
}
