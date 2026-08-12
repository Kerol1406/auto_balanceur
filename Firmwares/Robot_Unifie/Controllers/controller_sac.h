/*
 * controller_sac.h
 *
 *  Created on: Aug 7, 2026
 *      Author: mr_sun
 */

#ifndef CONTROLLER_SAC_H_
#define CONTROLLER_SAC_H_

#include "controller_config.h"

#if TYPE_CONTROLLER == 3

#include "controller_common.h"
#include "policy_sac.h"

float relu(float x);


#endif /* CONTROLLER_SAC_H_ */
#endif /* TYPE_CONTROLLER == 3 */


