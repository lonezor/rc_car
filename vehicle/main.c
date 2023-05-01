#include "vehicle.h"

//-------------------------------------------------------------------------------------------------------------------

void setup() {
#if defined(PCU_BUILD)
  pcu_setup();
#elif defined (SCU_BUILD)
  scu_setup();
#elif defined (VMU_BUILD)
  vmu_setup();
#endif
}

//-------------------------------------------------------------------------------------------------------------------

void loop() {
#if defined(PCU_BUILD)
  pcu_loop();
#elif defined (SCU_BUILD)
  scu_loop();
#elif defined (VMU_BUILD)
  vmu_loop();
#endif
}

//-------------------------------------------------------------------------------------------------------------------
