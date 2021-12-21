#ifndef I2C_WRAPPER_H
#define I2C_WRAPPER_H

#include <DebugConfig.h>
#include <HardwareConfig.h>

class I2CWrapper {
    
    public:

    /* init both devices but not the TW bus */
    static void init(void);
};

extern I2CWrapper i2CWrapper;

#endif //I2C_WRAPPER_H