/* Stack configuration for host unit tests. TEST_CLASSIC builds classic CAN (CiA 301), otherwise CANopen FD. */

#ifndef CO_DRIVER_CUSTOM_H
#define CO_DRIVER_CUSTOM_H

#ifndef TEST_CLASSIC
#define CO_CONFIG_FD (CO_CONFIG_FD_ENABLE)
#endif

#define CO_CONFIG_PDO                  (CO_CONFIG_RPDO_ENABLE | CO_CONFIG_TPDO_ENABLE | CO_CONFIG_PDO_OD_IO_ACCESS)
#define CO_CONFIG_USDO_SRV             (CO_CONFIG_USDO_SRV_SEGMENTED | CO_CONFIG_FLAG_TIMERNEXT)
#define CO_CONFIG_USDO_SRV_BUFFER_SIZE 64

#endif /* CO_DRIVER_CUSTOM_H */
