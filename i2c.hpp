#ifndef I2C_H
#define I2C_H

#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/param.h>	/* for NAME_MAX */
#include <sys/ioctl.h>
#include <string.h>
#include <strings.h>	/* for strcasecmp() */
#include <limits.h>
#include <dirent.h>
#include <fcntl.h>
#include <errno.h>
#include <QMutex>


#define     HW_I2C_DEV                      0x00
#define     ENCODER_BOARD_HW_BASE_ADDR      0x67

class i2c
{
public:
    i2c();

    void i2c_setDevice(int i2c_dev);
    int  i2c_read(int hw_address, int address);
    int  i2c_readWord(int hw_address, int address);
    int  i2c_read_continuous(int hw_address, int address, char* buf, size_t size);
    int  i2c_write(int hw_address, int address, int data);
    int  i2c_writeWord(char hw_address, char address, int16_t data);

private:
    QMutex mI2cMutex;
    int m_i2c_dev;

    int open_i2c_dev(int i2cbus, char *filename, size_t size, int quiet);
    __s32 i2c_smbus_access(int file, char read_write, __u8 command, int size, union i2c_smbus_data *data);
    __s32 i2c_smbus_read_byte_data(int file, __u8 command);
    __s32 i2c_smbus_read_word_data(int file, __u8 command);
    __s32 i2c_smbus_write_byte_data(int file, __u8 command, __u8 value);
    __s32 i2c_smbus_write_word_data(int file, __u8 command, __u16 value);

};

#endif // I2C_H
