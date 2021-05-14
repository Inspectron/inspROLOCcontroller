#include "i2c.hpp"

// static members
static QMutex mI2cMutex;

i2c::i2c()
{

}

void i2c::i2c_setDevice(int i2c_dev)
{
    m_i2c_dev = i2c_dev;
}

int i2c::open_i2c_dev(int i2cbus, char *filename, size_t size, int quiet)
{
    int file;

    snprintf(filename, size, "/dev/i2c-%d", i2cbus);
    filename[size - 1] = '\0';
    file = open(filename, O_RDWR);

    if (file < 0 && (errno == ENOENT || errno == ENOTDIR)) {
        sprintf(filename, "/dev/i2c-%d", i2cbus);
        file = open(filename, O_RDWR);
    }

    if (file < 0 && !quiet) {
        if (errno == ENOENT) {
            fprintf(stderr, "Error: Could not open file "
                "`/dev/i2c-%d' or `/dev/i2c/%d': %s\n",
                i2cbus, i2cbus, strerror(ENOENT));
        } else {
            fprintf(stderr, "Error: Could not open file "
                "`%s': %s\n", filename, strerror(errno));
            if (errno == EACCES)
                fprintf(stderr, "Run as root?\n");
        }
    }

    return file;
}

#if 0
int set_slave_addr(int file, int address, int force)
{
    /* With force, let the user read from/write to the registers
       even when a driver is also running */
    if (ioctl(file, force ? I2C_SLAVE_FORCE : I2C_SLAVE, address) < 0) {
        fprintf(stderr,
            "Error: Could not set address to 0x%02x: %s\n",
            address, strerror(errno));
        return -errno;
    }

    return 0;
}
#endif

__s32 i2c::i2c_smbus_access(int file, char read_write, __u8 command,
               int size, union i2c_smbus_data *data)
{
    struct i2c_smbus_ioctl_data args;
    __s32 err;

    args.read_write = read_write;
    args.command = command;
    args.size = size;
    args.data = data;

    err = ioctl(file, I2C_SMBUS, &args);
    if (err == -1)
    {
        perror("i2c_smbus_access ioctl");
        err = -errno;
    }
    return err;
}

__s32 i2c::i2c_smbus_read_byte_data(int file, __u8 command)
{
    union i2c_smbus_data data;
    int err;

    err = i2c_smbus_access(file, I2C_SMBUS_READ, command, I2C_SMBUS_BYTE_DATA, &data);
    if (err < 0)
        return err;

    return 0x0FF & data.byte;
}

__s32 i2c::i2c_smbus_read_word_data(int file, __u8 command)
{
    union i2c_smbus_data data;
    int err;

    err = i2c_smbus_access(file, I2C_SMBUS_READ, command, I2C_SMBUS_WORD_DATA, &data);
    if (err < 0)
        return err;

    return 0x0FFFF & data.word;
}

__s32 i2c::i2c_smbus_write_byte_data(int file, __u8 command, __u8 value)
{
    union i2c_smbus_data data;
    data.byte = value;
    return i2c_smbus_access(file, I2C_SMBUS_WRITE, command, I2C_SMBUS_BYTE_DATA, &data);
}

__s32 i2c::i2c_smbus_write_word_data(int file, __u8 command, __u16 value)
{
    union i2c_smbus_data data;
    data.word = value;
    return i2c_smbus_access(file, I2C_SMBUS_WRITE, command, I2C_SMBUS_WORD_DATA, &data);
}

int i2c::i2c_read(int hw_address, int address)
{
    int file;
    char filename[20];
    int res = 0;
    int reTries = 0;

    // equivalent to: i2cget -y 4 0x67 0x0 v
    mI2cMutex.lock();
    file = open_i2c_dev(m_i2c_dev, filename, sizeof(filename), 0);
    if (file < 0)
    {
        fprintf(stderr, "Error: Could not open i2c device: %s errno: %s\n", filename, strerror(errno));
        return -1;
    }

    if (ioctl(file, I2C_SLAVE, hw_address) < 0)
    {
        fprintf(stderr, "Error: Could not set address to 0x%02x: %s\n", address, strerror(errno));
        return -errno;
    }

    do
    {
        res = i2c_smbus_read_byte_data(file, address);
        ++reTries;
    } while (res < 0 && reTries < 3);

    if(reTries >= 3)
    {
        close(file);
        return -2;
    }

    close(file);

    mI2cMutex.unlock();
    return res;
}

int i2c::i2c_readWord(int hw_address, int address)
{
    int file;
    char filename[20];
    int res = 0;
    int reTries = 0;

    mI2cMutex.lock();
    file = open_i2c_dev(m_i2c_dev, filename, sizeof(filename), 0);
    if (file < 0)
    {
        fprintf(stderr, "Error: Could not open i2c device: %s errno: %s\n", filename, strerror(errno));
        mI2cMutex.unlock();
        return -1;
    }

    if (ioctl(file, I2C_SLAVE, hw_address) < 0)
    {
        fprintf(stderr, "Error: Could not set address to 0x%02x: %s\n", address, strerror(errno));
        mI2cMutex.unlock();
        return -errno;
    }

    do {
        res = i2c_smbus_read_word_data(file, address);
        ++reTries;
    } while (res < 0 && reTries < 3);

    if(reTries >= 3)
    {
        close(file);
        mI2cMutex.unlock();
        return -2;
    }

    close(file);
    mI2cMutex.unlock();
    return (res >> 8) | ((res & 0xFF) << 8);
}

int i2c::i2c_read_continuous(int hw_address, int address, char* buf, size_t size)
{
    int file;
    char filename[20];
    size_t res = 0;

    // equivalent to: i2cget -y 4 0x67 0x0 v

    file = open_i2c_dev(m_i2c_dev, filename, sizeof(filename), 0);
    if (file < 0)
    {
        fprintf(stderr, "Error: Could not open i2c device: %s errno: %s\n", filename, strerror(errno));
        return -1;
    }

    if (ioctl(file, I2C_SLAVE, hw_address) < 0)
    {
        fprintf(stderr, "Error: Could not set address to 0x%02x: %s\n", address, strerror(errno));
        return -errno;
    }

    res = read(file, buf, size);

    if(res != size)
    {
        fprintf(stderr, "Error: could not read %d bytes from 0x%02x: %s\n", res, address, strerror(errno));
    }

    close(file);
    return res;
}

int i2c::i2c_write(int hw_address, int address, int data)
{
    // equivalent to: i2cset 4 0x67 0x06 0x01

    int file;
    char filename[20];
    int res = 0;
    int reTries = 0;

    mI2cMutex.lock();
    file = open_i2c_dev(m_i2c_dev, filename, sizeof(filename), 0);
    if (file < 0)
    {
        return -1;
    }

    if (ioctl(file, I2C_SLAVE, hw_address) < 0)
    {
        fprintf(stderr, "Error: Could not set address to 0x%02x: %s\n", address, strerror(errno));
        return -errno;
    }

    do
    {
        res = i2c_smbus_write_byte_data(file, address, data);

        if (res < 0 && errno == EAGAIN)
        {
            fprintf(stderr, "i2c_write error, dev = %d, addr = %d, data = %d \n", m_i2c_dev, address, data);
            usleep(10000);
        }

        ++reTries;
    } while (res < 0 && reTries < 3);

#if 1
    if (res < 0)
    {
        fprintf(stderr, "i2c_write error, dev = %d, addr = 0x%02X, data = 0x%02X, err(%d) = %s\n", m_i2c_dev, address, data, errno, strerror(errno));
    }
#endif
    if(reTries >= 3)
    {
        close(file);
        return -2;
    }

    close(file);

    mI2cMutex.unlock();

    return res;
}

int i2c::i2c_writeWord(char hw_address, char address, int16_t data)
{
    int file;
    char filename[20];
    int res = 0;
    int reTries = 0;

    // reorient bytes in word to output correct byte first
    data = ((data & 0xFF00) >> 8) | ((data & 0x00FF) << 8);

    mI2cMutex.lock();
    file = open_i2c_dev(m_i2c_dev, filename, sizeof(filename), 0);
    if (file < 0)
    {
        mI2cMutex.unlock();
        return -1;
    }

    if (ioctl(file, I2C_SLAVE, hw_address) < 0)
    {
        mI2cMutex.unlock();
        fprintf(stderr, "Error: Could not set address to 0x%02x: %s\n", address, strerror(errno));
        return -errno;
    }

    do
    {
        res = i2c_smbus_write_word_data(file, address, data);

        if (res < 0 && errno == EAGAIN)
        {
            fprintf(stderr, "i2c_write error, dev = %d, addr = %d, data = %d \n", m_i2c_dev, address, data);
            usleep(10000);
        }

        ++reTries;
    } while (res < 0 && reTries < 3);

    if (res < 0)
    {
        fprintf(stderr, "i2c_write error, dev = %d, addr = 0x%02X, data = 0x%02X, err(%d) = %s\n", m_i2c_dev, address, data, errno, strerror(errno));
    }

    if(reTries >= 3)
    {
        close(file);
        mI2cMutex.unlock();
        return -2;
    }

    close(file);

    mI2cMutex.unlock();

    return res;
}
