#include "gpio.hpp"

using namespace GPIO_CTRL;

int gpio::exportIO(int gpio)
{
    QFile file("/sys/class/gpio/export");
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "[ERROR] - exporting gpio " << gpio;
        return(-1);
    }

    QTextStream out(&file);
    out << QString::number(gpio);

    file.close();

    return 0;
}

int gpio::setDirectionIO(int gpio, int dir)
{

    QFile file("/sys/class/gpio/gpio"+QString::number(gpio)+"/direction");
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "[ERROR] - setting direction of gpio " << gpio;
        return(-1);
    }

    QTextStream out(&file);

    switch(dir)
    {
        case 0:
            out << "out";
            break;
        case 1:
            out << "in";
            break;
        default:
            return(-2);

    }

    file.close();

    return 0;
}

int gpio::setEdgeIO(int gpio, int eType)
{
    QFile file("/sys/class/gpio/gpio"+QString::number(gpio)+"/edge");
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "[ERROR] - setting edge of gpio " << gpio;
        return(-1);
    }

    QTextStream out(&file);

    switch(eType)
    {
        case 0:
            out << "both";
            break;
        case 1:
            out << "falling";
            break;
        case 2:
            out << "rising";
            break;
        case 3:
            out << "none";
        default:
            return(-2);
    }

    file.close();

    return 0;
}

int gpio::setActivelowIO(int gpio, int active_low)
{
    QFile file("/sys/class/gpio/gpio"+QString::number(gpio)+"/active_low");
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "[ERROR] - setting active low of gpio " << gpio;
        return(-1);
    }

    QTextStream out(&file);

    out << QString::number(active_low);

    file.close();
    return 0;
}

int gpio::setOutputState(int gpio, int state)
{
    QFile file("/sys/class/gpio/gpio"+QString::number(gpio)+"/value");
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    {
        qDebug() << "[ERROR] - setting active low of gpio " << gpio;
        return(-1);
    }

    QTextStream out(&file);

    out << QString::number(state);

    file.close();
    return 0;
}

int gpio::configureIO(tGPIO gpio)
{
    int result = 0;
    qDebug() << "Configuring IO " << gpio.gpio;

    result = exportIO(gpio.gpio);
    result |= setDirectionIO(gpio.gpio, gpio.direction);

    if(gpio.direction == INPUT)
    {
        result |= setEdgeIO(gpio.gpio, gpio.edge);
        result |= setActivelowIO(gpio.gpio, gpio.active_low);
    }
    else
    {
       setOutputState(gpio.gpio, gpio.initial_state);
    }

    return result;
}

int gpio::configureIO(QVector<tGPIO> *io_map)
{
    int result = 0;

    for(int idx=0; idx<io_map->size(); idx++)
    {
        result |= configureIO(io_map->at(idx));
    }

    return result;
}
