#ifndef GPIO_H
#define GPIO_H

#include <QObject>
#include <QVector>
#include <QFile>
#include <QDebug>

namespace GPIO_CTRL
{
    static const int    INPUT            = 1;
    static const int    OUTPUT           = 0;
    static const int    EDGE_BOTH        = 0;
    static const int    EDGE_FALLING     = 1;
    static const int    EDGE_RISING      = 2;
    static const int    EDGE_NONE        = 3;
    static const int    HIGH             = 1;
    static const int    LOW              = 0;

    // The GPIO numbers can be extracted using signal name from below.
    // Example:
    // MX6QDL_PAD_EIM_DA10__GPIO3_IO10 is
    // GPIO(3,10) which is (3-1) * 32 + 10 = gpio 74
    //
    // i.e. The mapping of GPIO(X,Y) to Linux gpio number is:
    // gpio number = (X-1) * 32 + Y

    struct tGPIO
    {
        int gpio;
        int direction;
        int edge;
        int active_low;
        int initial_state;

        // default ctor
        tGPIO()
        {
            set ( 0, 0, 0, 0, 0 );
        }

        // data ctor
        tGPIO( int gpioNum, int gpioDir, int gpioEdge, int gpioActLow, int gpioInitial )
        {
            set (gpioNum, gpioDir, gpioEdge, gpioActLow, gpioInitial);
        }

        // set the member data all at once
        void set( int gpioNum, int gpioDir, int gpioEdge, int gpioActLow, int gpioInitial )
        {
            gpio          = gpioNum;
            direction     = gpioDir;
            edge          = gpioEdge;
            active_low    = gpioActLow;
            initial_state = gpioInitial;
        }

        // return a string printout of the gpio
        QString getString()
        {
            QString dirStr = ( direction == INPUT  ? "INPUT" :
                             ( direction == OUTPUT ? "OUTPUT" : "???" ));

            QString edgeStr = ( edge == EDGE_BOTH    ? "BOTH"   :
                              ( edge == EDGE_FALLING ? "FALLING":
                              ( edge == EDGE_RISING  ? "RISING" :
                              ( edge == EDGE_NONE    ? "NONE"   : "???" ))));

            QString activeStr = ( active_low == false   ? "false" :
                                ( active_low == true    ? "true"  : "???" ));

            QString initStr = ( initial_state == HIGH ? "HIGH" :
                              ( initial_state == LOW  ? "LOW"  : "???" ));

            QString str = QString("{ %1, %2, %3, %4, %5 }")
                    .arg( QString::number(gpio) )
                    .arg(dirStr)
                    .arg(edgeStr)
                    .arg(activeStr)
                    .arg(initStr) ;

            return str;
        }
    };
}

class gpio
{
public:
    static int configureIO(GPIO_CTRL::tGPIO gpio);
    static int configureIO(QVector<GPIO_CTRL::tGPIO> *io_map);
    static int setOutputState(int gpio, int state);
    static int setDirectionIO(int gpio, int dir);

private:
    // class is static. no need to call the ctor
    gpio();

private:
    static int exportIO(int gpio);
    static int setEdgeIO(int gpio, int eType);
    static int setActivelowIO(int gpio, int active_low);
};

#endif // GPIO_H
