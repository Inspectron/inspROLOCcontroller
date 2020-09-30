#ifndef INSPCORE_HPP
#define INSPCORE_HPP

/*
 * version 0.1
 * - initial creation for the setFormattedDebugOutput, which redirects qDebug messsages,
 * adds tags and updates the color of the msg on the console(stderr) in this case
 *
 * version 06.12.17
 * - add a printed out version on the console in the form (dd.mm.yy)
 * - add internal debug prints with libraryPrintMsg
 * - add function to set the overlay framebuffer alpha channel
 * - fix trimFunctionName to account for function names that do not have the desired markers
 *
 * version 08.06.18
 * - push messages to a fifo which is in turn pushed over a socket for debugging wirelessly
 * - include the global header locally here to save on the number of header files applications require
 * - optimize the string building for the dbg message
 */

#ifndef LIBINSPCORE_GLOBAL_H
#define LIBINSPCORE_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(LIBINSPCORE_LIBRARY)
#  define LIBINSPCORESHARED_EXPORT Q_DECL_EXPORT
#else
#  define LIBINSPCORESHARED_EXPORT Q_DECL_IMPORT
#endif

#endif // LIBINSPCORE_GLOBAL_H

#include <QString>

class LIBINSPCORESHARED_EXPORT InspCore
{

public:
    /*
     * Call this static function at the beginning of the main() in order to set
     * the formating of the debug message to appear with a  debug tag,
     * class name,function name, and colored text. No changes to qDebug/qWarning/etc
     * are needed
     *
     * param: applicationName - the name of the application to be appended to the log message.
     *
     * Coding Example:
     * ---------------
     * InpsCore::setFormattedDebugOutput()
     *
     * Example output:
     * ---------------
     * [M] UicHandler::openNode - success opening /dev/video1 node
     * [W] FileHandler::close - size of array is larger than expected
     * [E] GPIOChecker::read - couldn't read GPIO
     */
    static void setFormattedDebugOutput(QString applicationName);

    /*
     * Call this static function at the beginning of the main() in order to set
     * the alpha channel in the overlay buffer(/dev/fb1) to true
     *
     * returns: true on success, false on failure
     *
     * Coding Example:
     * ---------------
     * InpsCore::setOverlayFramebufferAlpha()
     *
     */
    static bool setOverlayFramebufferAlpha();

private:
    // ctor not to be called
    InspCore() {}
    static void    formattedDebugOutput(QtMsgType type, const QMessageLogContext &context, const QString &msg);
    static QString trimFunctionNameStr(QString fnc);
    static void    libraryPrintMsg(QtMsgType type, const char* funcName, QString msg);
    static void    initFIFO();
    static void    closeFIFO();
    static void    writeToFIFO(char type, QString msg);

};

#endif // INSPCORE_HPP
