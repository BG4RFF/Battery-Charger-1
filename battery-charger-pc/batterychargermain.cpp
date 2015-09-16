/**
@mainpage Battery Charger Project PC Control
@version 1.0.0
@author Ken Sarkies (www.jiggerjuice.net)
@date 31 March 2010

This describes the program structure and detail for the control application for
the Battery Charger Project. The development documentation is supplemented by a
user manual and overall design documentation. Refer also to documentation for
the firmware and serial bootloader.

The Battery Charger control application provides a GUI interface to the Battery
Charger Unit. The unit communicates by a message protocol over a serial
interface. This program provides:

-# configuration of the unit,
-# acquisition of historical internal status data
*/

/***************************************************************************
 *   Copyright (C) 2010 by Ken Sarkies                                     *
 *   ksarkies@trinity.asn.au ksarkies@internode.on.net                     *
 *                                                                         *
 *   This file is part of Acquisition.                                     *
 *                                                                         *
 *   Battery Charger is free software; you can redistribute it and/or      *
 *   modify it under the terms of the GNU General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   Battery Charger is distributed in the hope that it will be useful,    *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with Battery Charger. If not, write to the                      *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA.             *
 ***************************************************************************/

#include "batterychargercontrol.h"
#include "acqinfoblock.h"
#include "acqunit.h"
#include <QDebug>
#include <QApplication>
#include <QMessageBox>
#include <QString>

//-----------------------------------------------------------------------------
/** @brief Main Program

This opens the serial communications port and launches the main window if
it is successfully synchronized with the Regulator unit. The serial interface
used is defined in the code.

@todo Add a feature to allow the interface to be selected and tested.
*/

int main(int argc,char ** argv)
{
    bool finished = false;
    bool execOK;
    while (! finished)
    {
        finished = true;
        execOK = true;
        QApplication application(argc,argv);
//! Create the acquisition unit object to manage all communications.
        AcqUnit* unit = new AcqUnit();
//! First check that the unit has synchronized in baud rate when constructed.
        if (! unit->isSynchronized())
        {
            QMessageBox::critical(0,"Synchronization Problem",
                        QString("%1").arg(unit->error())+
                        "\n Check device power and serial port");
            execOK = false;
        }
/** Check that the unit object is properly configured (that is, the information
block has been pulled in and has valid settings). If not, call up a base
configuration screen to set it up. */
        else if (! unit->isConfigured())
        {
            execOK = false;
            AcqInfoBlockDialogue infoBlockDialogue(unit);
            if(infoBlockDialogue.exec())
/** When that has been completed, read the configuration again and check that
it was was done successfully. */
            {
                if (! unit->configure())
                {
                    QMessageBox::critical(0,"Configuration Problem\n",
                                QString("%1").arg(unit->error())+
                               "\n Device did not store Configuration\n"
                               "Did you Program it?");
                }
                else finished = false;            // Go back and redo windows
            }
        }
        if (execOK)
        {
//! Call up the main control screen ready to go.
            BatteryChargerControl controlDialogue(unit);
            controlDialogue.show();
            execOK = application.exec();
        }
        delete unit;
    }
    return execOK;
}
//-----------------------------------------------------------------------------
