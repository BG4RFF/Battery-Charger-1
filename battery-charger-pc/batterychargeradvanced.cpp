/*
Title:    Atmel Microcontroller Battery Charger Project
*/
/** @defgroup batterychargeradvanced Battery Charger Advanced Configuration Window

These functions provide functionality for the Battery Charger Advanced
Configuration Window which allows fine-grained control of some of the internal
timing and algorithm operation.

@{*/
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

#include "batterychargeradvanced.h"
#include "batterychargercalibrate.h"
#include <math.h>
#include <QDebug>

#define  high(x) ((uchar) (x >> 8) & 0xFF)
#define  low(x) ((uchar) (x & 0xFF))

//-----------------------------------------------------------------------------
/** @brief Constructor.

This sets up the various controls with default values either preset or taken
from the existing channel if it does exist. An existing channel will be read
from the device when the AcqAppConfigUnit object is created.

The Application's Configure Information Block has the following entries
[0] Channel Type/Number (0x30)
[1] Measurement Interval in RTC timer counts (default 32ms).
[2] Measurement Averaging Count - index of power of two.
[3] Set to TRUE to disable the algorithm in the device
[4] Storage Interval in measurement cycles.

@param u Acquisition Unit Object
@param parent Parent widget.
*/
BatteryChargerAdvanced::BatteryChargerAdvanced(AcqUnit* u,
          QWidget* parent) : QDialog(parent)
{
    unit = u;
//! Build the User Interface display from Ui object.
    BatteryChargerAdvancedDialogUi.setupUi(this);
// Setup the configure channel 0 for the Advanced settings
    appConfigureUnit = new AcqAppConfigUnit(unit,0);
//! Setup the Measurement Averaging Combo Box
    unsigned char measureAverage = 4;
    unsigned char measureInterval = 20;
    unsigned char storeInterval = 20;
    BatteryChargerAdvancedDialogUi.disableControlCheckBox->setChecked(true);
    for (uint n=1; n<8; n++)
        BatteryChargerAdvancedDialogUi.measureAverageComboBox->
                          insertItem(n-1,QString::number(1 << n));
    if (appConfigureUnit->isConfigured())
    {
        measureInterval = appConfigureUnit->getChannelInfoBlock(0);
        measureAverage = appConfigureUnit->getChannelInfoBlock(1);
        BatteryChargerAdvancedDialogUi.disableControlCheckBox->
                    setChecked(appConfigureUnit->getChannelInfoBlock(2));
        storeInterval = appConfigureUnit->getChannelInfoBlock(3);
    }
    BatteryChargerAdvancedDialogUi.measureAverageComboBox->setCurrentIndex(measureAverage);
//! Setup the measurement interval spinbox with a default
    BatteryChargerAdvancedDialogUi.measureIntervalSpinBox->setValue(measureInterval);
//! Setup the result store interval spinbox with a default
    BatteryChargerAdvancedDialogUi.storeIntervalSpinBox->setValue(storeInterval);
}

BatteryChargerAdvanced::~BatteryChargerAdvanced()
{
    delete appConfigureUnit;
}

//-----------------------------------------------------------------------------
/** @brief Close when OK is activated returning success.
*/

void BatteryChargerAdvanced::on_closeButton_clicked()
{
    accept();
}

//-----------------------------------------------------------------------------
/** @brief Channel Calibration.

This button opens the channel calibration window.
*/

void BatteryChargerAdvanced::on_calibrateButton_clicked()
{
    unsigned int channel = 0;
    if (BatteryChargerAdvancedDialogUi.channelB->isChecked()) channel = 1;
    else if (BatteryChargerAdvancedDialogUi.channelC->isChecked()) channel = 2;
    else if (BatteryChargerAdvancedDialogUi.channelD->isChecked()) channel = 3;
    AcqSimpleAdcUnit* channelUnit = new AcqSimpleAdcUnit(unit,channel);
// Go to the calibrate form
    BatteryChargerCalibrate calibrateForm(channelUnit);
    calibrateForm.exec();
}
//-----------------------------------------------------------------------------
/** @brief Write configuration to the Application IB.

This sends off a block program command to overwrite the configure channel
information with that set on the controls.
*/

void BatteryChargerAdvanced::on_updateButton_clicked()
{
    unsigned char infoBlock[10];
    unsigned char index = 0;
    bool ok;
    infoBlock[index++] = BatteryChargerAdvancedDialogUi.measureIntervalSpinBox->value();
    infoBlock[index++] = BatteryChargerAdvancedDialogUi.measureAverageComboBox->currentIndex();
    infoBlock[index++] = BatteryChargerAdvancedDialogUi.disableControlCheckBox->isChecked();
    infoBlock[index++] = BatteryChargerAdvancedDialogUi.storeIntervalSpinBox->value();
    ok = appConfigureUnit->programInformationBlock(index,infoBlock);
    if (! ok) displayErrorMessage("Could not program device");
}
//-----------------------------------------------------------------------------
/** @brief Error Message.

This provides access to error messages from outside the class.
@return The latest error message that was detected.
*/
void BatteryChargerAdvanced::displayErrorMessage(const QString message)
{
     BatteryChargerAdvancedDialogUi.errorLabel->setText(message);
}
/**@}*/
//-----------------------------------------------------------------------------
