/*
Title:    Atmel Microcontroller Battery Charger Project
*/
/** @defgroup batterychargerconfigure Battery Charger Configure Window

These functions provide functionality for the Battery Charger Configure
Window in which basic configuration controls are set by the user and
programmed into the unit information block. It provides access to
the advanced configuration dialogue.

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

#include "batterychargerconfigure.h"
#include "batterychargeradvanced.h"
#include "acqinfoblock.h"
#include <QDebug>
#include <math.h>

#define  high(x) ((uchar) (x >> 8) & 0xFF)
#define  low(x) ((uchar) (x & 0xFF))

//-----------------------------------------------------------------------------
/** @brief Constructor.

The Application's Configure Information Block has the following entries:
[1] Channel Type/Number (0x31)
[2-3] Battery Capacity "C" in AH (not used by the device but kept for reference)
[4-5] Float Charge Threshold Voltage (default 13.5V)
[6-7] Maximum Float Charge Current (default 0.3C)
[8-9] Cyclic Charge Threshold Voltage (default 14.4V)
[10-11] Cyclic Charge Cutoff Current (default 0.02C)
[12] Filter Time Constant
[13] Float Charge Mode only

@param u Acquisition Unit Object
@param parent Parent widget.
*/
BatteryChargerConfigure::BatteryChargerConfigure(AcqUnit* u,
          QWidget* parent) : QDialog(parent)
{
    unit = u;
// The Configure IB is kept separate from the one defined in Advanced Settings
// to allow each to be updated independently. Define as channel 1.
    appConfigureUnit = new AcqAppConfigUnit(unit,1);
//! Build the User Interface display from Ui object.
    BatteryChargerConfigureDialogUi.setupUi(this);
//! Setup the Charger Model defaults, safe for most LA batteries
    batteryCapacity = 1;
    floatChargeVoltageThreshold = 13.5;
    cyclicChargeCurrentThreshold = 0.3;
    cyclicChargeVoltageThreshold = 14.4;
    cyclicChargeCurrentCutoff = 0.02;
    unsigned int filterShift = 4;
    bool floatOnly = false;
    if (appConfigureUnit->isConfigured())
    {
        batteryCapacity = round((appConfigureUnit->getChannelInfoBlock(0)
                            +appConfigureUnit->getChannelInfoBlock(1)*256)/6.5535)/100;
        floatChargeVoltageThreshold = round((appConfigureUnit->getChannelInfoBlock(2)
                            +appConfigureUnit->getChannelInfoBlock(3)*256)/6.5535)/100;
        cyclicChargeCurrentThreshold = round((appConfigureUnit->getChannelInfoBlock(4)
                            +appConfigureUnit->getChannelInfoBlock(5)*256)/6.5535/batteryCapacity)/100;
        cyclicChargeVoltageThreshold = round((appConfigureUnit->getChannelInfoBlock(6)
                            +appConfigureUnit->getChannelInfoBlock(7)*256)/6.5535)/100;
        cyclicChargeCurrentCutoff = round((appConfigureUnit->getChannelInfoBlock(8)
                            +appConfigureUnit->getChannelInfoBlock(9)*256)/6.5535/batteryCapacity)/100;
        filterShift = appConfigureUnit->getChannelInfoBlock(10);
        floatOnly = appConfigureUnit->getChannelInfoBlock(11);
    }
    BatteryChargerConfigureDialogUi.batteryCapacityEdit->setText(QString("%1").arg(batteryCapacity));
    BatteryChargerConfigureDialogUi.floatChargeThresholdEdit->setText(QString("%1").arg(floatChargeVoltageThreshold));
    BatteryChargerConfigureDialogUi.maxChargeCurrentEdit->setText(QString("%1").arg(cyclicChargeCurrentThreshold));
    BatteryChargerConfigureDialogUi.cyclicChargeThresholdEdit->setText(QString("%1").arg(cyclicChargeVoltageThreshold));
    BatteryChargerConfigureDialogUi.cyclicChargeCutoffEdit->setText(QString("%1").arg(cyclicChargeCurrentCutoff));
    BatteryChargerConfigureDialogUi.filterTimeConstantSpinBox->setValue(filterShift);
    BatteryChargerConfigureDialogUi.batteryCapacityCheckBox->setChecked(! floatOnly);
    on_batteryCapacityCheckBox_stateChanged(0);
}

BatteryChargerConfigure::~BatteryChargerConfigure()
{
    delete appConfigureUnit;
}

//-----------------------------------------------------------------------------
/** @brief Close when OK is activated returning success.
*/

void BatteryChargerConfigure::on_closeButton_clicked()
{
    accept();
}
//-----------------------------------------------------------------------------
/** @brief Close when OK is activated returning success.
*/

void BatteryChargerConfigure::on_batteryCapacityCheckBox_stateChanged(int state)
{
    if (BatteryChargerConfigureDialogUi.batteryCapacityCheckBox->isChecked())
    {
        BatteryChargerConfigureDialogUi.batteryCapacityEdit->setEnabled(true);
        BatteryChargerConfigureDialogUi.floatChargeThresholdEdit->setEnabled(true);
        BatteryChargerConfigureDialogUi.maxChargeCurrentEdit->setEnabled(true);
        BatteryChargerConfigureDialogUi.cyclicChargeThresholdEdit->setEnabled(true);
        BatteryChargerConfigureDialogUi.cyclicChargeCutoffEdit->setEnabled(true);
    }
    else
    {
        BatteryChargerConfigureDialogUi.batteryCapacityEdit->setEnabled(false);
        BatteryChargerConfigureDialogUi.floatChargeThresholdEdit->setEnabled(true);
        BatteryChargerConfigureDialogUi.maxChargeCurrentEdit->setEnabled(true);
        BatteryChargerConfigureDialogUi.cyclicChargeThresholdEdit->setEnabled(false);
        BatteryChargerConfigureDialogUi.cyclicChargeCutoffEdit->setEnabled(false);
    }
}
//-----------------------------------------------------------------------------
/** @brief Show Microcontroller Information.

This button opens the advanced configuration window.
*/

void BatteryChargerConfigure::on_advancedButton_clicked()
{
    BatteryChargerAdvanced advancedForm(unit);
    advancedForm.exec();
}
//-----------------------------------------------------------------------------
/** @brief Open the Unit Configuration Dialogue.
*/

void BatteryChargerConfigure::on_unitConfigButton_clicked()
{
    AcqInfoBlockDialogue infoBlockDialogue(unit);
    infoBlockDialogue.exec();
}
//-----------------------------------------------------------------------------
/** @brief Write configuration to the Application IB.
*/

void BatteryChargerConfigure::on_updateButton_clicked()
{
    unsigned char infoBlock[16];
    unsigned char index = 0;
    bool ok;
    unsigned int batteryCapacity = floor(BatteryChargerConfigureDialogUi.batteryCapacityEdit->text().toFloat(&ok)*655.35);
    if (! ok)
    {
        displayErrorMessage("Invalid Battery Capacity");
        return;
    }
    infoBlock[index++] = low(batteryCapacity);
    infoBlock[index++] = high(batteryCapacity);
    unsigned int floatChargeVoltageThreshold
                    = floor(BatteryChargerConfigureDialogUi.floatChargeThresholdEdit->text().toFloat(&ok)*655.35);
    if (! ok)
    {
        displayErrorMessage("Invalid Float Charge Voltage Threshold");
        return;
    }
    infoBlock[index++] = low(floatChargeVoltageThreshold);
    infoBlock[index++] = high(floatChargeVoltageThreshold);
    unsigned int cyclicChargeCurrentThreshold
                    = floor(BatteryChargerConfigureDialogUi.maxChargeCurrentEdit->text().toFloat(&ok)*batteryCapacity);
    if (! ok)
    {
        displayErrorMessage("Invalid Maximum Float Charge Current");
        return;
    }
    infoBlock[index++] = low(cyclicChargeCurrentThreshold);
    infoBlock[index++] = high(cyclicChargeCurrentThreshold);
    unsigned int cyclicChargeVoltageThreshold
                    = floor(BatteryChargerConfigureDialogUi.cyclicChargeThresholdEdit->text().toFloat(&ok)*655.35);
    if (! ok)
    {
        displayErrorMessage("Invalid Cyclic Charge Voltage Threshold");
        return;
    }
    infoBlock[index++] = low(cyclicChargeVoltageThreshold);
    infoBlock[index++] = high(cyclicChargeVoltageThreshold);
    unsigned int cyclicChargeCurrentCutoff
                    = floor(BatteryChargerConfigureDialogUi.cyclicChargeCutoffEdit->text().toFloat(&ok)*batteryCapacity);
    if (! ok)
    {
        displayErrorMessage("Invalid Cyclic Charge Cutoff Current");
        return;
    }
    infoBlock[index++] = low(cyclicChargeCurrentCutoff);
    infoBlock[index++] = high(cyclicChargeCurrentCutoff);
    infoBlock[index++] = BatteryChargerConfigureDialogUi.filterTimeConstantSpinBox->value();
    infoBlock[index++] = ! BatteryChargerConfigureDialogUi.batteryCapacityCheckBox->isChecked();
    ok = appConfigureUnit->programInformationBlock(index,infoBlock);
    if (! ok) displayErrorMessage("Could not program device");
}
//-----------------------------------------------------------------------------
/** @brief Error Message.

This provides access to error messages from outside the class.
@return The latest error message that was detected.
*/
void BatteryChargerConfigure::displayErrorMessage(const QString message)
{
     BatteryChargerConfigureDialogUi.errorLabel->setText(message);
}
/**@}*/
//-----------------------------------------------------------------------------
