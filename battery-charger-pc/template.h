/*
Title:    Atmel Microcontroller Solar Regulator Project
*/

/***************************************************************************
 *   Copyright (C) 2010 by Ken Sarkies                                     *
 *   ksarkies@trinity.asn.au ksarkies@internode.on.net                     *
 *                                                                         *
 *   This file is part of Acquisition.                                     *
 *                                                                         *
 *   Solar Regulator is free software; you can redistribute it and/or      *
 *   modify it under the terms of the GNU General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the    *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   Solar Regulator is distributed in the hope that it will be useful,    *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with Solar Regulator. If not, write to the                      *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA.             *
 ***************************************************************************/

#ifndef SOLARREGULATORCONTROL_H
#define SOLARREGULATORCONTROL_H

#include "ui_solarregulatoraboutdialogue.h"
#include "acqunit.h"
#include <QString>

//-----------------------------------------------------------------------------
/** @brief Acquisition About Window.

This class provides an informational window with details of the MCU and analogue
channels, as well as version and copyright information.
*/

class SolarRegulatorAbout : public QDialog
{
    Q_OBJECT
public:
    SolarRegulatorAbout(AcqUnit*, QWidget* parent = 0);
    ~SolarRegulatorAbout();
private slots:
    void on_closeButton_clicked();
private:
    AcqUnit* unit;
    Ui::SolarRegulatorAboutDialogue SolarRegulatorAboutDialogUi;// User Interface
};

//-----------------------------------------------------------------------------
#endif
