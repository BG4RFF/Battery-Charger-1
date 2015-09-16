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

#include "solarregulatorabout.h"

//-----------------------------------------------------------------------------
/** @brief Constructor.

@param u Acquisition Unit Object
@param parent Parent widget.
*/
SolarRegulatorAbout::SolarRegulatorAbout(AcqUnit* u,
          QWidget* parent) : QDialog(parent)
{
}

SolarRegulatorAbout::~SolarRegulatorAbout()
{
}

//-----------------------------------------------------------------------------
/** @brief Close when OK is activated returning success.
*/

void SolarRegulatorAbout::on_closeButton_clicked()
{
    accept();
}

//-----------------------------------------------------------------------------
