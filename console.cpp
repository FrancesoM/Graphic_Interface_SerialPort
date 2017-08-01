/****************************************************************************
**
** Copyright (C) 2012 Denis Shienkov <denis.shienkov@gmail.com>
** Copyright (C) 2012 Laszlo Papp <lpapp@kde.org>
** Contact: https://www.qt.io/licensing/
**
** This file is part of the QtSerialPort module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "console.h"

#include <QScrollBar>

#include <QtCore/QDebug>

Console::Console(int n_ch, int y_min, int y_max, QCustomPlot *parent)
{
    n_channels = n_ch;

    for(int i = 0;i<n_channels;i++){
        this->addGraph();
        this->graph(i)->setPen(QPen(this->colorVector[i]));
    }

    this->xAxis->setRange(0,HISTORY);
    this->yAxis->setRange(y_min,y_max);

    //We want to define the indexes used to calculate the distances
    //Example: we want to find the distances using channels 3,5,7, so we
    //need the tuples (3,5), (3,7), (5,7) saved in the array "indexes"

}

void Console::putData(QVector<float> data)
{
    //For each channel, push the new value that has been read
    //Then plot the corresponding channel
    for(int i = 0;i<n_channels;i++){
        double temp = (double)data[i];
        this->channels[i].enqueue(temp);
        //We need a temporary t-axis
        QVector<double> t_axis = QVector<double>(this->channels[i].size() );
        //Fill vector with elements from 0 to SIZE-1
        for(int j = 0;j<this->channels[i].size();j++){
            t_axis[j] = j;
        }
        QVector<double> y_axis = this->channels[i].toVector();

        //Necessary to avoid unwanted lines on the screen
        this->graph(i)->data()->clear();
        this->graph(i)->addData(t_axis,y_axis);


    }

    //Replot everything, so it gives a sense of "REAL TIME"
    this->replot();
    this->update();

    //Now pop out the last element from the queue when we reach the max size of the axis
    //We use dequeue because when an element is read it is automatically removed
    //from the queue.
    if(channels[0].size() == HISTORY){
        for(int i = 0;i<n_channels;i++){
            this->channels[i].dequeue();
        }
    }
}

void Console::clearData(){

    for(int i = 0;i<n_channels;i++){
        this->graph(i)->data()->clear();
        this->channels[i].empty();
    }

    this->replot();
    this->update();
}

void Console::setLocalEchoEnabled(bool set)
{
//    localEchoEnabled = set;
}

void Console::keyPressEvent(QKeyEvent *e)
{
//    switch (e->key()) {
//    case Qt::Key_Backspace:
//    case Qt::Key_Left:
//    case Qt::Key_Right:
//    case Qt::Key_Up:
//    case Qt::Key_Down:
//        break;
//    default:
//        if (localEchoEnabled)
//            QPlainTextEdit::keyPressEvent(e);
//        emit getData(e->text().toLocal8Bit());
//    }
}

void Console::mousePressEvent(QMouseEvent *e)
{
    Q_UNUSED(e)
    setFocus();
}

void Console::mouseDoubleClickEvent(QMouseEvent *e)
{
    Q_UNUSED(e)
}

void Console::contextMenuEvent(QContextMenuEvent *e)
{
    Q_UNUSED(e)
}
