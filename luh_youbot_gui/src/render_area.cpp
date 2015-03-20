/* *****************************************************************
 *
 * luh_youbot_gui
 *
 * Copyright (c) 2015,
 * Institute of Mechatronic Systems,
 * Leibniz Universitaet Hannover.
 * (BSD License)
 * All rights reserved.
 *
 * http://www.imes.uni-hannover.de
 *
 * This software is distributed WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.
 *
 * For further information see http://www.linfo.org/bsdlicense.html
 *
 * Author: Simon Aden (simon.aden@mailbox.org)
 ******************************************************************/

#include "luh_youbot_gui/render_area.hpp"
#include <QPainter>
#include <QTimer>

using namespace luh_youbot_gui;

RenderArea::RenderArea(QWidget *parent) :
    QWidget(parent)
{
    model_ = new ArmSideView(this);

    connect(model_, SIGNAL(updateRequest()), this, SLOT(update()));
}

QSize RenderArea::sizeHint() const
{
    return QSize(200, 200);
}

QSize RenderArea::minimumSizeHint() const
{
    return QSize(100, 100);
}

 void RenderArea::paintEvent(QPaintEvent * /* event */)
 {
     QPainter painter(this);
     painter.setPen(QColor(90, 90, 90));
     painter.setBrush(QColor(70, 80, 80));
     painter.setRenderHint(QPainter::Antialiasing, true);

     int w = this->width();
     int h = this->height();

     // draw bounding box
     painter.drawRect(2,2,w-4, h-4);

     // draw grid
     int grid_width = std::min(w-4, h-4)/10;

     for(int i=grid_width+2; i<w-2; i+=grid_width)
     {
         painter.drawLine(i, 2, i, h-2);
     }
     for(int i=grid_width+2; i<h-2; i+=grid_width)
     {
         painter.drawLine(2, i, w-2, i);
     }

     // paint model
     model_->paint(painter, w, h);

 }

 void RenderArea::mouseReleaseEvent ( QMouseEvent * e )
 {
     if(mouse_state_ == PRESSED)
     {
         model_->release(e->pos());
         update();
     }

     if(mouse_state_ == PENDING)
     {
         model_->click(e->pos());
         update();
     }

     mouse_state_ = NONE;
 }

 void RenderArea::mousePressEvent ( QMouseEvent * e )
 {
     mouse_state_ = PENDING;
     mouse_position_ = e->pos();

     QTimer::singleShot(200, this, SLOT(mouseTimer()));
 }

void RenderArea::mouseMoveEvent(QMouseEvent *e)
{
   if(mouse_state_ == PENDING)
   {
       mouse_state_ = PRESSED;

       model_->press(mouse_position_);
   }

   mouse_position_ = e->pos();
   model_->move(mouse_position_);

   if(mouse_state_ == PRESSED)
       update();
}

void RenderArea::mouseTimer()
{
    if(mouse_state_ == PENDING)
    {
        mouse_state_ = PRESSED;

        model_->press(mouse_position_);
        update();
    }
}
