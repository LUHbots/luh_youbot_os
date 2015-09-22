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
 * Author: Simon Aden (info@luhbots.de)
 ******************************************************************/

#ifndef LUH_YOUBOT_GUI_RENDER_AREA_H
#define LUH_YOUBOT_GUI_RENDER_AREA_H

#include <QWidget>
#include <QMouseEvent>
#include "arm_side_view.hpp"

namespace luh_youbot_gui
{
class RenderArea : public QWidget
{
    Q_OBJECT
public:
    explicit RenderArea(QWidget *parent = 0);

    QSize sizeHint() const;
    QSize minimumSizeHint() const;
    ArmSideView* model(){return model_;}

protected Q_SLOTS:
    void mouseTimer();

protected:
    void paintEvent(QPaintEvent *event);
    void mouseReleaseEvent(QMouseEvent * e);
    void mousePressEvent(QMouseEvent * e);
    void mouseMoveEvent(QMouseEvent *e);

private:
    enum MouseState{NONE, PENDING, PRESSED} mouse_state_;

    QPoint mouse_position_;
    ArmSideView *model_;
};
}

#endif // RENDER_AREA_H
