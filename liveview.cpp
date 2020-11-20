#include "liveview.h"
#include <QEvent>
#include <QKeyEvent>

using namespace std;

LiveView::LiveView(string name)
{
    display=new QLabel();
    setWindowTitle(tr(name.c_str()));
    setCentralWidget(display);
    s_pressed=false;
    r_pressed=false;
    installEventFilter(this);
}

void LiveView::setImage(cv::Mat dis_mat){
    QImage qim(dis_mat.data,dis_mat.cols,dis_mat.rows,dis_mat.step,QImage::Format_RGB888);
    display->setPixmap(QPixmap::fromImage(qim));
}

bool LiveView::eventFilter(QObject *watched, QEvent *event){
    if(event->type()==QEvent::KeyPress){
        QKeyEvent *ke=static_cast<QKeyEvent*>(event);
        if(ke->key()==Qt::Key_R){
            r_pressed=true;
        }
        else if(ke->key()==Qt::Key_S){
            s_pressed=true;
        }
        else{
            return QMainWindow::eventFilter(watched,event);
        }
        return true;
    }
    else{
        return QMainWindow::eventFilter(watched,event);
    }
}
