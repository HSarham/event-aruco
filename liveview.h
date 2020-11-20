#ifndef LIVEVIEW_H
#define LIVEVIEW_H
#include <qt5/QtWidgets/QMainWindow>
#include <qt5/QtWidgets/QLabel>
#include <opencv2/core.hpp>
#include <atomic>

class LiveView: public QMainWindow{
    QLabel *display;
public:
    LiveView(std::string name);
    void setImage(cv::Mat dis_mat);
    std::atomic_bool r_pressed;
    std::atomic_bool s_pressed;
protected:
    bool eventFilter(QObject *watched, QEvent *event);
};

#endif // LIVEVIEW_H
