/*
Copyright (c) 2014-2018, Luca Longinotti, iniVation AG
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef EVENTCAM_H
#define EVENTCAM_H
#include <atomic>
#include <csignal>
#include <queue>
#include <libcaercpp/devices/dvs128.hpp>
#include <libcaercpp/filters/dvs_noise.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>
#include <memory>
#include "liveview.h"

class EventCam
{
    static std::atomic_bool globalShutdown;
    static void globalShutdownSignalHandler(int signal);
    static void usbShutdownHandler(void *ptr);

    std::atomic_bool recording,grabbing,playing;
    std::atomic_int container_index;
    std::thread live_view_t,grabber_t,player_t;

    LiveView *live_window;

    int num_containers;

    std::vector<long long int> timestamps;
    std::vector<std::shared_ptr<libcaer::events::PolarityEventPacket>> packets;

    // Open a DVS128, give it a device ID of 1, and don't care about USB bus or SN restrictions.
    std::shared_ptr<libcaer::devices::dvs128> cam_handle;

    struct PacketVisualizer{
        cv::Mat vis_im,vis_im_big;
        std::vector<cv::Vec3b*> fa_vis_im;

        cv::Size vis_im_size=cv::Size(128,128);
        cv::Size big_vis_size=cv::Size(500,500);

        PacketVisualizer(){
            vis_im=cv::Mat::zeros(vis_im_size,CV_8UC3);
            fa_vis_im.resize(vis_im.rows);
            for(int r=0;r<vis_im.rows;r++){//make the fast access arrays
                fa_vis_im[r]=vis_im.ptr<cv::Vec3b>(r);
            }
        }

        cv::Mat visualize(std::shared_ptr<const libcaer::events::PolarityEventPacket> pep){
            for(const libcaer::events::PolarityEvent &pe: *pep){
                int x = pe.getX();
                int y = pe.getY();
                if(pe.getPolarity()){//on event
                    fa_vis_im[y][x][2]=255;
                }
                else{//off event
                    fa_vis_im[y][x][0]=255;
                }
            }
            cv::resize(vis_im,vis_im_big,cv::Size(500,500),0,0,cv::INTER_NEAREST);
            vis_im=cv::Mat::zeros(vis_im_size,CV_8UC3);
            return vis_im_big;
        }
    };

    static void liveView(std::vector<std::unique_ptr<libcaer::events::EventPacketContainer>>* containers_p,
                         long long int *container_index_p, std::atomic_bool *live_view, LiveView *lw);

    static void grabber(int num_containers, libcaer::devices::dvs128* cam_handle_p, std::atomic_bool *grabbing_p,
                        std::atomic_bool *global_shutdown, std::atomic_bool* recording_p,
                        std::vector<std::shared_ptr<libcaer::events::PolarityEventPacket>> *packets_p,
                        std::vector<long long int> *packet_time_stamps_p, LiveView *lw);

    static void player(std::atomic_bool *playing, LiveView* window, std::vector<long long int> *time_stamps_p,
                          std::vector<std::shared_ptr<libcaer::events::PolarityEventPacket>> *packets_p, long long int s);
public:
    ~EventCam();
    EventCam(LiveView *lv,  int num_containers);
    std::vector<std::shared_ptr<libcaer::events::PolarityEventPacket>> getPackets();
    std::vector<long long int> getTimestamps();
    void record();
    void stop();
    void openCam();
    void start();
    void play(long long s);
    void writePacketsToFile(std::string file_path);
    void loadPacketsFile(std::string file_path);
};

#endif // EVENTCAM_H
