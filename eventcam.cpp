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

#include "eventcam.h"
#include <iostream>
#include <chrono>
#include <fstream>

using namespace std;

std::atomic_bool EventCam::globalShutdown;

void EventCam::globalShutdownSignalHandler(int signal) {
    // Simply set the running flag to false on SIGTERM and SIGINT (CTRL+C) for global shutdown.
    if (signal == SIGTERM || signal == SIGINT) {
        globalShutdown.store(true);
    }
}

void EventCam::usbShutdownHandler(void *ptr) {
    (void) (ptr); // UNUSED.

    globalShutdown.store(true);
}

EventCam::~EventCam(){

    if(player_t.joinable())
        player_t.join();
    if(live_view_t.joinable())
        live_view_t.join();

    if(live_window!=NULL)
        live_window->close();
}

void EventCam::loadPacketsFile(std::string file_path){
    ifstream in_file(file_path,std::ios_base::binary);
    if(!in_file.is_open()){
        throw runtime_error("Could not open a file to read at: "+file_path);
    }
    //read the number of packets
    size_t num_packets;
    in_file.read((char*)&num_packets,sizeof(num_packets));

    packets.resize(num_packets);
    timestamps.resize(num_packets);

    //read the packets
    for(size_t i=0;i<num_packets;i++){
        //read the packet timestamp
        in_file.read((char*)&timestamps[i],sizeof(long long int));

        //read number of events
        size_t num_events;
        in_file.read((char*)&num_events,sizeof(num_events));

        //read event source
        int16_t packet_source;
        in_file.read((char*)&packet_source,sizeof(packet_source));

        //get ts_overflow
        int32_t packet_ts_overflow;
        in_file.read((char*)&packet_ts_overflow,sizeof(packet_ts_overflow));

        packets[i]=make_shared<libcaer::events::PolarityEventPacket>(num_events,packet_source,packet_ts_overflow);
        libcaer::events::PolarityEventPacket &packet = *packets[i];

        //iterate on events and read them
        for(size_t k=0;k<num_events;k++){

            uint16_t x;
            in_file.read((char*)&x,sizeof x);

            uint16_t y;
            in_file.read((char*)&y,sizeof y);

            int32_t ts;
            in_file.read((char*)&ts,sizeof ts);

            bool polarity;
            in_file.read((char*)&polarity,sizeof polarity);

            bool validity;
            in_file.read((char*)&validity,sizeof validity);

            packet[k].setX(x);
            packet[k].setY(y);
            packet[k].setTimestamp(ts);
            packet[k].setPolarity(polarity);

            if(validity && !packet[k].isValid())
                packet[k].validate(packet);
            else if(!validity && packet[k].isValid())
                packet[k].invalidate(packet);
        }
    }

}

void EventCam::writePacketsToFile(std::string file_path){

    ofstream of(file_path,std::ios_base::binary);
    if(!of.is_open()){
        throw runtime_error("Could not open a file to write at: "+file_path);
    }

    //write the number of packets
    size_t num_packets=packets.size();
    of.write((char*)&num_packets,sizeof(num_packets));

    //iterate on containers
    for(size_t i=0;i<num_packets;i++){
        //write the packet time stamp
        of.write((char*)&timestamps[i],sizeof(long long int));

        //get the event packet
        const libcaer::events::PolarityEventPacket &packet=*(packets[i]);

        //write number of events
        size_t num_events=packet.size();
        of.write((char*)&num_events,sizeof(num_events));

        //write event source
        int16_t packet_source=packet.getEventSource();
        of.write((char*)&packet_source,sizeof(packet_source));

        //get ts_overflow
        int32_t packet_ts_overflow=packet.getEventTSOverflow();
        of.write((char*)&packet_ts_overflow,sizeof(packet_ts_overflow));

        //iterate on events and write them
        for(size_t k=0;k<num_events;k++){
            uint16_t x=packet[k].getX();
            of.write((char*)&x,sizeof x);
            uint16_t y=packet[k].getY();
            of.write((char*)&y,sizeof y);
            int32_t ts=packet[k].getTimestamp();
            of.write((char*)&ts,sizeof ts);
            bool polarity=packet[k].getPolarity();
            of.write((char*)&polarity,sizeof polarity);
            bool validity=packet[k].isValid();
            of.write((char*)&validity,sizeof validity);
        }
    }
}

void EventCam::liveView(std::vector<std::unique_ptr<libcaer::events::EventPacketContainer> > *containers_p, long long int *container_index_p, atomic_bool *live_view, LiveView *lw){

    PacketVisualizer pv;
    std::vector<std::unique_ptr<libcaer::events::EventPacketContainer> > &containers=*containers_p;

    long long int num_containers=containers.size();
    long long int container_index,prev_ci=num_containers-1;

    cv::Mat rgb;

    while(*live_view){
        //get previous index
        container_index=*container_index_p-1;
        //wrap around
        if(container_index<0)
            container_index+=num_containers;
        //get the container
        std::unique_ptr<libcaer::events::EventPacketContainer>& epc=containers[container_index];
        if(container_index!=prev_ci && epc!=nullptr){

            for ( size_t j=0; j<epc->size();j++) {
                const shared_ptr<libcaer::events::EventPacket> &packet=(*epc)[j];
                if (packet == nullptr) {
                    continue; // Skip if nothing there.
                }

                if (packet->getEventType() == POLARITY_EVENT) {
                    std::shared_ptr<const libcaer::events::PolarityEventPacket> polarity
                            = std::static_pointer_cast<libcaer::events::PolarityEventPacket>(packet);
                    //visualize the packet
                    cv::cvtColor(pv.visualize(polarity),rgb,CV_BGR2RGB);
                    lw->setImage(rgb);
                }
            }
        }
        else{
            std::this_thread::yield();
        }
        prev_ci=container_index;
    }
}

void EventCam::player(atomic_bool *playing, LiveView* window, vector<long long int> *time_stamps_p, std::vector<std::shared_ptr<libcaer::events::PolarityEventPacket>> *packets_p, long long int s){
    PacketVisualizer pv;
    cv::Mat rgb;
    std::vector<std::shared_ptr<libcaer::events::PolarityEventPacket>> &packets=*packets_p;
    vector<long long int> &time_stamps=*time_stamps_p;
    for(size_t i=0;i<packets.size() && *playing;i++){
        cv::cvtColor(pv.visualize(packets[i]),rgb,CV_BGR2RGB);
        window->setImage(rgb);
        if(i<packets.size()-1)//to make the frame rate according to the time stamps
            this_thread::sleep_for(chrono::milliseconds((time_stamps[i+1]-time_stamps[i]-1)*s));
    }
}

void EventCam::play(long long int s){
    playing=true;
    player_t=thread(&EventCam::player,&playing,live_window,&timestamps,&packets,s);
}

void EventCam::start(){
    // Now let's get start getting some data from the device. We just loop in blocking mode,
    // no notification needed regarding new events. The shutdown notification, for example if
    // the device is disconnected, should be listened to.
    cam_handle->dataStart(nullptr, nullptr, nullptr, &usbShutdownHandler, nullptr);

    // Let's turn on blocking data-get mode to avoid wasting resources.
    cam_handle->configSet(CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, true);

    grabbing=true;
    recording=false;
    grabber_t=std::thread(&EventCam::grabber,num_containers,cam_handle.get(),&grabbing,&globalShutdown,&recording,&packets,&timestamps,live_window);
}

void EventCam::stop(){
    recording=false;
    grabbing=false;
    if(grabber_t.joinable()){
        grabber_t.join();
        cam_handle->dataStop();
    }

    playing=false;
    if(player_t.joinable())
        player_t.join();
}

void EventCam::record(){
    recording=true;
}

void EventCam::grabber(int num_containers, libcaer::devices::dvs128* cam_handle_p, atomic_bool *grabbing_p, atomic_bool *global_shutdown, atomic_bool* recording_p, std::vector<std::shared_ptr<libcaer::events::PolarityEventPacket>> *packets_p, std::vector<long long int> *packet_time_stamps_p, LiveView *lw){
    std::vector<std::unique_ptr<libcaer::events::EventPacketContainer>> packet_containers(num_containers);

    std::vector<long long int> container_time_stamps(num_containers,-1);
    long long int time_stamp,container_index=0;

    atomic_bool &grabbing=*grabbing_p,&recording=*recording_p, live_view;

    live_view=true;
    thread live_view_t(&EventCam::liveView,&packet_containers,&container_index,&live_view,lw);

    while(grabbing && !global_shutdown->load(memory_order_relaxed)) {

        packet_containers[container_index]=cam_handle_p->dataGet();
        time_stamp=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();

        if (packet_containers[container_index] == nullptr) {
            container_time_stamps[container_index]=-1;
            continue; // Skip if nothing there.
        }
        else{
            if(recording){
                container_time_stamps[container_index]=time_stamp;
            }
            else{
                container_time_stamps[container_index]=-1;
            }
            container_index=(container_index+1)%num_containers;//loop over the buffers
        }
    }

    live_view=false;
    live_view_t.join();

    multimap<long long int,std::shared_ptr<libcaer::events::PolarityEventPacket>> timestamps_packets;//to sort according to the timestamps

    for(int i=0;i<packet_containers.size();i++){
        if(container_time_stamps[i]<0)
            continue;

//        printf("\nGot event container with %d packets (allocated).\n",packet_containers[i]->size());

        for(int j=0;j<packet_containers[i]->size();j++){
            shared_ptr<libcaer::events::EventPacket> packet=(*packet_containers[i])[j];

            if (packet == nullptr){
//                printf("Packet is empty (not present).\n");
                continue;
            }

//            printf("Packet of type %d -> %d events, %d capacity.\n", packet->getEventType(), packet->getEventNumber(),
//                   packet->getEventCapacity());

            if (packet->getEventType() == POLARITY_EVENT) {
                timestamps_packets.insert(make_pair(container_time_stamps[i],std::static_pointer_cast<libcaer::events::PolarityEventPacket>(packet)));
            }
        }
    }

    for(auto p:timestamps_packets){
        (*packet_time_stamps_p).push_back(p.first);
        (*packets_p).push_back(p.second);
    }
}

std::vector<long long int> EventCam::getTimestamps(){
    return timestamps;
}

std::vector<std::shared_ptr<libcaer::events::PolarityEventPacket>> EventCam::getPackets(){
    return packets;
}

void EventCam::openCam(){
    cam_handle=shared_ptr<libcaer::devices::dvs128>(new libcaer::devices::dvs128(1, 0, 0, ""));
    // Install signal handler for global shutdown.
    #if defined(_WIN32)
        if (signal(SIGTERM, &globalShutdownSignalHandler) == SIG_ERR) {
            libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
                "Failed to set signal handler for SIGTERM. Error: %d.", errno);
            return (EXIT_FAILURE);
        }

        if (signal(SIGINT, &globalShutdownSignalHandler) == SIG_ERR) {
            libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
                "Failed to set signal handler for SIGINT. Error: %d.", errno);
            return (EXIT_FAILURE);
        }
    #else
        struct sigaction shutdownAction;

        shutdownAction.sa_handler = &globalShutdownSignalHandler;
        shutdownAction.sa_flags   = 0;
        sigemptyset(&shutdownAction.sa_mask);
        sigaddset(&shutdownAction.sa_mask, SIGTERM);
        sigaddset(&shutdownAction.sa_mask, SIGINT);

        if (sigaction(SIGTERM, &shutdownAction, NULL) == -1) {
            libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
                "Failed to set signal handler for SIGTERM. Error: %d.", errno);
            throw runtime_error("Failed to set signal handler for SIGTERM");
        }

        if (sigaction(SIGINT, &shutdownAction, NULL) == -1) {
            libcaer::log::log(libcaer::log::logLevel::CRITICAL, "ShutdownAction",
                "Failed to set signal handler for SIGINT. Error: %d.", errno);
            throw runtime_error("Failed to set signal handler for SIGINT");
        }
    #endif

        // Let's take a look at the information we have on the device.
        struct caer_dvs128_info dvs128_info = cam_handle->infoGet();

        printf("%s --- ID: %d, Master: %d, DVS X: %d, DVS Y: %d, Logic: %d.\n", dvs128_info.deviceString,
            dvs128_info.deviceID, dvs128_info.deviceIsMaster, dvs128_info.dvsSizeX, dvs128_info.dvsSizeY,
            dvs128_info.logicVersion);


        // Send the default configuration before using the device.
        // No configuration is sent automatically!
        cam_handle->sendDefaultConfig();

        //set slow paramets
        cam_handle->configSet(DVS128_CONFIG_BIAS, DVS128_CONFIG_BIAS_REQ, 159147);
        cam_handle->configSet(DVS128_CONFIG_BIAS, DVS128_CONFIG_BIAS_REFR, 6);
        cam_handle->configSet(DVS128_CONFIG_BIAS, DVS128_CONFIG_BIAS_DIFFON, 482443);
        cam_handle->configSet(DVS128_CONFIG_BIAS, DVS128_CONFIG_BIAS_DIFF, 30153);
        cam_handle->configSet(DVS128_CONFIG_BIAS, DVS128_CONFIG_BIAS_FOLL, 51);
        cam_handle->configSet(DVS128_CONFIG_BIAS, DVS128_CONFIG_BIAS_PR, 3);

        // Tweak some biases, to increase bandwidth in this case.
        cam_handle->configSet(DVS128_CONFIG_BIAS, DVS128_CONFIG_BIAS_PR, 695);
        cam_handle->configSet(DVS128_CONFIG_BIAS, DVS128_CONFIG_BIAS_FOLL, 867);

        // increase the buffer size and num buffer we have a lot of events!
        cam_handle->configSet(CAER_HOST_CONFIG_USB, CAER_HOST_CONFIG_USB_BUFFER_SIZE, 32768);
        cam_handle->configSet(CAER_HOST_CONFIG_USB, CAER_HOST_CONFIG_USB_BUFFER_NUMBER, 4);

        // Let's verify they really changed!
        uint32_t prBias   = cam_handle->configGet(DVS128_CONFIG_BIAS, DVS128_CONFIG_BIAS_PR);
        uint32_t follBias = cam_handle->configGet(DVS128_CONFIG_BIAS, DVS128_CONFIG_BIAS_FOLL);

        printf("New bias values --- PR: %d, FOLL: %d.\n", prBias, follBias);

}

EventCam::EventCam(LiveView *lv,int nc)
{
    globalShutdown=false;
    num_containers=nc;
    live_window=lv;
}
