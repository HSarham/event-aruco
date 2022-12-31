#include "imageeventsynchronizer.h"
#include "filesystem.h"
#include <iostream>

using namespace std;

ImageEventSynchronizer::ImageEventSynchronizer(string in_folder_path)
{
    dataset_folder_path=in_folder_path;
    image_file_names=filesystem_extra::get_files_list(dataset_folder_path);
    std::sort(image_file_names.begin(),image_file_names.end());

    EventCam ec(NULL,0);
    cout<<"loading packet files.."<<flush;
    ec.loadPacketsFile(dataset_folder_path+"/packets.bin");
    cout<<"Done"<<endl;

    packets=ec.getPackets();
    packet_time_stamps=ec.getTimestamps();

    reset();
}

void ImageEventSynchronizer::reset(){
    curr_packet_index=0;
    curr_image_index=0;
    curr_image_time_stamp=0;
    end_of_images=false;
    end_of_packets=false;
}

bool ImageEventSynchronizer::goToPacket(size_t index, std::shared_ptr<libcaer::events::PolarityEventPacket> &io_packet, cv::Mat &io_image){
    if(index<curr_packet_index){
        reset();
    }

    bool ret_val=true;
    while(index>curr_packet_index)
        ret_val=getNextFrame(io_packet,io_image);

    return ret_val;
}

bool ImageEventSynchronizer::getNextFrame(std::shared_ptr<libcaer::events::PolarityEventPacket> &packet, cv::Mat &image, size_t *packet_index, size_t *image_index){

    if(end_of_packets)
        return false;

    if(packet_index!=NULL)
        *packet_index=curr_packet_index;
    if(image_index!=NULL)
        *image_index=curr_image_index;

    packet=packets[curr_packet_index];
    curr_packet_time_stamp=packet_time_stamps[curr_packet_index];

    curr_packet_index++;
    if(curr_packet_index>=packets.size())
        end_of_packets=true;

    while(curr_packet_time_stamp>curr_image_time_stamp && !end_of_images){//try to read the next image
        readNextImage();
    }

    if(end_of_images && curr_packet_time_stamp>curr_image_time_stamp){
        image=curr_im;
    }
    else{
        long long curr_im_ts_diff=curr_image_time_stamp-curr_packet_time_stamp;
        long long prev_im_ts_diff=curr_packet_time_stamp-prev_image_time_stamp;
        if(curr_im_ts_diff>prev_im_ts_diff)
            image=prev_im;
        else
            image=curr_im;

    }

    return true;
}

void ImageEventSynchronizer::readNextImage(){

    string file_name;
    cv::Mat im;
    while(im.empty() && curr_image_index<image_file_names.size()){//get the next image file
        file_name=image_file_names[curr_image_index++];
        im=cv::imread(dataset_folder_path+"/"+file_name);
    }

    if(curr_image_index>=image_file_names.size()){//the image sequence has ended
        end_of_images=true;
    }
    else{
        prev_im=curr_im;
        prev_image_time_stamp=curr_image_time_stamp;

        curr_im=im;
        curr_image_time_stamp=stoll(file_name.substr(0,file_name.find_last_of('.')));
    }

}
