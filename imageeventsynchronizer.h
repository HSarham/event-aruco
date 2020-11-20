#ifndef IMAGEEVENTSYNCHRONIZER_H
#define IMAGEEVENTSYNCHRONIZER_H

#include "eventcam.h"

class ImageEventSynchronizer
{
    std::string dataset_folder_path;
    std::vector<std::string> image_file_names;

    long long int curr_image_time_stamp, prev_image_time_stamp, curr_packet_time_stamp;
    std::vector<std::shared_ptr<libcaer::events::PolarityEventPacket>> packets;
    std::shared_ptr<libcaer::events::PolarityEventPacket> curr_packet;
    size_t curr_image_index,curr_packet_index;
    std::vector<long long int> packet_time_stamps;
    cv::Mat curr_im,prev_im;
    bool end_of_images, end_of_packets;
    void readNextImage();
public:
    ImageEventSynchronizer(std::string in_folder_path);
    bool getNextFrame(std::shared_ptr<libcaer::events::PolarityEventPacket> &packet, cv::Mat &image, size_t *packet_index=NULL, size_t *image_index=NULL);
    bool goToPacket(size_t index, std::shared_ptr<libcaer::events::PolarityEventPacket> &io_packet, cv::Mat &io_image);
    void reset();
};

#endif // IMAGEEVENTSYNCHRONIZER_H
