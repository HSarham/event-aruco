#include <iostream>
#include "eventpacketprocessor.h"
#include "eventcam.h"
#include "imageeventsynchronizer.h"

using namespace std;

void print_usage(){
    cout<<"Args: <folder_path>"<<endl;
}

int main(int argc, char* argv[]){
    if(argc<2){
        print_usage();
        return -1;
    }
    string dataset_folder=argv[1];

    //---------debug
//    double c,d;
//    vector<double> alphas={0,1,2};
//    vector<double> betas={0.41,0.39,0.47};

//    EventPacketProcessor::linear_regression(alphas, betas, c, d);//beta_i=c*alpha_i+d
//    cout<<"c: "<<c<<endl;
//    cout<<"d: "<<d<<endl;

//    return 0;

    //--------debug

    EventPacketProcessor epp(cv::Size(128,128));
//    EventCam ec(NULL,0);
//    cout<<"Loading packets.."<<endl;
//    ec.loadPacketsFile(dataset_folder+"/packets.bin");
//    cout<<"Done!"<<endl;
//    vector<std::shared_ptr<libcaer::events::PolarityEventPacket>> packets=ec.getPackets();
//    vector<long long int> timestamps=ec.getTimestamps();

    ImageEventSynchronizer ies(dataset_folder);

    std::shared_ptr<libcaer::events::PolarityEventPacket> packet;
    cv::Mat rgb;
    size_t curr_index;
    while(ies.getNextFrame(packet,rgb,&curr_index)){
        cv::imshow("rgb",rgb);
        epp.update(packet,curr_index);
        cv::waitKey();
    }

//    for(int i=0;i<packets.size();i++){
//        if(i>0){
////            this_thread::sleep_for(std::chrono::milliseconds((timestamps[i]-timestamps[i-1])*(long long int)10));
//        }
//        epp.update(packets[i]);
//        cv::waitKey();
//    }

    return 0;
}
