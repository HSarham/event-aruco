#include <iostream>
#include "eventpacketprocessor.h"
#include "eventcam.h"
#include "imageeventsynchronizer.h"

using namespace std;

void print_usage(){
    cout<<"Args: <folder_path>"<<endl;
}

double average(const vector<double> &input){
    if(input.size()==0)
        throw runtime_error("The input array is empty.");

    double sum=0;
    for(double val:input){
        sum+=val;
    }
    return sum/input.size();
}

double median_sorted_input(const vector<double> &input){
    if(input.size()==0)
        throw runtime_error("The input array is empty.");

    size_t num_elems=input.size();
    double median=input[num_elems/2];
    if(num_elems%2==0){
        median = (median+input[num_elems/2-1])/2;
    }
    return median;
}

int main(int argc, char* argv[]){
    if(argc<2){
        print_usage();
        return -1;
    }

    int start_index=-1,end_index=-1;
    if(argc==4){
        start_index=stoi(argv[2]);
        end_index=stoi(argv[3]);
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
    char index_str[100];

    vector<double> durations;
    vector<double> packets_per_seconds;

    while(ies.getNextFrame(packet,rgb,&curr_index)){
        if(curr_index<start_index)
            continue;
        if(curr_index>end_index)
            break;
        if(curr_index==start_index)
            epp.update(packet,curr_index);
//        cv::imshow("rgb",rgb);
//        sprintf(index_str,"%04d",curr_index);
//        cv::imwrite("/home/hamid/event_aruco/"+string(index_str)+"_rgb.png",rgb);
        std::chrono::high_resolution_clock::time_point before=std::chrono::high_resolution_clock::now(),now;
        epp.update(packet,curr_index);
        now=std::chrono::high_resolution_clock::now();
        double duration=std::chrono::duration_cast<std::chrono::milliseconds>(now-before).count();
//        if(curr_index>=start_index){
//            packets_per_seconds.push_back(1/duration);
//            durations.push_back(duration);
//        }
//        before=now;
//        cv::waitKey();
    }

//    cout<<endl;
//    for(double d:durations){
//        cout<<d<<endl;
//    }

    std::sort(durations.begin(),durations.end());
    std::sort(packets_per_seconds.begin(),packets_per_seconds.end());

    cout<<"num packets: "<<durations.size()<<endl;

    cout<<"median duration: "<<median_sorted_input(durations)<<endl;
    cout<<"average duration: "<<average(durations)<<endl;
    cout<<"min duration: "<<durations[0]<<endl;
    cout<<"max durations: "<<durations[durations.size()-1]<<endl;

    cout<<"median pps: "<<median_sorted_input(packets_per_seconds)<<endl;
    cout<<"average pps: "<<average(packets_per_seconds)<<endl;
    cout<<"min pps: "<<packets_per_seconds[0]<<endl;
    cout<<"max pps: "<<packets_per_seconds[packets_per_seconds.size()-1]<<endl;


//    for(int i=0;i<packets.size();i++){
//        if(i>0){
////            this_thread::sleep_for(std::chrono::milliseconds((timestamps[i]-timestamps[i-1])*(long long int)10));
//        }
//        epp.update(packets[i]);
//        cv::waitKey();
//    }

    return 0;
}
