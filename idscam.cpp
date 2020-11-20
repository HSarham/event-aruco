#include "idscam.h"
#include <iostream>
#include <vector>
#include <opencv2/highgui.hpp>
#include <chrono>
#include <opencv2/imgproc.hpp>
#include "filesystem.h"

using namespace std;

void check_ret_val(INT ret_val,string message){
    if(ret_val != IS_SUCCESS){
        throw runtime_error(message + " successfully.");
    }
}

void IDSCam::grabber(vector<cv::Mat>* frames_p, atomic_bool *live_view, atomic_bool *recording, vector<long long int> *t_sts, HIDS *cam_handle, LiveView *lw){

    const vector<cv::Mat> &frames=*frames_p;
    vector<long long int> &time_stamps=*t_sts;
    int num_frames=frames.size();

    int mem_index;
    cv::Mat rgb;
    long long int time_stamp;

    while(*live_view){
        //get the current memory index and time stamp
        is_WaitEvent(*cam_handle,IS_SET_EVENT_FIRST_PACKET_RECEIVED,1000);
        time_stamp=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch()).count();
        is_GetActSeqBuf(*cam_handle,&mem_index,NULL,NULL);
//        cout<<mem_index<<endl;
        if(mem_index!=0){
            //set the time_stamp
            if(*recording)
                time_stamps[mem_index-1]=time_stamp;
            else
                time_stamps[mem_index-1]=-1;
            //show the previous frame since current frame is not fully received
            mem_index=(mem_index-2)%num_frames;
            if(mem_index<0)
                mem_index+=num_frames;
            cv::cvtColor(frames[mem_index],rgb,cv::COLOR_BGR2RGB);
            lw->setImage(rgb);

        }
    }

    double frame_rate;
    is_GetFramesPerSecond(*cam_handle,&frame_rate);
    cout<<"recording frame rate: "<<frame_rate<<endl;
}

void IDSCam::start(){
    is_CaptureVideo(cam_handle,IS_DONT_WAIT);
    live_view=true;
    liveView_t=thread(&IDSCam::grabber,&frames,&live_view,&recording,&time_stamps,&cam_handle,liveWindow);
}

void IDSCam::stop(){
    recording=false;
    if(stamper_t.joinable())
        stamper_t.join();

    buffer_index=0;

    live_view=false;
    if(liveView_t.joinable())
    liveView_t.join();

    is_StopLiveVideo(cam_handle,IS_DONT_WAIT);
}

void IDSCam::record(){
    recording=true;
    cout<<"Started recording the timestamps."<<endl;
}

void IDSCam::player(atomic_bool *playing, LiveView* window, string folder_path, vector<long long int> *time_stamps_p, vector<cv::Mat> *frames_p, long long int s){
    cv::Mat rgb;
    if(folder_path.empty()){//just show what is in the arrays
        vector<cv::Mat> &frames=*frames_p;
        vector<long long int> &time_stamps=*time_stamps_p;
        for(size_t i=0;i<frames.size() && *playing;i++){
            cv::cvtColor(frames[i],rgb,CV_BGR2RGB);
            window->setImage(rgb);
            if(i<frames.size()-1)//to make the frame rate according to the time stamps
                this_thread::sleep_for(chrono::milliseconds((time_stamps[i+1]-time_stamps[i]-1)*s));
        }
    }
    else{//play the frames from file

        bool fill_arrays = time_stamps_p!=NULL && frames_p!=NULL;//load the frames into the arrays if possible

        if(fill_arrays){
            frames_p->clear();
            time_stamps_p->clear();
        }

        //get the list of files and sort them
        std::vector<string> file_names=filesystem::get_files_list(folder_path);
        std::sort(file_names.begin(),file_names.end());

        long long int prev_ts=-1;
        for(string file_name:file_names){
            cv::Mat im=cv::imread(folder_path+"/"+file_name);//try to open the files as images
            if(!im.empty()){
                //get the timestamp from filename
                string tss=file_name.substr(0,file_name.find_last_of('.'));
                long long int ts=stoll(tss);
                if(prev_ts!=-1)//to make the frame rate almost according to the time stamps
                    this_thread::sleep_for(chrono::milliseconds((ts-prev_ts-1)*s));
                //update the prev_ts
                prev_ts=ts;
                //show the image
                cv::cvtColor(im,rgb,CV_BGR2RGB);
                window->setImage(rgb);

                //take the timestamp
                cout<<tss<<endl;
                if(fill_arrays){//load the frames into the arrays if possible
                    time_stamps_p->push_back(ts);
                    frames_p->push_back(im);
                }
            }
        }

    }
}

void IDSCam::play(string folder_path, long long int s){
    playing=true;
    player_t=thread(&IDSCam::player,&playing,liveWindow,folder_path,(vector<long long int> *)NULL,(vector<cv::Mat> *)NULL,s);
}

void IDSCam::playAndLoad(string folder_path, long long int s){
    playing=true;
    player_t=thread(&IDSCam::player,&playing,liveWindow,folder_path,&time_stamps,&frames,s);
}

void IDSCam::play(long long int s){
    playing=true;
    player_t=thread(&IDSCam::player,&playing,liveWindow,string(),&time_stamps,&frames,s);
}

void IDSCam::loadFrames(string folder_path){
    stop();

    frames.clear();
    time_stamps.clear();

    std::vector<string> file_names=filesystem::get_files_list(folder_path);

    std::sort(file_names.begin(),file_names.end());

    for(string file_name:file_names){
        cv::Mat im=cv::imread(folder_path+"/"+file_name);
        if(!im.empty()){
            string tss=file_name.substr(0,file_name.find_last_of('.'));
            cout<<tss<<endl;
            time_stamps.push_back(stoll(tss));
            frames.push_back(im);
        }
    }
    num_frames=frames.size();
}

void IDSCam::saveFrames(string folder_path){
    for(int i=0;i<num_frames;i++){
//        cout<<time_stamps[i]<<endl;
        if(time_stamps[i]!=-1){
            string image_path=folder_path+"/"+to_string(time_stamps[i])+".png";
            cout<<"writing: "+image_path<<endl;
            cv::imwrite(image_path,frames[i]);
        }
    }
}


IDSCam::~IDSCam(){
    if(player_t.joinable())
        player_t.join();
    if(liveView_t.joinable())
        liveView_t.join();
    if(liveWindow!=NULL)
        liveWindow->close();
    check_ret_val(is_ExitCamera(cam_handle),"Could not exit camera with the given id");
}

void IDSCam::openCam(){
    INT num_cams;
    check_ret_val(is_GetNumberOfCameras(&num_cams),"Could not get number of the cameras");
    if(num_cams<1){
        throw runtime_error("No cameras are connected.");
    }

    vector<BYTE> cam_list_vec(sizeof(DWORD)+num_cams*sizeof(UEYE_CAMERA_INFO));

    UEYE_CAMERA_LIST* cam_list=(UEYE_CAMERA_LIST*)cam_list_vec.data();
    cam_list->dwCount=num_cams;

    check_ret_val(is_GetCameraList(cam_list),"Could not get the camera list");
    cout<<"camera ids: ";
    for(int i=0;i<cam_list->dwCount;i++){
        cout<<cam_list->uci[i].dwCameraID<<" ";
    }
    cout<<endl;

    //Open the camera
    cam_handle = 0;
    check_ret_val(is_InitCamera(&cam_handle,NULL),"Could not open camera with the given id");

    //get the width and height
    SENSORINFO sensor_info;
    check_ret_val(is_GetSensorInfo(cam_handle,&sensor_info),"Could not get sensor info");
    INT height=sensor_info.nMaxHeight, width=sensor_info.nMaxWidth;

    //enable 60 fps
    UINT clock_fq=28;
    is_PixelClock(cam_handle,IS_PIXELCLOCK_CMD_SET,(void*)&clock_fq,sizeof(clock_fq));
    double frame_rate;
    is_SetFrameRate(cam_handle,60,&frame_rate);
    cout<<"frame rate set to: "<<frame_rate<<endl;

    //create the memories and add them to the sequence
    for(int i=0;i<num_frames;i++){
        frames[i].create(height,width,CV_8UC3);
        is_SetAllocatedImageMem(cam_handle,width,height,24,(char*)frames[i].data,&mem_ids[i]);
        is_AddToSequence(cam_handle,(char*)frames[i].data,mem_ids[i]);
    }

    //enable the desired events
    is_EnableEvent(cam_handle,IS_SET_EVENT_FIRST_PACKET_RECEIVED);
//    is_EnableEvent(cam_handle,IS_SET_EVENT_FRAME_RECEIVED);

}

IDSCam::IDSCam(LiveView *lw, int nf)
{
    liveWindow=lw;
    recording = false;
    buffer_index=0;
    num_frames=nf;


    //resize the arrays
    frames.resize(num_frames);
    mem_ids.resize(num_frames);
    mem_indices.resize(num_frames);
    time_stamps.resize(num_frames,-1);

}
