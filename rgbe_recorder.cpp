#include <iostream>
#include <vector>
#include <qt5/QtWidgets/QApplication>
#include "liveview.h"
#include "idscam.h"
#include "eventcam.h"

using namespace std;

void main_process(string *folder_path, LiveView *lvc, LiveView *lve){


    IDSCam idsr(lvc,3600);//3600 frames which is almost 1 minutes
    EventCam ec(lve,6000);//6000 packet containers for almost 1 minutes
    cout<<"opening the cameras.."<<std::flush;
    idsr.openCam();
    ec.openCam();
    cout<<"done!"<<endl;
    cout<<"starting the cameras.."<<std::flush;
    idsr.start();
    ec.start();
    cout<<"started!"<<endl;

    bool exit=false;
    bool recording=false;
    while(!exit){
        if(lve->r_pressed || lvc->r_pressed){
            if(!recording){
                recording=true;
                ec.record();
                idsr.record();
            }
        }
        if(lve->s_pressed || lvc->s_pressed){
            exit=true;
            recording=false;
        }
    }
    idsr.stop();
    ec.stop();
    cout<<"saving frames"<<endl;
    idsr.saveFrames(*folder_path);
    ec.writePacketsToFile(*folder_path+"/packets.bin");
    cout<<"finished saving"<<endl;
    ec.loadPacketsFile(*folder_path+"/packets.bin");
    cout<<"showing the saved events!"<<endl;
}

void print_usage(){
    cout<<"Args: <path_to_folder>"<<endl;
}

int main(int argc, char* argv[]){
    if(argc<2){
        print_usage();
        return -1;
    }
    string folder_path(argv[1]);

    QApplication my_q(argc,argv);
    LiveView lvc("Color Camera"),lve("Event Camera");
    lvc.show();
    lve.show();

    thread main_process_t(&main_process,&folder_path,&lvc,&lve);
    my_q.exec();
    main_process_t.join();

    return 0;
}
