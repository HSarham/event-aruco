#include <iostream>
#include <qt5/QtWidgets/QApplication>
#include "eventcam.h"
#include "idscam.h"

using namespace std;

void main_process(string *folder_path_p,LiveView *lvc, LiveView *lve){
    EventCam ec(lve,0);
    IDSCam idsc(lvc,0);
    ec.loadPacketsFile(*folder_path_p+"/packets.bin");
    cout<<"packets loaded"<<endl;
    ec.play(5);
    idsc.playAndLoad(*folder_path_p,5);
    cout<<"playing started"<<endl;
    bool exit=false;
    while(!exit){
        if(lve->s_pressed || lvc->s_pressed){
            exit=true;
        }
    }
    idsc.stop();
    ec.stop();
    lvc->close();
    lve->close();
}

void print_usage(){
    cout<<"Args: <path_to_folder>"<<endl;
}

int main(int argc, char* argv[]){


    if(argc<1){
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
