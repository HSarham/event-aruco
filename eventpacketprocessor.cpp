#include "eventpacketprocessor.h"
#include <iostream>
#include <opencv2/imgproc.hpp>
#include <opencv2/ximgproc.hpp>
#include <algorithm>
#include <typeinfo>
extern "C"{
#include "LSD/lsd.h"
}
using namespace std;

template<typename T>
void init_mat_fa_array(cv::Mat &mat, vector<T*> &arrays){
    arrays.resize(mat.rows);
    for(int r=0;r<mat.rows;r++){
        arrays[r]=mat.ptr<T>(r);
    }
}

void EventPacketProcessor::linear_regression(const vector<double> &alphas, const vector<double> &betas, double &c, double &d){//beta_i=c*alpha_i+d
    double sum_alpha=0;
    double sum_beta=0;
    double sum_alpha_beta=0;
    double sum_alpha_sq=0;
    size_t n=alphas.size();
    for(size_t i=0;i<n;i++){
        const double &alpha=alphas[i];
        const double &beta=betas[i];
        sum_alpha+=alpha;
        sum_beta+=beta;
        sum_alpha_beta+=alpha*beta;
        sum_alpha_sq+=alpha*alpha;
    }
    c=(n*sum_alpha_beta-sum_alpha*sum_beta)/(n*sum_alpha_sq-sum_alpha*sum_alpha);
    d=(sum_beta-c*sum_alpha)/n;
}

bool in_range(int input, int l, int h){
    return input>=l && input<h;
}

bool isEventful(const LineSegment &ls,const EventPacketProcessor::FastMat &mask){
    float x_length=ls.p2.x-ls.p1.x;
    float y_length=ls.p2.y-ls.p1.y;
    int steps=std::round(std::max(std::abs(x_length),std::abs(y_length)));
    float x_step=x_length/steps;
    float y_step=y_length/steps;

    int num_points=0;
    for(int i=0;i<=steps;i++){
        int x = std::round(ls.p1.x+x_step*i);
        int y = std::round(ls.p1.y+y_step*i);
        if(mask[y][x]){
            num_points++;
        }
    }
    return num_points>steps/2;
}

void EventPacketProcessor::find_line_offset(const EventFrame &age_frame, const LineSegment &ls, float goal_age, double &x_offset, double &y_offset){
    const cv::Point2f mid_point=(ls.p1+ls.p2)/2;

    double x1=ls.p1.x, y1=ls.p1.y, x2=ls.p2.x, y2=ls.p2.y;
    double x_diff=x2-x1;
    double neg_y_diff=y1-y2;
    double norm_sq_1=x1*x1+y1*y1, norm_sq_2=x2*x2+y2*y2;
    double a,b,c,d;
    vector<double> alphas;
    vector<double> betas;

    alphas.push_back(0);
    betas.push_back(calcLineAge(age_frame,ls));

    if(int(x_diff)==-20 && int(neg_y_diff)==-2){

    }

    if(std::abs(x_diff)>std::abs(neg_y_diff)){//x=a*y+b
        a=neg_y_diff/x_diff;
        b=(norm_sq_2-norm_sq_1)/(2*x_diff);

        //traverse both directions
        for(int dir=-1;dir<=1;dir+=2){
            for(int abs_dy=1;;abs_dy++){
                int dy=abs_dy*dir;
                double dx=a*dy;

                cv::Point2f p1(x1+dx,y1+dy);
                cv::Point2f p2(x2+dx,y2+dy);
                int x1_int=std::round(p1.x);
                int y1_int=std::round(p1.y);
                int x2_int=std::round(p2.x);
                int y2_int=std::round(p2.y);

                //if getting out of the image break the loop
                if(!in_range(x1_int,0,age_frame.cols()))
                    break;
                if(!in_range(y1_int,0,age_frame.rows()))
                    break;
                if(!in_range(x2_int,0,age_frame.cols()))
                    break;
                if(!in_range(y2_int,0,age_frame.rows()))
                    break;

                LineSegment new_ls(p1,p2,0,0);

                if(isEventful(new_ls,age_frame.mask)){
                    alphas.push_back(dy);
                    betas.push_back(calcLineAge(age_frame,new_ls));
                }
                else{
                    break;
                }
            }
        }

        linear_regression(alphas,betas,c,d);
        //0.5=c*alpha+d => alpha=(0.5-d)/c;
        y_offset=(goal_age-d)/c;
        x_offset=a*y_offset;
    }
    else{//y=a*x+b
        a=x_diff/neg_y_diff;
        b=(norm_sq_1-norm_sq_2)/(2*neg_y_diff);

        //traverse both directions
        for(int dir=-1;dir<=1;dir+=2){
            //in each direction go forward one pixel at a time
            for(int abs_dx=1;;abs_dx++){
                int dx=abs_dx*dir;
                double dy=a*dx;

                cv::Point2f p1(x1+dx,y1+dy);
                cv::Point2f p2(x2+dx,y2+dy);
                int x1_int=std::round(p1.x);
                int y1_int=std::round(p1.y);
                int x2_int=std::round(p2.x);
                int y2_int=std::round(p2.y);

                //if getting out of the image break the loop
                if(!in_range(x1_int,0,age_frame.cols()))
                    break;
                if(!in_range(y1_int,0,age_frame.rows()))
                    break;
                if(!in_range(x2_int,0,age_frame.cols()))
                    break;
                if(!in_range(y2_int,0,age_frame.rows()))
                    break;

                LineSegment new_ls(p1,p2,0,0);

                if(isEventful(new_ls,age_frame.mask)){
                    alphas.push_back(dx);
                    betas.push_back(calcLineAge(age_frame,new_ls));
                }
                else{
                    break;
                }
            }
        }

        linear_regression(alphas,betas,c,d);
        //0.5=c*alpha+d => alpha=(0.5-d)/c;
        x_offset=(goal_age-d)/c;
        y_offset=a*x_offset;
    }
    if(std::abs(y_offset)>50){

    }

}

void EventPacketProcessor::preprocess(EventFrame &io){
    EventFrame output(io.rows(),io.cols());

    for(int y=0;y<im_size.height;y++){
        int yn_min=std::max(0,y-1);
        int yn_max=std::min(im_size.height-1,y+1);
        for(int x=0;x<im_size.width;x++){
            int xn_min=std::max(0,x-1);
            int xn_max=std::min(im_size.width-1,x+1);

            int half_total=((yn_max-yn_min+1)*(xn_max-xn_min+1)-1)/2;
            int num_valids=0;
            float sum=0;
            for(int yn=yn_min;yn<=yn_max;yn++)
                for(int xn=xn_min;xn<=xn_max;xn++){
                    if(io.mask[yn][xn] && (yn!=y || xn!=x)){
                        num_valids++;
                        sum+=io.image[yn][xn];
                    }
                }
            if(num_valids>half_total && io.mask[y][x]==0){//at least half of the neighbours should have events to turn it on
                output.image[y][x]=sum/num_valids;
                output.mask[y][x]=1;
            }
            else if(num_valids<half_total && io.mask[y][x]>0){//if less than helf of the neighbours are on, it should turn off
                output.image[y][x]=0;
                output.mask[y][x]=0;
            }
            else{//otherwise the mask and the events will be the same as input
                output.image[y][x]=io.image[y][x];
                output.mask[y][x]=io.mask[y][x];
            }
        }
    }

    io=output;
}

EventPacketProcessor::EventPacketProcessor(cv::Size is)
{
    im_size=is;
    display_size=im_size*display_scale;

    beginning=std::chrono::high_resolution_clock::now();

    display_frame=cv::Mat::zeros(display_size,CV_8UC3);
    init_mat_fa_array<cv::Vec3b>(display_frame, display_frame_fa);

    //init with no value argument sets elements to zero
    on_frame.init(im_size);
    off_frame.init(im_size);

    on_min.init(im_size,cv::Scalar(std::numeric_limits<float>::max()));
    off_min.init(im_size,cv::Scalar(std::numeric_limits<float>::max()));

    on_one_min.init(im_size);
    off_one_min.init(im_size);

    frame_num=0;

    //create the dictionary
    for(int i=0;i<codes.size();i++)
        dictionary[codes[i]]=i;

    dvsn=std::make_shared<libcaer::filters::DVSNoise>(im_size.width,im_size.height);

//    vw.open("edges.avi",cv::VideoWriter::fourcc('X','2','6','4'),30,cv::Size(im_size.width*4,im_size.height));
}

cv::Point2f project_point_to_line(cv::Point p, LineSegment ls){
    cv::Point2f res(NAN,NAN);
    float diff_x=ls.p1.x-ls.p2.x;
    float diff_y=ls.p1.y-ls.p2.y;
    if(abs(diff_x)>abs(diff_y)){
        float a=diff_y/diff_x;
        float b=ls.p1.y-a*ls.p1.x;
        res.x=(p.y+p.x/a-b)/(a+1.0/a);
        res.y=a*res.x+b;
    }else{
        float a=diff_x/diff_y;
        float b=ls.p1.x-a*ls.p1.y;
        res.y=(p.x+p.y/a-b)/(a+1.0/a);
        res.x=a*res.y+b;
    }
    return res;
}


float distance_squared(cv::Point2f p1, cv::Point2f p2){
    float dx=p1.x-p2.x;
    float dy=p1.y-p2.y;
    return dx*dx+dy*dy;
}

bool point_is_between(cv::Point2f p0, cv::Point p1, cv::Point p2){
    float d0sq = distance_squared(p1,p2);
    float d1sq = distance_squared(p1,p0);
    float d2sq = distance_squared(p2,p0);

    return d1sq+d2sq <= d0sq;
}

bool EventPacketProcessor::projects_in(cv::Point p, const LineSegment &ls, float &d){
//    cv::Point2f pp=project_point_to_line(p, ls.r, ls.th);//projected point
    cv::Point2f pp=project_point_to_line(p, ls);
    if(point_is_between(pp, ls.p1, ls.p2)){
        float dx=pp.x-p.x, dy=pp.y-p.y;
        d=sqrt(dx*dx+dy*dy);
        return true;
    }
    d=-1;
    return false;
}

bool EventPacketProcessor::compatible_segments(const LineSegment &ls1, const LineSegment &ls2){
    float theta_diff=std::abs(ls1.th-ls2.th);

    if(theta_diff>180*deg_in_rad)//take the minimum difference in orientation
        theta_diff=360*deg_in_rad-theta_diff;

    if(theta_diff>90*deg_in_rad)
        theta_diff=180*deg_in_rad-theta_diff;

    if(theta_diff>30*deg_in_rad)//if the difference of the line thetas is greater than 30 degrees
        return false;

    float d,min_d=std::numeric_limits<float>::max();
    if(projects_in(ls1.p1,ls2,d)){
        return true;
        min_d=std::min(d,min_d);
    }
    if(projects_in(ls1.p2,ls2,d)){
        return true;
        min_d=std::min(d,min_d);
    }
    if(projects_in(ls2.p1,ls1,d)){
        return true;
        min_d=std::min(d,min_d);
    }
    if(projects_in(ls2.p2,ls1,d)){
        return true;
        min_d=std::min(d,min_d);
    }

//    if(min_d>MarkerCandidate::side_size)
//        return true;
    return false;
}

void EventPacketProcessor::find_marker_candidates(LineSegments &on_segments, LineSegments &off_segments, vector<MarkerCandidate> &result){
    result.clear();
    on_segments.sort();
    off_segments.sort();
    for(const LineSegment &on_ls: on_segments.lines){
        if(on_ls.length<min_segment_size)
            continue;
        for(const LineSegment &off_ls: off_segments.lines){
            if(off_ls.length<min_segment_size)
                continue;
            if(off_ls.length*2<on_ls.length)
                continue;
            if(off_ls.length>on_ls.length*2)
                continue;
            if(compatible_segments(on_ls,off_ls)){
                result.emplace_back(on_ls,off_ls);
            }
        }
    }
}

void fix_candidate_segement_dirs(vector<MarkerCandidate> &candidates){
    for(MarkerCandidate candidate:candidates){
        cv::Vec2f off_vec=candidate.off_ls.p2-candidate.off_ls.p1;
        cv::Vec2f on_vec=candidate.on_ls.p2-candidate.on_ls.p1;
        if(off_vec.dot(on_vec)<0){//change the direction of the off segment
            cv::Point2f tmp=candidate.off_ls.p1;
            candidate.off_ls.p1=candidate.off_ls.p2;
            candidate.off_ls.p2=tmp;
        }
    }
}

//void intersect_candidates_and_segments(const vector<MarkerCandidate> &candidates, ){

//}

void draw_marker_candidates(const cv::Mat &in_image, const vector<MarkerCandidate> &candidates, int scale, vector<cv::Mat> &candidate_images, size_t packet_index){
    size_t num_candidates=candidates.size();
    candidate_images.resize(num_candidates);
    for(size_t i=0;i<num_candidates;i++){
        const MarkerCandidate& candidate=candidates[i];
        cv::Mat image=in_image.clone();

//        cv::line(image,candidate.p[1]*scale,candidate.p[2]*scale,cv::Scalar(255,0,255),3);
//        cv::line(image,candidate.p[3]*scale,candidate.p[0]*scale,cv::Scalar(255,255,0),3);

        cv::line(image,candidate.p[0]*scale,candidate.p[1]*scale,cv::Scalar(255,255,255),2);
        cv::line(image,candidate.p[1]*scale,candidate.p[2]*scale,cv::Scalar(255,255,255),2);
        cv::line(image,candidate.p[2]*scale,candidate.p[3]*scale,cv::Scalar(255,255,255),2);
        cv::line(image,candidate.p[3]*scale,candidate.p[0]*scale,cv::Scalar(255,255,255),2);

//        cv::putText(image,string("on age: ")+to_string(candidate.on_ls.age),cv::Point(0,10),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(0,0,255));
//        cv::putText(image,string("index: ")+to_string(packet_index),cv::Point(150,10),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255));
//        cv::putText(image,string("off age: ")+to_string(candidate.off_ls.age),cv::Point(0,image.rows-10),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,0,0));
        candidate_images[i]=image;
    }
}

void unwarp_candidates(const cv::Mat &image_on, const cv::Mat &image_off, vector<MarkerCandidate> &candidates){

    for(size_t i=0;i<candidates.size();i++){
        cv::Mat pt=cv::getPerspectiveTransform(MarkerCandidate::coords,candidates[i].p);
        const std::vector<cv::Point2f> &p=candidates[i].p;
        candidates[i].area=std::abs((p[0].x*p[1].y-p[1].x*p[0].y)+(p[1].x*p[2].y-p[2].x*p[1].y)+(p[2].x*p[3].y-p[3].x*p[2].y)+(p[3].x*p[1].y-p[1].x*p[3].y))/2;
        candidates[i].transform=pt;
        cv::warpPerspective(image_on,candidates[i].image_on,pt,MarkerCandidate::marker_size,cv::INTER_LINEAR+cv::WARP_INVERSE_MAP);
        cv::warpPerspective(image_off,candidates[i].image_off,pt,MarkerCandidate::marker_size,cv::INTER_LINEAR+cv::WARP_INVERSE_MAP);
    }
}

void EventPacketProcessor::maskBlur(EventFrame &input){
    int rows=input.rows();
    int cols=input.cols();

    EventFrame smooth_input(rows,cols);
    cv::GaussianBlur(input.mask.mat,smooth_input.mask.mat,kernel_2D_size,kernel_sigma,0,cv::BORDER_ISOLATED);//blur the mask
    cv::GaussianBlur(input.image.mat,smooth_input.image.mat,kernel_2D_size,kernel_sigma,0,cv::BORDER_ISOLATED);//blur the image

    for(int r=0;r<rows;r++)
        for(int c=0;c<cols;c++){
            if(input.mask[r][c]>0){
                smooth_input.image[r][c]/=smooth_input.mask[r][c];
            }
            else{
                smooth_input.image[r][c]=0;
            }
        }

    input.image=smooth_input.image;
    //preserving the original mask but not the original image
}

void EventPacketProcessor::fill_in_frames(std::shared_ptr<const libcaer::events::PolarityEventPacket> pep, string video_path){

    std::chrono::high_resolution_clock::time_point before=std::chrono::high_resolution_clock::now(),now;

    int max_ts,max_ts_on,max_ts_off;
    max_ts=max_ts_on=max_ts_off=-1;

    int min_ts_off,min_ts_on;
    min_ts=min_ts_off=min_ts_on=std::numeric_limits<int>::max();

    long long int sum_ts,sum_ts_on,sum_ts_off;
    sum_ts=sum_ts_on=sum_ts_off=0;

    size_t num_ts_on,num_ts_off;
    num_ts_on=num_ts_off=0;

    cv::VideoWriter avw;
    if(!video_path.empty()){
        avw.open(video_path,cv::VideoWriter::fourcc('X','2','6','4'),30,display_size);
    }

    std::vector<int> time_stamps;

    for(const libcaer::events::PolarityEvent &pe:*pep){
        int ts=pe.getTimestamp();

        sum_ts += ts;
        if(ts>max_ts)   max_ts=ts;
        if(ts<min_ts)   min_ts=ts;

        time_stamps.push_back(ts);

        if(pe.getPolarity()){
            num_ts_on++;
            sum_ts_on+=ts;
            if(ts>max_ts_on)   max_ts_on=ts;
            if(ts<min_ts_on)   min_ts_on=ts;
        }
        else{
            num_ts_off++;
            sum_ts_off+=ts;
            if(ts>max_ts_off)   max_ts_off=ts;
            if(ts<min_ts_off)   min_ts_off=ts;
        }
    }

    now=std::chrono::high_resolution_clock::now();
//    cout<<"------step 0: "<<std::chrono::duration_cast<std::chrono::milliseconds>(now-before).count()<<"s"<<endl;
    before=now;

    //claculate statistics about the events*************************************
    size_t num_ts=time_stamps.size();
    //get the average
    av_ts=double(sum_ts)/num_ts;
    double av_ts_on=double(sum_ts_on)/num_ts_on;
    double av_ts_off=double(sum_ts_off)/num_ts_off;
    //get the median
    sort(time_stamps.begin(),time_stamps.end());
    size_t med_ts_index=num_ts/2;
    med_ts=0;

    if(!time_stamps.empty()){
        med_ts=time_stamps[med_ts_index];
        if(num_ts%2==0)
            med_ts=(med_ts+time_stamps[med_ts_index-1])/2;
    }

    double ts_interval=max_ts-min_ts;
    double half_ts_interval=ts_interval/2;
    mid_ts=min_ts+half_ts_interval;


    now=std::chrono::high_resolution_clock::now();
//    cout<<"------step 1: "<<std::chrono::duration_cast<std::chrono::milliseconds>(now-before).count()<<"s"<<endl;
    before=now;
    //put the appropriate normalized event time stamps in the corresponding matrices********************
    for(const libcaer::events::PolarityEvent &pe:*pep){

        int x=pe.getX();
        int y=pe.getY();
        int p_index;

        float ratio=(pe.getTimestamp()-min_ts)/ts_interval;

        if(pe.getPolarity()){
            p_index=2;
            on_frame.image[y][x] += ratio;
            on_frame.mask[y][x] += 1;
            on_min.image[y][x] = std::min(on_min.image[y][x],ratio);
        }
        else{
            p_index=0;
            off_frame.image[y][x] += ratio;
            off_frame.mask[y][x] += 1;
            off_min.image[y][x] = std::min(off_min.image[y][x],ratio);
        }

        //Video visualization of events appearing through time
        if(!video_path.empty()){
            int min_y=y*display_scale;
            int max_y=min_y+display_scale;
            int min_x=x*display_scale;
            int max_x=min_x+display_scale;

            for(int i=min_y;i<max_y;i++)
                for(int j=min_x;j<max_x;j++)
                    display_frame_fa[i][j][p_index]=255;
            avw<<display_frame;
            display_frame=display_frame*(127.0/128.0);
        }
    }

    now=std::chrono::high_resolution_clock::now();
//    cout<<"------step 2: "<<std::chrono::duration_cast<std::chrono::milliseconds>(now-before).count()<<"s"<<endl;
    before=now;
    //put the average in on_frame and off_frame, calculate on_one_min and off_one_min**************************************
    for(int i=0;i<im_size.height;i++)
        for(int j=0;j<im_size.width;j++){
            int min_x=j*display_scale;
            int max_x=min_x+display_scale;
            int min_y=i*display_scale;
            int max_y=min_y+display_scale;
            if(on_frame.mask[i][j]>0){
                on_one_min.mask[i][j]=on_min.mask[i][j]=1;
                on_one_min.image[i][j]=1.0-on_min.image[i][j];

                on_frame.image[i][j]/=on_frame.mask[i][j];//put the average ts in on frame
                for(int y=min_y;y<max_y;y++)
                    for(int x=min_x;x<max_x;x++)
                        display_frame_fa[y][x]=cv::Vec3b(0,0,on_min.image[i][j]*255);
            }
            else{
                on_one_min.mask[i][j]=on_min.mask[i][j]=0;
                //if the position is under the mask set on_min to 0 for better visualization
                on_min.image[i][j]=0;//because not initialized as 0
            }
            if(off_frame.mask[i][j]>0){
                off_min.mask[i][j]=off_one_min.mask[i][j]=1;
                off_one_min.image[i][j]=1.0-off_min.image[i][j];
                off_frame.image[i][j]/=off_frame.mask[i][j];//put the average ts in off frame
                for(int y=min_y;y<max_y;y++)
                    for(int x=min_x;x<max_x;x++)
                        display_frame_fa[y][x]=cv::Vec3b(off_min.image[i][j]*255,0,0);
            }
            else{
                off_min.mask[i][j]=off_one_min.mask[i][j]=0;
                //if the position is under the mask set off_min to 0 for better visualization
                off_min.image[i][j]=0;//because not initialized as 0
            }
        }

    now=std::chrono::high_resolution_clock::now();
//    cout<<"------step 3: "<<std::chrono::duration_cast<std::chrono::milliseconds>(now-before).count()<<"s"<<endl;
    before=now;

    preprocess(on_one_min);
    maskBlur(on_one_min);

    preprocess(off_one_min);
    maskBlur(off_one_min);

//    for(int i=0;i<im_size.height;i++)
//        for(int j=0;j<im_size.width;j++){
//            if(on_one_min.mask[i][j]>0){
//                on_min.image[i][j]=1.0-on_one_min.image[i][j];
//                on_min.mask[i][j]=1;
//            }
//            else{
//                on_min.image[i][j]=0;
//                on_min.mask[i][j]=0;
//            }

//            if(off_one_min.mask[i][j]>0){
//                off_min.image[i][j]=1.0-off_one_min.image[i][j];
//                off_min.mask[i][j]=1;
//            }
//            else{
//                off_min.image[i][j]=0;
//                off_min.mask[i][j]=0;
//            }
//        }

//    preprocess(off_min);
//    maskBlur(off_min);

//    preprocess(on_min);
//    maskBlur(on_min);
    now=std::chrono::high_resolution_clock::now();
//    cout<<"------step 4: "<<std::chrono::duration_cast<std::chrono::milliseconds>(now-before).count()<<"s"<<endl;
    before=now;
}

void EventPacketProcessor::erase_frames(std::shared_ptr<const libcaer::events::PolarityEventPacket> pep){
    on_frame.image.mat.setTo(cv::Scalar(0));
    on_frame.mask.mat.setTo(cv::Scalar(0));

    off_frame.image.mat.setTo(cv::Scalar(0));
    off_frame.mask.mat.setTo(cv::Scalar(0));

    on_one_min.image.mat.setTo(cv::Scalar(0));
    off_one_min.image.mat.setTo(cv::Scalar(0));

    on_min.image.mat.setTo(cv::Scalar(std::numeric_limits<float>::max()));
    off_min.image.mat.setTo(cv::Scalar(std::numeric_limits<float>::max()));

    display_frame.setTo(cv::Scalar(0,0,0));
}

void EventPacketProcessor::find_end_points(int max_x, int max_y, const vector<cv::Vec2f> &rhos_and_thetas, LineSegments &end_points){

    for(size_t i=0;i<rhos_and_thetas.size();i++){
        float rho=rhos_and_thetas[i][0];
        float theta=rhos_and_thetas[i][1];
        float sin_theta=std::sin(theta);
        float cos_theta=std::cos(theta);

        float a,b;

        vector<cv::Point> points;
        if(cos_theta!=0){
            cv::Point2f p1,p2;
            a=-sin_theta/cos_theta;
            b=rho/cos_theta;

            //point on the upper border
            p1.y=0;
            p1.x=std::round(b);

            p2.y=max_y;
            p2.x=std::round(a*p2.y+b);

            if(p1.x>=0 && p1.x<=max_x){// if the point on the upper border is not in the image then the one on the lower border is
                points.push_back(p1);
            }

            if(p2.x>=0 && p2.x<=max_x){
                points.push_back(p2);
            }
        }

        if(sin_theta!=0 && points.size()<2){

            cv::Point2f p1,p2;
            a=-cos_theta/sin_theta;
            b=rho/sin_theta;

            p1.x=0;
            p1.y=std::round(b);

            p2.x=max_x;
            p2.y=std::round(a*p2.x+b);

            if(p1.y>=0 && p1.y<=max_y){
                points.push_back(p1);
            }

            if(p2.y>=0 && p2.y<=max_y && points.size()<2){
                points.push_back(p2);
            }
        }
        end_points.insert(LineSegment(points[0],points[1],rho,theta));
    }
}

void EventPacketProcessor::segment_lines(cv::Mat m, const LineSegments &end_points, LineSegments &segments, int min_length, int max_gap_pixels){
    vector<char*> mat_array(m.rows);
    for(size_t i=0;i<m.rows;i++)
        mat_array[i]=m.ptr<char>(i);

    for(const LineSegment &line: end_points.lines){
        int dx=line.p2.x-line.p1.x;
        int dy=line.p2.y-line.p1.y;
        int num_steps=std::max(dx,dy);
        if(num_steps==0)
            continue;

        float x_step=float(dx)/num_steps;
        float y_step=float(dy)/num_steps;

        bool segment_started=false;
        int gap_pixels=0;

        cv::Point first_pixel,last_pixel;
        for(int s=0;s<num_steps;s++){
            int x=line.p1.x+std::round(x_step*s);
            int y=line.p1.y+std::round(y_step*s);

            if(mat_array[y][x]){
                gap_pixels=0;
                if(!segment_started){
                    segment_started=true;

                    first_pixel.x=x;
                    first_pixel.y=y;
                }
                last_pixel.x=x;
                last_pixel.y=y;
            }
            else{
                gap_pixels++;
            }

            if(segment_started && (gap_pixels>max_gap_pixels || s==num_steps-1)){
                segment_started=false;
                int s_dx=last_pixel.x-first_pixel.x;
                int s_dy=last_pixel.y-first_pixel.y;

                float length_sqr=s_dx*s_dx+s_dy*s_dy;
                if( length_sqr >= min_length*min_length){
                    segments.insert(LineSegment(first_pixel,last_pixel,line.r,line.th, std::sqrt(length_sqr)));
                }
            }
        }
    }
}

void EventPacketProcessor::draw_segments(cv::Mat &im, LineSegments &segments, cv::Scalar color, int scale){
    for(LineSegment &line: segments.lines)
        cv::line(im,line.p1*scale,line.p2*scale,color,7);
}

void EventPacketProcessor::candidate_detector_hough(const cv::Mat& edge_frame_filtered, const cv::Mat& on_frame_filtered, const cv::Mat& off_frame_filtered, cv::Mat& edge_frame_processed_big, LineSegments& on_segments, LineSegments& off_segments, bool display){
    cv::Mat edge_frame_processed;
    cv::ximgproc::thinning(edge_frame_filtered,edge_frame_processed);

    if(display){
        cv::resize(edge_frame_processed,edge_frame_processed_big,cv::Size(0,0),5,5,cv::INTER_NEAREST);
        cv::imshow("edge_frame_thinned",edge_frame_processed_big);
    }

    vector<cv::Vec2f> lines;
    LineSegments line_ends;

    cv::HoughLines(edge_frame_processed,lines,1,deg_in_rad,MarkerCandidate::side_size);

    find_end_points(im_size.width-1,im_size.height-1,lines,line_ends);

    segment_lines(on_frame_filtered,line_ends,on_segments);
    segment_lines(off_frame_filtered,line_ends,off_segments);

}

cv::Mat make_displayable(const cv::Mat &input, float scale, cv::Size size=cv::Size(0,0)){//takes a gray scale floating point image with values between zero and one, give displayable image
    cv::Mat output;
    if(size!=cv::Size(0,0))
        scale=0;
    input.copyTo(output);
    cv::cvtColor(input,output,cv::COLOR_GRAY2BGR);
    cv::resize(output,output,size,scale,scale,cv::INTER_NEAREST);
    return output;
}

float EventPacketProcessor::calcLineAge(const EventFrame &ei, const LineSegment& ls){
    float x_length=ls.p2.x-ls.p1.x;
    float y_length=ls.p2.y-ls.p1.y;
    int steps=std::round(std::max(std::abs(x_length),std::abs(y_length)));
    float x_step=x_length/steps;
    float y_step=y_length/steps;

    float average_age=0;
    int num_points=0;
    for(int i=0;i<=steps;i++){
        int x = std::round(ls.p1.x+x_step*i);
        int y = std::round(ls.p1.y+y_step*i);
        if(ei.mask[y][x]){
            average_age+=ei.image[y][x];
            num_points++;
        }
    }
    average_age/=num_points;
    return average_age;
}

void EventPacketProcessor::detect_line_segments(const EventFrame &input, LineSegments &output){
    int rows=input.rows();
    int cols=input.cols();
    cv::Mat image;
    input.image.mat.convertTo(image,CV_64FC1,255);
    int n_out;
    const int n_bins=1024;
    const double scale=0.8,sigma_scale=0.6,quant=2.0,ang_th=30,log_eps=0.0,density_th=0.5;

    double *res = LineSegmentDetection( &n_out, (double *)image.data, image.cols, image.rows, scale, sigma_scale, quant, ang_th, log_eps, density_th, n_bins, NULL, NULL, NULL);
    free((void*)res);

    cv::Rect2i image_rect(cv::Point2i(0,0),cv::Size(cols,rows));

    for(size_t i=1;i<n_out;i++){
        size_t index=i*7;
        cv::Point2f p1(res[index],res[index+1]),p2(res[index+2],res[index+3]);
        cv::Point2f diff=p2-p1;
        float theta=std::atan(diff.y/diff.x),length=cv::norm(diff);

        if(length<min_segment_size)
            continue;
//        p1+=(-diff)/length*4;//extend it 4 pixels
//        p2+=(diff)/length*4;//extend it 4 pixels

        if(image_rect.contains(p1) && image_rect.contains(p2)){
            LineSegment ls(p1,p2,0,theta,cv::norm(p1-p2));
            ls.age=calcLineAge(input, ls);
            output.insert(ls);
        }
    }
}

void EventPacketProcessor::move_segments(const EventFrame &age_frame, LineSegments &line_segments, float goal_age){
    for(LineSegment &ls:line_segments.lines){
        double x_offset,y_offset;
//        cout<<ls.age;
        find_line_offset(age_frame, ls, goal_age, x_offset, y_offset);
//        cout<<"x_offset: "<<x_offset<<endl;
//        cout<<"y_offset: "<<y_offset<<endl;
        if(x_offset>=0){
            x_offset=std::min(x_offset,(double)std::min(age_frame.cols()-1-ls.p2.x,age_frame.cols()-1-ls.p1.x));
        }
        else{
            x_offset=std::max(x_offset,(double)std::max(-ls.p2.x,-ls.p1.x));
        }
        if(y_offset>=0){
            y_offset=std::min(y_offset,(double)std::min(age_frame.rows()-1-ls.p2.y,age_frame.rows()-1-ls.p1.y));
        }
        else{
            y_offset=std::max(y_offset,(double)std::max(-ls.p2.y,-ls.p1.y));
        }

        ls.p1.x+=x_offset;
        ls.p2.x+=x_offset;
        ls.p1.y+=y_offset;
        ls.p2.y+=y_offset;

        ls.age=calcLineAge(age_frame, ls);
//        cout<<"===>"<<ls.age<<endl;
    }
}

void EventPacketProcessor::trim_segments(const EventFrame &age_frame, LineSegments &segments){
    for(LineSegment &segment:segments.lines){
        trim_segment(age_frame,segment);
    }
}

void EventPacketProcessor::expand_segments(const EventFrame &age_frame, LineSegments &segments){
    for(LineSegment &segment:segments.lines){
        expand_segment(age_frame,segment);
    }
}

void EventPacketProcessor::expand_segment(const EventFrame &age_frame, LineSegment &ls){
    float x_length=ls.p2.x-ls.p1.x;
    float y_length=ls.p2.y-ls.p1.y;
    int steps=std::round(std::max(std::abs(x_length),std::abs(y_length)));
    float x_step=x_length/steps;
    float y_step=y_length/steps;

    cv::Point2f start_point,end_point,prev_point;

    //expand the begining
    prev_point=ls.p1;
    for(int i=0;;i--){
        cv::Point2f curr_point(ls.p1.x+x_step*i,ls.p1.y+y_step*i);
        int x = std::round(curr_point.x);
        int y = std::round(curr_point.y);
        if(x>=0 && x<age_frame.cols() && y>=0 && y<age_frame.rows()){//if x and y are in range
            if(!age_frame.mask[y][x]){
                start_point=prev_point;
                break;
            }
        }
        else{
            start_point=prev_point;
            break;
        }
        prev_point=curr_point;
    }

    prev_point=ls.p2;
    for(int i=steps;;i++){
        cv::Point2f curr_point(ls.p1.x+x_step*i,ls.p1.y+y_step*i);
        int x = std::round(curr_point.x);
        int y = std::round(curr_point.y);
        if(x>=0 && x<age_frame.cols() && y>=0 && y<age_frame.rows()){//if x and y are in range
            if(!age_frame.mask[y][x]){
                end_point=prev_point;
                break;
            }
        }
        else{
            end_point=prev_point;
            break;
        }
        prev_point=curr_point;
    }

    ls.p1=start_point;
    ls.p2=end_point;
}

void EventPacketProcessor::trim_segment(const EventFrame &age_frame, LineSegment &ls){
    float x_length=ls.p2.x-ls.p1.x;
    float y_length=ls.p2.y-ls.p1.y;
    int steps=std::round(std::max(std::abs(x_length),std::abs(y_length)));
    float x_step=x_length/steps;
    float y_step=y_length/steps;

    bool has_started=false, has_ended=false, prev_point_mask=false;
    cv::Point2f start_point,end_point,prev_point,prev_prev_point;

    for(int i=0;i<=steps;i++){
        cv::Point2f curr_point(ls.p1.x+x_step*i,ls.p1.y+y_step*i);
        int x = std::round(curr_point.x);
        int y = std::round(curr_point.y);
        if(age_frame.mask[y][x]){
            if(!has_started && prev_point_mask){
                start_point=prev_point;
                has_started=true;
            }
            prev_point_mask=true;
        }
        else{
            if(has_started && !has_ended && !prev_point_mask){
                end_point=prev_prev_point;
                has_ended=true;
                break;
            }
            prev_point_mask=false;
        }
        prev_prev_point=prev_point;
        prev_point=curr_point;
    }

    if(has_started && has_ended){
        ls.p1=start_point;
        ls.p2=end_point;
    }

}

void EventPacketProcessor::update(std::shared_ptr<const libcaer::events::PolarityEventPacket> pep, size_t curr_index){

    char index_str[100];
    sprintf(index_str,"%04d",curr_index);

    frame_num++;
    if(frame_num%skip_every_frames!=0)
        return;

    std::chrono::high_resolution_clock::time_point before,now,very_begining;
    very_begining=std::chrono::high_resolution_clock::now();

    dvsn->apply(&(*pep));//denoise

    //draw changed pixels on the frame
    //print filter run time
//    now=std::chrono::high_resolution_clock::now();
//    cout<<std::chrono::duration_cast<std::chrono::milliseconds>(now-before).count()<<" ";
//    before=now;

//    if(cv::waitKey(0)=='c'){
//        fill_in_frames(pep/*,"frame.avi"*/);
//    }
//    else{
        fill_in_frames(pep);
//    }

    bool display = false;

    cv::Mat on_one_min_8uc, off_one_min_8uc;

    //print denoising and event image creation time
    before=std::chrono::high_resolution_clock::now();

    LineSegments on_segments,off_segments;
    on_one_min.image.mat.convertTo(on_one_min_8uc,CV_8UC1,255);
    off_one_min.image.mat.convertTo(off_one_min_8uc,CV_8UC1,255);
    //detect the line segments (also calculates the ages)
    detect_line_segments(on_one_min, on_segments);
    detect_line_segments(off_one_min, off_segments);

//    trim_segments(on_min, on_segments);
    move_segments(on_min, on_segments, 0.5);

//    trim_segments(off_min, off_segments);
    move_segments(off_min, off_segments, 0.5);

    vector<MarkerCandidate> candidates;
    find_marker_candidates(on_segments, off_segments, candidates);

    //print line segment detection, age correction and candidate formation time
    now=std::chrono::high_resolution_clock::now();
    cout<<std::chrono::duration_cast<std::chrono::milliseconds>(now-before).count()<<", ";
    before=now;

    //draw the line segments
    int scale=5;
    cv::Mat on_one_min_display;

    if(display){
        on_one_min_display=make_displayable(on_one_min_8uc,scale);
        draw_segments(on_one_min_display,on_segments,cv::Scalar(0,0,255),scale);
        cv::imwrite("/home/hamid/event_aruco/"+string(index_str)+"_on_one_min.png",on_one_min_display);
        cv::imshow("on_one_min",on_one_min_display);
    }

    cv::Mat off_one_min_display;

    if(display){
        off_one_min_display=make_displayable(off_one_min_8uc,scale);
        draw_segments(off_one_min_display,off_segments,cv::Scalar(255,0,0),scale);
        cv::imwrite("/home/hamid/event_aruco/"+string(index_str)+"_off_one_min.png",off_one_min_display);
        cv::imshow("off_one_min",off_one_min_display);
    }

    cv::Mat curr_display_frame;

    cv::Mat edge_frame_processed_big;
//    candidate_detector_hough(edge_frame_filtered, on_frame_filtered, off_frame_filtered, edge_frame_processed_big, on_segments, off_segments, display);
//    keylines2linesegments(on_min_keylines,on_segments);
//    keylines2linesegments(off_min_keylines,off_segments);

    cv::Mat display_frame_on_lines,display_frame_off_lines,display_frame_lines;
    cv::Mat display_frame_detections;

    vector<cv::Mat> candidate_images;

    cv::Mat on_min_8u,off_min_8u,on_frame_8u,off_frame_8u;

    on_min.image.mat.convertTo(on_min_8u,CV_8UC1,255);
    off_min.image.mat.convertTo(off_min_8u,CV_8UC1,255);
//    on_frame.image.mat.convertTo(on_frame_8u,CV_8UC1,255);
//    off_frame.image.mat.convertTo(off_frame_8u,CV_8UC1,255);

    cv::Mat im_show_off, im_show_on;
    cv::Mat code_image, candidates_im;
    cv::Mat image;

    if(display){
        curr_display_frame=cv::Mat(display_frame.rows,0,CV_8UC3);
        cv::resize(off_frame.image.mat,display_frame_off_lines,cv::Size(0,0),display_scale,display_scale,cv::INTER_NEAREST);
        cv::resize(on_frame.image.mat,display_frame_on_lines,cv::Size(0,0),display_scale,display_scale,cv::INTER_NEAREST);

        display_frame.copyTo(display_frame_detections);
        display_frame.copyTo(display_frame_lines);

        draw_marker_candidates(display_frame,candidates,display_scale,candidate_images,curr_index);

        cv::imshow("candidates",display_frame_detections);


        draw_segments(display_frame_lines,off_segments,cv::Scalar(255,0,0),display_scale);
        draw_segments(display_frame_lines,on_segments,cv::Scalar(0,0,255),display_scale);

        cv::hconcat(curr_display_frame,display_frame_lines/*display_frame_detections*/,curr_display_frame);

        image=cv::Mat(128*5,128*5,CV_8UC3,cv::Scalar(255,255,255));

        candidates_im=cv::Mat(0,display_size.width,CV_8UC3);

        cv::Mat on_min_mask=make_displayable(on_min.mask.mat,5);
        cv::Mat on_one_min_mask=make_displayable(on_one_min.mask.mat,5);
        cv::Mat on_one_min_image=make_displayable(on_one_min.image.mat,5);
        cv::Mat on_min_image=make_displayable(on_min.image.mat,5);
        cv::Mat on_mask=make_displayable(on_frame.mask.mat,5);

        cv::imshow("on_min_mask",on_min_mask);
        cv::imshow("on_one_min_mask",on_one_min_mask);
        cv::imshow("on_min",on_min_image);
        cv::imshow("one_one_min",on_one_min_image);
        cv::imshow("on_frame_mask",on_mask);
    }

    unwarp_candidates(on_min_8u,off_min_8u,candidates);
    //print unwarping time
    now=std::chrono::high_resolution_clock::now();
    cout<<std::chrono::duration_cast<std::chrono::milliseconds>(now-before).count()<<", ";
    before=now;

//    std::sort(candidates.begin(),candidates.end(),[](const MarkerCandidate& a, const MarkerCandidate& b)->bool{return a.area < b.area;});

    for(size_t i=0;i<candidates.size();i++){
        candidates[i].decode_candidate();

        if(display/* && i<5*/){
            cv::Mat candidates_row(display_size.height,0,CV_8UC3);

            im_show_off=make_displayable(candidates[i].image_off,0,display_size);
            im_show_on=make_displayable(candidates[i].image_on,0,display_size);
            code_image=make_displayable(candidates[i].code_im*255,0,display_size);

            cv::hconcat(candidates_row,display_frame,candidates_row);
            cv::hconcat(candidates_row,candidate_images[i],candidates_row);
            cv::hconcat(candidates_row,im_show_on,candidates_row);
            cv::hconcat(candidates_row,im_show_off,candidates_row);

            float eightth_height=display_size.height/8.0;
            float eightth_width=display_size.width/8.0;
            for(int i=1;i<8;i++){
                int width=1;
//                if(i==0)
//                    width=3;
                cv::line(im_show_off,cv::Point(eightth_height,eightth_height*i),cv::Point(display_size.width-eightth_height,eightth_height*i),cv::Scalar(0,0,255),width);
                cv::line(im_show_on,cv::Point(eightth_height,eightth_height*i),cv::Point(display_size.width-eightth_height,eightth_height*i),cv::Scalar(0,0,255),width);
                cv::line(code_image,cv::Point(eightth_height,eightth_height*i),cv::Point(display_size.width-eightth_height,eightth_height*i),cv::Scalar(0,0,255),width);
                cv::line(im_show_off,cv::Point(eightth_width*i,eightth_height),cv::Point(eightth_width*i,display_size.height-eightth_height),cv::Scalar(0,0,255),width);
                cv::line(im_show_on,cv::Point(eightth_width*i,eightth_height),cv::Point(eightth_width*i,display_size.height-eightth_height),cv::Scalar(0,0,255),width);
                cv::line(code_image,cv::Point(eightth_width*i,eightth_height),cv::Point(eightth_width*i,display_size.height-eightth_height),cv::Scalar(0,0,255),width);
            }

            cv::Mat off_scores=candidates[i].off_score_im;
            for(int r=1;r<off_scores.rows-1;r++)
                for(int c=1;c<off_scores.cols-1;c++)
                    cv::putText(im_show_off,to_string(off_scores.at<uchar>(r,c)),cv::Point(eightth_width*(c)+eightth_width/4,eightth_height*r+eightth_height/2),cv::FONT_HERSHEY_PLAIN,0.75,cv::Scalar(0,255,0),1);

            cv::Mat on_scores=candidates[i].on_score_im;
            for(int r=1;r<on_scores.rows-1;r++)
                for(int c=1;c<on_scores.cols-1;c++)
                    cv::putText(im_show_on,to_string(on_scores.at<uchar>(r,c)),cv::Point(eightth_width*(c)+eightth_width/4,eightth_height*r+eightth_height/2),cv::FONT_HERSHEY_PLAIN,0.75,cv::Scalar(0,255,0),1);

            cv::hconcat(candidates_row,im_show_on,candidates_row);
            cv::hconcat(candidates_row,im_show_off,candidates_row);
            cv::hconcat(candidates_row,code_image,candidates_row);

            candidates_im.push_back(candidates_row);
//            vw<<curr_display_frame;

        }

        for(int o=0;o<4;o++){//rotates clockwise
            uint64_t curr_code=candidates[i].code[o];
            if(dictionary.count(curr_code)){
                candidates[i].orientation=o;
                candidates[i].marker_index=dictionary[curr_code];
            }
        }

        if(display && candidates[i].marker_index!=-1){
            cv::imwrite("/home/hamid/event_aruco/"+string(index_str)+"_decoding_on.png", im_show_on);
            cv::imwrite("/home/hamid/event_aruco/"+string(index_str)+"_decoding_off.png", im_show_off);
            cv::imwrite("/home/hamid/event_aruco/"+string(index_str)+"_candidate_image.png",candidate_images[i]);

            candidates[i].code_im.copyTo(code_image);

            cv::resize(code_image*255,code_image,MarkerCandidate::marker_size,0,0,cv::INTER_NEAREST);
            cv::cvtColor(code_image,code_image,cv::COLOR_GRAY2BGR);
            cv::warpPerspective(code_image,image,candidates[i].transform,cv::Size(128,128),cv::INTER_NEAREST,cv::BORDER_CONSTANT,cv::Scalar(255,255,255));
            cv::resize(code_image,code_image,cv::Size(640,640),0,0,cv::INTER_NEAREST);
            if(display){
                cv::resize(im_show_on,im_show_on,cv::Size(640,640),0,0,cv::INTER_NEAREST);
                cv::resize(im_show_off,im_show_off,cv::Size(640,640),0,0,cv::INTER_NEAREST);
            }
            cv::putText(image,to_string(candidates[i].marker_index),cv::Point(10,10),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(0,255,0));
            cv::resize(image,image,cv::Size(0,0),5,5,cv::INTER_NEAREST);
            break;
        }
    }

    //marker reconstruction and code lookup time
    now=std::chrono::high_resolution_clock::now();
    cout<<std::chrono::duration_cast<std::chrono::milliseconds>(now-before).count()<<", ";
    before=now;

    if(display && candidates_im.rows>0){
        cv::imwrite("/home/hamid/event_aruco/"+string(index_str)+"_candidates_im.png",candidates_im);
        cv::imshow("candidates_im",candidates_im);
    }

    if(display){
        cv::imwrite("/home/hamid/event_aruco/"+string(index_str)+"_display_frame.png",display_frame);
        cv::imwrite("/home/hamid/event_aruco/"+string(index_str)+"_image.png",image);
        cv::imshow("image",image);
    }

    char c=0;

    if(display)
        cv::waitKey(5);

//    if(c=='s'){//save the images
//        cv::imwrite("/home/hamid/event_aruco/"+to_string(frame_num)+"_events.png",display_frame);
//        cv::imwrite("/home/hamid/event_aruco/"+to_string(frame_num)+"_edge_frame_processed_big.png",edge_frame_processed_big);
//        cv::imwrite("/home/hamid/event_aruco/"+to_string(frame_num)+"_frame_candidates.png",display_frame_detections);
//        cv::imwrite("/home/hamid/event_aruco/"+to_string(frame_num)+"_line_segments.png",display_frame_lines);
//        cv::imwrite("/home/hamid/event_aruco/"+to_string(frame_num)+"_unwarped_off.png",im_show_on);
//        cv::imwrite("/home/hamid/event_aruco/"+to_string(frame_num)+"_unwarped_on.png",im_show_off);
//        cv::imwrite("/home/hamid/event_aruco/"+to_string(frame_num)+"_code_image.png",code_image);
//        cv::imwrite("/home/hamid/event_aruco/"+to_string(frame_num)+"_final_overlay.png",image);
//    }

    erase_frames(pep);

    now=std::chrono::high_resolution_clock::now();
    cout<<std::chrono::duration_cast<std::chrono::milliseconds>(now-very_begining).count()<<endl;
    before=now;
}
