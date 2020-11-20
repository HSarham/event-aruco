#ifndef MARKERCANDIDATE_H
#define MARKERCANDIDATE_H
#include <opencv2/imgproc.hpp>
#include <list>

struct LineSegment{

    float r,th;
    int event_type;

    float age;
    LineSegment(){}
    LineSegment(const cv::Point2f &in_p1, const cv::Point2f &in_p2, float rho, float theta, float l=-1){
        point1=in_p1;
        point2=in_p2;
        r=rho;
        th=theta;
    }
    LineSegment(const LineSegment &input){
        r=input.r;
        th=input.th;
        event_type=input.event_type;
        age=input.age;
        point1=input.p1();
        point2=input.p2();
    }
    cv::Point2f p1() const{
        return point1;
    }
    cv::Point2f p2() const{
        return point2;
    }
    void set_p1(cv::Point2f in_p){
        point1=in_p;
        len=-1;
    }
    void set_p2(cv::Point2f in_p){
        point2=in_p;
        len=-1;
    }
    float length() const{
        if(len<0){
            len=cv::norm(point1-point2);
        }
        return len;
    }
    bool operator < (const LineSegment& ls2) const{
        return len < ls2.len;
    }
private:
    cv::Point2f point1,point2;
    mutable float len=-1;
};

struct LineSegments{
    std::list <LineSegment> lines;
//    std::map <float,LineSegment*> rho_map;
//    std::map <float,LineSegment*> theta_map;
    void insert(const LineSegment &ls){
        lines.push_front(ls);
//        rho_map[ls.r]=&lines.front();
//        theta_map[ls.th]=&lines.front();
    }
    void sort(){
        lines.sort();
    }
};

class MarkerCandidate
{
public:

    const static int cell_size=20;
    const static int border_radius=1;//cell_size/3;
    const static int threshold_on_pixels=55;//(255*cell_size/3.0)*1.0/3.0;//(cell_size*(border_radius*2+1))/2;
    const static int threshold_off_pixels=55;//(255*cell_size/3.0)*1.0/3.0;//(cell_size*(border_radius*2+1))/2;
    const static int num_cells=8;
    const static int side_size=cell_size*num_cells;
    const static cv::Size marker_size;
    const static std::vector<cv::Point2f> coords;

    LineSegment on_ls,off_ls;

    std::vector<cv::Point2f> p;
    int off_side=-1;
    std::vector<uchar*> image_off_fa;
    std::vector<uchar*> image_on_fa;
    cv::Mat image_on;
    cv::Mat image_off;
    std::vector<uint64_t> code=std::vector<uint64_t>(4);
    int orientation=-1;
    int marker_index=-1;
    cv::Mat code_im;
    cv::Mat on_score_im;
    cv::Mat off_score_im;
    std::vector<uchar*> code_im_fa;
    cv::Mat transform;

    MarkerCandidate(const LineSegment &in_on_ls, const LineSegment &in_off_ls);//get the corners and fix their order
    void decode_candidate();

};

#endif // MARKERCANDIDATE_H
