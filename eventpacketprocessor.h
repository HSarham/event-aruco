#ifndef EVENTPACKETPROCESSOR_H
#define EVENTPACKETPROCESSOR_H

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/line_descriptor/descriptor.hpp>
#include <chrono>
#include <libcaercpp/devices/dvs128.hpp>
#include <libcaercpp/filters/dvs_noise.hpp>
#include <map>
#include "markercandidate.h"

//ARUCO_MIP_36h12

class EventPacketProcessor
{
public:
    struct FastMat{
        cv::Mat mat;
        std::vector<float*> array;
        int rows() const{return mat.rows;}
        int cols() const{return mat.cols;}
        //operator "=" for deep copying
        FastMat &operator=(const FastMat & rhs){
            rhs.mat.copyTo(mat);
            init_array();
            return *this;
        }
        FastMat(int rows=0,int cols=0, cv::Scalar v=cv::Scalar(0)){
            init(rows,cols,v);
        }
        FastMat(cv::Size s, cv::Scalar v=cv::Scalar(0)){
            init(s,v);
        }
        float* operator [](size_t i){
            return array[i];
        }
        const float* operator [](size_t i) const{
            return array[i];
        }
        void init(cv::Size s, cv::Scalar v=cv::Scalar(0)){
            init(s.height,s.width,v);
        }
        void init(int rows, int cols, cv::Scalar v=cv::Scalar(0)){
            mat=cv::Mat(rows,cols,CV_32FC1,v);//init with the scalar value
            init_array();
        }
        void init_array(){
            array.resize(mat.rows);
            for(int r=0;r<mat.rows;r++){
                array[r]=mat.ptr<float>(r);
            }
        }
    };

    struct EventFrame{
        FastMat image;
        FastMat mask;
        EventFrame(int rows=0,int cols=0, cv::Scalar vi=cv::Scalar(0), cv::Scalar vm=cv::Scalar(0)){
            init(rows,cols,vi,vm);
        }
        EventFrame(cv::Size s, cv::Scalar vi=cv::Scalar(0), cv::Scalar vm=cv::Scalar(0)){
            init(s.height,s.width,vi,vm);
        }
        void init(int rows, int cols, cv::Scalar vi=cv::Scalar(0), cv::Scalar vm=cv::Scalar(0)){
            image.init(rows,cols,vi);
            mask.init(rows,cols,vm);
        }
        void init(cv::Size s, cv::Scalar vi=cv::Scalar(0), cv::Scalar vm=cv::Scalar(0)){
            init(s.height,s.width,vi,vm);
        }
        int rows() const{
            return image.rows();
        }
        int cols() const{
            return image.cols();
        }
    };
    cv::Vec2f find_movement_dir(const EventPacketProcessor::EventFrame &on_one_min, const EventPacketProcessor::EventFrame &off_one_min);
    static cv::Vec2f calcGradient(const EventPacketProcessor::EventFrame &one_min);
private:
    std::shared_ptr<libcaer::filters::DVSNoise> dvsn;
    std::vector<uint64_t> codes={0xd2b63a09dUL,0x6001134e5UL,0x1206fbe72UL,0xff8ad6cb4UL,0x85da9bc49UL,0xb461afe9cUL,0x6db51fe13UL,0x5248c541fUL,0x8f34503UL,0x8ea462eceUL,0xeac2be76dUL,0x1af615c44UL,0xb48a49f27UL,0x2e4e1283bUL,0x78b1f2fa8UL,0x27d34f57eUL,0x89222fff1UL,0x4c1669406UL,0xbf49b3511UL,0xdc191cd5dUL,0x11d7c3f85UL,0x16a130e35UL,0xe29f27effUL,0x428d8ae0cUL,0x90d548477UL,0x2319cbc93UL,0xc3b0c3dfcUL,0x424bccc9UL,0x2a081d630UL,0x762743d96UL,0xd0645bf19UL,0xf38d7fd60UL,0xc6cbf9a10UL,0x3c1be7c65UL,0x276f75e63UL,0x4490a3f63UL,0xda60acd52UL,0x3cc68df59UL,0xab46f9daeUL,0x88d533d78UL,0xb6d62ec21UL,0xb3c02b646UL,0x22e56d408UL,0xac5f5770aUL,0xaaa993f66UL,0x4caa07c8dUL,0x5c9b4f7b0UL,0xaa9ef0e05UL,0x705c5750UL,0xac81f545eUL,0x735b91e74UL,0x8cc35cee4UL,0xe44694d04UL,0xb5e121de0UL,0x261017d0fUL,0xf1d439eb5UL,0xa1a33ac96UL,0x174c62c02UL,0x1ee27f716UL,0x8b1c5ece9UL,0x6a05b0c6aUL,0xd0568dfcUL,0x192d25e5fUL,0x1adbeccc8UL,0xcfec87f00UL,0xd0b9dde7aUL,0x88dcef81eUL,0x445681cb9UL,0xdbb2ffc83UL,0xa48d96df1UL,0xb72cc2e7dUL,0xc295b53fUL,0xf49832704UL,0x9968edc29UL,0x9e4e1af85UL,0x8683e2d1bUL,0x810b45c04UL,0x6ac44bfe2UL,0x645346615UL,0x3990bd598UL,0x1c9ed0f6aUL,0xc26729d65UL,0x83993f795UL,0x3ac05ac5dUL,0x357adff3bUL,0xd5c05565UL,0x2f547ef44UL,0x86c115041UL,0x640fd9e5fUL,0xce08bbcf7UL,0x109bb343eUL,0xc21435c92UL,0x35b4dfce4UL,0x459752cf2UL,0xec915b82cUL,0x51881eed0UL,0x2dda7dc97UL,0x2e0142144UL,0x42e890f99UL,0x9a8856527UL,0x8e80d9d80UL,0x891cbcf34UL,0x25dd82410UL,0x239551d34UL,0x8fe8f0c70UL,0x94106a970UL,0x82609b40cUL,0xfc9caf36UL,0x688181d11UL,0x718613c08UL,0xf1ab7629UL,0xa357bfc18UL,0x4c03b7a46UL,0x204dedce6UL,0xad6300d37UL,0x84cc4cd09UL,0x42160e5c4UL,0x87d2adfa8UL,0x7850e7749UL,0x4e750fc7cUL,0xbf2e5dfdaUL,0xd88324da5UL,0x234b52f80UL,0x378204514UL,0xabdf2ad53UL,0x365e78ef9UL,0x49caa6ca2UL,0x3c39ddf3UL,0xc68c5385dUL,0x5bfcbbf67UL,0x623241e21UL,0xabc90d5ccUL,0x388c6fe85UL,0xda0e2d62dUL,0x10855dfe9UL,0x4d46efd6bUL,0x76ea12d61UL,0x9db377d3dUL,0xeed0efa71UL,0xe6ec3ae2fUL,0x441faee83UL,0xba19c8ff5UL,0x313035eabUL,0x6ce8f7625UL,0x880dab58dUL,0x8d3409e0dUL,0x2be92ee21UL,0xd60302c6cUL,0x469ffc724UL,0x87eebeed3UL,0x42587ef7aUL,0x7a8cc4e52UL,0x76a437650UL,0x999e41ef4UL,0x7d0969e42UL,0xc02baf46bUL,0x9259f3e47UL,0x2116a1dc0UL,0x9f2de4d84UL,0xeffac29UL,0x7b371ff8cUL,0x668339da9UL,0xd010aee3fUL,0x1cd00b4c0UL,0x95070fc3bUL,0xf84c9a770UL,0x38f863d76UL,0x3646ff045UL,0xce1b96412UL,0x7a5d45da8UL,0x14e00ef6cUL,0x5e95abfd8UL,0xb2e9cb729UL,0x36c47dd7UL,0xb8ee97c6bUL,0xe9e8f657UL,0xd4ad2ef1aUL,0x8811c7f32UL,0x47bde7c31UL,0x3adadfb64UL,0x6e5b28574UL,0x33e67cd91UL,0x2ab9fdd2dUL,0x8afa67f2bUL,0xe6a28fc5eUL,0x72049cdbdUL,0xae65dac12UL,0x1251a4526UL,0x1089ab841UL,0xe2f096ee0UL,0xb0caee573UL,0xfd6677e86UL,0x444b3f518UL,0xbe8b3a56aUL,0x680a75cfcUL,0xac02baea8UL,0x97d815e1cUL,0x1d4386e08UL,0x1a14f5b0eUL,0xe658a8d81UL,0xa3868efa7UL,0x3668a9673UL,0xe8fc53d85UL,0x2e2b7edd5UL,0x8b2470f13UL,0xf69795f32UL,0x4589ffc8eUL,0x2e2080c9cUL,0x64265f7dUL,0x3d714dd10UL,0x1692c6ef1UL,0x3e67f2f49UL,0x5041dad63UL,0x1a1503415UL,0x64c18c742UL,0xa72eec35UL,0x1f0f9dc60UL,0xa9559bc67UL,0xf32911d0dUL,0x21c0d4ffcUL,0xe01cef5b0UL,0x4e23a3520UL,0xaa4f04e49UL,0xe1c4fcc43UL,0x208e8f6e8UL,0x8486774a5UL,0x9e98c7558UL,0x2c59fb7dcUL,0x9446a4613UL,0x8292dcc2eUL,0x4d61631UL,0xd05527809UL,0xa0163852dUL,0x8f657f639UL,0xcca6c3e37UL,0xcb136bc7aUL,0xfc5a83e53UL,0x9aa44fc30UL,0xbdec1bd3cUL,0xe020b9f7cUL,0x4b8f35fb0UL,0xb8165f637UL,0x33dc88d69UL,0x10a2f7e4dUL,0xc8cb5ff53UL,0xde259ff6bUL,0x46d070dd4UL,0x32d3b9741UL,0x7075f1c04UL,0x4d58dbea0UL};
    std::map<uint64_t,int> dictionary;

    std::vector<std::vector<cv::Vec2b*>> code_indices;

    std::chrono::high_resolution_clock::time_point prev_frame_tp,beginning;
    std::string window_name;
    cv::Mat display_frame;
    std::vector<cv::Vec3b*> display_frame_fa;
    EventFrame on_frame,off_frame,on_min,on_one_min,off_min,off_one_min;

    double mid_ts,med_ts,av_ts,min_ts;

    const int skip_every_frames=1;
    size_t frame_num;
    cv::Size im_size,display_size;
    int display_scale=2;
    cv::VideoWriter vw;

    const int kernel_size=3;
    float kernel_sigma=0.3*((kernel_size-1)*0.5-1)+0.8;
    cv::Size kernel_2D_size=cv::Size(kernel_size,kernel_size);
    std::vector<std::vector<std::vector<float>>> off_cell_events,on_cell_events;

    void preprocess(EventFrame &io);
    void maskBlur(EventFrame &input);
    float calcLineAge(const EventFrame &ei, const LineSegment& ls);
    void artifical_line(std::vector<float*> &io);
    void move_segments(const EventFrame &age_frame, LineSegments &line_segments, float goal_age);
    void soft_threshold(std::vector<float*> &io);
    void segment_lines(cv::Mat m, const LineSegments &end_points, LineSegments &segments, int min_length=MarkerCandidate::side_size, int max_gap_pixels=1);
    bool projects_in(cv::Point p, const LineSegment &ls, float &d);
    bool compatible_segments(const LineSegment &ls1, const LineSegment &ls2);
    void find_marker_candidates(LineSegments &on_segments, LineSegments &off_segments, std::vector<MarkerCandidate> &result, float min_size=0);
    void find_end_points(int max_x, int max_y, const std::vector<cv::Vec2f> &rhos_and_thetas, LineSegments &end_points);
    void fill_in_frames(std::shared_ptr<const libcaer::events::PolarityEventPacket> pep, std::string video_path="");
    void erase_frames(std::shared_ptr<const libcaer::events::PolarityEventPacket> pep);
    void draw_segments(cv::Mat &im, LineSegments &segments, cv::Scalar color, int scale=1);
    void candidate_detector_hough(const cv::Mat& edge_frame_filtered, const cv::Mat& on_frame_filtered, const cv::Mat& off_frame_filtered, cv::Mat& edge_frame_processed_big, LineSegments& on_segments, LineSegments& off_segments, bool display=false);
    void detect_line_segments(const EventFrame &input, LineSegments &output);
    void find_line_offset(const EventFrame &age_frame, const LineSegment &ls, float goal_age, double &x_offset, double &y_offset);
    void trim_segments(const EventFrame &age_frame, LineSegments &segments);
    void trim_segment(const EventFrame &age_frame, LineSegment &ls);
    void expand_segments(const EventFrame &age_frame, LineSegments &segments);
    void expand_segment(const EventFrame &age_frame, LineSegment &ls);

public:
    const double deg_in_rad=std::acos(-1)/180.0;
    static void linear_regression(const std::vector<double> &alphas, const std::vector<double> &betas, double &c, double &d);//beta_i=c*alpha_i+d
    EventPacketProcessor(cv::Size is=cv::Size(128,128));
    void update(std::shared_ptr<const libcaer::events::PolarityEventPacket> pep, size_t curr_index);
};

#endif // EVENTPACKETPROCESSOR_H
