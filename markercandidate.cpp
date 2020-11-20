#include "markercandidate.h"
#include <map>

using namespace std;

const cv::Size MarkerCandidate::marker_size(MarkerCandidate::side_size,MarkerCandidate::side_size);
const std::vector<cv::Point2f> MarkerCandidate::coords{cv::Point2f(0,0),cv::Point2f(MarkerCandidate::marker_size.width-1,0),cv::Point2f(MarkerCandidate::marker_size.width-1,MarkerCandidate::marker_size.height-1),cv::Point2f(0,MarkerCandidate::marker_size.height-1)};

MarkerCandidate::MarkerCandidate(const LineSegment& in_on_ls, const LineSegment& in_off_ls)
{
    on_ls=in_on_ls;
    off_ls=in_off_ls;

    cv::Point points[4];
    int input_indices[4],output_indices[4];
    points[0]=on_ls.p1;
    points[1]=on_ls.p2;
    points[2]=off_ls.p1;
    points[3]=off_ls.p2;

    std::multimap<int,int> x_index,y_index;
    for(int i=0;i<4;i++){
        x_index.emplace(points[i].x,i);
        y_index.emplace(points[i].y,i);
    }
    //the two points with the lowest x values
    int x_index_1=(*x_index.begin()).second;
    int x_index_2=(*std::next(x_index.begin())).second;
    int x_index_3=(*std::next(std::next(x_index.begin()))).second;
    int x_index_4=(*std::next(std::next(std::next(x_index.begin())))).second;

    //the two points with the lowest y values
    int y_index_1=(*y_index.begin()).second;
    int y_index_2=(*std::next(y_index.begin())).second;
    int y_index_3=(*std::next(std::next(y_index.begin()))).second;
    int y_index_4=(*std::next(std::next(std::next(y_index.begin())))).second;

    //p1 the point with the lowest x and then lowest y value
    if (points[x_index_1].y>points[x_index_2].y){
        int tmp=x_index_1;
        x_index_1=x_index_2;
        x_index_2=tmp;
    }

    input_indices[0]=x_index_1;
    input_indices[3]=x_index_2;
    output_indices[x_index_1]=0;
    output_indices[x_index_2]=3;

    if (points[x_index_3].y>points[x_index_4].y){
        int tmp=x_index_3;
        x_index_3=x_index_4;
        x_index_4=tmp;
    }

    input_indices[1]=x_index_3;
    input_indices[2]=x_index_4;
    output_indices[x_index_3]=1;
    output_indices[x_index_4]=2;

//    for(int i=0;i<4;i++)
//        if(i!=x_index_1 && i!=x_index_2 && i!=y_index_2){
//            input_indices[2]=i;
//            output_indices[i]=2;
//            break;
//        }

    int off_p1=std::min(output_indices[2],output_indices[3]);
    int off_p2=std::max(output_indices[2],output_indices[3]);
    if(off_p1==0 && off_p2==3)
        off_side=3;
    else
        off_side=off_p1;

    p.resize(4);
    for(int i=0;i<4;i++)
        p[i]=points[input_indices[(i+off_side+1)%4]];

}

void MarkerCandidate::decode_candidate(){
    //get the fast access arrays
    image_off_fa.resize(image_off.rows);
    image_on_fa.resize(image_on.rows);
    for(int r=0;r<image_on.rows;r++){
        image_on_fa[r]=image_on.ptr<uchar>(r);
        image_off_fa[r]=image_off.ptr<uchar>(r);
    }

    int gaussian_width=cell_size;
    int gaussian_height=cell_size;
    //        double sig=0.3*((gaussian_size-1)/2.0-1)+0.8;

    cv::Mat g_kernel_w=cv::getGaussianKernel(gaussian_width,-1);//default type: CV_64
    cv::Mat g_kernel_h=cv::getGaussianKernel(gaussian_height,-1);
    cv::Mat g_kernel=g_kernel_h*g_kernel_w.t();

    on_score_im=cv::Mat::zeros(num_cells,num_cells,CV_8UC1);
    off_score_im=cv::Mat::zeros(num_cells,num_cells,CV_8UC1);

    vector<uchar*> on_score_fa(on_score_im.rows), off_score_fa(off_score_im.rows);
    for(int r=0;r<on_score_im.rows;r++){
        on_score_fa[r]=on_score_im.ptr<uchar>(r);
        off_score_fa[r]=off_score_im.ptr<uchar>(r);
    }

    code_im=cv::Mat::zeros(num_cells,num_cells,CV_8UC1);
    code_im_fa.resize(num_cells);
    for(size_t r=0;r<num_cells;r++)
        code_im_fa[r]=code_im.ptr<uchar>(r);

    vector<vector<int>> on_scores(num_cells,vector<int>(num_cells,0));
    vector<vector<int>> off_scores(num_cells,vector<int>(num_cells,0));

    int max_on_score=-1,max_off_score=-1;

    for(int r=1;r<num_cells-1;r++){//loop on rows and columns
        for(int c=1;c<num_cells-1;c++){
            int y_min=r*cell_size+cell_size/2-gaussian_height/2;
            int y_max=r*cell_size+cell_size/2+gaussian_height/2;
            int x_min=c*cell_size-gaussian_width/2;
            int x_max=c*cell_size+gaussian_width/2;

            cv::Mat on_roi,off_roi;

            image_on(cv::Range(y_min,y_max),cv::Range(x_min-gaussian_width/4,x_max-gaussian_width/4)).convertTo(on_roi,CV_64FC1);
            int on_score=cv::sum(on_roi.mul(g_kernel))[0];
//            on_scores[r][c]=on_score;
            on_score_fa[r][c]=on_score;
            if(on_score>max_on_score)
                max_on_score=on_score;

            image_off(cv::Range(y_min,y_max),cv::Range(x_min,x_max)).convertTo(off_roi,CV_64FC1);
            int off_score=cv::sum(off_roi.mul(g_kernel))[0];
//            off_scores[r][c]=off_score;
            off_score_fa[r][c]=off_score;
            if(off_score>max_off_score)
                max_off_score=off_score;

        }
    }

    for(int r=1;r<num_cells-1;r++){//loop on rows and columns
        for(int c=1;c<num_cells-1;c++){
            if(on_score_fa[r][c]*100.0/max_on_score>threshold_on_pixels){
                on_scores[r][c]=1;
            }
            else{
                on_scores[r][c]=0;
            }

            if(off_score_fa[r][c]*100.0/max_off_score>threshold_off_pixels){
                off_scores[r][c]=1;
            }
            else{
                off_scores[r][c]=0;
            }
        }
    }

    //decoding loop
    for(int r=1;r<num_cells-1;r++){//loop on rows and columns
        code_im_fa[r][0]=0;//the first cell in each row is assumed to be zero
        for(int c=1;c<num_cells-1;c++){
            if(code_im_fa[r][c-1]==0){
                code_im_fa[r][c]=0;

                if(on_scores[r][c]>0)
                    code_im_fa[r][c]=1;

            }
            else{
                code_im_fa[r][c]=1;

                if(off_scores[r][c]>0)
                    code_im_fa[r][c]=0;
            }
        }
    }

    int code_size=6;
    for(int rot=0;rot<4;rot++){

        int i_step,j_step;
        int i_start,i_end,j_start,j_end;

        if(rot%3==0){
            i_start=0;
            i_end=code_size;
            i_step=1;
        }
        else{
            i_start=code_size-1;
            i_end=-1;
            i_step=-1;
        }
        if(rot<2){
            j_start=0;
            j_end=code_size;
            j_step=1;
        }
        else{
            j_start=code_size-1;
            j_end=-1;
            j_step=-1;
        }
        uint64_t &curr_code=code[rot];
        curr_code=0;
        for(int i=i_start;i!=i_end;i+=i_step){
            for(int j=j_start;j!=j_end;j+=j_step){
                curr_code=curr_code<<1;
                if(rot%2==0){
                    curr_code+=code_im_fa[i+1][j+1];
                }
                else{
                    curr_code+=code_im_fa[j+1][i+1];
                }
            }
        }
    }
}
