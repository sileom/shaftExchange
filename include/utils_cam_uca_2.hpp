//
// Created by sergio on 16/05/19.
//

#include "Model_uca_2.h"
#include "Tensor_uca_2.h"
#include <opencv2/opencv.hpp>
#include <numeric>
#include <iomanip>
#include <string> 
#include <iostream> 
#include <map>
#include <math.h>

#include <librealsense2/rs.hpp>
#include <Eigen/Core>

/*#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>*/

#include <chrono>

using namespace rs2;
using namespace cv;


typedef std::chrono::high_resolution_clock Clock;
typedef std::chrono::milliseconds milliseconds;
static Clock::time_point t0 = Clock::now();
rs2::pipeline_profile profile;

void tic(){ t0 = Clock::now();}
void toc(){ 
    Clock::time_point t1 = Clock::now();
    milliseconds ms = std::chrono::duration_cast<milliseconds>(t1-t0);
    std::cout << "*** ELAPSED TIME IS " << ms.count() << " milliseconds******\n";
}

typedef struct {
  int classId;
  int x;
  int y;
  int right;
  int bottom;
} objectDetected;


int width = 640;
int height = 480;
int fps = 15;
float threshold_net = 0.92;

double px = 585.8437284; //593.61; //602.71242507536715; // 613.6060181;
double py = 583.6422704; //589.70; //600.40978152205628; //575.42666411; //613.7553711;
double u0 = 311.4202191; //319.65; //333.41365317341609;//327.06022754;//324.6341248;
double v0 = 234.1084112; //241.51; //227.12562709177675;//238.04706309;//235.6944733;
double kud = 0.0; 
double kdu = 0.0;




bool IS_SHAFT_ROTATE = false;
double Z_CAMERA = 0.0;

objectDetected getShaft(std::vector<objectDetected> objs, int id);
std::vector<Eigen::Vector3d> getCoupleInShaft(std::vector<objectDetected> objs, objectDetected shaft);
Eigen::Vector3d getCenter(objectDetected r);
bool isIn(objectDetected o, objectDetected exhaust);
int getArea(objectDetected o);
void getPositionAndOrientation(rs2::depth_frame aligned_depth_frame, Eigen::Vector3d cPixel, Eigen::Vector3d vAlb, Eigen::VectorXd *cRisW, Eigen::Matrix3d *RrisW, int ALBERO);
Eigen::Vector3d getCenterIntake(std::vector<Eigen::Vector3d> couple, objectDetected intake);
Eigen::Vector3d getCenterExhaust(std::vector<Eigen::Vector3d> couple, objectDetected exhaust);
std::vector<objectDetected> extract_objs(std::vector<objectDetected> objs);

/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_pcl(const rs2::points& points, const rs2::video_frame& color);
std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords);
float getZValue(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float x, float y);
Eigen::Vector3d pixel2point(Eigen::Vector2d pixel, double depth);

void compute2D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2D);
float findZMax(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterPC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);*/

float checkOverlap(cv::Rect r1, cv::Rect r2);



Eigen::Vector3d pixel2point(Eigen::Vector2d pixel, double depth){
  //double r_quad = pow(((pixel[0] - u0)/px),2) + pow(((pixel[1] - v0)/py),2);
  Eigen::Vector3d result;
  result[0] = (pixel[0] - u0)/px;//(pixel[0] - u0) * (1+ (kdu*r_quad))/px;
  result[1] = (pixel[1] - v0)/py;//(pixel[1] - v0) * (1+ (kdu*r_quad))/py;
  result[2] = depth;
  return result;
}

pipeline startCamera() {
    pipeline pipe;
    rs2::config config;

    config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_RGB8, fps);//.as<video_stream_profile>();
    //config.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
    //config.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);
    //config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps);
    config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, fps);
    //rs2::align align_to(RS2_STREAM_COLOR);
    
    rs2::pipeline_profile pipeline_profile = pipe.start(config);
    profile = pipeline_profile;

    rs2::device dev = pipeline_profile.get_device();

    auto depth_sensor = dev.first<rs2::depth_sensor>();
    /*if (depth_sensor.supports(RS2_OPTION_EXPOSURE)) {
        depth_sensor.set_option(RS2_OPTION_EXPOSURE, 30000.f); //10715
    }
    if (depth_sensor.supports(RS2_OPTION_GAIN)) {
        depth_sensor.set_option(RS2_OPTION_GAIN, 148.f); //33
    }
    if (depth_sensor.supports(RS2_OPTION_LASER_POWER)){
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, 30); //30
    }
    if (depth_sensor.supports(RS2_OPTION_EMITTER_ENABLED)) {
        depth_sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);
    }*/

    return pipe;
}

cv::Mat frame_to_mat(const rs2::frame& f)
{
    using namespace cv;
    using namespace rs2;

    auto vf = f.as<video_frame>();
    const int w = vf.get_width();
    const int h = vf.get_height();

    if (f.get_profile().format() == RS2_FORMAT_BGR8)
    {
        return Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_RGB8)
    {
        auto r = Mat(Size(w, h), CV_8UC3, (void*)f.get_data(), Mat::AUTO_STEP);
        cvtColor(r, r, COLOR_RGB2BGR);
        return r;
    }
    else if (f.get_profile().format() == RS2_FORMAT_Z16)
    {
        return Mat(Size(w, h), CV_16UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_Y8)
    {
        return Mat(Size(w, h), CV_8UC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }
    else if (f.get_profile().format() == RS2_FORMAT_DISPARITY32)
    {
        return Mat(Size(w, h), CV_32FC1, (void*)f.get_data(), Mat::AUTO_STEP);
    }

    throw std::runtime_error("Frame format is not supported yet!");
}

void convertToWhite(cv::Mat img){
    for(int y=0;y<img.rows;y++) {
        for(int x=0;x<img.cols;x++) {
            cv::Vec3b& color = img.at<cv::Vec3b>(y,x);
            color[0] = 255;
            color[1] = 255;
            color[2] = 255;
            img.at<cv::Vec3b>(cv::Point(x,y)) = color;
        }
    }
}

bool xIsIn(int x, cv::Rect r){
    return x >= r.x && x <= (r.x+r.width);
}

bool yIsIn(int y, cv::Rect r){
    return y >= r.y && y <= (r.y+r.height);
}

void updateMask(cv::Mat img, cv::Mat img_mask, cv::Rect rec){
    for(int y=0;y<img.rows;y++) {
        for(int x=0;x<img.cols;x++) {
            if (xIsIn(x, rec) && yIsIn(y, rec)){
                cv::Vec3b& color = img.at<cv::Vec3b>(y,x);
                img_mask.at<cv::Vec3b>(cv::Point(x,y)) = color;
            }
        }
    }
}

int mainMethod(int ALBERO) {
    //Model model("../ssd_inception/frozen_inference_graph.pb");
    Model model("/home/labarea-franka/libfranka/uca_franka_lib1/resources/model/frozen_inference_graph.pb");
    Tensor outNames1{model, "num_detections"};
    Tensor outNames2{model, "detection_scores"};
    Tensor outNames3{model, "detection_boxes"};
    Tensor outNames4{model, "detection_classes"};

    Tensor inpName{model, "image_tensor"};

    std::vector<std::string> classes{"counter-rotating_shaft_exhaust", "counter-rotating_shaft_intake", "roller_bearing"}; 

    char c;
    bool runApp = true;

    //Start camera
    pipeline pipe = startCamera();
    rs2::align align_to_color(RS2_STREAM_COLOR);
    rs2::align align(RS2_STREAM_COLOR);
    
    //rs2::align align_to_depth(RS2_STREAM_DEPTH);

    // Capture 30 frames to give autoexposure, etc. a chance to settle
    for (int k = 0; k < 50; ++k) pipe.wait_for_frames();

    // Read image
    cv::Mat imgOr, cropped,  img, inp;
    //img = cv::imread("test.png", cv::IMREAD_COLOR);

    int rows, cols;

    std::vector<objectDetected> objs;
    std::vector<objectDetected> objs_filtered;
    //pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");

    rs2::colorizer color_map;
    
    FILE* file_res;
    file_res = fopen("/home/labarea-franka/libfranka/uca_franka_lib1/resources/shaft.txt", "w");
    FILE* file_rot;
    if(ALBERO==1)
        file_rot = fopen("/home/labarea-franka/libfranka/uca_franka_lib1/resources/isRotate.txt", "w");
    else{
        file_rot = fopen("/home/labarea-franka/libfranka/uca_franka_lib1/resources/isRotate.txt", "r");
        char stringa[80];
        fscanf(file_rot,"%s",stringa);
        int val = std::stoi(stringa);
        IS_SHAFT_ROTATE = (val == 0) ? false : true;
    }

    int numero_tentativi = 0;
    bool detection_ok = true;

    while(runApp){
        if(numero_tentativi >=1){
            std::cout << "SONO QUI" << std::endl;
            runApp = false;
            detection_ok = false;
            break;
        }
        objs.clear();
        objs_filtered.clear();
        auto data = pipe.wait_for_frames();
        auto color_frame = data.get_color_frame();
        auto depth_frame = data.get_depth_frame();

        auto aligned_frames = align.process(data);
        rs2::video_frame aligned_color_frame = aligned_frames.first(RS2_STREAM_COLOR);
        rs2::depth_frame aligned_depth_frame = aligned_frames.get_depth_frame();
        


        //CREATE PC
        rs2::pointcloud pc;
        rs2::points points;
        pc.map_to(color_frame);
        points = pc.calculate(depth_frame);
        //points.export_to_ply("1.ply", color_frame);
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = points_to_pcl(points, color_frame);
        //viewer.showCloud (cloud);
        
        imgOr = frame_to_mat(aligned_color_frame);

        //Rect myROI(240, 0, 1440, 1080);
        //Mat cropped = imgOr(myROI); //1440,1080

        //cv::flip(img, img, 0);
        //cv::flip(img, img, 1);
        //cv::resize(cropped, img, cv::Size(640, 480));
	    cv::resize(imgOr, img, cv::Size(640, 480));

        rows = img.rows;
        cols = img.cols;
        

        //cv::resize(img, inp, cv::Size(300, 300));
        cv::resize(img, inp, cv::Size(rows, cols));
        cv::cvtColor(inp, inp, cv::COLOR_BGR2RGB);

        // Put image in Tensor
        std::vector<uint8_t > img_data;
        img_data.assign(inp.data, inp.data + inp.total() * inp.channels());
        inpName.set_data(img_data, {1, 640,480,3}); //640, 480, 3}); //{1, 300, 300, 3});
        //tic();
        model.run(inpName, {&outNames1, &outNames2, &outNames3, &outNames4});
        //toc();

        // Visualize detected bounding boxes.
        int num_detections = (int)outNames1.get_data<float>()[0];
        int num_ooi = 0; //numero oggetti di interesse
        for (int i=0; i<num_detections; i++) {
            int classId = (int)outNames4.get_data<float>()[i];
            float score = outNames2.get_data<float>()[i];
            auto bbox_data = outNames3.get_data<float>();
            std::vector<float> bbox = {bbox_data[i*4], bbox_data[i*4+1], bbox_data[i*4+2], bbox_data[i*4+3]};
            if (score > 0.85) {
                num_ooi++;
                objectDetected o;
                o.classId = classId;
                o.x = bbox[1] * cols;
                o.y = bbox[0] * rows;
                o.right = bbox[3] * cols;
                o.bottom = bbox[2] * rows;

                std::stringstream ss;
                ss << score;
                std::string testo;
                testo = ss.str();
                //testo = classes[o.classId-1];// + " " + ss.str();
                //cv::putText(img, classes[classId-1], cv::Point(x,y-2.1), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 1.5);
                cv::putText(img, testo, cv::Point(o.x,o.y-(2.1*num_ooi)), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,69,255), 2.0);
                //cv::circle(img,  cv::Point(o.x, o.y), 0.5, Scalar(0, 0, 255), 10, 8, 0);
                cv::rectangle(img, {(int)o.x, (int)o.y}, {(int)o.right, (int)o.bottom}, {125, 255, 51}, 2);
                objs.push_back(o);
            }
        }

        //cv::imshow("Image22", img);
        //c = cv::waitKey(0);
        //if(c == 'c'){
		//	continue;
		//}

        objs_filtered = extract_objs(objs);
        std::cout << "Dimensione lista filtrata: " << objs_filtered.size() << std::endl;
        //for(int i=0; i<objs_filtered.size(); i++){
        //    cv::rectangle(img, {(int)objs_filtered[i].x, (int)objs_filtered[i].y}, {(int)objs_filtered[i].right, (int)objs_filtered[i].bottom}, {0,0,255}, 1);
        //}
        //cv::imshow("Image PARZIALE FINALE", img);
        //c = cv::waitKey(0);


        //if (!(num_ooi == 6 || num_ooi == 3)){
        if (!(objs_filtered.size() == 6 || objs_filtered.size() == 3)){
            std::cout << "Detection problems: All necessary components weren't detected." << std::endl;
            runApp=false;
            cv::imshow("Image for detect ERRORS", img);
            c = cv::waitKey(0);
            return -1;
        }
        
        // Qui mi ritrovo con objs_filtered riempito con gli oggetti rilevati e filtrati 

        objectDetected shaft;
        Eigen::MatrixXd passPoint = Eigen::MatrixXd::Identity(4,4);

        if(ALBERO == -1){ 
            shaft = getShaft(objs_filtered, 1); //exahust - corto
            passPoint << -0.00869494, 0.998429, 0.0551724, -0.491889,
			0.0108328, -0.0550767, 0.998423, 0.33308,
			0.999894, 0.0092789, -0.0103371, 0.3976,
			0, 0, 0, 1;
        } else {
            shaft = getShaft(objs_filtered, 2); //intake - lungo
            passPoint << -0.0165911, 0.0565945, -0.998259,-0.578073, 
                    0.0101416, -0.998326, -0.0567679, -0.65563, 
                    -0.999801, -0.0110658, 0.0159897,0.360183,
                    0,0,0,1;
        }

        //cv::rectangle(img, {(int)shaft.x, (int)shaft.y}, {(int)shaft.right, (int)shaft.bottom}, {255, 255, 255}, 1);

        std::vector<Eigen::Vector3d> couple = getCoupleInShaft(objs_filtered, shaft); 
        cv::line(img, cv::Point(couple[0][0], couple[0][1]), cv::Point(couple[1][0], couple[1][1]), cv::Scalar( 0, 89, 133), 3, cv::LINE_8);
        Eigen::Vector3d vAlb;
        Eigen::Vector3d pCentro;
        if(couple[0][0] > couple[1][0])
            vAlb = couple[0] - couple[1];
        else 
            vAlb = couple[1] - couple[0];

        vAlb = vAlb/vAlb.norm();

        if (ALBERO ==-1) {
            pCentro = (couple[0] + couple[1])/2;
        } else {
            pCentro = getCenterIntake(couple, shaft);
        }
        pCentro[2] = -0.14;
        cv::circle(img,  cv::Point(pCentro[0], pCentro[1]), 0.7, Scalar(0, 0, 255), 10, 8, 0);


        pCentro = pCentro*2.25;
        pCentro[0] += 240;
        cv::circle(imgOr,  cv::Point(pCentro[0], pCentro[1]), 0.7, Scalar(0, 0, 255), 10, 8, 0);
        //imshow("originale", imgOr);
        //waitKey(0);

        
            
        Eigen::VectorXd centro_w = Eigen::VectorXd::Zero(4); centro_w[3] = 1;
        Eigen::Matrix3d R_w = Eigen::Matrix3d::Identity(3,3);
        getPositionAndOrientation(aligned_depth_frame, pCentro, vAlb, &centro_w, &R_w, ALBERO);

        //if ((num_ooi == 6 || num_ooi == 3) && Z_CAMERA > 0.02){
        if ((objs_filtered.size() == 6 || objs_filtered.size() == 3) && Z_CAMERA > 0.02){
            std::cout << "NUMERO OOI: " << num_ooi << std::endl;
            std::cout << "DIMENSIONE FILTRATA: " << objs_filtered.size() << std::endl;
            std::cout << "Z_CAMERA: " << Z_CAMERA << std::endl;
            runApp=false;
        }
        numero_tentativi++;
    
        //centro_w[2] = -0.0948755; //interno scatola
        /*if (ALBERO ==-1) { //corto
            centro_w[2] = -0.0721125; //TEST 04 ottobre gripper CRF
        } else {
            centro_w[2] = -0.0791125; //TEST 04 ottobre gripper CRF
        }*/
        if (ALBERO ==-1) { //corto
            centro_w[2] = 0.0508238; //0.0514401; // TEST 10 dicembre //-0.0411456; //TEST 04 ottobre gripper Michelangelo
        } else {
            centro_w[2] = 0.0506139; //0.0514401; // TEST 10 dicembre //-0.0418173; //TEST 04 ottobre gripper Michelangelo
        }
        //std::cout << "Is rotate: " << IS_SHAFT_ROTATE << std::endl << std::endl << std::endl;
        //std::cout << "ALBERO: " << ALBERO << "\n" << centro_w << std::endl << std::endl << std::endl;
        //std::cout << R_w << std::endl << std::endl << std::endl;



        //Mostro l'immagine con i punti calcolati
        cv::imshow("Image", img);
        c = cv::waitKey(0);

        if(ALBERO==1){
            fprintf(file_rot, "%d\n", (IS_SHAFT_ROTATE ? 1 : 0));
        }

		//runApp=false;

        std::cout << "Punto di presa: " << centro_w  << std::endl << std::endl;

        if(ALBERO == 1) { //LUNGO
            fprintf(file_res, "%d %d\n", 3, 5);
            // approccio
            fprintf(file_res, "%f %f %f\nRd\n%f %f %f\n%f %f %f\n%f %f %f\n", centro_w[0], centro_w[1], centro_w[2]+0.10, R_w(0,0), R_w(0,1), R_w(0,2), R_w(1,0), R_w(1,1), R_w(1,2), R_w(2,0), R_w(2,1), R_w(2,2));
            // presa
            fprintf(file_res, "%f %f %f\nRd\n%f %f %f\n%f %f %f\n%f %f %f\n", centro_w[0], centro_w[1], centro_w[2], R_w(0,0), R_w(0,1), R_w(0,2), R_w(1,0), R_w(1,1), R_w(1,2), R_w(2,0), R_w(2,1), R_w(2,2));
            // approccio up        
            fprintf(file_res, "%f %f %f\nRd\n%f %f %f\n%f %f %f\n%f %f %f\n", centro_w[0], centro_w[1], centro_w[2]+0.10, R_w(0,0), R_w(0,1), R_w(0,2), R_w(1,0), R_w(1,1), R_w(1,2), R_w(2,0), R_w(2,1), R_w(2,2));
        //---------------------------	
        } else { //CORTO
            fprintf(file_res, "%d %d\n", 3, 5);
            // approccio
            fprintf(file_res, "%f %f %f\nRd\n%f %f %f\n%f %f %f\n%f %f %f\n", centro_w[0], centro_w[1], centro_w[2]+0.10, R_w(0,0), R_w(0,1), R_w(0,2), R_w(1,0), R_w(1,1), R_w(1,2), R_w(2,0), R_w(2,1), R_w(2,2));
            // presa
            fprintf(file_res, "%f %f %f\nRd\n%f %f %f\n%f %f %f\n%f %f %f\n", centro_w[0], centro_w[1], centro_w[2], R_w(0,0), R_w(0,1), R_w(0,2), R_w(1,0), R_w(1,1), R_w(1,2), R_w(2,0), R_w(2,1), R_w(2,2));
            // approccio up        
            fprintf(file_res, "%f %f %f\nRd\n%f %f %f\n%f %f %f\n%f %f %f\n", centro_w[0], centro_w[1], centro_w[2]+0.10, R_w(0,0), R_w(0,1), R_w(0,2), R_w(1,0), R_w(1,1), R_w(1,2), R_w(2,0), R_w(2,1), R_w(2,2));
        }
	
    }
    pipe.stop();
    if(!detection_ok){
        return -1; //NOK
    }
    return 0; // OK
}

std::vector<objectDetected> extract_objs(std::vector<objectDetected> objs){
    std::vector<objectDetected> objs_filtered;
    objectDetected scarico;
    objectDetected aspirazione;
    objectDetected corrente;
    double s_min = 10000;
    double a_max = -10;
    for(int i=0; i<objs.size(); i++){
        corrente = objs[i];
        if(corrente.classId == 3) { // roller bearing
            objs_filtered.push_back(corrente);
        } else if (corrente.classId == 1) { // scarico
            if(corrente.y < s_min){
                s_min = corrente.y;
                scarico.x = corrente.x;
                scarico.y = corrente.y;
                scarico.right = corrente.right;
                scarico.bottom = corrente.bottom;
                scarico.classId = corrente.classId;
            }
        } else if (corrente.classId == 2) { // aspirazione
            if(corrente.y > a_max){
                a_max = corrente.y;
                aspirazione.x = corrente.x;
                aspirazione.y = corrente.y;
                aspirazione.right = corrente.right;
                aspirazione.bottom = corrente.bottom;
                aspirazione.classId = corrente.classId;
            }
        }
    }
    if(s_min != 10000)
        objs_filtered.push_back(scarico);
    if(a_max != -10)
        objs_filtered.push_back(aspirazione);
    return objs_filtered;
}

//lungo
Eigen::Vector3d getCenterIntake(std::vector<Eigen::Vector3d> couple, objectDetected intake){
    double d1 = couple[0][0] - intake.x;
    double d2 = couple[1][0] - intake.x;
    double minimo = min(d1,d2);
    Eigen::Vector3d centro;
    double lunghezza = intake.right-intake.x;
    if(d1 < (10.0/33)*lunghezza || d2 < (10.0/33)*lunghezza){
        //albero testa a destra //così lo consideriamo dritto
        if(minimo == d1){
            centro = couple[1] *(2.0/3) + couple[0]*(1.0/3);
        } else{
            centro = couple[0] *(2.0/3) + couple[1]*(1.0/3);
        }

    } else {
        //albero testa a sinistra 
        if(minimo == d1){
            centro = couple[0] *(2.0/3) + couple[1]*(1.0/3);
        } else{
            centro = couple[1] *(2.0/3) + couple[0]*(1.0/3);
        }
        IS_SHAFT_ROTATE = true;        
    }
    return centro;
}

//corto
/*Eigen::Vector3d getCenterExhaust(std::vector<Eigen::Vector3d> couple, objectDetected exhaust){
    double d1 = couple[0][0] - exhaust.x;
    double d2 = couple[1][0] - exhaust.x;
    std::cout << d1 << " " << d2 << std::endl;
    double minimo = min(d1,d2);
    Eigen::Vector3d centro;
    double lunghezza = exhaust.right-exhaust.x;
    if(d1 < (3.0/27)*lunghezza || d2 < (3.0/27)*lunghezza){
        //albero testa a destra
        if(minimo == d1){
            centro = couple[1] *(3.0/4) + couple[0]*(1.0/4);
        } else{
            centro = couple[0] *(3.0/4) + couple[1]*(1.0/4);
        }
        IS_SHAFT_ROTATE = true;
    } else {
        //albero testa a sinistra //così lo consideriamo dritto
        if(minimo == d1){
            centro = couple[0] *(3.0/4) + couple[1]*(1.0/4);
        } else{
            centro = couple[1] *(3.0/4) + couple[0]*(1.0/4);
        }
    }
    return centro;
}
*/

/*
objectDetected getShaft_new(std::vector<objectDetected> objs, int id){
    objectDetected e;
    e.x = 0;
    e.y = (id == 1) ? 10000 : -10000; // id =1 corto # =2 lungo
    //e.y = 0;
    e.right = 1;
    e.bottom = 1;
    e.classId = 0;
    objectDetected c;
    bool esito_posizione;
    for(int i=0; i<objs.size(); i++){
        c = objs[i];
        //if(c.classId==id && (getArea(e) < getArea(c))){
        if(c.classId==id && (getArea(e) < getArea(c))){  
            esito_posizione = (id == 1) ? (min(e.y,c.y)==c.y) : (max(e.y,c.y)==c.y);
            std::cout << "e.y: " << e.y << " ||| c.y: " << c.y << std::endl;
            if(esito_posizione){
                e.x = c.x;
                e.y = c.y;
                e.right = c.right;
                e.bottom = c.bottom;
                e.classId = c.classId;
            }
        }
    }
    return e;
}
*/

objectDetected getShaft(std::vector<objectDetected> objs, int id){
    objectDetected e;
    e.x = 0;
    e.y = 0;
    e.right = 1;
    e.bottom = 1;
    e.classId = 0;
    objectDetected c;
    for(int i=0; i<objs.size(); i++){
        c = objs[i];
        if(c.classId==id && (getArea(e) < getArea(c))){
            e.x = c.x;
            e.y = c.y;
            e.right = c.right;
            e.bottom = c.bottom;
            e.classId = c.classId;
        }
    }
    return e;
}

std::vector<Eigen::Vector3d> getCoupleInShaft(std::vector<objectDetected> objs, objectDetected shaft){
    std::vector<objectDetected> rollers;
    int rollerFind = 0;
    objectDetected c;
    int i = 0;
    while(rollerFind < 2 && i<objs.size()) {
        c = objs[i];
        if(c.classId==3 && isIn(c, shaft)){
            rollers.push_back(c);
            rollerFind++;
        }
        i++;
    }
    //ho trovato i due cuscinetti in shaft
    std::vector<Eigen::Vector3d> p;
    Eigen::Vector3d temp = getCenter(rollers[0]);
    p.push_back(temp); 
    temp = getCenter(rollers[1]);
    p.push_back(temp);
    return p;
}

Eigen::Vector3d getCenter(objectDetected r){ 
    Eigen::Vector3d center;
    center[0] = (r.x + r.right)/2; //((r.x + (r.right/2)) - u0)/px;
    center[1] = (r.y + r.bottom)/2; //((r.y + (r.bottom/2)) - v0)/py;
    center[2] = 0; //DA METTERE CON LA DEPTH
    return center;
}

bool isIn(objectDetected o, objectDetected exhaust){
    Eigen::Vector3d center;
    center[0] = (o.x + o.right)/2;
    center[1] = (o.y + o.bottom)/2;
    if(exhaust.x <= center[0] && center[0] <= (exhaust.right)){
        if(exhaust.y <= center[1] && center[1] <= (exhaust.bottom)){
            return true;
        }
    }
    return false;
}

int getArea(objectDetected o){
    return (o.right-o.x)*(o.bottom-o.y);
}

/*
pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_to_pcl(const rs2::points& points, const rs2::video_frame& color){
    // OpenCV Mat for showing the rgb color image, just as part of processing
    Mat colorr(Size(640, 480), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
    //namedWindow("Display Image", WINDOW_AUTOSIZE );
    //imshow("Display Image", colorr);
        
    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    
    // Config of PCL Cloud object
    cloud->width = static_cast<uint32_t>(sp.width());
    cloud->height = static_cast<uint32_t>(sp.height());
    cloud->is_dense = false;
    cloud->points.resize(points.size());

    auto tex_coords = points.get_texture_coordinates();
    auto vertices = points.get_vertices();

    int idx = 0;
    for(int i = 0; i < cloud->height; ++i){
      for(int j = 0; j < cloud->width; ++j){
        //if(color[0] != 255 && color[1] != 255 && color[2] != 255){//     && (0 < vertices[idx].z && vertices[idx].z < 0.42)&& (-0.75 < vertices[idx].x && vertices[idx].x < 0.75) && (-0.75 < vertices[idx].y && vertices[idx].y < 0.75)){
          cloud->points[idx].x = vertices[idx].x;
          cloud->points[idx].y = vertices[idx].y;
          cloud->points[idx].z = vertices[idx].z;

          std::tuple<uint8_t, uint8_t, uint8_t> current_color;
          current_color = get_texcolor(color, tex_coords[idx]);

          // Reversed order- 2-1-0 because of BGR model used in camera
          cloud->points[idx].r = std::get<2>(current_color);
          cloud->points[idx].g = std::get<1>(current_color);
          cloud->points[idx].b = std::get<0>(current_color);
        //}
        idx++;
      }
    }
   return cloud;
}

float getZValue(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, float x, float y){
  pcl::PointXYZRGB point;
  float eps = 0.005;
  for (int i= 0; i < cloud->points.size(); i++){
    point = cloud->points[i];
    if((x-eps <= point.x && point.x <= x+eps) && (y-eps <= point.y && point.y <= y+eps)){
      return point.z;
    }
  }
  return 0.0f;
}
*/


void getPositionAndOrientation(rs2::depth_frame aligned_depth_frame, Eigen::Vector3d cPixel, Eigen::Vector3d vAlb, Eigen::VectorXd *cRisW, Eigen::Matrix3d *RrisW, int ALBERO){
    //cPixel[0] = 724;
    //cPixel[1] = 547;
    Eigen::MatrixXd Ae(4,4);
    Ae << -0.167693, 0.985335, 0.0312239,	-0.038289,
            0.985095, 0.168707, -0.0332647,	0.390762 ,
            -0.0380446, 0.0251802, -0.998959,	0.259696,
            0,0,0,1;


    //CRF
	//Ae << -0.0340116,	0.999389,	0.00668475,	0.738204,
	//	0.999024,	0.034184,	-0.0276255,	-0.0557818,
	//	-0.0278371,	0.00573864,	-0.999596,	0.39485,
	//	0,		0,		0,		1;




    
    Eigen::VectorXd L(11);
    //L 16 feb
   L << -0.0775284008525673,
        4.22686145200658,
        -2.71717222702774,
        1134.06871971009,
        4.08348636312813,
        0.158721742251983,
        -1.53414806078261,
        238.138428477327,
        2.04304839547653e-05,
        0.000143865377207217,
        -0.00298527393007430;

/* CRF
	L << -0.049166,
-1.7037,
-0.79516,
291.73,
-1.6767,
0.017722,
-0.59619,
407.04,
-0.000014398,
-0.00012475,
-0.0026644;
*/


	
		

    Eigen::MatrixXd Ace(4,4);

    //Ace presa il 16 Febbraio
 /*   Ace << 0.01999945272, -0.9990596861, 0.03846772089, 0.05834485203,
    0.9997803621, 0.01974315714, -0.007031030191, -0.03476564525,
    0.006264944557, 0.03859988867, 0.999235107, -0.06760482074,
    0, 0, 0, 1;
*/
    Ace << 0.01999945272, -0.9990596861, 0.03846772089, 0.05834485203,
    0.9997803621, 0.01974315714, -0.007031030191, -0.03476564525,
    0.006264944557, 0.03859988867, 0.999235107, -0.06760482074,
    0, 0, 0, 1;

	//if (ALBERO == -1){//corto
    //    Ace << -0.00526238455,  -0.9998912053,  0.01377987252,  0.06812248831,
    //        0.9999776697,  -0.005205077303,  0.004191332024,  -0.01571296662,
    //        -0.004119150728,  0.01380162121,  0.9998962685,  -0.07765977822,
    //        0,  0,  0,  1;
	//} else {
    //    Ace << -0.00526238455,  -0.9998912053,  0.01377987252,  0.06012248831,
    //        0.9999776697,  -0.005205077303,  0.004191332024,  -0.01671296662,
    //        -0.004119150728,  0.01380162121,  0.9998962685,  -0.07765977822,
    //        0,  0,  0,  1;
    //}


    Eigen::MatrixXd Act(4,4);
    //Act 16 feb
    Act << -0.023817,0.99976,0.0068357,0.068096,
        0.99857,0.020436,0.048135,-0.054515,
        0.04796,0.007827,-0.99882,0.33473,
        0,0,0,1;

	/*Act <<  0.58606,-0.0137,0.81017,-0.30079,
		-0.81023,-0.018659,0.58578,-0.22953,
		0.0084732,-0.99973,-0.022035,0.075805,
		0,0,0,1;
*/
    //CRF
	//Act << -0.026693, -0.9996, -0.0054, 0.1097,
	//	-0.9985,0.0275,-0.0468, -0.0065,
	//	0.0469, 0.0041, -0.9989, 0.3759,
	//	0,0,0,1;


    //std::cout << "CENTRO pixel: \n" << cPixel << std::endl;

    float z; //Valore della zeta preso dalla depth in corrispondenza del centro in pixel
    rs2_error** err;
    z = 0.431; //rs2_depth_frame_get_distance((rs2_frame*)aligned_depth_frame, (int)cPixel[0], (int)cPixel[1], err);
    std::cout << "z camera 707: \t" << z << std::endl;
    Z_CAMERA = z;

        
    //double zeta = 1000*distance;//0.47;//getZValue(cloud, cPixel[0], cPixel[1]);
    //std::cout << "ZETA scarico: " << getZValue(cloud, cPixel[0], cPixel[1]) << std::endl;
    //Eigen::VectorXd cc_ex(4);
    //cc_ex[0] = (u0 - zeta/(px*3/1000)*(cPixel[0] - u0)) * 3/pow(10,6); //  ( (cPixel[0]*(3/1000 - zeta/px)) - (u0*zeta/px) )/1000; //cPixel[0] * 3/ pow(10,6);
    //cc_ex[1] = (v0 - zeta/(py*3/1000)*(cPixel[1] - v0)) * 3/pow(10,6); //( (cPixel[1]*(3/1000 - zeta/py)) - (v0*zeta/py) )/1000;//cPixel[1] * 3/ pow(10,6);
    //cc_ex[2] = zeta/1000;
    //cc_ex[3] = 1;

    Eigen::VectorXd zt(4); //valore della zeta in terna target
    Eigen::VectorXd ztemp(4); 
    ztemp << 0,0,z,1;
    zt = Act*ztemp;
    z = zt[2];

    
    //std::cout << std::endl << "zt: \n" << zt << std::endl<< std::endl<< std::endl;

    //Conversione dei pixel in millimetri (la zeta è già in metri)
    float u_mm = cPixel[0];//*1.4/1000;
    float v_mm = cPixel[1];//*1.4/1000;
    float z_mm = z*1000;

    //std::cout << std::endl << u_mm << " " << v_mm << " " << z_mm << std::endl<< std::endl<< std::endl;

        
    Eigen::MatrixXd A(2,2);
    A << L[8]*u_mm-L[0], L[9]*u_mm-L[1],
         L[8]*v_mm-L[4], L[9]*v_mm-L[5];

    Eigen::VectorXd b(2);
    b << (L[2]*z_mm+L[3]-L[10]*z_mm*u_mm-u_mm),
         (L[6]*z_mm+L[7]-L[10]*z_mm*v_mm-v_mm);
    //std::cout << std::endl << A << " " << std::endl<< std::endl<< std::endl << b << std::endl<< std::endl<< std::endl;

    Eigen::VectorXd r(2); //Coordinate del centro in terna target e in millimetri
    r= A.inverse()*b;
    //std::cout << "r: \n" << r << std::endl<< std::endl<< std::endl;



    Eigen::VectorXd ct(4); //Coordinate del centro in terna target e in metri
    ct[0] = r[0]/1000;
    ct[1] = r[1]/1000;
    ct[2] = z;
    ct[3] = 1;
    std::cout << "ct: \n" << ct << std::endl<< std::endl<< std::endl;
    
    Eigen::VectorXd cw(4); //coordinate del centro in terna mondo
    cw = Ae*Ace*Act.inverse()*ct;
    *cRisW = Ae*Ace*Act.inverse()*ct;
    //std::cout << "target-world: \n" << Ae*Ace*Act.inverse() << std::endl<< std::endl;
    //std::cout << "c: \n" << Act.inverse()*ct << std::endl<< std::endl;
    //std::cout << "e: \n" << Ace*Act.inverse()*ct << std::endl<< std::endl;
    //std::cout << "w: \n" << cw << std::endl<< std::endl;

	/////////////
/*	auto stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
	auto intrinsics = stream.get_intrinsics();
	std::cout << intrinsics.ppx << std::endl;
	float upoint[3];
	float upixel[2];

	upixel[0] = cPixel[0];
	upixel[1] = cPixel[1];
	std::cout << "upixel " << upixel[0] << " " << upixel[1] << std::endl;
	auto udist = aligned_depth_frame.get_distance(upixel[0], upixel[1]);
	rs2_deproject_pixel_to_point(upoint, &intrinsics, upixel, udist);
	std::cout << "upoint " << upoint[0] << " " << upoint[1]<< " " << upoint[2] << std::endl;
	Eigen::VectorXd ucamera(4); //Coordinate del centro in terna target e in metri
	    ucamera << upoint[0], upoint[1], upoint[2], 1;
	std::cout << "ucamera " << ucamera;
	cw = Ae*Ace*ucamera;
	*cRisW = Ae*Ace*ucamera;
*/


    Eigen::Vector3d vw; //vettore normale in terna mondo
    vw = Ae.block<3,3>(0,0) *Ace.block<3,3>(0,0) * vAlb;

    double theta = atan(vw[1]/vw[0]); //rad 
    if(theta > M_PI) {
        theta = theta - M_PI;
    }
    //std::cout << "THETA: " << theta << std::endl;

    if (!IS_SHAFT_ROTATE)
        theta += M_PI;

    Eigen::MatrixXd Rz(3,3);
    Rz << cos(theta), -sin(theta), 0, sin(theta), cos(theta), 0, 0, 0, 1;

    Eigen::MatrixXd Rz180(3,3);
    Rz180 << cos(M_PI), -sin(M_PI), 0, sin(M_PI), cos(M_PI), 0, 0, 0, 1;

    Eigen::MatrixXd R_(3,3);
    R_ << 1, 0, 0, 0, cos(M_PI), -sin(M_PI), 0, sin(M_PI), cos(M_PI);

    Eigen::MatrixXd Rd(3,3);
    *RrisW = Rz180*Rz*R_;
    Rd = Rz180*Rz*R_;
    //std::cout <<"Rd:\n";
    //std::cout << Rd << std::endl;
}














std::tuple<uint8_t, uint8_t, uint8_t> get_texcolor(rs2::video_frame texture, rs2::texture_coordinate texcoords){
    const int w = texture.get_width(), h = texture.get_height();
    
    // convert normals [u v] to basic coords [x y]
    int x = std::min(std::max(int(texcoords.u*w + .5f), 0), w - 1);
    int y = std::min(std::max(int(texcoords.v*h + .5f), 0), h - 1);

    int idx = x * texture.get_bytes_per_pixel() + y * texture.get_stride_in_bytes();
    const auto texture_data = reinterpret_cast<const uint8_t*>(texture.get_data());
    return std::tuple<uint8_t, uint8_t, uint8_t>(texture_data[idx], texture_data[idx+1], texture_data[idx+2]);
}

/*
void compute2D(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2D){
  pcl::PointXYZRGB point;
  point.r = 255;
  point.g = 255;
  point.b = 255;
  for (int i= 0; i < cloud->points.size(); i++){
    point.x = cloud->points[i].x;
    point.y = cloud->points[i].y;
    point.z = 0.57;
    point.r = cloud->points[i].r;
    point.g = cloud->points[i].g;
    point.b = cloud->points[i].b;
    cloud2D->points.push_back(point);
  }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterPC(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
    float z_max = findZMax(cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_f->points.resize(cloud->points.size());
    //int idx = 0;
    //for(int i = 0; i < cloud->height; ++i){
    //  for(int j = 0; j < cloud->width; ++j){
    //    if(cloud->points[idx].z < (z_max-0.002)){
    //      std::cout << "Line 423: " << std::endl;
    //      cloud_f->points[idx].x = cloud->points[idx].x;
    //      cloud_f->points[idx].y = cloud->points[idx].y;
    //      cloud_f->points[idx].z = cloud->points[idx].z;
    //
    //      cloud_f->points[idx].r = cloud->points[idx].r; 
    //      cloud_f->points[idx].g = cloud->points[idx].g; 
    //      cloud_f->points[idx].b = cloud->points[idx].b; 
    //    }
    //    idx++;
    //  }
    //}
    pcl::PointXYZRGB point;
    for (int i= 0; i < cloud->points.size(); i++){
        point = cloud->points[i];
        if(point.z < ((z_max-0.01))){
            cloud_f->points[i].x = point.x;
            cloud_f->points[i].y = point.y;
            cloud_f->points[i].z = point.z;

            cloud_f->points[i].r = point.r; 
            cloud_f->points[i].g = point.g; 
            cloud_f->points[i].b = point.b; 
        }
    }
    return cloud_f;
}

float findZMax(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
    float max = -1;
    int idx = 0;
    for(int i = 0; i < cloud->height; ++i){
      for(int j = 0; j < cloud->width; ++j){
        //std::cout << cloud->points[idx].z << std::endl;
        if(cloud->points[idx].z > max){
          max = cloud->points[idx].z;
        }
        idx++;
      }
    }
    return max;
}
*/

float checkOverlap(cv::Rect r1, cv::Rect r2){
    //std::cout << r1.x << " " << r1.y << " " <<  (r1.x+r1.width) << " " << (r1.y+r1.height) << std::endl;
    //std::cout << r2.x << " " << r2.y << " " <<  (r2.x+r2.width) << " " << (r2.y+r2.height) << std::endl;
    float x_left = std::max(r1.x, r2.x);
    float x_right = std::min((r1.x+r1.width), (r2.x+r2.width));
    float y_top = std::max(r1.y, r2.y);
    float y_bottom = std::min((r1.y+r1.height), (r2.y+r2.height));
    //std::cout << x_left << " " <<  y_top << " " <<  x_right << " " << y_bottom << std::endl;
    if ((x_right < x_left) || (y_bottom < y_top))
        return 0.0;
    float insertion_area = (x_right - x_left) * (y_bottom - y_top);
    float r1_area = ((r1.x+r1.width)-r1.x)*((r1.y+r1.height)-r1.y);
    float r2_area = ((r2.x+r2.width)-r2.x)*((r2.y+r2.height)-r2.y);
    //std::cout << "AREE: " << r1_area << " " << r2_area << std::endl;
    float iou = insertion_area / float(r1_area+r2_area-insertion_area);
    //std::cout << "iou: " << iou << std::endl;
    return iou;
}
