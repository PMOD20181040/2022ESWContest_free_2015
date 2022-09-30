#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <JetsonGPIO.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>

#define LED 24

using namespace std;
using namespace cv;
using namespace dnn;
//using namespace GPIO;

//void GPIOsetup();

extern "C" void *predater(void*)
{
    system("canberra-gtk-play -f tiger.wav");
    sleep(1);
    return NULL;
}

extern "C" void *siren(void*)
{
    system("canberra-gtk-play -f siren.wav");
    sleep(1);
    return NULL;
}

int main()
{
    //Mat img = imread("boartest.jpg");
    pthread_t p_thread[2];      // 쓰레드 ID
    int thread_id = 0;
    int i = 0;
    //GPIOsetup();

    string class_name;
    std::vector<std::string> classes;
    std::ifstream file("boar_human_elk.names");
    std::string line;

    while (std::getline(file, line)) {
        classes.push_back(line);
    }

    Net net = readNetFromDarknet("yolov4-tiny-custom.cfg", "weights2/yolov4-tiny-custom_last.weights");

    VideoCapture cap(0);

    net.setPreferableBackend(DNN_BACKEND_CUDA);
    net.setPreferableTarget(DNN_TARGET_CUDA);

    while (cap.isOpened()) {
        Mat image;
        bool isSuccess = cap.read(image);

        if(!isSuccess){
            cout << "Could not load the image!" << endl;
            break;
        }

        auto start = getTickCount();

        DetectionModel model = DetectionModel(net);
        model.setInputParams(1 / 255.0, Size(512, 512), Scalar(), true);

        std::vector<int> classIds;
        std::vector<float> scores;
        std::vector<Rect> boxes;
        model.detect(image, classIds, scores, boxes, 0.5, 0.5);

        auto end = getTickCount();

        for (i = 0; i < classIds.size(); i++) {
            rectangle(image, boxes[i], Scalar(0, 255, 0), 2);

            char text[100];
            snprintf(text, sizeof(text), "%s: %.2f", classes[classIds[i]].c_str(), scores[i]);
            cout << scores[i] << endl;
            putText(image, text, Point(boxes[i].x, boxes[i].y - 5), cv::FONT_HERSHEY_SIMPLEX, 1,
             Scalar(0, 255, 0), 2);
             
        if(classIds[i] <= 0){
            thread_id = pthread_create(&p_thread[1], NULL, predater, NULL);
            if(thread_id < 0){
                perror("thread create error : predater");
                exit(1);
                }
                //output(LED, 0);
        }
        
        else if(classIds[i] == 1){
            usleep(5000);
            //output(LED, 1);
            }
        

        else {
            thread_id = pthread_create(&p_thread[2], NULL, siren, NULL);
            if(thread_id < 0){
                perror("thread create error : siren");
                exit(1);
                }
            
        }
       
        }
        
        auto totalTime = (end - start) / getTickFrequency();

        putText(image, "FPS" + to_string(int(1 / totalTime)), Point(50,50), FONT_HERSHEY_DUPLEX, 1, Scalar(0, 255, 0), 2, false);

        imshow("Image", image);

         if(scores[i]>=1.0){
            break;
        }
         else usleep(100);
        

        int k = waitKey(10);
            if (k == 113){
                break;
            }
        }
    cap.release();
    destroyAllWindows();
    return 0;
}

/*void GPIOsetup()
{
    setwarnings(false);

    setmode(GPIO::BCM);
    setup(LED, GPIO::OUT);
}*/
