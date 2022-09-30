/*
sudo sh -c 'echo 100 > /sys/devices/pwm-fan/target_pwm' 쿨링팬 켜기
g++ -o all all.cpp -lJetsonGPIO $(pkg-config opencv4 --libs --cflags)
sudo ./all
*/
#include <csignal>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>       // Used for UART
#include <sys/fcntl.h>    // Used for UART
#include <termios.h>      // Used for UART
#include <string>
#include <iostream>
#include <fcntl.h>
#include <JetsonGPIO.h>
#include "opencv2/opencv.hpp"
#include <pthread.h>
#include <errno.h>

#define LED 24
#define PIR_FRONT 16
#define PIR_BACK 19
//#define speaker 13

#define     NSERIAL_CHAR   256
#define     VMINX          1
#define     BAUDRATE       B115200

using namespace cv;
using namespace GPIO;
using namespace std;
using namespace dnn;

Mat frame, Big_frame, Gray_frame;
    //Mat dx, dy;
    //Mat fmag, mag;
    //Mat edge;
Mat Bin;
Mat dst, dst2, dst3, dst4;
Mat labels, stats, centroids, box;
    

VideoCapture cap(0, CAP_V4L);

void GPIOsetup();
void signalingHandler(int signo);
//void video();

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

// Define Constants
const char *uart_target = "/dev/ttyTHS1";
int fid = -1;
int thread_id = 0;

void (*breakCapture)(int);

int main(int argc, char const *argv[])
{
    pthread_t p_thread[1];      // 쓰레드 ID
    int thread_id=0;

    if(!cap.isOpened()){
            cout << "Camera open failed!" << endl;
            return -1;
    }
    
    GPIOsetup();
    //PWM a(speaker, 4000);
    //a.start(0);

	// SETUP SERIAL WORLD
    int fid = -1;
	struct termios  port_options;   // Create the structure                          
    tcgetattr(fid, &port_options);	// Get the current attributes of the Serial port 
    fid = open(uart_target, O_RDWR | O_NONBLOCK );
	tcflush(fid, TCIFLUSH);
 	tcflush(fid, TCIOFLUSH);
    usleep(1000000);  // 1 sec delay
    if (fid == -1)
	{
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	}
    port_options.c_cflag &= ~PARENB;            // Disables the Parity Enable bit(PARENB),So No Parity   
    port_options.c_cflag &= ~CSTOPB;            // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit 
    port_options.c_cflag &= ~CSIZE;	            // Clears the mask for setting the data size             
    port_options.c_cflag |=  CS8;               // Set the data bits = 8                                 	 
    port_options.c_cflag &= ~CRTSCTS;           // No Hardware flow Control                         
    port_options.c_cflag |=  CREAD | CLOCAL;                  // Enable receiver,Ignore Modem Control lines       				
    port_options.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both input & output
    port_options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non Cannonical mode                            
    port_options.c_oflag &= ~OPOST;                           // No Output Processing

    port_options.c_lflag = 0;               //  enable raw input instead of canonical,
 		
    port_options.c_cc[VMIN]  = VMINX;       // Read at least 1 character
    port_options.c_cc[VTIME] = 0;           // Wait indefinetly 
		
    cfsetispeed(&port_options,BAUDRATE);    // Set Read  Speed 
    cfsetospeed(&port_options,BAUDRATE);    // Set Write Speed 

    // Set the attributes to the termios structure
    int att = tcsetattr(fid, TCSANOW, &port_options);

    if (att != 0 )
    {
        printf("\nERROR in Setting port attributes");
    }
    else
    {
        printf("\nSERIAL Port Good to Go.\n");
    }

    // Flush Buffers
    tcflush(fid, TCIFLUSH);
    tcflush(fid, TCIOFLUSH);

    usleep(500000);   // 0.5 sec delay
   
    unsigned char rx_buffer[5]={};
	tcflush(fid, TCIOFLUSH);
    usleep(1000);   // .001 sec delay

    fcntl(0, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK);
    unsigned char tx_buffer;
    
    cout<<"모드를 선택하세요 \r\n전진 : g,  후진 :  b 정지 : s"<<endl;
   
    signal(SIGINT, signalingHandler);

    while(1){
        int tx_length = read(STDIN_FILENO ,&tx_buffer, 1);
        int rx_length = read(fid, (void*)rx_buffer,5);  

        if(tx_length>0) {
            usleep(1000);
            write(fid, &tx_buffer, 1);
        }
        
       /*
        if(rx_length>0) 
        {   
            std::basic_string<char> str1(rx_buffer,10);
            std::cout<<str1<<endl;
        }*/

         while(GPIO::input(PIR_FRONT)) //PIR센서 신호가 들어올 때만
        {
            tx_buffer='f'; //f 단어 보냄
            write(fid, &tx_buffer, 1);

            //video();

            cap >> frame; //영상처리를 위해 캠 영상을 영상 프레임에 주입

            resize(frame, Big_frame, Size(500, 500)); //열화상 카메라 resize
            cvtColor(Big_frame, Gray_frame, COLOR_BGR2GRAY); //영상을 그레이스케일로 변경

            threshold(Gray_frame, Bin, 110, 255, THRESH_BINARY); //이진화를 하여 열 정도가 175를 넘으면 하얀색으로 표시

            int cnt = connectedComponentsWithStats(Bin, labels, stats, centroids); //햐얀 영역의 가로 세로를 판단하기 위함

            for(int i = 1; i < cnt; i++){
                int* p = stats.ptr<int>(i);

                if(p[4] < 1500) continue; //햐얀 픽셀 수가 50개 이하인 객체는 객체 취급x

                if((p[2]/p[3])>=1) //객체의 가로가 세로보다 더 길 때(유해 야생동물로 취급)
                {
                    thread_id = pthread_create(&p_thread[1], NULL, predater, NULL);
                    if(thread_id < 0)
	                {
		                perror("thread create error : predater");
		                exit(1);
	                }
                    usleep(5000);
                    output(LED, 1);
                }
                else if((p[2]/p[3])<1) //객체의 세로가 가로보다 더 길 때(사람으로 취급)
                {
                    //a.ChangeDutyCycle(0);
                    usleep(5000);
                    output(LED, 0);
                }
                rectangle(Gray_frame, Rect(p[0], p[1], p[2], p[3]), Scalar(255,255,255), 4);
                rectangle(Bin, Rect(p[0], p[1], p[2], p[3]), Scalar(255,255,255), 4);
            }

            //morphologyEx(Bin, dst, MORPH_ERODE, Mat());
            morphologyEx(Bin, dst2, MORPH_OPEN, Mat());
            //morphologyEx(Bin, dst3, MORPH_DILATE, Mat());
            //morphologyEx(Bin, dst4, MORPH_CLOSE, Mat());

            //Sobel(Gray_frame, dx, CV_32FC1, 1, 0);
            //Sobel(Gray_frame, dy, CV_32FC1, 0, 1);

            //magnitude(dx, dy, fmag);
            //fmag.convertTo(mag, CV_8UC1);

            //Mat edge = mag > 50;

            //output(LED, 1);

            imshow("Gray_frame", Gray_frame);
            //imshow("edge", edge);
            imshow("Bin", Bin);
            //imshow("ERODE", dst);
            imshow("OPEN", dst2);
            //imshow("DILATE", dst3);
            //imshow("CLOSE", dst4);

            waitKey(10);

            if(GPIO::input(PIR_FRONT)==0)
            {
                //a.ChangeDutyCycle(0);
                
                output(LED, 1);
                usleep(5000);
            }
            if(frame.empty())
            break;
        }

        while(GPIO::input(PIR_BACK)) //PIR센서 신호가 들어올 때만
        {
            tx_buffer='b'; //b 단어 보냄
            write(fid, &tx_buffer, 1);

            //video();

            cap >> frame; //영상처리를 위해 캠 영상을 영상 프레임에 주입

            resize(frame, Big_frame, Size(500, 500)); //열화상 카메라 resize
            cvtColor(Big_frame, Gray_frame, COLOR_BGR2GRAY); //영상을 그레이스케일로 변경

            threshold(Gray_frame, Bin, 110, 255, THRESH_BINARY); //이진화를 하여 열 정도가 175를 넘으면 하얀색으로 표시

            int cnt = connectedComponentsWithStats(Bin, labels, stats, centroids); //햐얀 영역의 가로 세로를 판단하기 위함

            for(int i = 1; i < cnt; i++){
                int* p = stats.ptr<int>(i);

                if(p[4] < 1500) continue; //햐얀 픽셀 수가 50개 이하인 객체는 객체 취급x

                if((p[2]/p[3])>=1) //객체의 가로가 세로보다 더 길 때(유해 야생동물로 취급)
                {
                    thread_id = pthread_create(&p_thread[1], NULL, predater, NULL);
                    if(thread_id < 0)
	                {
		                perror("thread create error : predater");
		                exit(1);
	                }
                    usleep(5000);
                    output(LED, 1);
                }
                else if((p[2]/p[3])<1) //객체의 세로가 가로보다 더 길 때(사람으로 취급)
                {
                    //a.ChangeDutyCycle(0);
                    usleep(5000);
                    output(LED, 0);
                }
                rectangle(Gray_frame, Rect(p[0], p[1], p[2], p[3]), Scalar(255,255,255), 4);
                rectangle(Bin, Rect(p[0], p[1], p[2], p[3]), Scalar(255,255,255), 4);
            }

            //morphologyEx(Bin, dst, MORPH_ERODE, Mat());
            morphologyEx(Bin, dst2, MORPH_OPEN, Mat());
            //morphologyEx(Bin, dst3, MORPH_DILATE, Mat());
            //morphologyEx(Bin, dst4, MORPH_CLOSE, Mat());

            //Sobel(Gray_frame, dx, CV_32FC1, 1, 0);
            //Sobel(Gray_frame, dy, CV_32FC1, 0, 1);

            //magnitude(dx, dy, fmag);
            //fmag.convertTo(mag, CV_8UC1);

            //Mat edge = mag > 50;

            //output(LED, 1);

            imshow("Gray_frame", Gray_frame);
            //imshow("edge", edge);
            imshow("Bin", Bin);
            //imshow("ERODE", dst);
            imshow("OPEN", dst2);
            //imshow("DILATE", dst3);
            //imshow("CLOSE", dst4);

            waitKey(10);

            if(GPIO::input(PIR_BACK)==0)
            {
                //a.ChangeDutyCycle(0);
                
                output(LED, 1);
                usleep(5000);
            }
            if(frame.empty())
            break;
        }
        destroyAllWindows();
        
    }
    return 0;
}

void GPIOsetup()
{
    setwarnings(false);

    setmode(GPIO::BCM);
    setup(PIR_FRONT, GPIO::IN);
    setup(PIR_BACK, GPIO::IN);
    setup(LED, GPIO::OUT);
    //setup(speaker, GPIO::OUT, GPIO::HIGH);
}

void signalingHandler(int signo) 
{
    close(fid);
    printf("\n\nGoodbye World\n\n");
    cap.release();
    cleanup();
    exit(signo);
}

/*void video()
{
    cap >> frame;

    resize(frame, Big_frame, Size(500, 500));
    cvtColor(Big_frame, Gray_frame, COLOR_BGR2GRAY);

     threshold(Gray_frame, Bin, 150, 255, THRESH_BINARY);

            int cnt = connectedComponentsWithStats(Bin, labels, stats, centroids);

            for(int i = 1; i < cnt; i++){
                int* p = stats.ptr<int>(i);

                if(p[4] < 20) continue;

                if((p[2]/p[3])>=1)
                {
                    a.ChangeDutyCycle(90);
                    usleep(100000);
                }
                else if((p[2]/p[3])<1)
                {
                    a.ChangeDutyCycle(0);
                    usleep(100000);
                }
            
                rectangle(Gray_frame, Rect(p[0], p[1], p[2], p[3]), Scalar(255,255,255), 4);
                rectangle(Bin, Rect(p[0], p[1], p[2], p[3]), Scalar(255,255,255), 4);
            }

            //morphologyEx(Bin, dst, MORPH_ERODE, Mat());
            morphologyEx(Bin, dst2, MORPH_OPEN, Mat());
            //morphologyEx(Bin, dst3, MORPH_DILATE, Mat());
            //morphologyEx(Bin, dst4, MORPH_CLOSE, Mat());

            //Sobel(Gray_frame, dx, CV_32FC1, 1, 0);
            //Sobel(Gray_frame, dy, CV_32FC1, 0, 1);

            //magnitude(dx, dy, fmag);
            //fmag.convertTo(mag, CV_8UC1);

            //Mat edge = mag > 50;

            output(LED, 1);

            imshow("Gray_frame", Gray_frame);
            //imshow("edge", edge);
            imshow("Bin", Bin);
            //imshow("ERODE", dst);
            imshow("OPEN", dst2);
            //imshow("DILATE", dst3);
            //imshow("CLOSE", dst4);

            waitKey(10);
}*/