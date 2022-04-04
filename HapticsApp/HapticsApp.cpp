/*****************************************************************************
HapticsApp.cpp
*******************************************************************************/
#ifdef  _WIN64
#pragma warning (disable:4996)
#endif

// Standard Library
#include <iostream>
#include <string>
#include <cstdio>
#include <cassert>
#include <sstream>
#include <vector>
using namespace std;

// Windows Library
#if defined(WIN32)
# include <conio.h>
#else
# include "conio.h"
#endif

// Haptics Device Library
#include <HD/hd.h>
#include <HDU/hduVector.h>
#include <HDU/hduError.h>

// UDP Librara
#pragma comment(lib, "ws2_32.lib")
#include <WinSock2.h>


// Global variables
float hd_position[3];
float hd_force[3];
int gimbal_angle = 0;
int button = 0;




/*******************************************************************************
 * Callback
 ******************************************************************************/
HDCallbackCode HDCALLBACK HapticsAppCallback(void* data)
{
    double stiffness = 0.05;
    static hduVector3Dd anchor = { 0, 0, 0 };

    hdBeginFrame(hdGetCurrentDevice());

    // Get the position of the device.
    hduVector3Dd position;
    hdGetDoublev(HD_CURRENT_POSITION, position);
    hd_position[0] = position[0];
    hd_position[1] = position[1];
    hd_position[2] = position[2];
    // cout << "position: " << position << endl;

    // Get angle
    hduVector3Dd gimbalAngles;
    hduVector3Dd jointAngles;
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbalAngles);
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES, jointAngles);
    gimbal_angle = gimbalAngles[1] * 100;
    // cout << gimbalAngles << endl;

    // Get the button state
    HDint nCurrentButtons;
    hdGetIntegerv(HD_CURRENT_BUTTONS, &nCurrentButtons);
    button = nCurrentButtons;

    // Get anchor
    hduVector3Dd force = { 0, 0, 0 };
    //force[0] = hd_force[0];
    //force[1] = hd_force[1];
    //force[2] = hd_force[2];
    

    // Move arm freely
    if (button == 1 || button == 3) {
        force[0] = hd_force[0];
        force[1] = hd_force[1];
        force[2] = hd_force[2];
        cout << "Applied Force: " << force << endl;
        stiffness = 1.0;
    }

    // Fix arm pos
    else {
        hduVecSubtract(force, anchor, position);
    }


    // Apply force
    force = stiffness * force;
    hdSetDoublev(HD_CURRENT_FORCE, force);


    hdEndFrame(hdGetCurrentDevice());

    // In case of error, terminate the callback.
    HDErrorInfo error;
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Error detected during main scheduler callback\n");

        if (hduIsSchedulerError(&error))
        {
            return HD_CALLBACK_DONE;
        }
    }
    return HD_CALLBACK_CONTINUE;
}
/*****************************************************************************/




/*******************************************************************************
 * main function
******************************************************************************/
int main(int argc, char* argv[])
{
    WSAData wsaData;
    WSAStartup(MAKEWORD(2, 0), &wsaData);


    // Create sender socket
    SOCKET sock_send;
    sock_send = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr_send;
    addr_send.sin_family = AF_INET;  // IPv4
    addr_send.sin_port = htons(10010); // Port
    addr_send.sin_addr.S_un.S_addr = inet_addr("163.221.150.145"); // Address


    // Create receiver socket
    SOCKET sock_receive;
    sock_receive = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in addr_receive;
    addr_receive.sin_family = AF_INET;  // IPv4
    addr_receive.sin_port = htons(10030); // Port
    addr_receive.sin_addr.S_un.S_addr = INADDR_ANY; // Address
    bind(sock_receive, (struct sockaddr*)&addr_receive, sizeof(addr_receive));
    u_long val = 1;
    ioctlsocket(sock_receive, FIONBIO, &val);


    // Initialize the default haptic device.
    HDErrorInfo error;
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;
    }


    // Start the servo scheduler and enable forces.
    hdEnable(HD_FORCE_OUTPUT);
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start the scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        getch();
        return -1;
    }


    // Schedule the frictionless plane callback, which will then run at servoloop rates and command forces.
    HDCallbackCode hPlaneCallback = hdScheduleAsynchronous(HapticsAppCallback, 0, HD_DEFAULT_SCHEDULER_PRIORITY);

    printf("My HapticsApp\n");
    printf("Press any key to quit.\n\n");

    // Main loop
    while (!_kbhit())
    {
        if (!hdWaitForCompletion(hPlaneCallback, HD_WAIT_CHECK_STATUS))
        {
            fprintf(stderr, "\nThe main scheduler callback has exited\n");
            fprintf(stderr, "\nPress any key to quit.\n");
            getch();
            break;
        }

        // receive 
        char buf_receive[1024];
        memset(buf_receive, 0, sizeof(buf_receive));
        recv(sock_receive, buf_receive, sizeof(buf_receive), 0);
        string data = string(buf_receive);
        stringstream s_stream(data);
        vector<string> result;
        int i = 0;
        while (s_stream.good()) {
            string substr;
            getline(s_stream, substr, ',');
            result.push_back(substr);
            i = i + 1;
        }
        if (i == 4) {
            float max_force = 3.0;
            hd_force[0] = stod(result[1]); // y
            hd_force[1] = stod(result[2]); // z
            hd_force[2] = stod(result[0]); // x
            
            //printf("Receive: Fx %f Fy %f Fz %f\n", hd_force[2], hd_force[0], hd_force[1]);

            // Fx limit
            if (hd_force[2] > max_force) hd_force[2] = max_force;
            else if (hd_force[2] < -max_force) hd_force[2] = -max_force;
            //else hd_force[2] = 0;

            // Fy limit
            if (hd_force[0] > max_force) hd_force[0] = max_force;
            else if (hd_force[0] < -max_force) hd_force[0] = -max_force;
            //else hd_force[0] = 0;

            // Fz limit
            if (hd_force[1] > max_force) hd_force[1] = max_force;
            else if (hd_force[1] < -max_force) hd_force[1] = -max_force;
            //else if (hd_force[1] < 0) hd_force[1] = 0;
        }

        // send
        int32_t buf_send[5];
        buf_send[0] = hd_position[0] * 10;
        buf_send[1] = hd_position[1] * 10;
        buf_send[2] = hd_position[2] * 10;
        buf_send[3] = gimbal_angle;
        buf_send[4] = button;
        if (button == 0) {
            buf_send[0] = 0;
            buf_send[1] = 0;
            buf_send[2] = 0;
        }
        if (button == 0 || button == 1) {
            buf_send[3] = 0;
        }
        //printf("Send: %d %d %d %d %d\n", buf_send[0], buf_send[1], buf_send[2], buf_send[3], buf_send[4]);
        sendto(sock_send, (const char*)buf_send, sizeof(buf_send), 0, (struct sockaddr*)&addr_send, sizeof(addr_send));
    }

    // Cleanup and shutdown the haptic device, cleanup all callbacks.
    hdStopScheduler();
    hdUnschedule(hPlaneCallback);
    hdDisableDevice(hHD);

    return 0;
}
/*****************************************************************************/