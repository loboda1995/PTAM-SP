//
// Created by luka on 19. 09. 20.
//

#ifndef PTAM_VIDEODISPLAY_H
#define PTAM_VIDEODISPLAY_H


#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <glad/glad.h>

class VideoDisplay {
public:
    VideoDisplay(const std::string &videoPath, std::vector<cv::Point3f> displayPosition);

    void Show();

private:
    cv::VideoCapture pcap;
    int lastFrameN, width, height;
    double tFrame;
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::duration<float>> start;
    cv::Mat lastFrame;

    unsigned int videoTextureID;

    const GLfloat bgTextureCoords[8]{0, 0, 1, 0, 0, 1, 1, 1 };
    GLfloat bgTextureVertices[12];
};


#endif //PTAM_VIDEODISPLAY_H
