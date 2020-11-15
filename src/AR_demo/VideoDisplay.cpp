//
// Created by luka on 19. 09. 20.
//

#include "VideoDisplay.h"
#include <GL/glext.h>
#include <opencv2/highgui.hpp>

VideoDisplay::VideoDisplay(const std::string &videoPath, std::vector<cv::Point3f> displayPosition) {
    pcap.open(videoPath);
    lastFrameN = -1;

    if (!pcap.isOpened()) {
        std::cerr << "Cannot read '" << videoPath << "'. Exiting... " << std::endl;
        exit(-1);
    }
    auto fps = (double) pcap.get(cv::CAP_PROP_FPS);
    tFrame = 1000 / fps;

    width = (int) pcap.get(cv::CAP_PROP_FRAME_WIDTH);
    height = (int) pcap.get(cv::CAP_PROP_FRAME_HEIGHT);

    for (int i = 0; i < 4; i++) {
        bgTextureVertices[i*3] = displayPosition[i].x;
        bgTextureVertices[i*3+1] = displayPosition[i].y;
        bgTextureVertices[i*3+2] = displayPosition[i].z;
    }

    glGenTextures(1, &videoTextureID);
    glBindTexture(GL_TEXTURE_2D, videoTextureID);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
}

void VideoDisplay::Show() {

    if (lastFrameN < 0) {
        start = std::chrono::steady_clock::now();
    }
    double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
    int n = floor(elapsed / tFrame);
    if (n > lastFrameN) {
        while (pcap.get(cv::CAP_PROP_POS_FRAMES) < n - 1) {
            pcap.grab();
        }
        lastFrameN = n;
        pcap.read(lastFrame);
    }


    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glBindTexture(GL_TEXTURE_2D, videoTextureID);

    // Upload new texture data:
    if (lastFrame.channels() == 3)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_BGR_EXT, GL_UNSIGNED_BYTE, lastFrame.data);
    else if(lastFrame.channels() == 4)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, lastFrame.data);
    else if (lastFrame.channels()==1)
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_LUMINANCE, GL_UNSIGNED_BYTE, lastFrame.data);


    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, videoTextureID);

    // Update attribute values.
    glEnableClientState(GL_VERTEX_ARRAY);
    glEnableClientState(GL_TEXTURE_COORD_ARRAY);

    glVertexPointer(3, GL_FLOAT, 0, bgTextureVertices);
    glTexCoordPointer(2, GL_FLOAT, 0, bgTextureCoords);

    glColor4f(1,1,1,1);
    glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

    glDisableClientState(GL_VERTEX_ARRAY);
    glDisableClientState(GL_TEXTURE_COORD_ARRAY);
    glDisable(GL_TEXTURE_2D);
}