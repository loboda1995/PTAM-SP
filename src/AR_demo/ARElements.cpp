//
// Created by luka on 18. 09. 20.
//
#include "ARElements.h"
#include <glad/glad.h>

void ARElements::DrawCube(float cx, float cy, float cz, float sx, float sy, float sz, float r, float g, float b,
                     float borderOffset) {
    auto sizex = sx / 2.0;
    auto sizey = sy / 2.0;
    auto sizez = sz / 2.0;

    glEnable(GL_DEPTH_TEST);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
    glTranslatef(cx, cy, cz);

    glBegin(GL_QUADS);

    glColor3f(r, g, b);

    // FRONT
    glVertex3f(-sizex, -sizey, sizez);
    glVertex3f(sizex, -sizey, sizez);
    glVertex3f(sizex, sizey, sizez);
    glVertex3f(-sizex, sizey, sizez);

    // BACK
    glVertex3f(-sizex, -sizey, -sizez);
    glVertex3f(-sizex, sizey, -sizez);
    glVertex3f(sizex, sizey, -sizez);
    glVertex3f(sizex, -sizey, -sizez);

    // LEFT
    glVertex3f(-sizex, -sizey, sizez);
    glVertex3f(-sizex, sizey, sizez);
    glVertex3f(-sizex, sizey, -sizez);
    glVertex3f(-sizex, -sizey, -sizez);

    // RIGHT
    glVertex3f(sizex, -sizey, -sizez);
    glVertex3f(sizex, sizey, -sizez);
    glVertex3f(sizex, sizey, sizez);
    glVertex3f(sizex, -sizey, sizez);

    // TOP
    glVertex3f(-sizex, sizey, sizez);
    glVertex3f(sizex, sizey, sizez);
    glVertex3f(sizex, sizey, -sizez);
    glVertex3f(-sizex, sizey, -sizez);

    // BOTTOM
    glVertex3f(-sizex, -sizey, sizez);
    glVertex3f(-sizex, -sizey, -sizez);
    glVertex3f(sizex, -sizey, -sizez);
    glVertex3f(sizex, -sizey, sizez);

    glEnd();

    if (borderOffset > 0) {
        glColor3d(0, 0, 0);

        // FRONT
        glBegin(GL_LINE_STRIP);
        glVertex3f(-sizex - borderOffset, -sizey - borderOffset, sizez + borderOffset);
        glVertex3f(sizex + borderOffset, -sizey - borderOffset, sizez + borderOffset);
        glVertex3f(sizex + borderOffset, sizey + borderOffset, sizez + borderOffset);
        glVertex3f(-sizex - borderOffset, sizey + borderOffset, sizez + borderOffset);
        glEnd();

        // BACK
        glBegin(GL_LINE_STRIP);
        glVertex3f(-sizex - borderOffset, -sizey - borderOffset, -sizez - borderOffset);
        glVertex3f(-sizex - borderOffset, sizey + borderOffset, -sizez - borderOffset);
        glVertex3f(sizex + borderOffset, sizey + borderOffset, -sizez - borderOffset);
        glVertex3f(sizex + borderOffset, -sizey - borderOffset, -sizez - borderOffset);
        glEnd();

        // LEFT
        glBegin(GL_LINE_STRIP);
        glVertex3f(-sizex - borderOffset, -sizey - borderOffset, sizez + borderOffset);
        glVertex3f(-sizex - borderOffset, sizey + borderOffset, sizez + borderOffset);
        glVertex3f(-sizex - borderOffset, sizey + borderOffset, -sizez - borderOffset);
        glVertex3f(-sizex - borderOffset, -sizey - borderOffset, -sizez + borderOffset);
        glEnd();

        // RIGHT
        glBegin(GL_LINE_STRIP);
        glVertex3f(sizex + borderOffset, -sizey - borderOffset, -sizez - borderOffset);
        glVertex3f(sizex + borderOffset, sizey + borderOffset, -sizez - borderOffset);
        glVertex3f(sizex + borderOffset, sizey + borderOffset, sizez + borderOffset);
        glVertex3f(sizex + borderOffset, -sizey - borderOffset, sizez + borderOffset);
        glEnd();

        // TOP
        glBegin(GL_LINE_STRIP);
        glVertex3f(-sizex - borderOffset, sizey + borderOffset, sizez + borderOffset);
        glVertex3f(sizex + borderOffset, sizey + borderOffset, sizez + borderOffset);
        glVertex3f(sizex + borderOffset, sizey + borderOffset, -sizez - borderOffset);
        glVertex3f(-sizex - borderOffset, sizey + borderOffset, -sizez - borderOffset);
        glEnd();

        // BOTTOM
        glBegin(GL_LINE_STRIP);
        glVertex3f(-sizex - borderOffset, -sizey - borderOffset, sizez + borderOffset);
        glVertex3f(-sizex - borderOffset, -sizey - borderOffset, -sizez - borderOffset);
        glVertex3f(sizex + borderOffset, -sizey - borderOffset, -sizez - borderOffset);
        glVertex3f(sizex + borderOffset, -sizey - borderOffset, sizez + borderOffset);
        glEnd();
    }


    glTranslatef(-cx, -cy, -cz);
    glDisable(GL_DEPTH_TEST);
}


