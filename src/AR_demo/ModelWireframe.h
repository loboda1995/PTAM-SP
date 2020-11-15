//
// Created by luka on 19. 09. 20.
//

#ifndef PTAM_MODELWIREFRAME_H
#define PTAM_MODELWIREFRAME_H

#include <vector>
#include <glad/glad.h>

class ModelWireframe {
    struct Vertex{
        GLfloat x, y, z;

        void Draw() {
            glVertex3f(x, y, z);
        }
    };
public:
    ModelWireframe(const std::string &modelPath);

    void Show();
private:
    std::vector<Vertex> vertices;
    std::vector<std::vector<unsigned int>> faces;
};


#endif //PTAM_MODELWIREFRAME_H
