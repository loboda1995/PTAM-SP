//
// Created by luka on 19. 09. 20.
//

#include <string>
#include "ModelWireframe.h"
#include "opencv2/highgui.hpp"
#include <OpenMVS/MVS.h>

ModelWireframe::ModelWireframe(const std::string &modelPath) {
    MVS::Scene mvs_scene;
    mvs_scene.Load(modelPath);

    for (const auto &v : mvs_scene.mesh.vertices) {
        Vertex p;
        p.x = v.x;
        p.y = v.y;
        p.z = v.z;
        vertices.push_back(p);
    }
    for (const auto &f : mvs_scene.mesh.faces) {
        faces.push_back({f[0], f[1], f[2]});
    }
    mvs_scene.Release();
}

void ModelWireframe::Show() {
    glColor4d(1, 0, 0, 0.8);
    glLineWidth(1);
    glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );
    glBegin(GL_TRIANGLES);
    for (const auto &face : faces) {
        vertices[face[0]].Draw();
        vertices[face[1]].Draw();
        vertices[face[2]].Draw();
    }
    glEnd();
    glPolygonMode( GL_FRONT_AND_BACK, GL_FILL );
}
