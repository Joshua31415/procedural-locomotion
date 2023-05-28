#pragma once

#include "crl-basic/gui/guiMath.h"
#include "crl-basic/gui/model.h"
#include "crl-basic/utils/logger.h"

namespace crl {
namespace gui {

void drawSphere(const P3D &p, double r, const Shader &shader, const V3D &color = V3D(1, 0, 0), float alpha = 1.0);

/**
 * p is center of ellipsode
 * orientation is orientation of ellipsoid
 * dims is radius along each axis (in ellipsoid's frame)
 */
void drawEllipsoid(const P3D &p, const Quaternion &orientation, const V3D &dims, const Shader &shader, const V3D &color = V3D(1, 0, 0), float alpha = 1.0);

void drawCuboid(const P3D &p, const Quaternion &orientation, const V3D &dims, const Shader &shader, const V3D &color = V3D(1, 0, 0), float alpha = 1.0);

void drawWireFrameCuboid(const P3D &p, const Quaternion &orientation, const V3D &dims, const Shader &shader, const V3D &color = V3D(1, 0, 0),
                         float alpha = 1.0);

void drawCylinder(const P3D &startPosition, const P3D &endPosition, const double &radius, const Shader &shader, const V3D &color = V3D(1, 0, 0),
                  float alpha = 1.0);

void drawCylinder(const P3D &startPosition, const V3D &direction, const double &radius, const Shader &shader, const V3D &color = V3D(1, 0, 0),
                  float alpha = 1.0);

void drawCone(const P3D &origin, const V3D &direction, const double &radius, const Shader &shader, const V3D &color = V3D(1, 0, 0), float alpha = 1.0);

void drawArrow3d(const P3D &origin, const V3D &direction, const double &radius, const Shader &shader, const V3D &color = V3D(1, 0, 0), float alpha = 1.0);

void drawCapsule(const P3D &sP, const P3D &eP, const double &radius, const Shader &shader, const V3D &color = V3D(1, 0, 0), float alpha = 1.0);

void drawCapsule(const P3D &startPosition, const V3D &direction, const double &radius, const Shader &shader, const V3D &color = V3D(1, 0, 0),
                 float alpha = 1.0);

void drawCircle(const P3D &center, const V3D &normal, const double &radius, const Shader &shader, const V3D &color = V3D(1, 0, 0), float alpha = 1.0);

void drawRectangle(const P3D &p, const V3D &normal, const double &angle, const Vector2d &dims, const Shader &shader, const V3D &color = V3D(1, 0, 0),
                   float alpha = 1.0);

void drawEnvMap(const Shader &shader);

/**
 * we could go from the vector "from" the long way, or the short way. The Vector up will tell us which one is meant
 */
void drawSector(const P3D &p, const V3D &from, const V3D &to, const V3D &up, const Shader &shader, const V3D &color = V3D(1, 0, 0), float alpha = 1.0);

Model getGroundModel(double s = 100);

class SizableGroundModel {
public:
    SizableGroundModel(int size);

    void setSize(int size);

    int getSize() const;

    void draw(const Shader &shader, const double &intensity = 1.0, const V3D &groundColor = V3D(0.95, 0.95, 0.95),
              const V3D &gridColor = V3D(0.78431, 0.78431, 0.78431)) const;

    [[nodiscard]] double getHeight(const P3D &position) const {
        return 0.0;
    }

private:
    int size;
    Model ground;

public:
    double gridThickness = 0.025;
    bool showGrid = true;
};


class SimpleGroundModel {
public:

    static bool isFlat;

    // static Model groundFlat;
    static Model groundUneven;

    static SizableGroundModel groundFlat;
    // static Model grid1;
    // static Model grid2;


    static void draw(const Shader &shader, double intensity, const V3D &col = V3D(0.7, 0.7, 0.9)) {
        if(isFlat){
            groundFlat.draw(shader, intensity, col);
        } else {
            groundUneven.draw(shader, col * intensity);
        }
        // getGround().draw(shader, col*intensity);
//        sphere.draw(shader, col*intensity);
    }

    static bool hitByRay(const P3D &r_o, const V3D &r_v, P3D &hitPoint) {
        P3D bestHit = hitPoint;
        double currentBestSq = std::numeric_limits<double>::max();

        for(const auto &model : {SimpleGroundModel::groundUneven /*sphere*/}){
            if(model.hitByRay(r_o, r_v, hitPoint)){
                const auto distSq = V3D(r_o, hitPoint).squaredNorm();
                if(distSq < currentBestSq){
                    bestHit = hitPoint;
                    currentBestSq = distSq;
                }
            }
        }


        hitPoint = bestHit;
        const bool didHit = currentBestSq != std::numeric_limits<double>::max();
        return didHit;
    }

    //assumes that the terrain has no obstruction above
    //returns absolute height of terrain, not relative to pos
    [[nodiscard]] static double getHeight(const P3D& position) {
        if (isFlat) {
            return 0.0;
        } else {
            constexpr double maxHeight = 1000;

            P3D hitPoint{};

            if (hitByRay(P3D(position[0], maxHeight, position[2]), V3D(0, -1, 0), hitPoint)) {
                return hitPoint[1];
            }
            return 0.0;
        }
    }
};

namespace rendering {

struct RenderingContext {
    // subcontext
    MeshRenderingContext *mctx = nullptr;

    // reusable primitive models
    Model sphere = Model(CRL_DATA_FOLDER "/meshes/sphere.obj");
    Model cube = Model(CRL_DATA_FOLDER "/meshes/cube.obj");
    Model environmentMap = Model(CRL_DATA_FOLDER "/meshes/envmap.obj");
    Model cubeFrame = Model(CRL_DATA_FOLDER "/meshes/cube_frame.obj");
    Model cylinder = Model(CRL_DATA_FOLDER "/meshes/cylinder.obj");
    Model cone = Model(CRL_DATA_FOLDER "/meshes/cone.obj");
    Model sector = Model(CRL_DATA_FOLDER "/meshes/sector.obj");
};

RenderingContext *CreateContext();

// if ctx is null, it destroy current context
void DestroyContext(RenderingContext *ctx = NULL);

// getter for current context
RenderingContext *GetCurrentContext();

// setter for current context
void SetCurrentContext(RenderingContext *ctx);

}  // namespace rendering
}  // namespace gui
}  // namespace crl