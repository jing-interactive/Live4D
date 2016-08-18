#include "cinder/app/RendererGl.h"
#include "cinder/app/App.h"
#include "cinder/gl/gl.h"
#include "cinder/params/Params.h"
#include "cinder/Log.h"
#include "cinder/PolyLine.h"
#include "cinder/CameraUi.h"

#include "Cinder/osc/Osc.h"
#include "CinderOpenCV.h"

#include "DepthSensor.h"
#include "Cinder-VNM/include/MiniConfig.h"
#include "Cinder-VNM/include/TextureHelper.h"
#include "Cinder-VNM/include/AssetManager.h"

#include <vector>

#pragma comment (lib, "OpenNI2.lib")

using namespace std;
using namespace ci;
using namespace ci::app;
using namespace cv;

static const int VBO_X_RES = 640;
static const int VBO_Y_RES = 480;

using namespace ci;
using namespace ci::app;
using namespace std;

gl::VertBatchRef createGrid()
{
    auto grid = gl::VertBatch::create(GL_LINES);
    grid->begin(GL_LINES);
    float scale = 0.1f;
    for (int i = -10; i <= 10; ++i) {
        grid->color(Color(0.25f, 0.25f, 0.25f));
        grid->color(Color(0.25f, 0.25f, 0.25f));
        grid->color(Color(0.25f, 0.25f, 0.25f));
        grid->color(Color(0.25f, 0.25f, 0.25f));

        grid->vertex(float(i)*scale, 0.0f, -10.0f*scale);
        grid->vertex(float(i)*scale, 0.0f, +10.0f*scale);
        grid->vertex(-10.0f*scale, 0.0f, float(i)*scale);
        grid->vertex(+10.0f*scale, 0.0f, float(i)*scale);
    }
    grid->end();

    return grid;
}

class PointCloudGl : public App {
public:

    PointCloudGl();
    void           update()override;
    void           draw()override;

    gl::VboMeshRef createVboMesh();

    // PARAMS
    params::InterfaceGlRef	mParams;

    // CAMERA
    CameraPersp         mCam;
    CameraUi            mCamUi;

    // KINECT AND TEXTURES
    ds::DeviceRef		mDevice;
    gl::TextureRef  mDepthTexture;
    gl::TextureRef  mColorTexture;

    // BATCH AND SHADER
    gl::VertBatch pointCloud;

    gl::GlslProgRef	mPointCloudShader;

    gl::VertBatchRef grid;
};


PointCloudGl::PointCloudGl()
{
    readConfig();

    mCam.setEyePoint({2, 2, 2});
    mCam.lookAt({ 0, 0, 0 });
    mCamUi = CameraUi(&mCam, getWindow(), -1);

    // SETUP PARAMS
    mParams = createConfigUI({ 200, 180 });

    // SETUP KINECT AND TEXTURES
    ds::DeviceType type = ds::DeviceType(SENSOR_TYPE);
    ds::Option option;
    option.enableColor = true;
    option.enablePointCloud = true;
    mDevice = ds::Device::create(type, option);
    if (!mDevice->isValid())
    {
        quit();
        return;
    }

    mDevice->signalDepthDirty.connect([&]{
        updateTexture(mDepthTexture, mDevice->depthChannel);

        pointCloud.clear();
        auto count = mDevice->pointCloudXYZ.size();
        for (int i = 0; i < count; i++) {
            pointCloud.texCoord(mDevice->pointCloudUV[i]);
            pointCloud.vertex(mDevice->pointCloudXYZ[i]);
        }
    });
    mDevice->signalColorDirty.connect([&]{
        updateTexture(mColorTexture, mDevice->colorSurface);
    });

#if 0
    mDepthTexture = gl::Texture::create(mDevice->getDepthSize().x, mDevice->getDepthSize().y,
        gl::Texture::Format().internalFormat(GL_R16UI).dataType(GL_UNSIGNED_SHORT).minFilter(GL_NEAREST).magFilter(GL_NEAREST));
    mColorTexture = gl::Texture::create(mDevice->getDepthSize().x, mDevice->getDepthSize().y,
        gl::Texture::Format().internalFormat(GL_RGB8).dataType(GL_UNSIGNED_BYTE).wrap(GL_CLAMP_TO_EDGE));
#endif

    // SETUP VBO AND SHADER

    try {
        mPointCloudShader = gl::GlslProg::create(loadAsset("pointcloud.vert"), loadAsset("pointcloud.frag"));
    }
    catch (const gl::GlslProgCompileExc &e) {
        console() << e.what() << endl;
    }

    auto mesh = createVboMesh();
    mPointCloudShader->uniform("uDepthTexture", 0);
    mPointCloudShader->uniform("uDepthToMmScale", mDevice->getDepthToMmScale());

    grid = createGrid();

    // SETUP GL
    gl::enableDepthWrite();
    gl::enableDepthRead();
}

gl::VboMeshRef PointCloudGl::createVboMesh()
{
    vector<float> data;

    int numVertices = VBO_X_RES * VBO_Y_RES;

    for (int x = 0; x<VBO_X_RES; ++x){
        for (int y = 0; y<VBO_Y_RES; ++y){

            float xPer = x / (float)(VBO_X_RES - 1);
            float yPer = y / (float)(VBO_Y_RES - 1);

            data.push_back(x);
            data.push_back(y);
            data.push_back(0);
            data.push_back(xPer);
            data.push_back(yPer);

        }
    }

    geom::BufferLayout data_layout;
    data_layout.append(geom::POSITION, 3, sizeof(float) * 5, 0);
    data_layout.append(geom::TEX_COORD_0, 2, sizeof(float) * 5, sizeof(float) * 3);

    auto data_buffer = gl::Vbo::create(GL_ARRAY_BUFFER, sizeof(float)*data.size(), data.data(), GL_STATIC_DRAW);

    vector<pair<geom::BufferLayout, gl::VboRef>> layouts(1, make_pair(data_layout, data_buffer));

    return gl::VboMesh::create(numVertices, GL_POINTS, layouts);
}

void PointCloudGl::update()
{
    FPS = getAverageFps();
}

void PointCloudGl::draw()
{
    gl::clear(Color(0.0f, 0.0f, 0.0f), true);
    gl::setMatrices(mCam);

    if (COORDINATE_VISIBLE)
    {
        gl::ScopedDepthWrite depthWrite(false);
        gl::ScopedGlslProg glsl(am::glslProg("color"));
        grid->draw();
        gl::drawCoordinateFrame(1, 0.1, 0.01);
    }

    {
        gl::ScopedGlslProg glsl(am::glslProg("texture"));
        gl::ScopedTextureBind tb(mColorTexture, 0);
        gl::ScopedModelMatrix model;
        gl::scale({ SCALE, SCALE, SCALE });

        pointCloud.draw();
    }

    mParams->draw();
}

void prepareSettings(App::Settings* settings)
{
    settings->setWindowSize(1280, 720);
}

CINDER_APP(PointCloudGl, RendererGl, prepareSettings)
