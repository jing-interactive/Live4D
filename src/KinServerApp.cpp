#include "cinder/app/RendererGl.h"
#include "cinder/app/App.h"
#include "cinder/gl/gl.h"
#include "cinder/params/Params.h"
#include "cinder/Log.h"
#include "cinder/PolyLine.h"
#include "cinder/CameraUi.h"

#include "Cinder-DepthSensor/include/DepthSensor.h"
#include "Cinder-VNM/include/MiniConfigImgui.h"
#include "Cinder-VNM/include/TextureHelper.h"
#include "Cinder-VNM/include/AssetManager.h"

#include "RenderdocManager/include/RenderDocManager.h"

#include <vector>

using namespace std;
using namespace ci;
using namespace ci::app;

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
    for (int i = -10; i <= 10; ++i)
    {
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

class Live4D : public App {
public:

    Live4D();
    void           update()override;
    void           draw()override;

    gl::VboMeshRef createVboMesh();

    // CAMERA
    CameraPersp         mCam;
    CameraUi            mCamUi;

    // KINECT AND TEXTURES
    ds::DeviceRef		mDevice;
    gl::TextureRef  mDepthTexture;
    gl::TextureRef  mColorTexture;
    gl::TextureRef  mDepthToCameraTableTexture;
    gl::TextureRef  mDepthToColorTableTexture;

    // BATCH AND SHADER
    gl::VertBatch pointCloud;
    gl::VboMeshRef mVboMesh;

    gl::GlslProgRef	mPointCloudShader;

    gl::VertBatchRef grid;

	RenderDocManager renderdoc;
};


Live4D::Live4D()
{
    readConfig();

    mCam.setEyePoint({2, 2, 2});
    mCam.lookAt({ 0, 0, 0 });
    mCamUi = CameraUi(&mCam, getWindow(), -1);

    // SETUP PARAMS
    createConfigImgui();

    // SETUP KINECT AND TEXTURES
    ds::DeviceType type = ds::DeviceType(_SENSOR_TYPE);
    ds::Option option;
    option.enableColor = true;
    option.enablePointCloud = true;
    mDevice = ds::Device::create(type, option);
    if (!mDevice->isValid())
    {
        quit();
        return;
    }

	renderdoc.setup(getAssetPath("").string().c_str());

    mDevice->signaldepthToCameraTableDirty.connect([&]{
        updateTexture(mDepthToCameraTableTexture, mDevice->depthToCameraTable);
    });
    mDevice->signalDepthDirty.connect([&]{
        updateTexture(mDepthTexture, mDevice->depthChannel);
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
    grid = createGrid();
    mVboMesh = createVboMesh();

    mPointCloudShader = am::glslProg("pointcloud.vert", "pointcloud.frag");
    mPointCloudShader->uniform("uTextureDepth", 0);
    mPointCloudShader->uniform("uTextureColor", 1);
    mPointCloudShader->uniform("uTextureDepthToCameraTable", 2);
    mPointCloudShader->uniform("uTextureDepthToColorTable", 3);

    // SETUP GL
    gl::enableDepthWrite();
    gl::enableDepthRead();
}

gl::VboMeshRef Live4D::createVboMesh()
{
    ivec2 sz = mDevice->getDepthSize();
    vector<vec2> vertices;
    for (int32_t x = 0; x < sz.x; ++x) {
        for (int32_t y = 0; y < sz.y; ++y) {
            vertices.push_back(vec2(x, y) / vec2(sz));
        }
    }

    gl::VboRef vbo = gl::Vbo::create(GL_ARRAY_BUFFER, vertices.size() * sizeof(vec2), &vertices[0], GL_STATIC_DRAW);

    geom::BufferLayout layout;
    layout.append(geom::Attrib::POSITION, 2, sizeof(vec2), 0);
    auto vertexArrayBuffers = { make_pair(layout, vbo) };

    return gl::VboMesh::create(vertices.size(), GL_POINTS, vertexArrayBuffers);
}

void Live4D::update()
{
    _FPS = getAverageFps();

    if (mDepthToCameraTableTexture) ui::Image(mDepthToCameraTableTexture, mDepthToCameraTableTexture->getSize());
}

void Live4D::draw()
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

    if (mDepthTexture && mColorTexture)
    {
        gl::ScopedGlslProg glsl(mPointCloudShader);
        gl::ScopedTextureBind t0(mDepthTexture, 0);
        gl::ScopedTextureBind t1(mColorTexture, 1);
        gl::ScopedTextureBind t2(mDepthToCameraTableTexture, 2);
        //gl::ScopedTextureBind t3(mDepthToColorTableTexture, 3);
        gl::ScopedModelMatrix model;
        gl::scale({ SCALE, SCALE, SCALE });

        gl::draw(mVboMesh);

        pointCloud.draw();
    }
}

void prepareSettings(App::Settings* settings)
{
    settings->setWindowSize(1280, 720);
}

CINDER_APP(Live4D, RendererGl, prepareSettings)
