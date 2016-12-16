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

    gl::VboMeshRef createVboMesh(bool isPointCloud);

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
        auto format = gl::Texture::Format()
            .immutableStorage()
            .loadTopDown();
        updateTexture(mDepthToCameraTableTexture, mDevice->depthToCameraTable, format);
    });

    mDevice->signalDepthToColorTableDirty.connect([&]{
        auto format = gl::Texture::Format()
            .dataType(GL_FLOAT)
            .immutableStorage()
            .loadTopDown();
        updateTexture(mDepthToColorTableTexture, mDevice->depthToColorTable, format);
    });

    mDevice->signalDepthDirty.connect([&]{
        static auto format = gl::Texture::Format()
            .dataType(GL_UNSIGNED_SHORT)
            .internalFormat(GL_R16UI)
            .immutableStorage()
            .loadTopDown();
        updateTexture(mDepthTexture, mDevice->depthChannel, format);
    });

    mDevice->signalColorDirty.connect([&]{
        auto format = gl::Texture::Format()
            .immutableStorage()
            .loadTopDown();
        updateTexture(mColorTexture, mDevice->colorSurface, format);
    });

#if 0
    mDepthTexture = gl::Texture::create(mDevice->getDepthSize().x, mDevice->getDepthSize().y,
        gl::Texture::Format().internalFormat(GL_R16UI).dataType(GL_UNSIGNED_SHORT).minFilter(GL_NEAREST).magFilter(GL_NEAREST));
    mColorTexture = gl::Texture::create(mDevice->getDepthSize().x, mDevice->getDepthSize().y,
        gl::Texture::Format().internalFormat(GL_RGB8).dataType(GL_UNSIGNED_BYTE).wrap(GL_CLAMP_TO_EDGE));
#endif

    // SETUP VBO AND SHADER
    grid = createGrid();
    mVboMesh = createVboMesh(false);

    mPointCloudShader = am::glslProg("pointcloud.vert", "pointcloud.frag");
    mPointCloudShader->uniform("uTextureDepth", 0);
    mPointCloudShader->uniform("uTextureColor", 1);
    mPointCloudShader->uniform("uTextureDepthToCameraTable", 2);
    mPointCloudShader->uniform("uTextureDepthToColorTable", 3);

    // SETUP GL
    gl::enableDepthWrite();
    gl::enableDepthRead();
}

gl::VboMeshRef Live4D::createVboMesh(bool isPointCloud)
{
    vec2 sz = mDevice->getDepthSize();
    vector<vec2> vertices;
    for (int32_t x = 0; x < sz.x; ++x)
    {
        for (int32_t y = 0; y < sz.y; ++y)
        {
            vertices.push_back(vec2(x, y) / sz);
        }
    }

    gl::VboRef vbo = gl::Vbo::create(GL_ARRAY_BUFFER, vertices, GL_STATIC_DRAW);

    geom::BufferLayout layout;
    layout.append(geom::Attrib::POSITION, 2, sizeof(vec2), 0);
    auto vertexArrayBuffers = { make_pair(layout, vbo) };

    if (isPointCloud)
    {
        return gl::VboMesh::create(vertices.size(), GL_POINTS, vertexArrayBuffers);
    }
    else
    {
        int I = sz.x - 1;
        int J = sz.y - 1;
        vector<uint16_t> indices;
        for (int i = 0; i < I; i++)
            for (int j = 0; j < J; j++)
            {
                int idx_00 = (i + 0) + (j + 0) * sz.x;
                int idx_10 = (i + 1) + (j + 0) * sz.x;
                int idx_01 = (i + 0) + (j + 1) * sz.x;
                int idx_11 = (i + 1) + (j + 1) * sz.x;
                indices.emplace_back(idx_00);
                indices.emplace_back(idx_10);
                indices.emplace_back(idx_11);

                indices.emplace_back(idx_00);
                indices.emplace_back(idx_11);
                indices.emplace_back(idx_01);
            }

        gl::VboRef indexVbo = gl::Vbo::create(GL_ELEMENT_ARRAY_BUFFER, indices, GL_STATIC_DRAW);

        return gl::VboMesh::create(vertices.size(), GL_TRIANGLES, vertexArrayBuffers, 
                                    indices.size(), GL_UNSIGNED_SHORT, indexVbo);
    }
}

void Live4D::update()
{
    _FPS = getAverageFps();

    if (mDepthToColorTableTexture) ui::Image(mDepthToColorTableTexture, mDepthToColorTableTexture->getSize());

    mPointCloudShader->uniform("uMinDistance", MIN_DISTANCE_MM);
    mPointCloudShader->uniform("uMaxDistance", MAX_DISTANCE_MM);
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
        gl::ScopedTextureBind t3(mDepthToColorTableTexture, 3);
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
