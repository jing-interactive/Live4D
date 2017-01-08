#include "cinder/app/RendererGl.h"
#include "cinder/app/App.h"
#include "cinder/gl/gl.h"
#include "cinder/params/Params.h"
#include "cinder/Log.h"
#include "cinder/CameraUi.h"

#include "Cinder-DepthSensor/include/DepthSensor.h"
#include "Cinder-VNM/include/MiniConfigImgui.h"
#include "Cinder-VNM/include/TextureHelper.h"
#include "Cinder-VNM/include/AssetManager.h"

#include <vector>

using namespace std;
using namespace ci;
using namespace ci::app;

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

    void    setup() override;
    void    update()override;
    void    draw() override;

    // CAMERA
    CameraPersp         mCam;
    CameraUi            mCamUi;

    struct DeviceInfo
    {
        string name;
        ds::DeviceRef device;
        bool isVisible = true;
        bool showImage = false;
        mat4 transform;
        gl::TextureRef  depthTexture;
        gl::TextureRef  colorTexture;
        gl::TextureRef  depthToCameraTableTexture;
        gl::TextureRef  depthToColorTableTexture;
    };
    vector<DeviceInfo> mDeviceInfos;
    int selectedIndex = 0;

    gl::VboMeshRef mVboMesh;

    static gl::VboMeshRef createVboMesh(vec2 depthSize, bool isPointCloud);

    gl::GlslProgRef	mPointCloudShader;

    gl::VertBatchRef grid;

    bool isPointCloud = true;
};


void Live4D::setup()
{
    readConfig();
    log::makeLogger<log::LoggerFile>();

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

    auto deviceCount = ds::Device::getDeviceCount(type);
    if (deviceCount == 0)
    {
        CI_LOG_E("There is no connected sensors for " << ds::strFromType(type));
        //quit();
        //return;
    }

    mDeviceInfos.resize(deviceCount);
    int idx = 0;
    for (auto& info : mDeviceInfos)
    {
        option.deviceId = idx++;
        info.device = ds::Device::create(type, option);
        if (!info.device->isValid())
        {
            //quit();
            //return;
        }

        info.name = "#" + toString(option.deviceId + 1);
        info.device->signalDepthToCameraTableDirty.connect([&]{
            auto format = gl::Texture::Format()
                .immutableStorage()
                .loadTopDown();
            updateTexture(info.depthToCameraTableTexture, info.device->depthToCameraTable, format);
        });

        info.device->signalDepthToColorTableDirty.connect([&]{
            auto format = gl::Texture::Format()
                .dataType(GL_FLOAT)
                .immutableStorage()
                .loadTopDown();
            updateTexture(info.depthToColorTableTexture, info.device->depthToColorTable, format);
        });

        info.device->signalDepthDirty.connect([&]{
            static auto format = gl::Texture::Format()
                .dataType(GL_UNSIGNED_SHORT)
                .internalFormat(GL_R16UI)
                .immutableStorage()
                .loadTopDown();
            updateTexture(info.depthTexture, info.device->depthChannel, format);

            if (mVboMesh == nullptr)
            {
                mVboMesh = createVboMesh(info.device->getDepthSize(), isPointCloud);
            }
        });

        info.device->signalColorDirty.connect([&]{
            auto format = gl::Texture::Format()
                .immutableStorage()
                .loadTopDown();
            updateTexture(info.colorTexture, info.device->colorSurface, format);
        });
    }

#if 0
    depthTexture = gl::Texture::create(info.device->getDepthSize().x, info.device->getDepthSize().y,
        gl::Texture::Format().internalFormat(GL_R16UI).dataType(GL_UNSIGNED_SHORT).minFilter(GL_NEAREST).magFilter(GL_NEAREST));
    colorTexture = gl::Texture::create(info.device->getDepthSize().x, info.device->getDepthSize().y,
        gl::Texture::Format().internalFormat(GL_RGB8).dataType(GL_UNSIGNED_BYTE).wrap(GL_CLAMP_TO_EDGE));
#endif

    // SETUP VBO AND SHADER
    grid = createGrid();

    mPointCloudShader = am::glslProg("pointcloud.vert", "pointcloud.frag");
    mPointCloudShader->uniform("uTextureDepth", 0);
    mPointCloudShader->uniform("uTextureColor", 1);
    mPointCloudShader->uniform("uTextureDepthToCameraTable", 2);
    mPointCloudShader->uniform("uTextureDepthToColorTable", 3);

    // SETUP GL
    gl::enableDepthWrite();
    gl::enableDepthRead();

    getWindow()->getSignalKeyUp().connect([&](KeyEvent& event) {
        auto key = event.getCode();
        if (key == KeyEvent::KEY_ESCAPE) quit();
        if (key >= KeyEvent::KEY_1 && key <= KeyEvent::KEY_9)
        {
            selectedIndex = key - KeyEvent::KEY_1;
        }
    });
}

gl::VboMeshRef Live4D::createVboMesh(vec2 depthSize, bool isPointCloud)
{
    vector<vec2> vertices;
    for (int32_t x = 0; x < depthSize.x; ++x)
    {
        for (int32_t y = 0; y < depthSize.y; ++y)
        {
            vertices.push_back(vec2(x, y) / depthSize);
        }
    }

    gl::VboRef vbo = gl::Vbo::create(GL_ARRAY_BUFFER, vertices, GL_STATIC_DRAW);

    geom::BufferLayout layout;
    layout.append(geom::Attrib::POSITION, 2, sizeof(vec2), 0);
    auto vertexArrayBuffers = { make_pair(layout, vbo) };

    int spc = 1;
    int I = depthSize.x - spc;
    int J = depthSize.y - spc;
    vector<uint32_t> indices;
    for (int i = 0; i < I - spc; i += spc)
        for (int j = 0; j < J - spc; j += spc)
        {
            int idx_00 = (i + 0) + (j + 0) * depthSize.x;
            int idx_10 = (i + spc) + (j + 0) * depthSize.x;
            int idx_01 = (i + 0) + (j + spc) * depthSize.x;
            int idx_11 = (i + spc) + (j + spc) * depthSize.x;
            indices.emplace_back(idx_00);
            indices.emplace_back(idx_10);
            indices.emplace_back(idx_11);

            indices.emplace_back(idx_00);
            indices.emplace_back(idx_11);
            indices.emplace_back(idx_01);
        }

    gl::VboRef indexVbo = gl::Vbo::create(GL_ELEMENT_ARRAY_BUFFER, indices, GL_STATIC_DRAW);

    return gl::VboMesh::create(vertices.size(), isPointCloud ? GL_POINTS : GL_TRIANGLES, vertexArrayBuffers,
        indices.size(), GL_UNSIGNED_INT, indexVbo);
}

void Live4D::update()
{
    _FPS = getAverageFps();

    //if (ui::Checkbox("point cloud", &isPointCloud))
    //{
    //    mVboMesh = createVboMesh(isPointCloud);
    //}
    {
        ui::ScopedWindow window("Devices");
        // selectable list
        ui::ListBoxHeader("");
        int idx = 0;
        for (auto& info : mDeviceInfos)
        {
            if (ui::Selectable(info.name.c_str(), idx == selectedIndex))
            {
                selectedIndex = idx;
            }
            idx++;
        }
        ui::ListBoxFooter();

        if (selectedIndex != -1 && selectedIndex >= mDeviceInfos.size() - 1)
        {
            auto& info = mDeviceInfos[selectedIndex];
            ui::Checkbox("Visible", &info.isVisible);
            if (ui::Button("Reset"))
            {
                info.transform = mat4();
            }

            if (ui::EditGizmo(mCam.getViewMatrix(), mCam.getProjectionMatrix(), &info.transform))
            {
                mCamUi.disable();
            }
            else
            {
                mCamUi.enable();
            }

            ui::Checkbox("View Images", &info.showImage);
            if (info.showImage)
            {
                if (info.depthTexture) ui::Image(info.depthTexture, info.depthTexture->getSize());
                if (info.colorTexture) ui::Image(info.colorTexture, info.colorTexture->getSize());
                if (info.depthToColorTableTexture) ui::Image(info.depthToColorTableTexture, info.depthToColorTableTexture->getSize());
                if (info.depthToCameraTableTexture) ui::Image(info.depthToCameraTableTexture, info.depthToCameraTableTexture->getSize());
            }
        }
    }

    // TODO: use UBO
    mPointCloudShader->uniform("uMinDistance", MIN_DISTANCE_MM);
    mPointCloudShader->uniform("uMaxDistance", MAX_DISTANCE_MM);
    mPointCloudShader->uniform("uFlipX", FLIP_X);
    mPointCloudShader->uniform("uFlipY", FLIP_Y);
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

    for (auto& info : mDeviceInfos)
    {
        if (info.isVisible &&
            info.depthTexture &&
            info.colorTexture &&
            info.depthToCameraTableTexture &&
            info.depthToColorTableTexture)
        {
            gl::ScopedGlslProg glsl(mPointCloudShader);
            gl::ScopedTextureBind t0(info.depthTexture, 0);
            gl::ScopedTextureBind t1(info.colorTexture, 1);
            gl::ScopedTextureBind t2(info.depthToCameraTableTexture, 2);
            gl::ScopedTextureBind t3(info.depthToColorTableTexture, 3);
            gl::ScopedModelMatrix model;
            gl::multModelMatrix(info.transform);
            gl::scale({ SCALE, SCALE, SCALE });

            gl::draw(mVboMesh);
        }
    }
}

void prepareSettings(App::Settings* settings)
{
    settings->setWindowSize(1280, 720);
}

CINDER_APP(Live4D, RendererGl, prepareSettings)
