#include "cinder/app/RendererGl.h"
#include "cinder/app/App.h"
#include "cinder/gl/gl.h"
#include "cinder/params/Params.h"
#include "cinder/Log.h"
#include "cinder/PolyLine.h"

#include "Cinder/osc/Osc.h"

#include "DepthSensor.h"
#include "Cinder-VNM/include/MiniConfig.h"
#include "Cinder-VNM/include/TextureHelper.h"

using namespace ci;
using namespace ci::app;

class KinServerApp : public App
{
public:
    void setup() override
    {
        readConfig();
        log::makeLogger<log::LoggerFile>();

        auto params = createConfigUI({ 400, 400 });
        params->addParam("FPS", &mFps, true);

        Kinect::DeviceType type = Kinect::DeviceType(SENSOR_TYPE);
        Kinect::Device::Option option;
        option.enableColor = true;
        mDevice = Kinect::Device::create(type, option);
        if (!mDevice->isValid())
        {
            quit();
            return;
        }

        getWindow()->setTitle("Live4D");
        getWindow()->setSize(1024, 768);
    }

    void resize() override
    {
    }

    void draw() override
    {
        gl::clear(ColorA::gray(0.5f));
    }

    void keyUp(KeyEvent event) override
    {
        int code = event.getCode();
        if (code == KeyEvent::KEY_ESCAPE)
        {
            quit();
        }
    }

    void update() override
    {
        mFps = getAverageFps();
    }

private:

    float mFps = 0;
    Kinect::DeviceRef mDevice;
};

CINDER_APP(KinServerApp, RendererGl)
