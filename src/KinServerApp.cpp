#include "cinder/app/RendererGl.h"
#include "cinder/app/App.h"
#include "cinder/gl/gl.h"
#include "cinder/params/Params.h"
#include "cinder/Log.h"
#include "cinder/PolyLine.h"

#include "Cinder/osc/Osc.h"
#include "CinderOpenCV.h"

#include "DepthSensor.h"
#include "Cinder-VNM/include/MiniConfig.h"
#include "Cinder-VNM/include/TextureHelper.h"

#include <vector>

using namespace std;
using namespace ci;
using namespace ci::app;
using namespace cv;

// Store the correspondences and centroids
// Vec3f is an OpenCV data structure
typedef pair< Vec3f, Vec3f > match;

// Used for interactive transformations 
// see transformation() and KeyPressed()
enum Transform_Mode{ rotation, translation, full_transform, none };
Transform_Mode transform_mode = none;

// Variables for Calculations and Loops
const int NUM_CAMS = 2;
int mx = -1, my = -1;        // Prevous mouse coordinates
int rotangles[2] = { 0 }; // Panning angles
float zoom = 1;         // zoom factor
match centroids;
Mat P, Q, trans, rot; // P and Q store the points for procrustes analysis
vector< Vec3f > P_pts, Q_pts; // This is how they're collected
// NOTE: Simplify correspondence containers above

// This is used for collecting Correspondences. It keeps track of where the points were just collected from.
enum PointsType{ P_POINTS, Q_POINTS, NONE };
PointsType pointsType = NONE;

// Window Size and Position
const int window_width = 640, window_height = 480;
int window_xpos = 1000, window_ypos = 100;

// Handy functions to call in the render function
void transformation(int cam); // This applies the procrustes transformations
void loadVertexMatrix(); // This applies the projection transformation
void noKinectQuit();
void draw_axes();

// Computer Vision functions
void displayCVcams(); // Calls joinFrames and displays the RGB images
Mat joinFrames(const Mat& img1, const Mat& img2); // Puts images side by side
Mat convert_vector2Mat(const vector< Vec3f > vec);
Vec3f transformPoint(const Vec3f& pt); // Transforms pt from image space
// to Kinect space
match calculateCentroids(const Mat& verts1, const Mat& verts2);
void procrustes(const vector< Vec3f >&, const vector< Vec3f >&, Mat&, Mat&);

// Collects the information from a (cameraIndx) Kinect
void loadBuffers(int cameraIndx,
    unsigned int indices[window_height][window_width],
    short xyz[window_height][window_width][3],
    unsigned char rgb[window_height][window_width][3]);

// getDepth (poorly) attempts to ameliorate the bad depth measurements
// by checking the neighbors in a 3x3 grid around a pixel which got a 
// depth measurement > 2047
float getDepth(int cam, int x, int y);
void printMat(const Mat& A);

// Store the matrices from all cameras here
vector<Mat> rgbCV;
vector<Mat> depthCV;

void loadBuffers(int cameraIndx,
    unsigned int indices[window_height][window_width],
    short xyz[window_height][window_width][3],
    unsigned char rgb[window_height][window_width][3]) {

    // TODO:
    //rgbCV.push_back(freenect_sync_get_rgb_cv(cameraIndx));
    //depthCV.push_back(freenect_sync_get_depth_cv(cameraIndx));

    if (rgbCV[cameraIndx].empty() || depthCV[cameraIndx].empty())
        noKinectQuit();

    for (int row = 0; row < window_height; row++) {
        for (int col = 0; col < window_width; col++) {
            xyz[row][col][0] = col;
            xyz[row][col][1] = row;
            xyz[row][col][2] = getDepth(cameraIndx, row, col);
            indices[row][col] = (window_height + row)*window_width + col;
            vec3 color = rgbCV[cameraIndx].at<vec3>(row, col);
            rgb[row][col][0] = color[0];
            rgb[row][col][1] = color[1];
            rgb[row][col][2] = color[2];
        }
    }
}

// Do the projection from u,v,depth to X,Y,Z directly in an opengl matrix
// These numbers come from a combination of the ros kinect_node wiki, and
// nicolas burrus' posts.
void loadVertexMatrix() {

    float fx = 594.21f;
    float fy = 591.04f;
    float a = -0.0030711f;
    float b = 3.3309495f;
    float cx = 339.5f;
    float cy = 242.7f;
    GLfloat mat[16] = {
        1 / fx, 0, 0, 0,
        0, -1 / fy, 0, 0,
        0, 0, 0, a,
        -cx / fx, cy / fy, -1, b
    };

    // TODO: proj?
    gl::multModelMatrix(glm::make_mat4(mat));
}

void printMat(const Mat& A) {

    printf("| ");
    for (int i = 0; i < A.rows; i++) {
        if (i > 0 && i < A.rows) {
            printf("|\n");
            printf("| ");
        }
        for (int j = 0; j < A.cols; j++) {
            printf("%f ", A.at<float>(i, j));
        }
    }
    printf("|\n");
}

// Definitely need to come up with a better
// way to find good depth measurements...
float getDepth(int cam, int x, int y) {

    float d = (float)(depthCV[cam].at<short>(x, y));

    if (d >= 2047)
        d = (float)(depthCV[cam].at<short>(x, y + 1));

    if (d >= 2047)
        d = (float)(depthCV[cam].at<short>(x, y - 1));

    if (d >= 2047)
        d = (float)(depthCV[cam].at<short>(x + 1, y));

    if (d >= 2047)
        d = (float)(depthCV[cam].at<short>(x - 1, y));

    if (d >= 2047)
        d = (float)(depthCV[cam].at<short>(x - 1, y - 1));

    if (d >= 2047)
        d = (float)(depthCV[cam].at<short>(x - 1, y + 1));

    if (d >= 2047)
        d = (float)(depthCV[cam].at<short>(x + 1, y - 1));

    if (d >= 2047)
        d = (float)(depthCV[cam].at<short>(x + 1, y + 1));

    return d;
}

void draw_axes() {

    //X Axis

    gl::color(Color8u(1, 0, 0));    //red
    vec3 r1(0, 0, 0);
    vec3 r2(1, 0, 0);
    gl::drawLine(r1, r2);

    //Y Axis

    gl::color(Color8u(0, 1, 0));    //green
    vec3 g1(0, 0, 0);
    vec3 g2(0, 1, 0);
    gl::drawLine(g1, g2);

    //Z Axis

    gl::color(Color8u(0, 0, 1));    //blue
    vec3 b1(0, 0, 0);
    vec3 b2(0, 0, 1);
    gl::drawLine(b1, b2);

}

// Puts two RGB images side by side in one Mat
Mat joinFrames(const Mat& img1, const Mat& img2) {

    Mat rslt = Mat::zeros(img1.rows, img1.cols * 2, img1.type());
    for (int i = 0; i < img1.rows; i++)
        for (int j = 0; j < img1.cols; j++) {
            vec3 p = img1.at<vec3>(i, j);
            vec3 q = img2.at<vec3>(i, j);
            rslt.at<vec3>(i, j) = p;
            rslt.at<vec3>(i, j + window_width) = q;
        }

    return rslt;
}

// Display the joined frames in one window
void displayCVcams() {

    Mat tmp = Mat::zeros(window_width, window_height, CV_8U);
    for (int cam = 0; cam < NUM_CAMS; cam++) {
        cvtColor(rgbCV[cam], tmp, CV_RGB2BGR);
        rgbCV[cam] = tmp.clone();
    }

    Mat rgb = joinFrames(rgbCV[0], rgbCV[1]);
    imshow("Camera 0 | Camera 1", rgb);
}

match calculateCentroids(const Mat& verts1, const Mat& verts2) {

    Vec3f centroid1(0, 0, 0), centroid2(0, 0, 0);

    for (int i = 0; i < verts1.cols; i++) {

        // Sum of components of first point cloud
        centroid1[0] += verts1.at<float>(0, i);
        centroid1[1] += verts1.at<float>(1, i);
        centroid1[2] += verts1.at<float>(2, i);

        // Sum of components of second point cloud
        centroid2[0] += verts2.at<float>(0, i);
        centroid2[1] += verts2.at<float>(1, i);
        centroid2[2] += verts2.at<float>(2, i);
    }

    // First centroid
    centroid1[0] /= verts1.cols;
    centroid1[1] /= verts1.cols;
    centroid1[2] /= verts1.cols;
    printf(" First centroid = (%f,%f,%f)\n", centroid1[0], centroid1[1], centroid1[2]);

    // Second centroid
    centroid2[0] /= verts1.cols;
    centroid2[1] /= verts1.cols;
    centroid2[2] /= verts1.cols;
    printf(" Second centroid = (%f,%f,%f)\n", centroid2[0], centroid2[1], centroid2[2]);

    match centroids(centroid1, centroid2);

    return centroids;
}

void procrustes(const vector< Vec3f >& P_pts,
    const vector< Vec3f >& Q_pts,
    Mat& trans, Mat& rot) {
    printf("\n\n------------ENTERING procrustes()------------\n");

    if (P_pts.size() == 0 || Q_pts.size() == 0) {
        printf("There are no correspondences for Registration...\n");
        return;
    }

    P = convert_vector2Mat(P_pts);
    printf("P:\n");
    printMat(P);
    Q = convert_vector2Mat(Q_pts);
    printf("Q:\n");
    printMat(Q);
    centroids = calculateCentroids(P, Q);

    // To calculate the rotation we assume the centroids of their
    // correspondences are aligned. So move the centroids of each
    // collection of correspondences to the origin.
    for (int pt = 0; pt < P.cols; pt++) {
        P.at<float>(0, pt) -= centroids.first[0];
        P.at<float>(1, pt) -= centroids.first[1];
        P.at<float>(2, pt) -= centroids.first[2];
    }
    for (int pt = 0; pt < Q.cols; pt++) {
        Q.at<float>(0, pt) -= centroids.second[0];
        Q.at<float>(1, pt) -= centroids.second[1];
        Q.at<float>(2, pt) -= centroids.second[2];
    }
    // Procrustes in 3 lines baby
    Mat PQt = P*Q.t();
    SVD svd(PQt);
    Mat rotMat = svd.u*svd.vt;

    // change the rotMat to column major (OpenGL stores matrices in column
    // major order, while OpenCV row major, and add the extra column and row)
    float rotCpy[16] = { rotMat.at<float>(0, 0), rotMat.at<float>(0, 1), rotMat.at<float>(0, 2), 0,
        rotMat.at<float>(1, 0), rotMat.at<float>(1, 1), rotMat.at<float>(1, 2), 0,
        rotMat.at<float>(2, 0), rotMat.at<float>(2, 1), rotMat.at<float>(2, 2), 0,
        0, 0, 0, 1 };
    rot = Mat(4, 4, CV_32F, rotCpy).clone();
    printf(" rot:\n");
    printMat(rot);

    printf("\n\n------------LEAVING procrustes()------------\n");
}

// Apply certain transformations to each camera
void transformation(int cam) {

    if (transform_mode == none)
        return;
    else if (transform_mode == rotation && cam == 0) {
        glm::mat4 mat = glm::make_mat4(rot.data);
        gl::multModelMatrix(mat);
    }
    else if (transform_mode == translation && cam == 0) {
        gl::translate(-centroids.first[0], -centroids.first[1], -centroids.first[2]);
    }
    else if (transform_mode == translation && cam == 1) {
        gl::translate(-centroids.second[0], -centroids.second[1], -centroids.second[2]);
    }
    else if (transform_mode == full_transform && cam == 0) {
        glm::mat4 mat = glm::make_mat4(rot.data);
        gl::multModelMatrix(mat);
        gl::translate(-centroids.first[0], -centroids.first[1], -centroids.first[2]);
    }
    else if (transform_mode == full_transform && cam == 1) {
        gl::translate(-centroids.second[0], -centroids.second[1], -centroids.second[2]);
    }

}

// Callback function
void cbMouseEvent(int event, int col, int row, int flags, void* param) {

    switch (event) {
    case CV_EVENT_LBUTTONDOWN:
        if (col <= 640) { // This is how it know which points you're
            // collecting (images side by side)
            printf(" Click in P ( %d, %d, %f )\n", col, row, getDepth(0, row, col));
            if (pointsType == NONE) {
                // If first click, grab the point and
                P_pts.push_back(Vec3f(col, row, getDepth(0, row, col)));
                //  state that the last point grabbed was a point in P
                pointsType = P_POINTS;
                break;
            }
            if (pointsType == P_POINTS) {
                printf("Can't match to the same image! Erasing previous point...\n");
                P_pts.pop_back();
                pointsType = NONE;
                break;
            }
            // If not NONE and not P_POINTS must be Q_POINTS
            P_pts.push_back(Vec3f(col, row, getDepth(0, row, col)));
            float Pz = P_pts.back()[2];
            float Qz = Q_pts.back()[2];
            // Check if the depth measurements are bad, if so remove them
            if (Pz >= 2047 || Pz <= 0 || Qz >= 2047 || Qz <= 0) {
                printf("\nDepths are bad! Erasing correspondence...\n");
                P_pts.pop_back();
                Q_pts.pop_back();
                pointsType = NONE;
                break;
            }
            else {
                // If the depths are good, transform the points and store them!
                Vec3f transP_pt = transformPoint(P_pts.back());
                P_pts.pop_back();
                P_pts.push_back(transP_pt);
                Vec3f transQ_pt = transformPoint(Q_pts.back());
                Q_pts.pop_back();
                Q_pts.push_back(transQ_pt);
                printf(" P_pts: \n");
                for (int i = 0; i < (int)P_pts.size(); i++)
                    printf("%d - (%f,%f,%f)\n", i, P_pts[i][0], P_pts[i][1], P_pts[i][2]);
                printf(" Q_pts: \n");
                for (int i = 0; i < (int)Q_pts.size(); i++)
                    printf("%d - (%f,%f,%f)\n", i, Q_pts[i][0], Q_pts[i][1], Q_pts[i][2]);
            }
            pointsType = NONE;
        }
        else {
            // SAME AS ABOVE JUST FOR OTHER SIDE OF WINDOW ( Q POINTS )
            printf(" Click in Q ( %d, %d, %f )\n", col - 640, row, getDepth(1, row, col - 640));
            if (pointsType == NONE) {
                Q_pts.push_back(Vec3f(col - 640, row, getDepth(1, row, col - 640)));
                pointsType = Q_POINTS;
                break;
            }
            if (pointsType == Q_POINTS) {
                printf("Can't match to the same image! Erasing previous point...\n");
                Q_pts.pop_back();
                pointsType = NONE;
                break;
            }
            Q_pts.push_back(Vec3f(col - 640, row, getDepth(1, row, col - 640)));
            float Pz = P_pts.back()[2];
            float Qz = Q_pts.back()[2];
            if (Pz >= 2047 || Pz <= 0 || Qz >= 2047 || Qz <= 0) {
                P_pts.pop_back();
                Q_pts.pop_back();
                printf("\nDepths are bad! Erasing correspondence...\n");
                pointsType = NONE;
                break;
            }
            else {
                Vec3f transP_pt = transformPoint(P_pts.back());
                P_pts.pop_back();
                P_pts.push_back(transP_pt);
                Vec3f transQ_pt = transformPoint(Q_pts.back());
                Q_pts.pop_back();
                Q_pts.push_back(transQ_pt);
                printf("\n P_pts: \n");
                for (int i = 0; i < (int)P_pts.size(); i++)
                    printf("%d - (%f,%f,%f)\n", i, P_pts[i][0], P_pts[i][1], P_pts[i][2]);
                printf(" Q_pts: \n");
                for (int i = 0; i < (int)Q_pts.size(); i++)
                    printf("%d - (%f,%f,%f)\n", i, Q_pts[i][0], Q_pts[i][1], Q_pts[i][2]);
            }

            pointsType = NONE;
        }
    }
}

Mat convert_vector2Mat(const vector< Vec3f > vec) {

    printf("\n\n---------- ENTERING convert_vector2Mat() ----------\n ");
    printf(" Incoming vector \n");
    for (int i = 0; i < (int)vec.size(); i++)
        printf(" vec[%d] - ( %f, %f, %f )\n", i, vec[i][0], vec[i][1], vec[i][2]);

    Mat m(3, vec.size(), CV_32F);

    for (int i = 0; i < (int)vec.size(); i++) {
        m.at<float>(0, i) = vec[i][0];
        m.at<float>(1, i) = vec[i][1];
        m.at<float>(2, i) = vec[i][2];
    }

    printf(" Mat version of vec:\n");
    printMat(m);

    printf("--------- LEAVING convert_vector2Mat() ---------\n\n");
    return m;

}

Vec3f transformPoint(const Vec3f& pt) {

    printf("\n\n----- ENTERING transformPoint() -------\n");
    printf(" Incoming Point = ( %f, %f, %f )\n", pt[0], pt[1], pt[2]);
    Mat point(pt);
    point.resize(4);
    point.at<float>(0, 3) = 1;
    printf(" Mat version of point\n");
    printMat(point);

    float fx = 594.21f;
    float fy = 591.04f;
    float a = -0.0030711f;
    float b = 3.3309495f;
    float cx = 339.5f;
    float cy = 242.7f;
    float data[16] = {
        1 / fx, 0, 0, -cx / fx,
        0, -1 / fy, 0, cy / fy,
        0, 0, 0, -1,
        0, 0, a, b
    };

    Mat transform(4, 4, CV_32F, data);
    printf("Transformation Matrix:\n");
    printMat(transform);
    Mat tmp = transform*point;
    printf("Transformation Matrx * Point:\n");
    printMat(tmp);
    float w = tmp.at<float>(0, 3);
    float x = tmp.at<float>(0, 0) / w;
    float y = tmp.at<float>(0, 1) / w;
    float z = tmp.at<float>(0, 2) / w;
    Vec3f transformedPoint(x, y, z);
    printf(" Perspective Division = ( %f, %f, %f )", transformedPoint[0],
        transformedPoint[1],
        transformedPoint[2]);

    printf("\n----- LEAVING transformPoint() -------\n\n");
    return transformedPoint;
}

float flann_knn(Mat& m_destinations, Mat& m_object, vector<int>& ptpairs, vector<float>& dists = vector<float>()) {
    // find nearest neighbors using FLANN
    Mat m_indices(m_object.rows, 1, CV_32S);
    Mat m_dists(m_object.rows, 1, CV_32F);

    Mat dest_32f; m_destinations.convertTo(dest_32f, CV_32FC2);
    Mat obj_32f; m_object.convertTo(obj_32f, CV_32FC2);

    assert(dest_32f.type() == CV_32F);

    cv::flann::Index flann_index(dest_32f, cv::flann::KDTreeIndexParams(2));  // using 2 randomized kdtrees
    flann_index.knnSearch(obj_32f, m_indices, m_dists, 1, cv::flann::SearchParams(64));

    int* indices_ptr = m_indices.ptr<int>(0);
    //float* dists_ptr = m_dists.ptr<float>(0);
    for (int i = 0; i < m_indices.rows; ++i) {
        ptpairs.push_back(indices_ptr[i]);
    }

    dists.resize(m_dists.rows);
    m_dists.copyTo(Mat(dists));

    return cv::sum(m_dists)[0];
}

void findBestReansformSVD(Mat& _m, Mat& _d) {
    Mat m; _m.convertTo(m, CV_32F);
    Mat d; _d.convertTo(d, CV_32F);

    Scalar d_bar = mean(d);
    Scalar m_bar = mean(m);
    Mat mc = m - m_bar;
    Mat dc = d - d_bar;

    mc = mc.reshape(1); dc = dc.reshape(1);

    Mat H(2, 2, CV_32FC1);
    for (int i = 0; i < mc.rows; i++) {
        Mat mci = mc(Range(i, i + 1), Range(0, 2));
        Mat dci = dc(Range(i, i + 1), Range(0, 2));
        H = H + mci.t() * dci;
    }

    cv::SVD svd(H);

    Mat R = svd.vt.t() * svd.u.t();
    double det_R = cv::determinant(R);
    if (abs(det_R + 1.0) < 0.0001) {
        float _tmp[4] = { 1, 0, 0, cv::determinant(svd.vt*svd.u) };
        R = svd.u * Mat(2, 2, CV_32FC1, _tmp) * svd.vt;
    }
    float* _R = R.ptr<float>(0);
    Scalar T(d_bar[0] - (m_bar[0] * _R[0] + m_bar[1] * _R[1]), d_bar[1] - (m_bar[0] * _R[2] + m_bar[1] * _R[3]));

    m = m.reshape(1);
    m = m * R;
    m = m.reshape(2);
    m = m + T;// + m_bar;
    m.convertTo(_m, CV_32S);
}

#if 0
void icp() {
    vector<int> pair;
    vector<float> dists;

    while (true) {
        pair.clear(); dists.clear();
        double dist = flann_knn(destination, X, pair, dists);

        if (lastDist <= dist) {
            X = lastGood;
            break;  //converged?
        }
        lastDist = dist;
        X.copyTo(lastGood);

        cout << "distance: " << dist << endl;

        Mat X_bar(X.size(), X.type());
        for (int i = 0; i < X.rows; i++) {
            Point p = destination.at<Point>(pair[i], 0);
            X_bar.at<Point>(i, 0) = p;
        }

        ShowQuery(destination, X, X_bar);

        X = X.reshape(2);
        X_bar = X_bar.reshape(2);
        findBestReansformSVD(X, X_bar);
        X = X.reshape(1); // back to 1-channel
    }
}
#endif

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

        for (int cam = 0; cam < NUM_CAMS; cam++) {
            rgbCV.push_back(freenect_sync_get_rgb_cv(cam));
            depthCV.push_back(freenect_sync_get_depth_cv(cam));
        }
    }

    void resize() override
    {
    }

    void draw() override
    {
        gl::clear(ColorA::gray(0.5f));

        short xyz[window_height][window_width][3];
        unsigned char rgb[window_height][window_width][3];
        unsigned int indices[window_height][window_width];

        // Flush the OpenCV Mat's from last frame
        rgbCV.clear();
        depthCV.clear();

#if 0
        glEnable(GL_DEPTH_TEST);
        glPushMatrix();
        glScalef(zoom, zoom, 1);
        gluLookAt(0, 0, 3.5, 0, 0, 0, 0, 1.0, 0);
        glRotatef(rotangles[0], 1, 0, 0);
        glRotatef(rotangles[1], 0, 1, 0);
        draw_axes();

        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_COLOR_ARRAY);
        glPointSize(2);
        //--------Camera 0 (P)-----------
        loadBuffers(0, indices, xyz, rgb);
        glVertexPointer(3, GL_SHORT, 0, xyz);
        glColorPointer(3, GL_UNSIGNED_BYTE, 0, rgb);
        glPushMatrix();
        // transform centroid of P to origin and rotate
        transformation(0);
        // projection matrix (camera specific - Can be improved)
        loadVertexMatrix();
        glDrawArrays(GL_POINTS, 0, window_width*window_height);
        glPopMatrix();
        //--------Camera 1 (Q)-----------
        loadBuffers(1, indices, xyz, rgb);
        glVertexPointer(3, GL_SHORT, 0, xyz);
        glColorPointer(3, GL_UNSIGNED_BYTE, 0, rgb);
        glPushMatrix();
        // translate centroid of Q to origin
        transformation(1);
        loadVertexMatrix();
        glDrawArrays(GL_POINTS, 0, window_width*window_height);
        glPopMatrix();
        glPopMatrix();

        displayCVcams();
        glFlush();
        glutSwapBuffers();
        glDisable(GL_DEPTH_TEST);
#endif
    }

    void keyUp(KeyEvent event) override
    {
        int code = event.getCode();
        switch (code)
        {
        case KeyEvent::KEY_ESCAPE: quit(); break;
        case KeyEvent::KEY_p:
        {
            procrustes(P_pts, Q_pts, trans, rot);
            // The correspondence arrays are flushed after the transformations
            // are calculated. Just in case the registration attempt is poor 
            // and you want to try again, you can just start collecting
            // correspondences again without restarting the program
            P_pts.clear();
            Q_pts.clear();
            break;
        }
        case KeyEvent::KEY_r: 	transform_mode = rotation; 		break;
        case KeyEvent::KEY_t: 	transform_mode = translation;	break;
        case KeyEvent::KEY_a:	transform_mode = full_transform; break;
        case KeyEvent::KEY_n:	transform_mode = none;			break;
        case KeyEvent::KEY_z:	zoom *= 1.1f;					break;
        case KeyEvent::KEY_x:	zoom /= 1.1f;					break;
        }
    }

    void mouseUp(MouseEvent event)
    {
        if (event.isLeft()) {
            mx = -1;
            my = -1;
        }
    }

    void mouseDown(MouseEvent event)
    {
        if (event.isLeft()) {
            mx = event.getX();
            my = event.getY();
        }
    }

    void mouseDrag(MouseEvent event)
    {
        if (mx >= 0 && my >= 0) {
            rotangles[0] += event.getY() - my;
            rotangles[1] += event.getX() - mx;
        }

        mx = event.getX();
        my = event.getY();
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
