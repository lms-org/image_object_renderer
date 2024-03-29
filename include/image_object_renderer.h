#ifndef IMAGE_HINT_TRANSFORMER_H
#define IMAGE_HINT_TRANSFORMER_H

#include "lms/imaging/graphics.h"
#include "lms/imaging/image.h"
#include "lms/imaging/image.h"
#include "lms/math/point_cloud.h"
#include "lms/module.h"
#include "street_environment/basic_obstacle.h"
#include "street_environment/bounding_box.h"
#include "street_environment/crossing.h"
#include "street_environment/obstacle.h"
#include "street_environment/road.h"
#include "street_environment/roadmatrix.h"
#include "street_environment/trajectory.h"

class ImageObjectRenderer : public lms::Module {
public:
    bool initialize() override;
    bool deinitialize() override;
    bool cycle() override;
private:
    std::vector<std::string> vertex4f;
    std::vector<std::string> drawObjectStrings;
    std::vector<lms::ReadDataChannel<lms::Any>> drawObjects;

    lms::WriteDataChannel<lms::imaging::Image> image;
    lms::imaging::BGRAImageGraphics *graphics;
    std::vector<lms::ReadDataChannel<street_environment::EnvironmentObjects>> toDrawEnv;
    std::vector<lms::ReadDataChannel<lms::math::polyLine2f>> toDrawPolyLines;
    std::vector<lms::ReadDataChannel<lms::math::vertex2f>> toDrawVertex2f;
    std::vector<lms::ReadDataChannel<std::pair<lms::math::vertex2f,lms::math::vertex2f>>> toDrawVertex4f;
    int xToImage(float x);
    int yToImage(float y);
    void drawObstacle(const street_environment::Obstacle *obstacle);
    void drawPolyLine(const lms::math::polyLine2f *lane);
    void drawVertex2f(const lms::math::vertex2f &v,int length = 5);
    void drawTrajectoryPoint(const street_environment::TrajectoryPoint &v);
    void drawObject(const street_environment::EnvironmentObject *eo,bool customColori);
    void drawRect(const lms::math::Rect &r);
    void drawRect(float x, float y, float width, float height, bool filled);//TODO
    void drawLine(float x1, float y1, float x2, float y2);
    void drawLine(lms::math::vertex2f p1, lms::math::vertex2f p2);
    void drawTrajectory(const street_environment::Trajectory &tra);
    void drawRoadMatrix(const street_environment::RoadMatrix &rm);
    void drawTriangle(lms::math::vertex2f v1, lms::math::vertex2f v2, lms::math::vertex2f v3,bool filled);
    void drawPointCloud2f(const lms::math::PointCloud2f& pointCloud);
    void drawBasicObstacles(const street_environment::BasicObstacleVector &obstacles);
    void drawBoundingBox(const street_environment::BoundingBox2f &boundingBox);
    /**
     * @brief setColor
     * @param toDrawName
     * @return false if the default color was used
     */
    bool setColor(std::string toDrawName);

};

#endif /* IMAGE_HINT_TRANSFORMER_H */
